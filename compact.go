package slam

import "math"

// compactGraphLocked merges keyframes from DIFFERENT sessions that describe
// the same physical spot after optimization. It is what keeps repeated
// "save -> reload -> re-scan overlapping area" cycles from multiplying
// nodes, constraints and map cells:
//
//   - only cross-session node pairs are merged (within a session the
//     front-end's own keyframe spacing already prevents duplicates, and
//     chaining must stay intact);
//   - matching is 1:1 and chained: new-session nodes snap onto distinct
//     older-session nodes within CompactDistXY / CompactDistTh;
//   - edges are remapped, deduplicated per (kind, pair) and re-checked:
//     an odom-chain edge whose endpoints collapsed onto incompatible poses
//     is dropped (it crossed the relocated session boundary);
//   - keyframes are re-indexed and node 0 is preserved, so the optimizer
//     keeps a fixed anchor.
//
// Returns true when something changed. The caller must re-optimize and
// rebuild afterwards.
func (p *Pipeline) compactGraphLocked() bool {
	n := len(p.graph.Nodes)
	if n < 2 {
		return false
	}
	parent := make([]int, n)
	for i := range parent {
		parent[i] = i
	}
	var find func(int) int
	find = func(a int) int {
		for parent[a] != a {
			parent[a] = parent[parent[a]]
			a = parent[a]
		}
		return a
	}
	union := func(a, b int) {
		ra, rb := find(a), find(b)
		if ra != rb {
			// Keep the lower index (older session) as the representative.
			if ra > rb {
				ra, rb = rb, ra
			}
			parent[rb] = ra
		}
	}
	used := make(map[int]bool)
	// Process sessions oldest first so representatives are stable.
	for sNew := 1; sNew <= p.session; sNew++ {
		var newNodes []int
		for i, kf := range p.graph.Nodes {
			if kf.Session == sNew {
				newNodes = append(newNodes, i)
			}
		}
		for _, i := range newNodes {
			a := p.graph.Nodes[i]
			bestJ, bestD := -1, math.MaxFloat64
			for j, b := range p.graph.Nodes {
				if b.Session >= sNew || find(j) == find(i) {
					continue
				}
				root := find(j)
				if used[root] {
					continue // 1:1: one old keyframe absorbs at most one new one
				}
				d := PoseDist(a.Pose, b.Pose)
				dth := math.Abs(normAngle(a.Pose.Theta - b.Pose.Theta))
				if d <= p.cfg.CompactDistXY && dth <= p.cfg.CompactDistTh &&
					d < bestD {
					bestD, bestJ = d, j
				}
			}
			if bestJ >= 0 {
				union(i, bestJ)
				used[find(bestJ)] = true
			}
		}
	}
	// Representatives only change when at least one union happened.
	changed := false
	for i := range parent {
		if find(i) != i {
			changed = true
			break
		}
	}
	if !changed {
		return false
	}

	// Remap edges onto representatives; drop odom edges that now connect
	// two merged keyframes with contradictory relative motion.
	type edgeKey struct {
		lo, hi int
		kind   EdgeKind
	}
	kept := make(map[edgeKey]*Edge)
	for _, e := range p.graph.Edges {
		ri, rj := find(e.I), find(e.J)
		if ri == rj {
			continue // self-constraint after merge
		}
		if ri > rj {
			ri, rj = rj, ri
		}
		ne := *e
		ne.I, ne.J = find(e.I), find(e.J)
		if ne.Kind == EdgeOdom {
			want := Between(p.graph.Nodes[ne.I].Pose, p.graph.Nodes[ne.J].Pose)
			d := PoseDist(want, ne.Measure)
			dth := math.Abs(normAngle(want.Theta - ne.Measure.Theta))
			if d > p.cfg.CompactDistXY || dth > p.cfg.CompactDistTh {
				continue
			}
			ne.Measure = want
		}
		key := edgeKey{ri, rj, ne.Kind}
		if old, ok := kept[key]; ok {
			// Keep the stronger constraint of the duplicate pair.
			if edgeStrength(&ne) > edgeStrength(old) {
				kept[key] = &ne
			}
			continue
		}
		kept[key] = &ne
	}
	var edges []*Edge
	for _, e := range kept {
		edges = append(edges, e)
	}

	// Rebuild the node slice in representative order, re-indexing.
	var reps []*Keyframe
	oldToNew := make(map[int]int, n)
	zero := -1
	for i, kf := range p.graph.Nodes {
		if find(i) != i {
			continue
		}
		if i == 0 {
			zero = len(reps)
		}
		oldToNew[i] = len(reps)
		reps = append(reps, kf)
	}
	// Node 0 must keep index 0 for the fixed-anchor optimizer convention.
	if zero > 0 {
		reps[0], reps[zero] = reps[zero], reps[0]
		for oi, ni := range oldToNew {
			if ni == 0 {
				oldToNew[oi] = zero
			}
		}
		oldToNew[0] = 0
	}
	for idx, kf := range reps {
		kf.ID = idx
	}
	for _, e := range edges {
		e.I = oldToNew[e.I]
		e.J = oldToNew[e.J]
	}
	// Deterministic edge order.
	sortEdges(edges)

	// Apply the same node mapping to trajectory keyframe references:
	// retargetTrajectoryLocked rebuilds from keyframe FrameIDs afterwards,
	// but FrameIDs are duplicated across sessions, so match on Seq.
	exact := make(map[int64]Pose, len(reps))
	for _, kf := range reps {
		exact[kf.Seq] = kf.Pose
	}
	p.graph.Nodes = reps
	p.graph.Edges = edges
	p.lastKF = len(reps) - 1
	p.lastKFBySession = map[int]int{}
	// Rebuild per-session keyframe tails in node order.
	for idx, kf := range reps {
		p.lastKFBySession[kf.Session] = idx
	}
	return true
}

// edgeStrength ranks duplicate constraints (trace of information matrix).
func edgeStrength(e *Edge) float64 {
	return e.Info[0][0] + e.Info[1][1] + e.Info[2][2]
}
