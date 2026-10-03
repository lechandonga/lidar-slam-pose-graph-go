package slam

import (
	"fmt"
	"math"
	"sort"
)

// loopCandidate is a scan-aligned revisit hypothesis for keyframe k.
type loopCandidate struct {
	j                int
	score            float64
	scanRel, odomRel Pose
}

// tryLoopClosure searches for previously visited places near new keyframe k.
// Proximity is tested in the drift-free odometry frame; each near candidate
// is scan-aligned in the old keyframe's LOCAL frame. Accepted candidates add
// loop edges (possibly several, so the whole returning leg is corrected);
// weak alignments are downweighted and contradictions rejected, all with
// distinct, logged reasons.
func (p *Pipeline) tryLoopClosure(k int) {
	kf := p.graph.Nodes[k]
	var cands []loopCandidate
	checked := 0
	for j := 0; j < k; j++ {
		old := p.graph.Nodes[j]
		crossSession := old.Session != kf.Session
		// Within a session require index separation to skip the odom
		// chain; across sessions the index gap is irrelevant, only world
		// proximity matters.
		if !crossSession && k-j < p.cfg.LoopMinFrames {
			continue
		}
		// Odometry-frame proximity is meaningful only within one session
		// (each session's odom frame restarts at zero). Across sessions we
		// test proximity in the common world frame instead.
		if !crossSession &&
			PoseDist(old.PoseOdom, kf.PoseOdom) > p.cfg.LoopOdomRadius {
			continue
		}
		crossRadius := p.cfg.LoopOdomRadius * 2.5
		if crossSession &&
			(PoseDist(old.Pose, kf.Pose) > crossRadius ||
				math.Abs(normAngle(old.Pose.Theta-kf.Pose.Theta)) > 1.2) {
			continue
		}
		checked++
		if checked > 8 {
			break
		}
		// Alignment prediction: local odom chain within a session; the
		// current world estimate across sessions.
		var odomRel Pose
		if crossSession {
			odomRel = Between(old.Pose, kf.Pose)
		} else {
			odomRel = Between(old.PoseOdom, kf.PoseOdom)
		}
		localSurface := rasterizeHits(old.Scan, p.cfg.MapResolution)
		lcfg := p.cfg
		lcfg.MatchWinXY = math.Max(p.cfg.LoopSearchRadius, 1.0)
		lcfg.MatchWinTheta = math.Pi
		lcfg.CoarseStepXY = 0.12
		lcfg.CoarseStepTheta = 0.08
		lcfg.RefineIter = 10
		mr := matchScan(lcfg, localSurface, subsample(kf.Scan, 240), odomRel)
		if !mr.CoarseOK {
			continue
		}
		fine := buildLikelihoodField(localSurface,
			p.cfg.FieldResolution, p.cfg.FieldInflate)
		if s, n := fine.score(mr.Pose, kf.Scan); n > 0 {
			mr.Score = s
		}
		cands = append(cands, loopCandidate{
			j: j, score: mr.Score, scanRel: mr.Pose, odomRel: odomRel,
		})
	}
	if len(cands) == 0 {
		return
	}
	sort.Slice(cands, func(a, b int) bool {
		return cands[a].score > cands[b].score
	})
	raw := diagInfo(p.cfg.LoopSigmaXY, p.cfg.LoopSigmaTh)
	// Cross-session strong alignments are the ONLY link between two
	// independently drifted odom frames. Weight them like the (stiff) odom
	// chain so the optimizer actually pulls the overlap together instead of
	// letting the drifted chain dominate.
	rawCross := diagInfo(p.cfg.OdomEdgeSigmaXY*1.5, p.cfg.OdomEdgeSigmaTh*1.5)
	added := 0
	for _, cd := range cands {
		if cd.score < p.cfg.LoopMinScore {
			p.logLoop(k, cd, LoopRejected, 0, raw, 0, "loop_match_score_low")
			continue
		}
		dxy := math.Hypot(cd.scanRel.X-cd.odomRel.X, cd.scanRel.Y-cd.odomRel.Y)
		dth := math.Abs(normAngle(cd.scanRel.Theta - cd.odomRel.Theta))
		err := [3]float64{
			cd.scanRel.X - cd.odomRel.X,
			cd.scanRel.Y - cd.odomRel.Y,
			normAngle(cd.scanRel.Theta - cd.odomRel.Theta),
		}
		chi2 := 0.0
		for r := 0; r < 3; r++ {
			for c := 0; c < 3; c++ {
				chi2 += err[r] * raw[r][c] * err[c]
			}
		}
		switch {
		case dxy > p.cfg.LoopOdomMaxDiffXY || dth > p.cfg.LoopOdomMaxDiffTh:
			p.logLoop(k, cd, LoopRejected, 0, raw, chi2,
				fmt.Sprintf("loop_contradicts_odom dxy=%.2f dth=%.2f", dxy, dth))
			continue
		case cd.score >= p.cfg.LoopStrongScore:
			// Strong scan alignment. Convert the scan-frame relative
			// translation into node i's world frame for the edge measure.
			worldMeasure := relToWorld(p.graph.Nodes[cd.j].Pose, cd.scanRel)
			info := raw
			if p.graph.Nodes[cd.j].Session != kf.Session {
				info = rawCross
			}
			p.appendEdgeDedupLocked(&Edge{
				I: cd.j, J: k, Kind: EdgeLoop, Measure: worldMeasure,
				Info: info, RawInfo: info, Status: LoopAccepted, Chi2: chi2,
			})
			p.logLoop(k, cd, LoopAccepted, 1, raw, chi2,
				fmt.Sprintf("loop_strong chi2=%.1f dxy=%.2f", chi2, dxy))
			added++
		default:
			scale := (cd.score - p.cfg.LoopMinScore) /
				(p.cfg.LoopStrongScore - p.cfg.LoopMinScore)
			if scale <= 0 || scale > 1 {
				scale = 0.5
			}
			p.appendEdgeDedupLocked(&Edge{
				I: cd.j, J: k, Kind: EdgeLoop, Measure: cd.odomRel,
				Info: scaleInfo(raw, scale), RawInfo: raw, Status: LoopSoft,
				Chi2: chi2, Reason: "loop_weak_align",
			})
			p.logLoop(k, cd, LoopSoft, scale, raw, chi2,
				fmt.Sprintf("loop_weak_align score=%.2f chi2=%.1f",
					cd.score, chi2))
			added++
		}
		if added >= 2 {
			break
		}
	}
}

func (p *Pipeline) logLoop(k int, cd loopCandidate, status LoopStatus,
	scale float64, raw [3][3]float64, chi2 float64, reason string) {
	dec := LoopDecision{
		Time: p.graph.Nodes[k].Time, From: cd.j, To: k,
		Chi2: chi2, Score: cd.score, Measure: cd.odomRel,
		Status: status, WeightScale: scale, Reason: reason,
	}
	p.loopLog = append(p.loopLog, dec)
	p.log.Loopf("kf=%d -> %d status=%s score=%.2f chi2=%.1f scale=%.2f %s",
		k, cd.j, status, cd.score, chi2, scale, reason)
}

func scaleInfo(in [3][3]float64, s float64) [3][3]float64 {
	var out [3][3]float64
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			out[i][j] = in[i][j] * s
		}
	}
	return out
}

// ScanInWorld returns the keyframe scan in world coordinates.
func (k *Keyframe) ScanInWorld() []Point {
	out := make([]Point, len(k.Scan))
	for i, pt := range k.Scan {
		out[i] = k.Pose.Transform(pt)
	}
	return out
}

// SurfaceWorld returns the dense matching surface corrected to the current
// optimized pose.
func (k *Keyframe) SurfaceWorld() []Point {
	corr := k.Pose.Compose(k.surfaceBase.Inv())
	out := make([]Point, len(k.Surface))
	for i, pt := range k.Surface {
		out[i] = corr.Transform(pt)
	}
	return out
}

// runOptimizeLocked optimizes the graph and retargets non-keyframe frame
// poses at Finalize.
func (p *Pipeline) runOptimizeLocked(why string) OptimizeResult {
	before := graphCost(p.graph)
	prevPoses := make([]Pose, len(p.graph.Nodes))
	for i, kf := range p.graph.Nodes {
		prevPoses[i] = kf.Pose
	}
	p.lastPrevPoses = prevPoses
	r := optimizeGraph(p.graph, p.cfg)
	maxMove := 0.0
	for i := range p.graph.Nodes {
		if d := poseMove(prevPoses[i], p.graph.Nodes[i].Pose); d > maxMove {
			maxMove = d
		}
	}
	after := graphCost(p.graph)
	p.lastOpt = r
	p.optLog = append(p.optLog, OptLogEntry{
		Nodes: len(p.graph.Nodes), Edges: len(p.graph.Edges),
		Iterations: r.Iterations, Converged: r.Converged,
		FinalDelta: r.FinalDelta, CostBefore: before, CostAfter: after,
	})
	p.log.Optf("%s nodes=%d edges=%d iters=%d converged=%v cost %.3f -> %.3f",
		why, len(p.graph.Nodes), len(p.graph.Edges),
		r.Iterations, r.Converged, before, after)
	p.lastOptMaxMove = maxMove
	for _, kf := range p.graph.Nodes {
		kf.UpdatedAt = p.frameCount
	}
	return r
}

func poseMove(a, b Pose) float64 {
	d := math.Max(math.Abs(a.X-b.X), math.Abs(a.Y-b.Y))
	if t := math.Abs(normAngle(a.Theta - b.Theta)); t > d {
		d = t
	}
	return d
}

// retargetTrajectoryLocked applies nearest-keyframe corrections to all
// non-keyframe trajectory points; keyframe frames take optimized poses.
func (p *Pipeline) retargetTrajectoryLocked(prevPoses []Pose) {
	if len(p.graph.Nodes) == 0 {
		return
	}
	type cand struct {
		seq      int64
		old, new Pose
	}
	cands := make([]cand, 0, len(p.graph.Nodes))
	for i, kf := range p.graph.Nodes {
		cands = append(cands, cand{kf.Seq, prevPoses[i], kf.Pose})
	}
	sessBySeq := make(map[int64]int, len(cands))
	for _, kf := range p.graph.Nodes {
		sessBySeq[kf.Seq] = kf.Session
	}
	for i := range p.traj {
		tp := &p.traj[i]
		best, bestD := 0, int64(math.MaxInt64)
		bestSession := -1
		bestAny, bestAnyD := 0, 1<<52
		for j := range cands {
			d := cands[j].seq - tp.Seq
			if d < 0 {
				d = -d
			}
			if d < int64(bestAnyD) {
				bestAnyD, bestAny = int(d), j
			}
			// Prefer a keyframe from the same session: FrameIDs and clocks
			// restart independently per session, so Seq distance is only
			// meaningful within one session.
			if sessBySeq[cands[j].seq] == tp.Session {
				d -= 1 << 40
			}
			if d < bestD {
				bestD, best = d, j
				bestSession = sessBySeq[cands[j].seq]
			}
		}
		c := cands[best]
		if bestSession == tp.Session && bestD < 1<<40 {
			tp.Pose = c.new.Compose(c.old.Inv()).Compose(tp.Pose)
		} else if bestAnyD < 1<<52 {
			// Keyframe was merged into another session's node during
			// compaction: snap to the surviving node (same physical spot).
			ca := cands[bestAny]
			tp.Pose = ca.new
		}
	}
}

// trajKeyframeSession returns the session that produced keyframe seq.
func (p *Pipeline) trajKeyframeSession(seq int64) int {
	for _, kf := range p.graph.Nodes {
		if kf.Seq == seq {
			return kf.Session
		}
	}
	return -1
}

// Finalize runs the last optimization and rebuilds the grid when poses
// moved, guaranteeing map/trajectory alignment after loop closure.
func (p *Pipeline) Finalize() OptimizeResult {
	p.mu.Lock()
	defer p.mu.Unlock()
	// 1) Bring every session's keyframes to a consistent optimum.
	r := p.runOptimizeLocked("final_no_retarget")
	// 2) Merge keyframes describing the same physical spot across sessions
	//    (prevents re-scanned overlaps from duplicating nodes/edges/cells).
	if p.compactGraphLocked() {
		p.log.Optf("compaction merged cross-session keyframes: nodes=%d edges=%d",
			len(p.graph.Nodes), len(p.graph.Edges))
		pre := make([]Pose, len(p.graph.Nodes))
		for i, kf := range p.graph.Nodes {
			pre[i] = kf.Pose
		}
		r = p.runOptimizeLocked("post_compact")
		p.lastPrevPoses = pre
	}
	// 3) Apply keyframe corrections to non-keyframe trajectory points, then
	//    rebuild the grid once from final poses: trajectory and map share
	//    exactly the same poses, with no doubled contours.
	p.retargetTrajectoryLocked(p.lastPrevPoses)
	p.grid.rebuildFromKeyframes(p.graph.Nodes)
	p.lastOptMaxMove = 0
	return r
}

func (p *Pipeline) OptLog() []OptLogEntry {
	p.mu.RLock()
	defer p.mu.RUnlock()
	out := make([]OptLogEntry, len(p.optLog))
	copy(out, p.optLog)
	return out
}
