package slam

import "math"

func (p *Pipeline) maybeKeyframe(fr Frame, clean []Point, pose, odomPose Pose, session int) int {
	prevInSession, hasPrev := p.lastKFBySession[session]
	if !hasPrev {
		return p.addKeyframeLocked(fr, clean, pose, odomPose, session)
	}
	prev := p.graph.Nodes[prevInSession]
	d := PoseDist(prev.Pose, pose)
	dt := math.Abs(normAngle(pose.Theta - prev.Pose.Theta))
	if d < p.cfg.KeyframeMinDist && dt < p.cfg.KeyframeMinTheta &&
		d < p.cfg.KeyframeMaxGap {
		return -1
	}
	return p.addKeyframeLocked(fr, clean, pose, odomPose, session)
}

func (p *Pipeline) addKeyframeLocked(fr Frame, clean []Point, pose, odomPose Pose, session int) int {
	idx := len(p.graph.Nodes)
	scan := subsample(clean, p.cfg.MaxScanPointsKF)
	// Subsample uniformly for the surface too, but keep a denser budget:
	// matching benefits from coverage while memory stays bounded.
	surfaceScan := subsample(clean, 360)
	surface := rasterizeHits(transformAll(pose, surfaceScan), p.cfg.MapResolution)
	kf := &Keyframe{
		ID: idx, FrameID: fr.Scan.ID, Seq: p.frameCount, Session: session,
		Time: fr.Scan.Time,
		Pose: pose, PoseOdom: odomPose, Scan: scan,
		surfaceBase: pose, Surface: surface,
		UpdatedAt: p.frameCount,
	}
	p.graph.Nodes = append(p.graph.Nodes, kf)
	if prevIdx, ok := p.lastKFBySession[session]; ok {
		prev := p.graph.Nodes[prevIdx]
		rel := Between(prev.Pose, pose)
		info := diagInfo(p.cfg.OdomEdgeSigmaXY, p.cfg.OdomEdgeSigmaTh)
		p.appendEdgeDedupLocked(&Edge{
			I: prevIdx, J: idx, Kind: EdgeOdom, Measure: rel,
			Info: info, RawInfo: info, Status: LoopAccepted,
		})
	}
	p.lastKF = idx
	p.lastKFOdom = odomPose
	p.lastKFBySession[session] = idx
	return idx
}

// appendEdgeDedupLocked keeps at most one edge of the same kind per node
// pair: repeatedly revisiting the same overlap across saved/reloaded
// sessions must not pile up duplicate constraints. An existing edge is
// replaced only when the new measure has a smaller residual energy.
func (p *Pipeline) appendEdgeDedupLocked(e *Edge) int {
	lo, hi := e.I, e.J
	if lo > hi {
		lo, hi = hi, lo
	}
	for k, old := range p.graph.Edges {
		if old.Kind == e.Kind {
			a, b := old.I, old.J
			if a > b {
				a, b = b, a
			}
			if a == lo && b == hi {
				xi := p.graph.Nodes[old.I].Pose
				xj := p.graph.Nodes[old.J].Pose
				oldE := edgeError(xi, xj, old.Measure)
				newE := edgeError(xi, xj, e.Measure)
				oldQ, newQ := 0.0, 0.0
				for r := 0; r < 3; r++ {
					oldQ += oldE[r] * e.Info[r][r] * oldE[r]
					newQ += newE[r] * e.Info[r][r] * newE[r]
				}
				if newQ < oldQ {
					p.graph.Edges[k] = e
				}
				return k
			}
		}
	}
	p.graph.Edges = append(p.graph.Edges, e)
	return len(p.graph.Edges) - 1
}

func diagInfo(sigXY, sigTh float64) [3][3]float64 {
	xy := 1 / (sigXY * sigXY)
	th := 1 / (sigTh * sigTh)
	return [3][3]float64{
		{xy, 0, 0},
		{0, xy, 0},
		{0, 0, th},
	}
}

// subsample keeps at most maxN points with a fixed stride (deterministic).
func subsample(pts []Point, maxN int) []Point {
	if maxN <= 0 || len(pts) <= maxN {
		out := make([]Point, len(pts))
		copy(out, pts)
		return out
	}
	out := make([]Point, maxN)
	step := float64(len(pts)) / float64(maxN)
	for i := 0; i < maxN; i++ {
		out[i] = pts[int(float64(i)*step)]
	}
	return out
}

func transformAll(pose Pose, pts []Point) []Point {
	out := make([]Point, len(pts))
	for i, pt := range pts {
		out[i] = pose.Transform(pt)
	}
	return out
}
