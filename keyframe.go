package slam

import "math"

func (p *Pipeline) maybeKeyframe(fr Frame, clean []Point, pose, odomPose Pose) int {
	if p.lastKF < 0 {
		return p.addKeyframeLocked(fr, clean, pose, odomPose)
	}
	// Revisit suppression: if the candidate pose coincides with an
	// already stored keyframe other than the odom-chain predecessor, reuse
	// that node instead of adding a duplicate. Repeatedly loading and
	// re-sweeping the SAME area then converges to a fixed graph/state
	// size, while genuinely new terrain (farther than the threshold from
	// every existing node) keeps adding keyframes.
	if dup := p.findRevisitKeyframeLocked(pose, p.lastKF); dup >= 0 {
		return -dup - 2
	}
	prev := p.graph.Nodes[p.lastKF]
	d := PoseDist(prev.Pose, pose)
	dt := math.Abs(normAngle(pose.Theta - prev.Pose.Theta))
	if d < p.cfg.KeyframeMinDist && dt < p.cfg.KeyframeMinTheta &&
		d < p.cfg.KeyframeMaxGap {
		return -1
	}
	return p.addKeyframeLocked(fr, clean, pose, odomPose)
}

// findRevisitKeyframeLocked returns the index of an existing keyframe whose
// optimized pose is within the revisit-suppression radius of pose, or -1.
// skip is the odom-chain predecessor, which is excluded: two consecutive
// keyframes in a fresh session are naturally spaced roughly one keyframe
// interval apart and must not suppress each other.
func (p *Pipeline) findRevisitKeyframeLocked(pose Pose, skip int) int {
	dxy := p.cfg.RevisitSuppressionXY
	dth := p.cfg.RevisitSuppressionTheta
	for i, kf := range p.graph.Nodes {
		if i == skip {
			continue
		}
		if PoseDist(kf.Pose, pose) <= dxy &&
			math.Abs(normAngle(kf.Pose.Theta-pose.Theta)) <= dth {
			return i
		}
	}
	return -1
}

func (p *Pipeline) addKeyframeLocked(fr Frame, clean []Point, pose, odomPose Pose) int {
	idx := len(p.graph.Nodes)
	scan := subsample(clean, p.cfg.MaxScanPointsKF)
	// Subsample uniformly for the surface too, but keep a denser budget:
	// matching benefits from coverage while memory stays bounded.
	surfaceScan := subsample(clean, 360)
	surface := rasterizeHits(transformAll(pose, surfaceScan), p.cfg.MapResolution)
	kf := &Keyframe{
		ID: idx, FrameID: fr.Scan.ID, Time: fr.Scan.Time,
		Pose: pose, PoseOdom: odomPose, Session: p.session, Scan: scan,
		SurfaceBase: pose, Surface: surface,
		UpdatedAt: p.frameCount,
	}
	p.graph.Nodes = append(p.graph.Nodes, kf)
	if p.lastKF >= 0 {
		prev := p.graph.Nodes[p.lastKF]
		if prev.Session != p.session {
			// Different odom origins: no relative odom edge exists.
			p.lastKF = idx
			p.lastKFOdom = odomPose
			return idx
		}
		rel := Between(prev.Pose, pose)
		info := diagInfo(p.cfg.OdomEdgeSigmaXY, p.cfg.OdomEdgeSigmaTh)
		p.graph.Edges = append(p.graph.Edges, &Edge{
			I: p.lastKF, J: idx, Kind: EdgeOdom, Measure: rel,
			Info: info, RawInfo: info, Status: LoopAccepted,
		})
	}
	p.lastKF = idx
	p.lastKFOdom = odomPose
	return idx
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
