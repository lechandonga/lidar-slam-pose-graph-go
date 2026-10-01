package slam

import "math"

func (p *Pipeline) maybeKeyframe(fr Frame, clean []Point, pose, odomPose Pose) int {
	if p.lastKF < 0 {
		return p.addKeyframeLocked(fr, clean, pose, odomPose)
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

func (p *Pipeline) addKeyframeLocked(fr Frame, clean []Point, pose, odomPose Pose) int {
	idx := len(p.graph.Nodes)
	scan := subsample(clean, p.cfg.MaxScanPointsKF)
	// Subsample uniformly for the surface too, but keep a denser budget:
	// matching benefits from coverage while memory stays bounded.
	surfaceScan := subsample(clean, 360)
	surface := rasterizeHits(transformAll(pose, surfaceScan), p.cfg.MapResolution)
	kf := &Keyframe{
		ID: idx, FrameID: fr.Scan.ID, Time: fr.Scan.Time,
		Pose: pose, PoseOdom: odomPose, Scan: scan,
		surfaceBase: pose, Surface: surface,
		UpdatedAt: p.frameCount,
	}
	p.graph.Nodes = append(p.graph.Nodes, kf)
	if p.lastKF >= 0 {
		prev := p.graph.Nodes[p.lastKF]
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
