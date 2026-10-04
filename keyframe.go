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
	// Revisit convergence: if the robot is at a vantage point another
	// session already keyframed and the scans align strongly, chain the
	// odometry edge to the existing keyframe instead of inserting a
	// duplicate node. Map and graph size then stay stable when the same
	// area is re-surveyed; new terrain (no nearby matching keyframe) grows
	// normally.
	if reuse := p.findReuseKeyframeLocked(pose, clean, -1); reuse >= 0 &&
		reuse != p.lastKF {
		p.linkReuseChainLocked(fr, reuse, pose)
		return reuse
	}
	return p.addKeyframeLocked(fr, clean, pose, odomPose)
}

// linkReuseChainLocked records that the current frame's vantage point is
// the existing node reuseIdx from another session. No new node or scan is
// added: a strong cross-session confirmation edge ties the new session's
// odom chain to the existing keyframe (same mechanism as an accepted
// cross-session revisit), so optimization keeps both sessions aligned while
// the graph cannot grow from re-surveying mapped terrain.
func (p *Pipeline) linkReuseChainLocked(fr Frame, reuseIdx int, pose Pose) {
	reuse := p.graph.Nodes[reuseIdx]
	// Only add a constraint when none exists yet (hasEdgeBetween covers
	// both the anchor and earlier revisits), so repeated rounds converge.
	if p.lastKF >= 0 && p.lastKF != reuseIdx &&
		!p.hasEdgeBetween(p.lastKF, reuseIdx) {
		info := diagInfo(p.cfg.LocalizeEdgeSigmaXY, p.cfg.LocalizeEdgeSigmaTh)
		measure := relToWorld(p.graph.Nodes[p.lastKF].Pose,
			Between(p.graph.Nodes[p.lastKF].Pose, pose))
		p.graph.Edges = append(p.graph.Edges, &Edge{
			I: p.lastKF, J: reuseIdx, Kind: EdgeLoop, Measure: measure,
			Info: info, RawInfo: info, Status: LoopAccepted,
			Reason: "revisit_reuse",
		})
	}
	p.lastKF = reuseIdx
	p.log.Framef("id=%d REVISIT reuse existing kf=%d pose=%s",
		fr.Scan.ID, reuseIdx, reuse.Pose)
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
			// Different odom origins: no ordinary odom edge exists. When the
			// session anchored by coalescing onto an existing keyframe (or
			// reuses one), the first NEW keyframe of this session still has
			// to connect the session chain to that foreign node, otherwise
			// the new leg dangles from the graph. Insert one anchor-style
			// strong link using the world-frame relative transform.
			if !p.hasEdgeBetween(p.lastKF, idx) {
				info := diagInfo(p.cfg.LocalizeEdgeSigmaXY, p.cfg.LocalizeEdgeSigmaTh)
				measure := relToWorld(prev.Pose, Between(prev.Pose, pose))
				p.graph.Edges = append(p.graph.Edges, &Edge{
					I: p.lastKF, J: idx, Kind: EdgeLoop, Measure: measure,
					Info: info, RawInfo: info, Status: LoopAccepted,
					Reason: "session_anchor_chain",
				})
			}
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
