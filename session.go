// Multi-session anchoring: committing a cross-session localization result
// and then running a new session (own odom/clock origin) inside the old
// world frame.
package slam

import (
	"fmt"
	"math"
)

// PrepareNewSession arms a loaded pipeline for a brand-new independent
// session (separate power cycle, own odom/clock origin). After this call
// ProcessFrame rejects frames until BeginSession commits a successful
// Localize result. Same-session resume after Save/Load does NOT call this:
// loaded mid-session state continues accepting frames directly.
func (p *Pipeline) PrepareNewSession() error {
	p.mu.Lock()
	defer p.mu.Unlock()
	if len(p.graph.Nodes) == 0 {
		return fmt.Errorf("slam: no prior map to localize against")
	}
	p.awaitAnchor = true
	return nil
}

// BeginSession commits a localization result and starts a new independent
// sensor session. It must be called after LoadState before processing the
// new session's frames.
//
// The first frame of the new session is placed at lr.Pose in the old WORLD
// frame. Its odometry sample becomes the new session's odom origin (odom
// deltas, not absolute odom poses, are used from here on). Anchor loop edges
// tie the new keyframe to the old keyframes whose surfaces the alignment was
// measured against, so later optimization keeps both sessions consistent.
//
// On any non-unique result the call fails and the loaded map, graph and
// trajectory are left exactly as they were: a failed or ambiguous
// localization never pollutes the old state.
func (p *Pipeline) BeginSession(fr Frame, lr LocalizeResult) error {
	p.mu.Lock()
	defer p.mu.Unlock()
	canAnchor := p.awaitAnchor ||
		// A finalized session-0 pipeline (e.g. save at end of first session)
		// is also a valid starting point for a new session.
		(p.session == 0 && len(p.graph.Nodes) > 0)
	if !canAnchor {
		if len(p.graph.Nodes) == 0 && !p.started {
			return fmt.Errorf("slam: nothing to anchor to; process frames normally for the first session")
		}
		return fmt.Errorf("slam: BeginSession called while a session is already active")
	}
	if lr.Status != LocalizeUnique {
		return fmt.Errorf("slam: refusing to anchor session: localization status=%s reason=%s score=%.2f rms=%.3f",
			lr.Status, lr.Reason, lr.Score, lr.Residual)
	}
	clean, reason := validateScan(p.cfg, fr.Scan)
	if reason != ReasonOK {
		return fmt.Errorf("slam: anchor frame rejected: %s", reason)
	}

	// Unique, validated result: commit.
	p.session++
	// Offset future frame IDs so the new session's zero-restarted source IDs
	// never collide with stored ones. Base is derived from the largest ID
	// ever seen in this pipeline state.
	var maxID int64 = -1
	for _, kf := range p.graph.Nodes {
		if kf.FrameID > maxID {
			maxID = kf.FrameID
		}
	}
	for _, tp := range p.traj {
		if tp.FrameID > maxID {
			maxID = tp.FrameID
		}
	}
	// Anchor frame gets ID maxID+1 (it is itself one new frame); the next
	// source frame (source ID 1) then maps to maxID+2 with no collision and
	// no gap.
	// Anchor source frame 0 maps to maxID+1 uniquely; subsequent source
	// frame i maps to maxID+1+i, i.e. source ID + (maxID+1). The base below
	// is set so stored ID = sourceID + frameIDBase with frameIDBase=maxID+1.
	p.frameIDBase = maxID + 1
	p.awaitAnchor = false
	p.sessionStartNodes = len(p.graph.Nodes)
	p.sessionStartTraj = len(p.traj)

	pose := lr.Pose
	p.lastPose = pose
	p.odomBase = fr.Odom.Pose // session odom origin
	p.lastOdom = Pose{}
	p.lastKFOdom = Pose{}
	p.started = true
	p.lastScan = append(p.lastScan[:0], clean...)

	idx, newNode := p.addSessionKeyframeLocked(fr, clean, pose, Pose{})
	// Only a genuinely new anchor keyframe contributes a fresh scan to the
	// grid; a reused (revisit) node's cells are already mapped.
	if newNode {
		p.grid.integrateScanLocked(pose, clean)
	}
	p.traj = append(p.traj, TrajectoryPoint{
		FrameID: fr.Scan.ID + p.frameIDBase,
		Time:    fr.Scan.Time, Pose: pose,
	})
	// Anchor edges tie a NEW session node into the old graph. When the
	// anchor reused an existing node it is already part of the old graph
	// and needs no extra constraints (adding them every round would grow
	// the edge set and the state file for an unchanged area).
	if newNode {
		p.addAnchorEdgesLocked(idx, lr)
	}

	p.frameCount++
	d := p.decide(Frame{
		Scan: Scan{ID: fr.Scan.ID + p.frameIDBase, Time: fr.Scan.Time},
	}, OutcomeAccepted, ReasonLocOK,
		fmt.Sprintf("session=%d anchored score=%.2f rms=%.3f pose=%s cands=%d",
			p.session, lr.Score, lr.Residual, pose, len(lr.Candidates)))
	p.log.Framef("session=%d ANCHOR score=%.2f rms=%.3f pose=%s",
		p.session, lr.Score, lr.Residual, pose)
	_ = d
	return nil
}

// AwaitAnchor reports whether the pipeline is loaded and waiting for a
// successful BeginSession before accepting new frames.
func (p *Pipeline) AwaitAnchor() bool {
	p.mu.RLock()
	defer p.mu.RUnlock()
	return p.awaitAnchor
}

// Session returns the current session index.
func (p *Pipeline) Session() int {
	p.mu.RLock()
	defer p.mu.RUnlock()
	return p.session
}

// SessionInfo reports the current session index and the FrameID offset:
// frames belonging to session s have stored FrameID >= the offset of the
// first session whose index is s. Offsets[i] is the first FrameID of
// session i. This lets tests partition a multi-session trajectory without
// guessing at rejected-frame gaps.
func (p *Pipeline) SessionInfo() (session int, frameIDBase int64) {
	p.mu.RLock()
	defer p.mu.RUnlock()
	return p.session, p.frameIDBase
}

// addSessionKeyframeLocked appends a keyframe tagged with the new session.
// No odom-chain edge is created to the previous keyframe: sessions have
// unrelated odom frames, and their connection is expressed exclusively by
// anchor/cross-session loop edges.
func (p *Pipeline) addSessionKeyframeLocked(fr Frame, clean []Point, pose, odomPose Pose) (int, bool) {
	scan := subsample(clean, p.cfg.MaxScanPointsKF)
	surfaceScan := subsample(clean, 360)
	surface := rasterizeHits(transformAll(pose, surfaceScan), p.cfg.MapResolution)
	// Reuse a pre-existing keyframe when the new session boots at an
	// already mapped pose (repeated load -> run overlap -> save). This is
	// the session-start counterpart of maybeKeyframe's revisit suppression
	// and is what makes repeated rounds converge to a fixed graph size.
	for i, kf := range p.graph.Nodes {
		if kf.Session == p.session {
			continue
		}
		if PoseDist(kf.Pose, pose) <= p.cfg.RevisitSuppressionXY &&
			math.Abs(normAngle(kf.Pose.Theta-pose.Theta)) <=
				p.cfg.RevisitSuppressionTheta {
			// The reused node now carries the new session's chain: retag
			// it so subsequent frames build odom edges onto it normally.
			kf.Session = p.session
			p.lastKF = i
			p.lastKFOdom = odomPose
			return i, false
		}
	}
	idx := len(p.graph.Nodes)
	kf := &Keyframe{
		ID:      idx,
		FrameID: fr.Scan.ID + p.frameIDBase,
		Time:    fr.Scan.Time,
		Pose:    pose, PoseOdom: odomPose, Session: p.session,
		Scan: scan, SurfaceBase: pose, Surface: surface,
		UpdatedAt: p.frameCount,
	}
	p.graph.Nodes = append(p.graph.Nodes, kf)
	p.lastKF = idx
	p.lastKFOdom = odomPose
	return idx, true
}

// addAnchorEdgesLocked links the new session's first keyframe to up to
// LocalizeMaxAnchors old-session keyframes near the localized pose. Each edge
// measurement is the scan-aligned world transform implied by the localizer.
func (p *Pipeline) addAnchorEdgesLocked(newKF int, lr LocalizeResult) {
	type nearKF struct {
		idx int
		d   float64
	}
	var near []nearKF
	for j, kf := range p.graph.Nodes {
		if j == newKF || kf.Session == p.session {
			continue
		}
		if d := PoseDist(kf.Pose, lr.Pose); d <= p.cfg.LocalizeKFRefRadius {
			near = append(near, nearKF{j, d})
		}
	}
	// Deterministic nearest-first order.
	for i := 0; i < len(near); i++ {
		for j := i + 1; j < len(near); j++ {
			if near[j].d < near[i].d {
				near[i], near[j] = near[j], near[i]
			}
		}
	}
	info := diagInfo(p.cfg.LocalizeEdgeSigmaXY, p.cfg.LocalizeEdgeSigmaTh)
	added := 0
	for _, nk := range near {
		old := p.graph.Nodes[nk.idx]
		// World-frame relative measurement, consistent with strong loop
		// edges (edgeError expects translation rotated by old's heading).
		measure := relToWorld(old.Pose, Between(old.Pose, lr.Pose))
		if p.hasEdgeBetween(nk.idx, newKF) {
			continue
		}
		p.graph.Edges = append(p.graph.Edges, &Edge{
			I: nk.idx, J: newKF, Kind: EdgeLoop, Measure: measure,
			Info: info, RawInfo: info, Status: LoopAccepted,
			Reason: "session_anchor",
		})
		p.log.Loopf("ANCHOR edge kf=%d -> %d measure=%s",
			nk.idx, newKF, measure)
		added++
		if added >= p.cfg.LocalizeMaxAnchors {
			break
		}
	}
}

// hasEdgeBetween reports an undirected edge between two keyframes.
func (p *Pipeline) hasEdgeBetween(a, b int) bool {
	for _, e := range p.graph.Edges {
		if (e.I == a && e.J == b) || (e.I == b && e.J == a) {
			return true
		}
	}
	return false
}
