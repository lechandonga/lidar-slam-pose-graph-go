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

	pose := lr.Pose
	p.lastPose = pose
	p.odomBase = fr.Odom.Pose // session odom origin
	p.lastOdom = Pose{}
	p.lastKFOdom = Pose{}
	p.started = true
	p.lastScan = append(p.lastScan[:0], clean...)

	idx := p.addSessionKeyframeLocked(fr, clean, pose, Pose{})
	p.grid.integrateScanLocked(pose, clean)
	p.traj = append(p.traj, TrajectoryPoint{
		FrameID: fr.Scan.ID + p.frameIDBase,
		Time:    fr.Scan.Time, Pose: pose,
	})
	p.addAnchorEdgesLocked(idx, lr)
	// If the anchored vantage point is geometrically the same place an
	// earlier session already keyframed (repeated load/run/save over the
	// same overlap), coalesce the duplicate node: keep the new trajectory
	// point (its FrameID must stay unique) but retarget it at the existing
	// keyframe and drop the just-added node/anchor edges. Repeated
	// re-surveys then cannot grow the graph round after round.
	if reuse := p.findReuseKeyframeLocked(pose, clean, idx); reuse >= 0 {
		p.coalesceAnchorNodeLocked(idx, reuse, fr, pose)
		idx = reuse
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

// findReuseKeyframeLocked returns an existing keyframe of another session
// close to pose whose scan aligns strongly with clean at pose, or -1.
func (p *Pipeline) findReuseKeyframeLocked(pose Pose, clean []Point, newIdx int) int {
	best, bestScore := -1, p.cfg.RevisitReuseMinScore
	var globalField *likelihoodField
	if p.grid != nil && p.grid.w > 0 {
		globalField = buildLikelihoodField(p.grid.hitCellsWorld(),
			p.cfg.FieldResolution, p.cfg.FieldInflate)
	}
	for j, kf := range p.graph.Nodes {
		if j == newIdx || kf.Session == p.session {
			continue
		}
		// Pre-filter generously: the live pose estimate may have drifted by
		// a fraction of a metre relative to the earlier pass. The tight
		// decision is made on the CONVERGED scan alignment below, not on
		// this raw distance.
		if PoseDist(kf.Pose, pose) > p.cfg.RevisitSearchRadius {
			continue
		}
		if math.Abs(normAngle(kf.Pose.Theta-pose.Theta)) > p.cfg.RevisitSearchTheta {
			continue
		}
		localSurface := rasterizeHits(kf.Scan, p.cfg.MapResolution)
		lcfg := p.cfg
		// Search window covers inter-pass drift; reuse is accepted only if
		// the converged relative transform is ~zero (same physical vantage)
		// AND the implied world pose lands on the candidate.
		lcfg.MatchWinXY = p.cfg.RevisitSearchRadius
		lcfg.MatchWinTheta = p.cfg.RevisitSearchTheta
		lcfg.CoarseStepXY = 0.12
		lcfg.CoarseStepTheta = 0.08
		lcfg.RefineIter = 12
		mr := matchScan(lcfg, localSurface, subsample(clean, 240),
			Between(kf.Pose, pose))
		moved := math.Hypot(mr.Pose.X, mr.Pose.Y)
		turned := math.Abs(mr.Pose.Theta)
		implied := kf.Pose.Compose(mr.Pose)
		sameVantage := moved <= p.cfg.RevisitReuseRadius &&
			turned <= p.cfg.RevisitReuseTheta &&
			PoseDist(implied, kf.Pose) <= p.cfg.RevisitReuseRadius &&
			math.Abs(normAngle(implied.Theta-kf.Pose.Theta)) <= p.cfg.RevisitReuseTheta
		if mr.CoarseOK && mr.Score > bestScore && sameVantage {
			// Independent confirmation against the GLOBAL map: a nearby
			// keyframe in freshly explored terrain is absent from the grid,
			// so a local scan that merely resembles a distant corridor
			// segment cannot trigger reuse of an actually-new vantage.
			if globalField != nil {
				gs, _ := relocWeightedScore(globalField, pose, clean)
				if gs < p.cfg.LocalizeMinScore {
					continue
				}
			}
			// Prefer a candidate already connected to the running chain:
			// this makes the reuse mapping identical across repeated
			// replays of the same data, so graph size converges instead of
			// linking a slightly different keyframe each round.
			best, bestScore = j, mr.Score
		}
	}
	return best
}

// coalesceAnchorNodeLocked removes the freshly appended duplicate anchor
// node newIdx and its edges, links session bookkeeping to the existing
// keyframe reuseIdx, and retargets the anchor trajectory point at it.
func (p *Pipeline) coalesceAnchorNodeLocked(newIdx, reuseIdx int, fr Frame, pose Pose) {
	reuse := p.graph.Nodes[reuseIdx]
	kept := p.graph.Edges[:0]
	for _, e := range p.graph.Edges {
		if e.I == newIdx || e.J == newIdx {
			continue
		}
		kept = append(kept, e)
	}
	p.graph.Edges = kept
	p.graph.Nodes = append(p.graph.Nodes[:newIdx], p.graph.Nodes[newIdx+1:]...)
	for i, kf := range p.graph.Nodes {
		kf.ID = i
	}
	// Retarget the just-appended anchor trajectory point to the reused
	// keyframe's pose (same physical vantage point).
	last := &p.traj[len(p.traj)-1]
	last.Pose = reuse.Pose
	p.lastPose = reuse.Pose
	p.lastKF = reuseIdx
	p.lastKFOdom = Pose{}
	p.log.Framef("session=%d anchor COALESCED onto existing kf=%d pose=%s",
		p.session, reuseIdx, reuse.Pose)
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
func (p *Pipeline) addSessionKeyframeLocked(fr Frame, clean []Point, pose, odomPose Pose) int {
	idx := len(p.graph.Nodes)
	scan := subsample(clean, p.cfg.MaxScanPointsKF)
	surfaceScan := subsample(clean, 360)
	surface := rasterizeHits(transformAll(pose, surfaceScan), p.cfg.MapResolution)
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
	return idx
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
