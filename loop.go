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
	cross := map[int]loopCandidate{}
	checked := 0
	for j := 0; j < k; j++ {
		old := p.graph.Nodes[j]
		sameSession := old.Session == kf.Session
		if sameSession {
			// Within one session, proximity is tested in the drift-free
			// odometry frame; far or too-recent keyframes cannot be loops
			// but can still be cross-session revisits below.
			if k-j < p.cfg.LoopMinFrames {
				continue
			}
			if PoseDist(old.PoseOdom, kf.PoseOdom) > p.cfg.LoopOdomRadius {
				continue
			}
			checked++
			if checked > 8 {
				break
			}
			odomRel := Between(old.PoseOdom, kf.PoseOdom)
			mr := p.alignKeyframesLocked(old, kf, odomRel,
				math.Max(p.cfg.LoopSearchRadius, 1.0), true)
			if !mr.CoarseOK {
				continue
			}
			cands = append(cands, loopCandidate{
				j: j, score: mr.Score, scanRel: mr.Pose, odomRel: odomRel,
			})
			continue
		}
		// Different session: odom frames are unrelated, so proximity and
		// the alignment seed come from the WORLD frame.
		if PoseDist(old.Pose, kf.Pose) > p.cfg.LocalizeCrossSessionRadius {
			continue
		}
		if p.hasEdgeBetween(j, k) {
			continue
		}
		// Seed in old's LOCAL frame: alignKeyframes matches the new local
		// scan against the old local scan. The resulting scanRel is then
		// rotated into the world frame via relToWorld, exactly like
		// same-session strong loop edges.
		localSeed := Pose{Theta: normAngle(kf.Pose.Theta - old.Pose.Theta)}
		mr := p.alignKeyframesLocked(old, kf, localSeed, 1.0, true)
		if !mr.CoarseOK {
			continue
		}
		if mr.Score < p.cfg.LocalizeCrossMinScore {
			continue
		}
		cross[j] = loopCandidate{
			j: j, score: mr.Score, scanRel: mr.Pose, odomRel: mr.Pose,
		}
	}
	// Commit strong cross-session revisits immediately. They bypass odom
	// gating (the two odom frames are incomparable) and only strong
	// alignments are accepted, so a wrong place cannot silently merge.
	crossList := make([]loopCandidate, 0, len(cross))
	for _, cd := range cross {
		crossList = append(crossList, cd)
	}
	sort.Slice(crossList, func(a, b int) bool {
		if crossList[a].score != crossList[b].score {
			return crossList[a].score > crossList[b].score
		}
		return crossList[a].j < crossList[b].j
	})
	crossAdded := 0
	for _, cd := range crossList {
		// One strong cross-session link per new keyframe is enough given
		// the intra-session chain and the start anchor; adding several
		// identical-score edges for the same overlap only bloats the graph.
		if crossAdded >= 1 {
			break
		}
		p.addCrossSessionEdgeLocked(k, cd)
		crossAdded++
	}
	if len(cands) == 0 {
		return
	}
	sort.Slice(cands, func(a, b int) bool {
		return cands[a].score > cands[b].score
	})
	raw := diagInfo(p.cfg.LoopSigmaXY, p.cfg.LoopSigmaTh)
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
			worldMeasure := relToWorld(p.graph.Nodes[cd.j].Pose, cd.scanRel)
			p.graph.Edges = append(p.graph.Edges, &Edge{
				I: cd.j, J: k, Kind: EdgeLoop, Measure: worldMeasure,
				Info: raw, RawInfo: raw, Status: LoopAccepted, Chi2: chi2,
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
			p.graph.Edges = append(p.graph.Edges, &Edge{
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

// alignKeyframesLocked scan-matches new keyframe k against old keyframe j's
// local surface, starting from seed (relative motion in old's local frame
// for same-session candidates, world-relative for cross-session ones).
func (p *Pipeline) alignKeyframesLocked(old, kf *Keyframe, seed Pose,
	winXY float64, rescore bool) MatchResult {
	localSurface := rasterizeHits(old.Scan, p.cfg.MapResolution)
	lcfg := p.cfg
	lcfg.MatchWinXY = winXY
	lcfg.MatchWinTheta = math.Pi
	lcfg.CoarseStepXY = 0.12
	lcfg.CoarseStepTheta = 0.08
	lcfg.RefineIter = 10
	mr := matchScan(lcfg, localSurface, subsample(kf.Scan, 240), seed)
	if !mr.CoarseOK {
		return mr
	}
	if rescore {
		fine := buildLikelihoodField(localSurface,
			p.cfg.FieldResolution, p.cfg.FieldInflate)
		if s, n := fine.score(mr.Pose, kf.Scan); n > 0 {
			mr.Score = s
		}
	}
	return mr
}

// addCrossSessionEdgeLocked inserts an accepted cross-session revisit edge.
// The measure is the scan-aligned world-frame relative transform; only
// strong matches reach here (see LocalizeCrossMinScore).
func (p *Pipeline) addCrossSessionEdgeLocked(k int, cd loopCandidate) {
	info := diagInfo(p.cfg.LocalizeEdgeSigmaXY, p.cfg.LocalizeEdgeSigmaTh)
	worldMeasure := relToWorld(p.graph.Nodes[cd.j].Pose, cd.scanRel)
	if p.hasEdgeBetween(cd.j, k) {
		return
	}
	p.graph.Edges = append(p.graph.Edges, &Edge{
		I: cd.j, J: k, Kind: EdgeLoop, Measure: worldMeasure,
		Info: info, RawInfo: info, Status: LoopAccepted,
		Reason: "cross_session_revisit",
	})
	dec := LoopDecision{
		Time: p.graph.Nodes[k].Time, From: cd.j, To: k,
		Score: cd.score, Measure: worldMeasure,
		Status: LoopAccepted, WeightScale: 1,
		Reason: "cross_session_revisit",
	}
	p.loopLog = append(p.loopLog, dec)
	p.trimLoopLogLocked()
	p.log.Loopf("kf=%d -> %d CROSS-SESSION score=%.2f measure=%s",
		k, cd.j, cd.score, worldMeasure)
}

func (p *Pipeline) logLoop(k int, cd loopCandidate, status LoopStatus,
	scale float64, raw [3][3]float64, chi2 float64, reason string) {
	dec := LoopDecision{
		Time: p.graph.Nodes[k].Time, From: cd.j, To: k,
		Chi2: chi2, Score: cd.score, Measure: cd.odomRel,
		Status: status, WeightScale: scale, Reason: reason,
	}
	p.loopLog = append(p.loopLog, dec)
	p.trimLoopLogLocked()
	p.log.Loopf("kf=%d -> %d status=%s score=%.2f chi2=%.1f scale=%.2f %s",
		k, cd.j, status, cd.score, chi2, scale, reason)
}

func (p *Pipeline) trimLoopLogLocked() {
	if m := p.cfg.MaxLoopLog; m > 0 && len(p.loopLog) > m {
		p.loopLog = p.loopLog[len(p.loopLog)-m:]
	}
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
	corr := k.Pose.Compose(k.SurfaceBase.Inv())
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
	if m := p.cfg.MaxOptLog; m > 0 && len(p.optLog) > m {
		p.optLog = p.optLog[len(p.optLog)-m:]
	}
	p.log.Optf("%s nodes=%d edges=%d iters=%d converged=%v cost %.3f -> %.3f",
		why, len(p.graph.Nodes), len(p.graph.Edges),
		r.Iterations, r.Converged, before, after)
	if why == "final" {
		p.retargetTrajectoryLocked(prevPoses)
	}
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
	exact := make(map[int64]Pose, len(p.graph.Nodes))
	type cand struct {
		fid      int64
		old, new Pose
	}
	cands := make([]cand, 0, len(p.graph.Nodes))
	for i, kf := range p.graph.Nodes {
		exact[kf.FrameID] = kf.Pose
		cands = append(cands, cand{kf.FrameID, prevPoses[i], kf.Pose})
	}
	for i := range p.traj {
		tp := &p.traj[i]
		if q, ok := exact[tp.FrameID]; ok {
			tp.Pose = q
			continue
		}
		best, bestD := 0, int64(math.MaxInt64)
		for j := range cands {
			d := cands[j].fid - tp.FrameID
			if d < 0 {
				d = -d
			}
			if d < bestD {
				bestD, best = d, j
			}
		}
		c := cands[best]
		tp.Pose = c.new.Compose(c.old.Inv()).Compose(tp.Pose)
	}
}

// Finalize runs the last optimization and rebuilds the grid when poses
// moved, guaranteeing map/trajectory alignment after loop closure.
func (p *Pipeline) Finalize() OptimizeResult {
	p.mu.Lock()
	defer p.mu.Unlock()
	// A session that added no keyframes only re-swept existing terrain.
	// Its trajectory segment is physically identical to one already stored,
	// so drop it before persisting: repeated load/run/save over an
	// unchanged area then keeps the exact single-merge trajectory.
	if p.session > 0 && len(p.graph.Nodes) == p.sessionStartNodes &&
		len(p.traj) > p.sessionStartTraj {
		p.traj = p.traj[:p.sessionStartTraj]
	}
	r := p.runOptimizeLocked("final")
	if p.lastOptMaxMove > p.cfg.OptConvergeDelta {
		p.grid.rebuildFromKeyframes(p.graph.Nodes)
		p.lastOptMaxMove = 0
	}
	return r
}

func (p *Pipeline) OptLog() []OptLogEntry {
	p.mu.RLock()
	defer p.mu.RUnlock()
	out := make([]OptLogEntry, len(p.optLog))
	copy(out, p.optLog)
	return out
}
