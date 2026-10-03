package slam

import (
	"fmt"
	"math"
	"strings"
	"sync"
)

// TrajectoryPoint is one fused per-frame pose in the world frame.
type TrajectoryPoint struct {
	FrameID int64
	Time    float64
	Pose    Pose
	Seq     int64 // pipeline-wide unique sequence across sessions
	Session int   // session this point belongs to
}

// Pipeline is the local 2-D SLAM front/back-end. State mutates only under
// mu; queries and persistence take the same lock, so concurrent collection,
// optimization and map reads never observe an intermediate state.
type Pipeline struct {
	mu    sync.RWMutex
	cfg   Config
	log   Logger
	graph *Graph
	grid  *Grid
	traj  []TrajectoryPoint

	frameCount int64
	lastPose   Pose
	lastOdom   Pose
	started    bool
	lastScan   []Point // cleaned scan at last accepted pose

	lastKF      int
	lastKFOdom  Pose
	kfsSinceOpt int

	// Session support. Each session has its own odom frame and clock;
	// session >= 1 is "in session". lastKFBySession[s] is the most recent
	// keyframe created by session s, so odom-chain edges never span the
	// reset boundary between sessions.
	session         int
	pendingSession  bool // BeginSessionLocalize failed/ambiguous; awaiting retry
	localizeStart   Pose // accepted fix for the pending/active session
	lastKFBySession map[int]int
	localizeLog     []LocalizeDecision

	decisions      []FrameDecision
	loopLog        []LoopDecision
	optLog         []OptLogEntry
	lastOpt        OptimizeResult
	lastOptMaxMove float64
	lastPrevPoses  []Pose
}

// LoopDecision records an attempted loop constraint and its classification.
type LoopDecision struct {
	Time        float64
	From, To    int
	Status      LoopStatus
	Chi2        float64
	Score       float64
	WeightScale float64
	Reason      string
	Measure     Pose
}

// LocalizeDecision records a cross-session localization attempt with its
// quality numbers so operators can judge why a fix was or was not applied.
type LocalizeDecision struct {
	Session     int
	Status      LocalizeStatus
	Reason      string
	Best        LocalizeHypothesis
	Second      LocalizeHypothesis
	ScoreMargin float64
	WithHint    bool
}

// OptLogEntry captures one optimization call for tests and logs.
type OptLogEntry struct {
	Nodes      int
	Edges      int
	Iterations int
	Converged  bool
	FinalDelta float64
	CostBefore float64
	CostAfter  float64
}

func NewPipeline(cfg Config, logger Logger) *Pipeline {
	cfg.validate()
	if logger == nil {
		logger = NopLogger()
	}
	return &Pipeline{
		cfg:             cfg,
		log:             logger,
		graph:           &Graph{},
		grid:            newGrid(cfg),
		lastKF:          -1,
		lastKFBySession: map[int]int{},
	}
}

// SessionID reports the currently active session (0 for the first run).
func (p *Pipeline) SessionID() int {
	p.mu.RLock()
	defer p.mu.RUnlock()
	return p.session
}

// LocalizeDecisions returns the log of all cross-session localization
// attempts, including failures and ambiguous fixes that changed nothing.
func (p *Pipeline) LocalizeDecisions() []LocalizeDecision {
	p.mu.RLock()
	defer p.mu.RUnlock()
	out := make([]LocalizeDecision, len(p.localizeLog))
	copy(out, p.localizeLog)
	return out
}

func (p *Pipeline) Run(src DataSource) error {
	for {
		fr, ok := src.Next()
		if !ok {
			return nil
		}
		p.ProcessFrame(fr)
	}
}

func (p *Pipeline) Decisions() []FrameDecision {
	p.mu.RLock()
	defer p.mu.RUnlock()
	out := make([]FrameDecision, len(p.decisions))
	copy(out, p.decisions)
	return out
}

func (p *Pipeline) LoopDecisions() []LoopDecision {
	p.mu.RLock()
	defer p.mu.RUnlock()
	out := make([]LoopDecision, len(p.loopLog))
	copy(out, p.loopLog)
	return out
}

func (p *Pipeline) Trajectory() []TrajectoryPoint {
	p.mu.RLock()
	defer p.mu.RUnlock()
	out := make([]TrajectoryPoint, len(p.traj))
	copy(out, p.traj)
	return out
}

// MapSnapshot returns an immutable grid copy. It takes the pipeline read
// lock, so map reads never observe a half-updated grid while the SLAM loop
// is integrating a scan or rebuilding after loop closure.
func (p *Pipeline) MapSnapshot() *MapSnapshot {
	p.mu.RLock()
	defer p.mu.RUnlock()
	return p.grid.snapshot()
}

func (p *Pipeline) GraphSummary() (nodes, edges int) {
	p.mu.RLock()
	defer p.mu.RUnlock()
	return len(p.graph.Nodes), len(p.graph.Edges)
}

func (p *Pipeline) LastOptimize() OptimizeResult {
	p.mu.RLock()
	defer p.mu.RUnlock()
	return p.lastOpt
}

// ProcessFrame handles one synchronized input frame and records a decision.
func (p *Pipeline) ProcessFrame(fr Frame) FrameDecision {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.frameCount++

	clean, scanReason := validateScan(p.cfg, fr.Scan)
	if scanReason != ReasonOK {
		d := p.decide(fr, OutcomeRejected, scanReason,
			fmt.Sprintf("raw_points=%d", len(fr.Scan.Points)))
		p.log.Framef("id=%d t=%.3f REJECT %s raw_points=%d",
			fr.Scan.ID, fr.Scan.Time, scanReason, len(fr.Scan.Points))
		return d
	}

	if p.pendingSession {
		d := p.decide(fr, OutcomeRejected, ReasonSessionNotLocalized,
			"session start not localized; call BeginSessionLocalize")
		p.log.Framef("id=%d REJECT %s", fr.Scan.ID,
			ReasonSessionNotLocalized)
		return d
	}

	if !p.started {
		return p.bootstrap(fr, clean)
	}

	if fr.Restart {
		d := p.advanceScanOnly(fr, clean, true)
		p.lastOdom = fr.Odom.Pose
		return d
	}
	if !fr.HasOdom || !fr.Odom.Valid {
		return p.advanceScanOnly(fr, clean, false)
	}

	odomDelta := Between(p.lastOdom, fr.Odom.Pose)
	if math.Hypot(odomDelta.X, odomDelta.Y) > p.cfg.RestartJumpXY ||
		math.Abs(odomDelta.Theta) > p.cfg.RestartJumpTheta {
		d := p.advanceScanOnly(fr, clean, true)
		p.lastOdom = fr.Odom.Pose
		return d
	}

	pred := p.lastPose.Compose(odomDelta)
	ref := p.referencePointsLocked(pred)
	mr := matchScan(p.cfg, ref, clean, pred)
	// Frame-to-frame ICP fallback using the previous scan only.
	if !mr.CoarseOK && len(p.lastScan) > 0 {
		prevWorld := transformAll(p.lastPose, p.lastScan)
		fcfg := p.cfg
		fcfg.MatchWinXY = 0.35
		fcfg.MatchWinTheta = 0.3
		mr = matchScan(fcfg, prevWorld, clean, pred)
	}
	if !mr.CoarseOK {
		return p.commitDegraded(fr, odomDelta, ReasonDegenerate, "no coarse match")
	}
	dTrans := math.Hypot(mr.Pose.X-pred.X, mr.Pose.Y-pred.Y)
	dRot := math.Abs(normAngle(mr.Pose.Theta - pred.Theta))
	if dTrans > p.cfg.MaxMatchTransErr || dRot > p.cfg.MaxMatchRotErr {
		return p.commitDegraded(fr, odomDelta, ReasonMatchDivergence,
			fmt.Sprintf("dxy=%.3f dth=%.3f score=%.2f", dTrans, dRot, mr.Score))
	}

	matchDelta := Between(p.lastPose, mr.Pose)
	weakAny := mr.WeakAxes[0] || mr.WeakAxes[1] || mr.WeakAxes[2]
	// Relative-motion consistency: even when the absolute match is close
	// to the prediction, its implied per-frame delta must agree with the
	// odometry delta. A sudden disagreement on an otherwise fine-looking
	// match is a wrong-seed alignment; reject the match and trust odom.
	ddx := matchDelta.X - odomDelta.X
	ddy := matchDelta.Y - odomDelta.Y
	if d := math.Hypot(ddx, ddy); d > p.cfg.MaxDeltaConflict {
		return p.commitDegraded(fr, odomDelta, ReasonMatchDivergence,
			fmt.Sprintf("delta_conflict=%.3f score=%.2f", d, mr.Score))
	}
	if mr.EigRatio < p.cfg.MinEigRatio {
		return p.advanceFused(fr, clean, odomDelta, matchDelta,
			p.cfg.DegradedMatchWeightXY, p.cfg.DegradedMatchWeightTheta,
			OutcomeDegraded, ReasonDegenerate,
			fmt.Sprintf("eig=%.2e score=%.2f", mr.EigRatio, mr.Score))
	}

	wxy := gaussianWeights(p.cfg.SigmaMatchXY, p.cfg.SigmaOdomXY)
	wth := gaussianWeights(p.cfg.SigmaMatchTheta, p.cfg.SigmaOdomTheta)
	// Confidence-adaptive weighting: the inverse-variance weight is scaled
	// by the ICP inlier fraction. On a poor match the odometry (which is
	// locally smooth and unbiased) dominates instead of being overruled by
	// a marginal scan alignment.
	wxy *= mr.Score * mr.Score
	wth *= mr.Score * mr.Score
	if wxy > 1 {
		wxy = 1
	}
	if wth > 1 {
		wth = 1
	}
	outcome2, reason2 := OutcomeAccepted, ReasonOK
	detail2 := fmt.Sprintf("score=%.2f eig=%.2e dxy=%.3f dth=%.3f",
		mr.Score, mr.EigRatio, dTrans, dRot)
	if weakAny {
		// Degenerate along at least one SE2 direction: fuse component-wise
		// so the weak direction follows odometry while strong directions
		// keep scan alignment. Reported as a degraded frame.
		if mr.WeakAxes[0] || mr.WeakAxes[1] {
			wxy = p.cfg.DegradedMatchWeightXY
		}
		if mr.WeakAxes[2] {
			wth = p.cfg.DegradedMatchWeightTheta
		}
		outcome2 = OutcomeDegraded
		reason2 = ReasonDegenerate
		detail2 = fmt.Sprintf("weak_axes=%v %s", mr.WeakAxes, detail2)
	}
	return p.advanceFused(fr, clean, odomDelta, matchDelta, wxy, wth,
		outcome2, reason2, detail2)
}

func (p *Pipeline) decide(fr Frame, o Outcome, reason, detail string) FrameDecision {
	d := FrameDecision{FrameID: fr.Scan.ID, Time: fr.Scan.Time,
		Outcome: o, Reason: reason, Detail: detail}
	p.decisions = append(p.decisions, d)
	return d
}

func (p *Pipeline) bootstrap(fr Frame, clean []Point) FrameDecision {
	base := Pose{}
	p.lastPose = base
	p.lastOdom = fr.Odom.Pose
	p.started = true
	p.addKeyframeLocked(fr, clean, base, fr.Odom.Pose, p.session)
	p.grid.integrateScanLocked(base, clean)
	p.traj = append(p.traj, TrajectoryPoint{
		FrameID: fr.Scan.ID, Time: fr.Scan.Time, Pose: base,
		Seq: p.frameCount, Session: p.session,
	})
	d := p.decide(fr, OutcomeAccepted, ReasonOK, "bootstrap")
	p.log.Framef("id=%d t=%.3f ACCEPT bootstrap points=%d",
		fr.Scan.ID, fr.Scan.Time, len(clean))
	return d
}

// anchorSessionLocked fixes the first accepted scan of a new, independently
// clocked/odometry session at its globally localized world pose and anchors
// it to the old graph with a relocation constraint. The odom chain for the
// new session starts from zero and is NOT linked to the old session's tail.
func (p *Pipeline) anchorSessionLocked(fr Frame, clean []Point, fix Pose) (FrameDecision, int) {
	p.lastPose = fix
	p.lastOdom = fr.Odom.Pose
	p.lastScan = append(p.lastScan[:0], clean...)
	p.started = true
	p.pendingSession = false
	kfIdx := p.addKeyframeLocked(fr, clean, fix, fr.Odom.Pose, p.session)
	// Relocation edge to the geometrically nearest older-session keyframe.
	// The measure is Between(old,new); node 0's fixed pose is unaffected.
	bestJ, bestD := -1, math.MaxFloat64
	for j, kf := range p.graph.Nodes {
		if j == kfIdx || kf.Session >= p.session {
			continue
		}
		if d := PoseDist(kf.Pose, fix); d < bestD {
			bestD, bestJ = d, j
		}
	}
	edgeIdx := -1
	if bestJ >= 0 {
		info := diagInfo(p.cfg.OdomEdgeSigmaXY*1.5, p.cfg.OdomEdgeSigmaTh*1.5)
		edgeIdx = p.appendEdgeDedupLocked(&Edge{
			I: bestJ, J: kfIdx, Kind: EdgeReloc,
			Measure: Between(p.graph.Nodes[bestJ].Pose, fix),
			Info:    info, RawInfo: info, Status: LoopAccepted,
			Reason: "session_relocation",
		})
	}
	p.grid.integrateScanLocked(fix, clean)
	p.traj = append(p.traj, TrajectoryPoint{
		FrameID: fr.Scan.ID, Time: fr.Scan.Time, Pose: fix,
		Seq: p.frameCount, Session: p.session,
	})
	d := p.decide(fr, OutcomeAccepted, ReasonLocalized, fmt.Sprintf(
		"session=%d fix=%s anchor_kf=%d", p.session, fix, bestJ))
	p.log.Framef("id=%d SESSION %d anchored fix=%s anchor_kf=%d edge=%d",
		fr.Scan.ID, p.session, fix, bestJ, edgeIdx)
	return d, edgeIdx
}

// BeginSessionLocalize starts a new mapping session whose start pose, clock
// and wheel odometry are unrelated to the loaded state. The first scan is
// aligned globally against the old map. On a unique strong fix the session
// is anchored in the old world frame and subsequent ProcessFrame calls run
// there; on failure or ambiguity NOTHING in the old map/trajectory/graph is
// modified and the session stays pending (retry with a hint). hint may be
// nil for an unconstrained full-map search.
func (p *Pipeline) BeginSessionLocalize(first Frame, hint *LocalizeHint) LocalizeResult {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.beginSessionLocalizeLocked(first, hint)
}

func (p *Pipeline) beginSessionLocalizeLocked(fr Frame, hint *LocalizeHint) LocalizeResult {
	clean, reason := validateScan(p.cfg, fr.Scan)
	res := LocalizeResult{Status: LocalizeFailed, AppliedEdge: -1}
	if reason != ReasonOK {
		res.Reason = reason
		p.recordLocalizeLocked(res, hint)
		return res
	}
	if !p.started {
		// Nothing to localize against: behave as a normal first run.
		p.session = 0
		p.bootstrap(fr, clean)
		res.Status = LocalizeUnique
		res.Reason = "fresh_start"
		p.recordLocalizeLocked(res, hint)
		return res
	}
	lr := p.globalLocalize(clean, hint)
	if lr.Status != LocalizeUnique {
		// Old state stays byte-for-byte intact; remember intent so frames
		// cannot slip in un-localized.
		p.pendingSession = true
		p.recordLocalizeLocked(lr, hint)
		return lr
	}
	p.session++
	p.pendingSession = false
	p.frameCount++
	fr2 := fr
	fr2.Restart = true
	_, edgeIdx := p.anchorSessionLocked(fr2, clean, lr.Applied)
	lr.AppliedEdge = edgeIdx
	p.recordLocalizeLocked(lr, hint)
	return lr
}

func (p *Pipeline) recordLocalizeLocked(res LocalizeResult, hint *LocalizeHint) {
	d := LocalizeDecision{
		Session: p.session, Status: res.Status, Reason: res.Reason,
		Best: res.Best, Second: res.Second, ScoreMargin: res.ScoreMargin,
		WithHint: hint != nil && hint.HasPose,
	}
	p.localizeLog = append(p.localizeLog, d)
	with := "no-hint"
	if d.WithHint {
		with = "hint"
	}
	p.log.Loopf("localize session=%d %s %s best_score=%.2f rms=%.3f "+
		"margin=%.2f %s fix=%s", p.session, res.Status, res.Reason,
		res.Best.Score, res.Best.ResidualRMS, res.ScoreMargin, with,
		res.Applied)
}

func (p *Pipeline) advanceScanOnly(fr Frame, clean []Point, restart bool) FrameDecision {
	pred := p.lastPose
	ref := p.referencePointsLocked(pred)
	mr := matchScan(p.cfg, ref, clean, pred)
	reason := ReasonNoOdom
	if restart {
		reason = ReasonRestart
	}
	if !mr.CoarseOK {
		d := p.decide(fr, OutcomeRejected, reason, "scan-only matching failed")
		p.log.Framef("id=%d t=%.3f REJECT %s scan-only failed",
			fr.Scan.ID, fr.Scan.Time, reason)
		return d
	}
	return p.advanceFused(fr, clean, Pose{}, Between(p.lastPose, mr.Pose),
		1, 1, OutcomeDegraded, reason,
		fmt.Sprintf("score=%.2f scan-only", mr.Score))
}

func (p *Pipeline) commitDegraded(fr Frame, odomDelta Pose, reason, detail string) FrameDecision {
	return p.advanceFused(fr, nil, odomDelta, odomDelta, 0, 0,
		OutcomeDegraded, reason, detail)
}

func (p *Pipeline) advanceFused(fr Frame, clean []Point,
	odomDelta, matchDelta Pose, wxy, wth float64,
	outcome Outcome, reason, detail string) FrameDecision {
	fused := fuseDelta(odomDelta, matchDelta, wxy, wth)
	pose := p.lastPose.Compose(fused)
	p.lastPose = pose
	if clean != nil {
		p.lastScan = append(p.lastScan[:0], clean...)
	}
	if fr.HasOdom && fr.Odom.Valid {
		p.lastOdom = fr.Odom.Pose
	}
	p.traj = append(p.traj, TrajectoryPoint{
		FrameID: fr.Scan.ID, Time: fr.Scan.Time, Pose: pose,
		Seq: p.frameCount, Session: p.session,
	})

	if clean != nil {
		kf := p.maybeKeyframe(fr, clean, pose, fr.Odom.Pose, p.session)
		if kf >= 0 {
			p.grid.integrateScanLocked(p.graph.Nodes[kf].Pose,
				p.graph.Nodes[kf].Scan)
			if kf%p.cfg.LoopCheckEvery == 0 {
				p.tryLoopClosure(kf)
			}
			p.kfsSinceOpt++
			if p.kfsSinceOpt >= p.cfg.OptEveryNKeyframes {
				hasLoop := false
				for _, e := range p.graph.Edges {
					if e.Kind == EdgeLoop {
						hasLoop = true
						break
					}
				}
				// Defer global correction once a loop exists: applying it
				// repeatedly without rebuilding the map would desync map
				// and trajectory. Finalize performs the single aligned
				// optimize+rebuild pass.
				if !hasLoop {
					p.runOptimizeLocked("incremental")
				}
				p.kfsSinceOpt = 0
			}
		}
	}
	d := p.decide(fr, outcome, reason, detail)
	p.log.Framef("id=%d t=%.3f %s %s %s pose=%s",
		fr.Scan.ID, fr.Scan.Time, strings.ToUpper(string(outcome)),
		reason, detail, pose)
	return d
}

func (p *Pipeline) referencePointsLocked(pred Pose) []Point {
	const maxRefKeyframes = 6
	const refRadius = 4.0
	var pts []Point
	n := len(p.graph.Nodes)
	start := n - maxRefKeyframes
	if start < 0 {
		start = 0
	}
	for i := n - 1; i >= start; i-- {
		kf := p.graph.Nodes[i]
		if PoseDist(kf.Pose, pred) > refRadius {
			continue
		}
		pts = append(pts, kf.SurfaceWorld()...)
		// Also include raw scans of the two newest keyframes for fine
		// detail near the current pose.
	}
	// Deduplicate surface cells across keyframes.
	pts = rasterizeHits(pts, p.cfg.MapResolution)
	return pts
}
