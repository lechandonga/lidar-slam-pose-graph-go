package slam

import (
	"fmt"
	"math"
	"sort"
	"strings"
	"sync"
)

// TrajectoryPoint is one fused per-frame pose in the world frame.
type TrajectoryPoint struct {
	FrameID int64
	Time    float64
	Pose    Pose
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

	// session is the current independent sensor session. Session 0 is the
	// original run. Every BeginSession increments it. Each session's wheel
	// odometry and clock start at zero, so PoseOdom comparisons are only
	// meaningful within the same session.
	session int
	// frameIDBase makes source frame IDs globally unique across sessions:
	// stored FrameID/TrajectoryPoint.FrameID = source ID + frameIDBase.
	frameIDBase int64
	// odomBase is the raw odom pose the current session's deltas are
	// measured from. lastOdom stores odomBase + consumed delta; sessions
	// after the first are anchored in the world but their odom hardware
	// reports motion relative to a freshly powered-on origin.
	odomBase Pose
	// awaitAnchor is true after loading a non-empty state: new frames must
	// not touch anything until BeginSession commits a localization result.
	awaitAnchor bool

	// sessionStartNodes/sessionStartTraj record the graph/trajectory sizes
	// when the current session began. A session that adds NO new keyframes
	// re-swept only already-mapped terrain; after Finalize its duplicate
	// trajectory segment is dropped so repeated load/run/save over the same
	// area converges to the single-session trajectory instead of stacking
	// physically identical copies round after round.
	sessionStartNodes int
	sessionStartTraj  int

	decisions      []FrameDecision
	loopLog        []LoopDecision
	optLog         []OptLogEntry
	lastOpt        OptimizeResult
	lastOptMaxMove float64
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
		cfg:    cfg,
		log:    logger,
		graph:  &Graph{},
		grid:   newGrid(cfg),
		lastKF: -1,
	}
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
	if p.awaitAnchor {
		// A loaded map is waiting for cross-session localization. Feeding
		// frames straight through here would silently append the new,
		// unlocalized session to the old trajectory and could corrupt the
		// old map. Reject and leave every byte of state untouched.
		d := p.decide(fr, OutcomeRejected, ReasonAwaitingAnchor,
			"call BeginSession after Localize before processing frames")
		p.log.Framef("id=%d t=%.3f REJECT %s",
			fr.Scan.ID, fr.Scan.Time, ReasonAwaitingAnchor)
		return d
	}
	p.frameCount++
	fr = p.remapFrameLocked(fr)

	clean, scanReason := validateScan(p.cfg, fr.Scan)
	if scanReason != ReasonOK {
		d := p.decide(fr, OutcomeRejected, scanReason,
			fmt.Sprintf("raw_points=%d", len(fr.Scan.Points)))
		p.log.Framef("id=%d t=%.3f REJECT %s raw_points=%d",
			fr.Scan.ID, fr.Scan.Time, scanReason, len(fr.Scan.Points))
		return d
	}

	if !p.started {
		return p.bootstrap(fr, clean)
	}

	if fr.Restart {
		d := p.advanceScanOnly(fr, clean, true)
		p.odomBase = fr.Odom.Pose
		p.lastOdom = Pose{}
		return d
	}
	if !fr.HasOdom || !fr.Odom.Valid {
		return p.advanceScanOnly(fr, clean, false)
	}

	// Session-relative odom sample (raw odom minus this session's origin),
	// then the per-frame delta. Every session powers on at raw odom zero,
	// but subtracting the base is still required after in-session restarts
	// and after BeginSession (whose first raw sample need not be exactly 0).
	sessOdom := Between(p.odomBase, fr.Odom.Pose)
	odomDelta := Between(p.lastOdom, sessOdom)
	if math.Hypot(odomDelta.X, odomDelta.Y) > p.cfg.RestartJumpXY ||
		math.Abs(odomDelta.Theta) > p.cfg.RestartJumpTheta {
		d := p.advanceScanOnly(fr, clean, true)
		p.odomBase = fr.Odom.Pose
		p.lastOdom = Pose{}
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
	if m := p.cfg.MaxDecisionLog; m > 0 && len(p.decisions) > m {
		p.decisions = p.decisions[len(p.decisions)-m:]
	}
	return d
}

func (p *Pipeline) bootstrap(fr Frame, clean []Point) FrameDecision {
	base := Pose{}
	p.lastPose = base
	p.odomBase = fr.Odom.Pose
	p.lastOdom = Pose{}
	p.lastKFOdom = Pose{}
	p.started = true
	p.addKeyframeLocked(fr, clean, base, Pose{})
	p.grid.integrateScanLocked(base, clean)
	p.traj = append(p.traj, TrajectoryPoint{fr.Scan.ID, fr.Scan.Time, base})
	d := p.decide(fr, OutcomeAccepted, ReasonOK, "bootstrap")
	p.log.Framef("id=%d t=%.3f ACCEPT bootstrap points=%d",
		fr.Scan.ID, fr.Scan.Time, len(clean))
	return d
}

// ReasonAwaitingAnchor marks frames pushed after LoadState but before a
// successful cross-session localization.
const ReasonAwaitingAnchor = "awaiting_session_anchor"

// remapFrameLocked gives the frame a process-wide unique ID and a session
// time offset so sessions with reset clocks never collide. The source values
// are preserved in the decision Detail/logs via the pre-call IDs.
func (p *Pipeline) remapFrameLocked(fr Frame) Frame {
	fr.Scan.ID += p.frameIDBase
	fr.Odom.ID += p.frameIDBase
	return fr
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
		p.lastOdom = Between(p.odomBase, fr.Odom.Pose)
	}
	p.traj = append(p.traj, TrajectoryPoint{fr.Scan.ID, fr.Scan.Time, pose})

	if clean != nil {
		kf := p.maybeKeyframe(fr, clean, pose,
			Between(p.odomBase, fr.Odom.Pose))
		if kf < -1 {
			// Revisit of an existing keyframe: reuse its node, do NOT add
			// a duplicate and do NOT integrate the scan again (that would
			// only re-accumulate evidence and bloat the saved grid).
			p.lastKF = -kf - 2
			p.lastKFOdom = Between(p.odomBase, fr.Odom.Pose)
		} else if kf >= 0 {
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
				// optimize+rebuild pass. A cross-session graph is likewise
				// optimized only at Finalize: early in a new session the
				// anchor has just one or two constraints while the new
				// odom chain is already long, so incremental optimization
				// would drag the freshly anchored start off the old map.
				if !hasLoop && p.session == 0 {
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
			// Keyframes from a previous session are skipped here rather
			// than counted against the budget (their odom/session frame is
			// unrelated); fall through to the geometric fallback below.
			continue
		}
		pts = append(pts, kf.SurfaceWorld()...)
	}
	// Cross-session anchor fallback: right after BeginSession the newest
	// keyframes belong to the old session and are far away, so the loop
	// above contributes nothing. Fill the reference from geometrically
	// nearby keyframes of ANY session (the overlap area).
	if len(pts) < 8 {
		type near struct{ idx, rank int }
		var cands []near
		for i, kf := range p.graph.Nodes {
			if PoseDist(kf.Pose, pred) <= refRadius {
				rank := 0
				if kf.Session != p.session {
					rank = 1
				}
				cands = append(cands, near{i, rank})
			}
		}
		sort.Slice(cands, func(a, b int) bool {
			if cands[a].rank != cands[b].rank {
				return cands[a].rank < cands[b].rank
			}
			return cands[a].idx > cands[b].idx
		})
		for k := 0; k < len(cands) && k < maxRefKeyframes; k++ {
			pts = append(pts, p.graph.Nodes[cands[k].idx].SurfaceWorld()...)
		}
	}
	// Deduplicate surface cells across keyframes.
	pts = rasterizeHits(pts, p.cfg.MapResolution)
	return pts
}
