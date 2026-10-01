package slam

import (
	"math"
	"sync"

	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/fusion"
	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/geom"
	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/gridmap"
	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/posegraph"
	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/scanmatch"
	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/sensor"
)

// Pipeline is the whole SLAM front- and back-end plus the live map.
// One read/write mutex guards every mutable state object as a group, so
// concurrent frame ingestion, optimization and map queries can never observe
// a half-updated trajectory or map.
type Pipeline struct {
	mu     sync.RWMutex
	cfg    Config
	graph  *posegraph.Graph
	mapRef *gridmap.Map

	keyframes []Keyframe
	records   []FrameRecord

	lastFramePose geom.Pose
	lastOdom      geom.Pose
	lastRestart   int
	haveOdom      bool
	lastKeyNode   int
	localField    *gridmap.Field

	loopAttempts int
}

// New constructs an empty pipeline.
func New(cfg Config) *Pipeline {
	return &Pipeline{
		cfg:   cfg,
		graph: posegraph.New(),
		mapRef: gridmap.NewMap(cfg.MapResolution, cfg.MapHalfSize,
			cfg.MapHitLogOdds, cfg.MapFreeLogOdds, cfg.MapMinLogOdds, cfg.MapMaxLogOdds),
		lastKeyNode: -1,
	}
}

// Config returns the configuration in use.
func (p *Pipeline) Config() Config { return p.cfg }

// Process ingests one synchronized frame and returns its decision record.
// The whole update is atomic under the write lock.
func (p *Pipeline) Process(f sensor.Frame) FrameRecord {
	p.mu.Lock()
	defer p.mu.Unlock()

	rec := FrameRecord{Seq: f.Seq, Time: f.Scan.Time}
	odomPose := geom.Pose{X: f.Odom.PoseX, Y: f.Odom.PoseY, Theta: f.Odom.PoseTheta}

	restart := false
	if p.haveOdom {
		restart = fusion.IsRestart(p.lastRestart, f.Odom.Restart, p.lastOdom, odomPose, p.cfg.RestartJump)
	}
	rec.Restart = restart

	var predicted geom.Pose
	if !p.haveOdom || restart {
		predicted = p.lastFramePose
	} else {
		step := geom.Between(odomPose, p.lastOdom)
		predicted = geom.Compose(step, p.lastFramePose)
	}
	rec.PredictedPose = predicted

	points := scanmatch.ValidPoints(f.Scan)
	rec.ValidPoints = len(points)

	var match scanmatch.Result
	match.Pose = predicted
	switch {
	case f.Scan.Bad:
		match.Reason = scanmatch.ReasonRejectBadFrame
	case len(points) < p.cfg.MinValidPoints:
		match.Reason = scanmatch.ReasonRejectFewPoints
	default:
		if p.localField == nil {
			p.localField = p.localFieldLocked(predicted)
		}
		field := p.localField
		match = scanmatch.Match(points, predicted, field, scanmatch.Params{
			MaxIter:           p.cfg.MatchMaxIter,
			ConvergeStep:      p.cfg.MatchConvergeStep,
			HuberSigma:        p.cfg.MaxResidualSigma * p.cfg.MapResolution,
			MinInfoEigenvalue: p.cfg.MinInfoEigenvalue,
			MaxResidual:       p.cfg.MatchMaxError,
			MaxPosErr:         p.cfg.MatchMaxPosErr,
			MaxAngErr:         p.cfg.MatchMaxAngErr,
		}, p.cfg.MatchMaxPosErr, p.cfg.MatchMaxAngErr)
	}
	rec.ScanPose = match.Pose
	rec.MeanResidual = match.MeanResid
	rec.MinEigen = match.MinEigen
	rec.MatchIter = match.Iterations
	rec.InBounds = match.InBounds
	rec.Reason = match.Reason

	d := fusion.Fuse(predicted, match, restart || !p.haveOdom, fusion.Weights{
		Scan: p.cfg.ScanWeight, Odom: p.cfg.OdomWeight,
	})
	rec.Mode = d.Mode
	rec.FusedPose = d.Fused
	rec.ScanWeight = d.ScanW
	rec.OdomWeight = d.OdomW

	p.lastFramePose = d.Fused
	p.lastOdom = odomPose
	p.lastRestart = f.Odom.Restart
	p.haveOdom = true

	if p.shouldKeyframeLocked(d.Fused) {
		kfPose := d.Fused
		if d.Mode == fusion.ModeFused {
			kfPose = match.Pose
		}
		nodeID := p.graph.AddNode(f.Seq, kfPose)
		kf := Keyframe{NodeID: nodeID, Seq: f.Seq, Pose: kfPose, Points: append([]geom.Point(nil), points...)}
		p.lastFramePose = kfPose
		rec.FusedPose = kfPose
		d.Fused = kfPose
		p.keyframes = append(p.keyframes, kf)
		rec.NewKeyframe = true
		rec.NodeID = nodeID
		if p.lastKeyNode >= 0 {
			prev, _ := p.graph.NodePose(p.lastKeyNode)
			rel := geom.Between(d.Fused, prev)
			p.graph.AddEdge(posegraph.Edge{
				From: p.lastKeyNode, To: nodeID, Type: posegraph.EdgeOdom,
				Rel: rel,
				Info: [3][3]float64{
					{p.cfg.SeqInfoXY, 0, 0},
					{0, p.cfg.SeqInfoXY, 0},
					{0, 0, p.cfg.SeqInfoTh},
				},
				Status: posegraph.StatusAccepted,
				Reason: "sequential_odom_edge",
			})
		}
		p.lastKeyNode = nodeID
		p.localField = nil // next frame builds a field including the new kf
		p.tryLoopClosureLocked(kf, &rec)
		if rec.LoopStatus == string(posegraph.StatusAccepted) ||
			rec.LoopStatus == string(posegraph.StatusSoft) {
			p.rebuildMapLocked(&rec)
		} else {
			// incremental update: bounded per-keyframe cost
			rasterize(p.mapRef, kf.Pose, kf.Points, p.cfg.MaxRange)
			rec.MapRebuilt = true
		}
	}

	p.records = append(p.records, rec)
	return rec
}

func (p *Pipeline) shouldKeyframeLocked(pose geom.Pose) bool {
	if p.lastKeyNode < 0 {
		return true
	}
	last, ok := p.graph.NodePose(p.lastKeyNode)
	if !ok {
		return true
	}
	d := geom.PoseDist(pose, last)
	a := math.Abs(geom.WrapAngle(pose.Theta - last.Theta))
	return d >= p.cfg.KeyframeDist || a >= p.cfg.KeyframeAngle
}

// localFieldLocked builds a field from the most recent keyframes only,
// keeping per-frame work bounded by LocalKeyframes.
// localFieldLocked builds a field from the most recent keyframes only. The
// window is a pure function of the keyframe set (bounding box + sensor
// radius), so the same keyframes always yield the same field, which keeps
// save/resume continuation bit-reproducible.
func (p *Pipeline) localFieldLocked(_ geom.Pose) *gridmap.Field {
	if len(p.keyframes) == 0 {
		return nil
	}
	start := 0
	if len(p.keyframes) > p.cfg.LocalKeyframes {
		start = len(p.keyframes) - p.cfg.LocalKeyframes
	}
	minX, minY := math.Inf(1), math.Inf(1)
	maxX, maxY := math.Inf(-1), math.Inf(-1)
	for _, kf := range p.keyframes[start:] {
		minX = math.Min(minX, kf.Pose.X)
		minY = math.Min(minY, kf.Pose.Y)
		maxX = math.Max(maxX, kf.Pose.X)
		maxY = math.Max(maxY, kf.Pose.Y)
	}
	pad := p.cfg.LocalFieldRange + 1
	minX -= pad
	maxX += pad
	minY -= pad
	maxY += pad
	cx, cy := (minX+maxX)/2, (minY+maxY)/2
	half := math.Max(maxX-minX, maxY-minY)/2 + 0.5
	window := 2*half + 1
	tmp := gridmap.NewMap(p.cfg.MapResolution, window, 1, -1, -10, 10)
	for _, kf := range p.keyframes[start:] {
		rasterize(tmp, kf.Pose, kf.Points, p.cfg.LocalFieldRange)
	}
	return tmp.BuildField(cx, cy, half)
}

func (p *Pipeline) tryLoopClosureLocked(kf Keyframe, rec *FrameRecord) {
	p.loopAttempts++
	if p.loopAttempts%p.cfg.LoopTryEvery != 0 {
		return
	}
	rec.LoopTried = true
	type cand struct {
		id   int
		dist float64
	}
	var cands []cand
	for _, other := range p.keyframes {
		if other.NodeID == kf.NodeID {
			continue
		}
		if kf.Seq-other.Seq < p.cfg.LoopMinFrames {
			continue
		}
		op, _ := p.graph.NodePose(other.NodeID)
		if d := geom.PoseDist(op, kf.Pose); d <= p.cfg.LoopSearchDist {
			// A revisit requires the SAME place AND orientation; rotating in
			// place creates spatially coincident keyframes with different
			// headings that must never be linked as loops.
			if math.Abs(geom.WrapAngle(op.Theta-kf.Pose.Theta)) > p.cfg.LoopMaxHeadDiff {
				continue
			}
			cands = append(cands, cand{other.NodeID, d})
		}
	}
	rec.LoopCandidates = len(cands)
	if len(cands) == 0 {
		return
	}
	// Deterministic tie break: closest, then lowest node id.
	best := cands[0]
	for _, c := range cands[1:] {
		if c.dist < best.dist || (c.dist == best.dist && c.id < best.id) {
			best = c
		}
	}
	other, _ := p.graph.NodePose(best.id)
	hyp := geom.Between(kf.Pose, other)

	// Field around the old keyframe; initial guess for the match is that the
	// new keyframe currently coincides with the old one (standard loop try).
	tmp := gridmap.NewMap(p.cfg.MapResolution, float64(p.cfg.LocalKeyframes)*p.cfg.KeyframeDist+3, 1, -1, -10, 10)
	start := best.id - p.cfg.LocalKeyframes/2
	if start < 0 {
		start = 0
	}
	end := best.id + p.cfg.LocalKeyframes/2 + 1
	if end > len(p.keyframes) {
		end = len(p.keyframes)
	}
	for _, k := range p.keyframes[start:end] {
		rasterize(tmp, k.Pose, k.Points, p.cfg.MaxRange)
	}
	field := tmp.BuildField(other.X, other.Y, float64(p.cfg.LocalKeyframes)*p.cfg.KeyframeDist+2.5)
	// The edge claims kf = other * rel; at loop time the scan is currently
	// at kf.Pose, so the correct registration initial guess is kf.Pose, and
	// rel = matchedKf * other^-1.
	res := scanmatch.Match(kf.Points, kf.Pose, field, scanmatch.Params{
		MaxIter:           p.cfg.MatchMaxIter,
		ConvergeStep:      p.cfg.MatchConvergeStep,
		HuberSigma:        p.cfg.MaxResidualSigma * p.cfg.MapResolution,
		MinInfoEigenvalue: 0,
		MaxResidual:       p.cfg.LoopMaxError,
		MaxPosErr:         p.cfg.LoopMaxPosErr,
		MaxAngErr:         p.cfg.LoopMaxAngErr,
	}, p.cfg.LoopMaxPosErr, p.cfg.LoopMaxAngErr)
	rel := geom.Between(res.Pose, other)
	posChi := math.Hypot(rel.X-hyp.X, rel.Y-hyp.Y) / 0.5
	angChi := math.Abs(geom.WrapAngle(rel.Theta-hyp.Theta)) / p.cfg.LoopMaxAngErr
	chi2 := posChi*posChi + angChi*angChi
	rec.LoopChi2 = chi2

	status := posegraph.StatusAccepted
	reason := "loop_within_soft_band"
	switch {
	case res.Reason == scanmatch.ReasonRejectHighResidual:
		status, reason = posegraph.StatusRejected, "loop_match_residual_too_high"
	case res.Reason == scanmatch.ReasonRejectDeviation:
		status, reason = posegraph.StatusRejected, "loop_match_deviation_too_large"
	case chi2 >= p.cfg.LoopRejectChi2:
		status, reason = posegraph.StatusRejected, "loop_chi2_above_reject_threshold"
	case chi2 > p.cfg.LoopSoftChi2:
		status, reason = posegraph.StatusSoft, "loop_chi2_in_soft_band"
	}
	rec.LoopStatus = string(status)
	rec.LoopReason = reason
	p.graph.AddEdge(posegraph.Edge{
		From: best.id, To: kf.NodeID, Type: posegraph.EdgeLoop,
		Rel: rel, Info: diagInfo(p.cfg.ScanWeight),
		Status: status, Reason: reason, Chi2: chi2,
	})
	if status == posegraph.StatusRejected {
		return
	}
	rep := p.graph.Optimize(posegraph.OptimizeParams{
		MaxIter: p.cfg.OptMaxIter, ConvergeChi2: p.cfg.OptConvergeChi2,
		ConvergeStep: p.cfg.OptConvergeStep, HuberDelta: p.cfg.HuberDelta,
	})
	rec.OptConverged = rep.Converged
	rec.OptIter = rep.Iterations
	for i := range p.keyframes {
		if np, ok := p.graph.NodePose(p.keyframes[i].NodeID); ok {
			p.keyframes[i].Pose = np
		}
	}
	p.lastFramePose = p.keyframes[len(p.keyframes)-1].Pose
}

// rebuildMapLocked regenerates the occupancy map from keyframes at their
// current (post-optimization) poses. This is the guarantee that map and
// trajectory can never disagree.
func (p *Pipeline) rebuildMapLocked(rec *FrameRecord) {
	m := gridmap.NewMap(p.cfg.MapResolution, p.cfg.MapHalfSize,
		p.cfg.MapHitLogOdds, p.cfg.MapFreeLogOdds, p.cfg.MapMinLogOdds, p.cfg.MapMaxLogOdds)
	for _, kf := range p.keyframes {
		rasterize(m, kf.Pose, kf.Points, p.cfg.MaxRange)
	}
	p.mapRef = m
	rec.MapRebuilt = true
}

func rasterize(m *gridmap.Map, pose geom.Pose, points []geom.Point, maxRange float64) {
	origin := geom.Point{X: pose.X, Y: pose.Y}
	step := 1
	if len(points) > 400 {
		step = 2
	}
	for i, q := range points {
		if i%step != 0 {
			continue
		}
		w := pose.Transform(q)
		if geom.Dist(origin, w) > maxRange {
			continue
		}
		m.AddLine(origin.X, origin.Y, w.X, w.Y, true)
	}
}

func diagInfo(w float64) [3][3]float64 {
	return [3][3]float64{
		{w, 0, 0},
		{0, w, 0},
		{0, 0, w * 0.5},
	}
}
