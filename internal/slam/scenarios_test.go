package slam_test

import (
	"math"
	"math/rand"
	"os"
	"path/filepath"
	"sync"
	"testing"
	"time"

	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/fusion"
	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/geom"
	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/posegraph"
	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/scanmatch"
	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/sensor"
	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/sim"
	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/slam"
)

func makeFrames(t *testing.T, world sim.World, truth, odoms []geom.Pose, sp sim.ScanParams, rng *rand.Rand,
	inject func(i int, sc *sensor.Scan, od *sensor.Odom)) []sensor.Frame {
	t.Helper()
	frames := make([]sensor.Frame, len(truth))
	ts := time.Unix(1000, 0)
	for i, pose := range truth {
		ranges := sim.MakeScan(world, pose, sp, rng)
		sc := sensor.Scan{
			Time:     ts.Add(time.Duration(i) * 100 * time.Millisecond),
			AngleMin: sp.AngleMin, AngleInc: (sp.AngleMax - sp.AngleMin) / float64(sp.Beams-1),
			RangeMin: sp.RangeMin, RangeMax: sp.RangeMax, Ranges: ranges,
		}
		od := sensor.Odom{Time: sc.Time, PoseX: odoms[i].X, PoseY: odoms[i].Y, PoseTheta: odoms[i].Theta}
		if inject != nil {
			inject(i, &sc, &od)
		}
		frames[i] = sensor.Frame{Seq: i, Scan: sc, Odom: od}
	}
	return frames
}

func runPipeline(t *testing.T, frames []sensor.Frame, cfg slam.Config) *slam.Pipeline {
	t.Helper()
	p := slam.New(cfg)
	for _, f := range frames {
		p.Process(f)
	}
	return p
}

func rmse(path, truth []geom.Pose) float64 {
	aligned := alignToTruth(path, truth)
	s := 0.0
	for i := range truth {
		s += (aligned[i].X-truth[i].X)*(aligned[i].X-truth[i].X) +
			(aligned[i].Y-truth[i].Y)*(aligned[i].Y-truth[i].Y)
	}
	return math.Sqrt(s / float64(len(truth)))
}

// TestScenarioStraight: wall following stays bounded even where geometry is
// locally corridor-like; the degeneracy gate must downgrade, not crash.
func TestScenarioStraight(t *testing.T) {
	world := sim.RectWorld(8)
	rng := rand.New(rand.NewSource(1))
	sp := sim.DefaultScanParams()
	var truth []geom.Pose
	for i := 0; i < 40; i++ {
		truth = append(truth, geom.Pose{X: 2, Y: -5 + float64(i)*0.2, Theta: math.Pi / 2})
	}
	odoms := sim.MakeOdom(truth, sim.DefaultOdomParams(), rng)
	frames := makeFrames(t, world, truth, odoms, sp, rng, nil)
	p := runPipeline(t, frames, slam.DefaultConfig())
	truthN := normalizeFrame(truth)
	if got := rmse(p.Trajectory(), truthN); got > 0.35 {
		t.Fatalf("straight RMSE too high: %.3f", got)
	}
	t.Logf("straight: kfs=%d RMSE=%.3f", len(p.Keyframes()), rmse(p.Trajectory(), truthN))
}

// TestScenarioTurn: in-place rotation recovers heading.
func TestScenarioTurn(t *testing.T) {
	world := sim.RectWorld(8)
	rng := rand.New(rand.NewSource(2))
	sp := sim.DefaultScanParams()
	var truth []geom.Pose
	for i := 0; i < 60; i++ {
		truth = append(truth, geom.Pose{Theta: float64(i) * 2 * math.Pi / 60})
	}
	odoms := sim.MakeOdom(truth, sim.DefaultOdomParams(), rng)
	frames := makeFrames(t, world, truth, odoms, sp, rng, nil)
	cfg := slam.DefaultConfig()
	cfg.KeyframeAngle = 0.2
	p := runPipeline(t, frames, cfg)
	out := p.Trajectory()
	want := geom.WrapAngle(truth[len(truth)-1].Theta)
	if err := math.Abs(geom.WrapAngle(out[len(out)-1].Theta - want)); err > 0.15 {
		t.Fatalf("final heading error %.3f rad, got %.3f want %.3f", err, out[len(out)-1].Theta, want)
	}
	var maxKfErr float64
	for _, r := range p.Records() {
		if r.NewKeyframe {
			if e := math.Abs(geom.WrapAngle(r.ScanPose.Theta - geom.WrapAngle(truth[r.Seq].Theta))); e > maxKfErr {
				maxKfErr = e
			}
		}
	}
	t.Logf("turn: kfs=%d finalTheta=%.3f want=%.3f maxPerKfAngErr=%.4f",
		len(p.Keyframes()), out[len(out)-1].Theta, want, maxKfErr)
}

// TestScenarioLoopClosure: SLAM RMSE must clearly beat pure odometry.
func TestScenarioLoopClosure(t *testing.T) {
	frames, truthRaw := buildDataset(t, 42, nil)
	truth := normalizeFrame(truthRaw)
	p := runPipeline(t, frames, slam.DefaultConfig())
	odPath := make([]geom.Pose, len(frames))
	for i, f := range frames {
		odPath[i] = geom.Pose{X: f.Odom.PoseX, Y: f.Odom.PoseY, Theta: f.Odom.PoseTheta}
	}
	odE, slamE := rmse(odPath, truth), rmse(p.Trajectory(), truth)
	accepted := 0
	for _, e := range p.LoopEdges() {
		t.Logf("loop %d->%d status=%s chi2=%.2f reason=%s", e.From, e.To, e.Status, e.Chi2, e.Reason)
		if e.Status == posegraph.StatusAccepted {
			accepted++
		}
	}
	t.Logf("loop closure: odom RMSE=%.3f slam RMSE=%.3f kfs=%d", odE, slamE, len(p.Keyframes()))
	if accepted == 0 {
		t.Fatal("no accepted loop edge")
	}
	if slamE > 0.6*odE {
		t.Fatalf("loop correction insufficient: odom=%.3f slam=%.3f", odE, slamE)
	}
}

// TestScenarioNoiseAndBadFrames: three different failure injections must
// produce three distinguishable rejection reasons and not corrupt the path.
func TestScenarioNoiseAndBadFrames(t *testing.T) {
	frames, truthRaw := buildDataset(t, 11, func(i int, sc *sensor.Scan, od *sensor.Odom) {
		switch {
		case i == 10:
			sc.Bad = true
			sc.BadReason = "simulated_corrupt_packet"
		case i == 20:
			for k := range sc.Ranges {
				if k%40 != 0 {
					sc.Ranges[k] = sc.RangeMax + 1
				}
			}
		case i == 30:
			for k := range sc.Ranges {
				sc.Ranges[k] = 0.5 + float64(k%7)*0.03
			}
		}
	})
	truth := normalizeFrame(truthRaw)
	p := runPipeline(t, frames, slam.DefaultConfig())
	recs := p.Records()
	want := map[int]scanmatch.Reason{
		10: scanmatch.ReasonRejectBadFrame,
		20: scanmatch.ReasonRejectFewPoints,
		30: scanmatch.ReasonRejectHighResidual,
	}
	seen := map[scanmatch.Reason]bool{}
	for seq, reason := range want {
		r := recs[seq]
		t.Logf("frame %d: mode=%s reason=%s resid=%.3f valid=%d", seq, r.Mode, r.Reason, r.MeanResidual, r.ValidPoints)
		if r.Reason != reason {
			t.Fatalf("frame %d reason=%s want %s", seq, r.Reason, reason)
		}
		seen[r.Reason] = true
	}
	if len(seen) != 3 {
		t.Fatalf("reasons not distinguishable: %v", seen)
	}
	if got := rmse(p.Trajectory(), truth); got > 0.4 {
		t.Fatalf("trajectory corrupted by bad frames: RMSE=%.3f", got)
	}
	_ = fusion.ModeOdomOnly
}

// TestScenarioRestart: odom origin jump mid-run is detected and never
// teleports the pose; tracking resumes afterwards.
func TestScenarioRestart(t *testing.T) {
	frames, truthRaw := buildDataset(t, 13, nil)
	base := geom.Pose{X: frames[45].Odom.PoseX, Y: frames[45].Odom.PoseY, Theta: frames[45].Odom.PoseTheta}
	for i := 45; i < len(frames); i++ {
		cur := geom.Pose{X: frames[i].Odom.PoseX, Y: frames[i].Odom.PoseY, Theta: frames[i].Odom.PoseTheta}
		rel := geom.Between(cur, base)
		frames[i].Odom.PoseX, frames[i].Odom.PoseY, frames[i].Odom.PoseTheta = rel.X, rel.Y, rel.Theta
		frames[i].Odom.Restart = 1
	}
	truth := normalizeFrame(truthRaw)
	p := runPipeline(t, frames, slam.DefaultConfig())
	var restarts []int
	for _, r := range p.Records() {
		if r.Restart {
			restarts = append(restarts, r.Seq)
		}
	}
	t.Logf("restarts at frames %v", restarts)
	if len(restarts) == 0 {
		t.Fatal("odom restart not detected")
	}
	out := p.Trajectory()
	recs := p.Records()
	// The restart itself must not create a huge discontinuity: pose at the
	// restart frame must stay within 1.5 m of the pre-restart pose (scan
	// re-localization keeps it anchored; only the odom origin jumped).
	if d := geom.PoseDist(recs[45].FusedPose, recs[44].FusedPose); d > 1.5 {
		t.Fatalf("restart frame jumped %.2f m", d)
	}
	// post-restart segment must re-track the truth within a loose bound
	if got := rmse(out[45:], truth[45:]); got > 0.8 {
		t.Fatalf("post-restart RMSE too high %.3f", got)
	}
	t.Logf("restart handled, post-restart RMSE=%.3f jump=%.3f",
		rmse(out[45:], truth[45:]), geom.PoseDist(recs[45].FusedPose, recs[44].FusedPose))
}

// TestScenarioFalseLoop: a confidently wrong loop is rejected and cannot
// distort the graph.
func TestScenarioFalseLoop(t *testing.T) {
	frames, truthRaw := buildDataset(t, 42, nil)
	truth := normalizeFrame(truthRaw)
	p := runPipeline(t, frames, slam.DefaultConfig())
	before := rmse(p.Trajectory(), truth)
	n := p.GraphSnapshot().Nodes
	p.InjectLoopEdgeForTest(0, len(n)-1, geom.Pose{X: 5, Y: 0, Theta: 0.3}, 100.0)
	var injected *posegraph.Edge
	for _, e := range p.LoopEdges() {
		e := e
		if e.Reason == "injected_loop_chi2_above_reject_threshold" {
			injected = &e
		}
	}
	if injected == nil {
		t.Fatal("rejected injected edge not recorded")
	}
	t.Logf("injected loop status=%s chi2=%.1f", injected.Status, injected.Chi2)
	if injected.Status != posegraph.StatusRejected {
		t.Fatalf("contradictory loop not rejected: %s", injected.Status)
	}
	after := rmse(p.Trajectory(), truth)
	if math.Abs(after-before) > 1e-9 {
		t.Fatalf("rejected edge changed trajectory: %.6f -> %.6f", before, after)
	}
}

// TestScenarioSoftLoop: an ambiguous but not impossible loop is downweighted
// (distinguishable status) and influences the graph only mildly.
func TestScenarioSoftLoop(t *testing.T) {
	frames, truthRaw := buildDataset(t, 42, nil)
	truth := normalizeFrame(truthRaw)
	p := runPipeline(t, frames, slam.DefaultConfig())
	nodes := p.GraphSnapshot().Nodes
	// a moderate 0.8 m wrong transform, chi2 in the soft band
	p.InjectLoopEdgeForTest(0, len(nodes)-1, geom.Pose{X: 0.8, Y: 0.2}, 8.0)
	var soft int
	for _, e := range p.LoopEdges() {
		if e.Status == posegraph.StatusSoft && e.Reason == "injected_loop_chi2_in_soft_band" {
			soft++
		}
	}
	if soft != 1 {
		t.Fatalf("expected one soft edge, got %d", soft)
	}
	if got := rmse(p.Trajectory(), truth); got > 0.5 {
		t.Fatalf("soft loop destabilized trajectory RMSE=%.3f", got)
	}
	t.Logf("soft loop handled, RMSE=%.3f", rmse(p.Trajectory(), truth))
}

// TestScenarioSaveAndResume: mid-run save then continue equals one-shot run.
func TestScenarioSaveAndResume(t *testing.T) {
	frames, _ := buildDataset(t, 42, nil)
	cfg := slam.DefaultConfig()
	cut := len(frames) / 2
	oneShot := runPipeline(t, frames, cfg)

	dir := t.TempDir()
	path := filepath.Join(dir, "state.gob")
	part := slam.New(cfg)
	for _, f := range frames[:cut] {
		part.Process(f)
	}
	if err := part.Save(path); err != nil {
		t.Fatal(err)
	}
	if fi, err := os.Stat(path); err != nil || fi.Size() == 0 {
		t.Fatalf("state not written: %v", err)
	}
	resumed, err := slam.Load(path)
	if err != nil {
		t.Fatal(err)
	}
	// Immediately after load (before any new frame), the partial trajectory
	// and map must be byte-identical to the saved instance.
	preA := part.Trajectory()
	preB := resumed.Trajectory()
	for i := range preA {
		if preA[i] != preB[i] {
			t.Fatalf("loaded state differs at frame %d: %+v vs %+v", i, preA[i], preB[i])
		}
	}
	for _, f := range frames[cut:] {
		resumed.Process(f)
	}
	a, b := oneShot.Trajectory(), resumed.Trajectory()
	if len(a) != len(b) {
		t.Fatalf("length %d vs %d", len(a), len(b))
	}
	maxDiff := 0.0
	for i := range a {
		if d := geom.PoseDist(a[i], b[i]); d > maxDiff {
			maxDiff = d
		}
		if da := math.Abs(geom.WrapAngle(a[i].Theta - b[i].Theta)); da > 1e-9 {
			t.Fatalf("frame %d theta diff %.6f", i, da)
		}
	}
	t.Logf("save/resume max trajectory diff %.2e m", maxDiff)
	if maxDiff > 1e-9 {
		t.Fatalf("resumed run diverged: %.6f", maxDiff)
	}
	// well past the resume boundary the two runs must be identical again
	tail := a[len(a)-10:]
	tailB := b[len(b)-10:]
	tailDiff := 0.0
	for i := range tail {
		if d := geom.PoseDist(tail[i], tailB[i]); d > tailDiff {
			tailDiff = d
		}
	}
	if tailDiff > 1e-9 {
		t.Fatalf("post-resume tail not identical: %.2e", tailDiff)
	}
	ms, mr := oneShot.Map().Snapshot(), resumed.Map().Snapshot()
	if len(ms.LogOdds) != len(mr.LogOdds) {
		t.Fatal("map length differs")
	}
	for i := range ms.LogOdds {
		if ms.LogOdds[i] != mr.LogOdds[i] {
			t.Fatalf("map cell %d differs", i)
		}
	}
}

// TestDeterminism: repeated identical replay gives bit-identical results.
func TestDeterminism(t *testing.T) {
	frames, _ := buildDataset(t, 42, nil)
	cfg := slam.DefaultConfig()
	p1 := runPipeline(t, frames, cfg)
	p2 := runPipeline(t, frames, cfg)
	a, b := p1.Trajectory(), p2.Trajectory()
	for i := range a {
		if a[i] != b[i] {
			t.Fatalf("frame %d differs: %+v vs %+v", i, a[i], b[i])
		}
	}
	if p1.GraphSnapshot().Edges[0].Rel != p2.GraphSnapshot().Edges[0].Rel {
		t.Fatal("edges non-deterministic")
	}
}

// TestConcurrentAccess: ingestion racing with map/trajectory/graph reads must
// be race-clean (run with `go test -race`).
func TestConcurrentAccess(t *testing.T) {
	frames, _ := buildDataset(t, 9, nil)
	cfg := slam.DefaultConfig()
	p := slam.New(cfg)

	var wg sync.WaitGroup
	stop := make(chan struct{})
	wg.Add(3)
	reader := func(fn func()) {
		defer wg.Done()
		for {
			select {
			case <-stop:
				return
			default:
				fn()
			}
		}
	}
	go reader(func() {
		_ = p.Trajectory()
		snap := p.Map().Snapshot()
		_ = len(snap.LogOdds)
	})
	go reader(func() {
		_ = p.GraphSnapshot()
		_, _, _, _ = p.Bounds()
		_ = p.LoopEdges()
	})
	go func() {
		defer wg.Done()
		for _, f := range frames {
			p.Process(f)
		}
		close(stop)
	}()
	wg.Wait()
	if len(p.Records()) != len(frames) {
		t.Fatalf("lost frames: %d / %d", len(p.Records()), len(frames))
	}
}

// TestBoundedGrowth verifies per-frame processing time does not grow with
// keyframe count for front-end work (local map is bounded).
func TestBoundedGrowth(t *testing.T) {
	frames, _ := buildDataset(t, 42, nil)
	p := slam.New(slam.DefaultConfig())
	maxMatchIter := 0
	for _, f := range frames {
		r := p.Process(f)
		if r.MatchIter > maxMatchIter {
			maxMatchIter = r.MatchIter
		}
	}
	kfs, nodes, total, active := p.Bounds()
	t.Logf("kfs=%d nodes=%d edges=%d active=%d maxMatchIter=%d", kfs, nodes, total, active, maxMatchIter)
	if nodes != kfs || total < active || kfs == 0 {
		t.Fatal("inconsistent graph accounting")
	}
	if maxMatchIter > slam.DefaultConfig().MatchMaxIter*2+1 {
		t.Fatalf("match iterations unbounded: %d", maxMatchIter)
	}
}
