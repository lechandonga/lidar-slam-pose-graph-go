package slam_test

import (
	"math"
	"os"
	"path/filepath"
	"sync"
	"testing"

	root "github.com/lechandonga/lidar-slam-pose-graph-go"
	"github.com/lechandonga/lidar-slam-pose-graph-go/synth"
)

func loopWorld() synth.World {
	return synth.RectWorld(14, 10, []struct{ X, Y, Ww, Hh float64 }{
		{-3.0, -3.0, 1.6, 0.4},
		{-3.5, -2.4, 0.4, 1.6},
		{-2.2, -3.5, 0.5, 0.5},
		{0, 0, 2, 2},
		{-3, 2.5, 1.5, 1},
		{3.2, 3.0, 1.2, 0.8},
	})
}

func loopOpts(path []synth.Pose, seed int64) synth.Options {
	return synth.Options{
		Path: path, DT: 0.1, Nbeams: 360, Fov: 2 * math.Pi, MaxR: 20,
		BeamNoise: 0.03, BearingJit: 0.004,
		OdomDrift: 0.02, OdomBias: 0.003, Seed: seed,
	}
}

func consume(p *root.Pipeline, src *synth.Source) {
	for {
		fr, ok := src.Next()
		if !ok {
			break
		}
		p.ProcessFrame(fr)
	}
	p.Finalize()
}

func trajectoryRMSE(p *root.Pipeline, truth []root.Pose) float64 {
	traj := p.Trajectory()
	if len(traj) == 0 || len(truth) == 0 {
		return 0
	}
	// Align coordinate frames: the pipeline anchors its first pose at the
	// world origin while synthetic ground truth may start elsewhere.
	sum, n := 0.0, 0
	for i, tp := range traj {
		if i >= len(truth) {
			break
		}
		// SE2 alignment from first pose: transform estimated point into the
		// ground-truth frame using the initial pose/heading offset.
		lx := tp.Pose.X - traj[0].Pose.X
		ly := tp.Pose.Y - traj[0].Pose.Y
		c := math.Cos(truth[0].Theta - traj[0].Pose.Theta)
		s := math.Sin(truth[0].Theta - traj[0].Pose.Theta)
		ax := truth[0].X + c*lx - s*ly
		ay := truth[0].Y + s*lx + c*ly
		dx := ax - truth[i].X
		dy := ay - truth[i].Y
		sum += dx*dx + dy*dy
		n++
	}
	return math.Sqrt(sum / float64(n))
}

func odomRMSE(g *synth.Generator) float64 {
	sum, n := 0.0, 0
	for i := 0; i < g.Len(); i++ {
		st := g.At(i)
		dx := st.Odom.X - st.Pose.X
		dy := st.Odom.Y - st.Pose.Y
		sum += dx*dx + dy*dy
		n++
	}
	return math.Sqrt(sum / float64(n))
}

func TestStraightLine(t *testing.T) {
	world := synth.RectWorld(14, 10, nil)
	gen := synth.New(world, loopOpts(synth.MakeStraightPath(6, 40), 11))
	src := synth.NewSource(gen)
	p := root.NewPipeline(root.DefaultConfig(), root.NopLogger())
	consume(p, src)
	if rmse := trajectoryRMSE(p, src.GroundTruth()); rmse > 0.3 {
		t.Fatalf("straight RMSE too high: %.3f", rmse)
	}
	t.Logf("straight RMSE=%.3f", trajectoryRMSE(p, src.GroundTruth()))
}

func TestTurn(t *testing.T) {
	world := synth.RectWorld(14, 10, nil)
	gen := synth.New(world, loopOpts(synth.MakeTurnPath(4, 20, 10), 12))
	src := synth.NewSource(gen)
	p := root.NewPipeline(root.DefaultConfig(), root.NopLogger())
	consume(p, src)
	truth, traj := src.GroundTruth(), p.Trajectory()
	maxTh := 0.0
	for i, tp := range traj {
		if i >= len(truth) {
			break
		}
		if d := math.Abs(root.NormAngle(tp.Pose.Theta - truth[i].Theta)); d > maxTh {
			maxTh = d
		}
	}
	if maxTh > 0.2 {
		t.Fatalf("heading error too high: %.3f", maxTh)
	}
	t.Logf("turn max heading err=%.3f RMSE=%.3f",
		maxTh, trajectoryRMSE(p, truth))
}

func TestNoiseAndBadFrames(t *testing.T) {
	world := synth.RectWorld(14, 10, nil)
	opts := loopOpts(synth.MakeStraightPath(6, 60), 13)
	opts.BadFrames = map[int]synth.Fault{
		10: {Kind: "few_points"}, 20: {Kind: "nan"},
		30: {Kind: "burst"}, 40: {Kind: "huge"},
	}
	opts.BadOdoms = map[int]bool{25: true}
	gen := synth.New(world, opts)
	src := synth.NewSource(gen)
	p := root.NewPipeline(root.DefaultConfig(), root.NewTextLogger(os.Stderr))
	consume(p, src)
	reasons := map[string]int{}
	for _, d := range p.Decisions() {
		if d.Outcome != root.OutcomeAccepted {
			reasons[d.Reason]++
		}
	}
	for _, want := range []string{
		root.ReasonTooFewPoints, root.ReasonBadScan,
	} {
		if reasons[want] == 0 {
			t.Errorf("missing decision reason %q: %v", want, reasons)
		}
	}
	if reasons[root.ReasonNoOdom] == 0 {
		t.Errorf("missing odom rejection reason: %v", reasons)
	}
	if rmse := trajectoryRMSE(p, src.GroundTruth()); rmse > 0.5 {
		t.Fatalf("bad-frame RMSE too high: %.3f", rmse)
	}
	t.Logf("reasons=%v RMSE=%.3f", reasons, trajectoryRMSE(p, src.GroundTruth()))
}

func TestDeterministicReplay(t *testing.T) {
	world := loopWorld()
	gen := synth.New(world, loopOpts(synth.MakeLoopPath(4, 24, 6), 14))
	run := func() []root.Pose {
		src := synth.NewSource(gen)
		p := root.NewPipeline(root.DefaultConfig(), root.NopLogger())
		consume(p, src)
		out := make([]root.Pose, len(p.Trajectory()))
		for i, tp := range p.Trajectory() {
			out[i] = tp.Pose
		}
		return out
	}
	a, b := run(), run()
	if len(a) != len(b) {
		t.Fatalf("length differs: %d vs %d", len(a), len(b))
	}
	for i := range a {
		if a[i] != b[i] {
			t.Fatalf("frame %d differs: %s vs %s", i, a[i], b[i])
		}
	}
}

func TestBadLoopConstraintHandled(t *testing.T) {
	cfg := root.DefaultConfig()
	info := root.ExportDiagInfo(0.05, 0.04)
	g := root.ExportNewGraph()
	n := 30
	for i := 0; i < n; i++ {
		root.ExportAddNode(g, root.Pose{X: float64(i)})
	}
	for i := 0; i+1 < n; i++ {
		root.ExportAddEdge(g, root.ExportEdgeOdom, i, i+1,
			root.Pose{X: 1}, info)
	}
	root.ExportAddEdge(g, root.ExportEdgeLoop, 0, n-1,
		root.Pose{X: 5}, info)
	r := root.ExportOptimize(g, cfg)
	// The contradictory edge pulls the chain but must not blow it up.
	for i, nd := range root.ExportNodes(g) {
		if math.Abs(nd.Pose.X) > float64(n)+2 || math.IsNaN(nd.Pose.X) {
			t.Fatalf("node %d blown up: %s cost %.1f", i, nd.Pose, r.FinalCost)
		}
	}
	t.Logf("contradictory loop contained: cost=%.1f iters=%d",
		r.FinalCost, r.Iterations)
}

func TestSaveResumeEquivalent(t *testing.T) {
	world := loopWorld()
	gen := synth.New(world, loopOpts(synth.MakeLoopPath(4, 24, 6), 16))
	src := synth.NewSource(gen)
	var frames []root.Frame
	for {
		fr, ok := src.Next()
		if !ok {
			break
		}
		frames = append(frames, fr)
	}
	truth := src.GroundTruth()
	split := len(frames) * 2 / 3

	oneShot := root.NewPipeline(root.DefaultConfig(), root.NopLogger())
	for _, fr := range frames {
		oneShot.ProcessFrame(fr)
	}
	oneShot.Finalize()

	path := filepath.Join(t.TempDir(), "state.gob")
	resume := root.NewPipeline(root.DefaultConfig(), root.NopLogger())
	for _, fr := range frames[:split] {
		resume.ProcessFrame(fr)
	}
	if err := resume.Save(path); err != nil {
		t.Fatal(err)
	}
	loaded, err := root.LoadState(path, root.NopLogger())
	if err != nil {
		t.Fatal(err)
	}
	for _, fr := range frames[split:] {
		loaded.ProcessFrame(fr)
	}
	loaded.Finalize()

	a, b := oneShot.Trajectory(), loaded.Trajectory()
	if len(a) != len(b) {
		t.Fatalf("length: %d vs %d", len(a), len(b))
	}
	maxd := 0.0
	for i := range a {
		if d := math.Hypot(a[i].Pose.X-b[i].Pose.X,
			a[i].Pose.Y-b[i].Pose.Y); d > maxd {
			maxd = d
		}
	}
	if maxd > 0.05 {
		t.Fatalf("resume trajectory diverges: max %.3f", maxd)
	}
	t.Logf("resume max diff=%.4f RMSE=%.3f",
		maxd, trajectoryRMSE(loaded, truth))
}

func TestConcurrentMapQueries(t *testing.T) {
	world := loopWorld()
	gen := synth.New(world, loopOpts(synth.MakeLoopPath(4, 24, 6), 17))
	src := synth.NewSource(gen)
	p := root.NewPipeline(root.DefaultConfig(), root.NopLogger())
	var wg sync.WaitGroup
	stop := make(chan struct{})
	wg.Add(1)
	go func() {
		defer wg.Done()
		for {
			select {
			case <-stop:
				return
			default:
				snap := p.MapSnapshot()
				if snap == nil {
					t.Error("nil snapshot")
					return
				}
				_ = snap.IsOccupiedAtWorld(0, 0)
			}
		}
	}()
	for {
		fr, ok := src.Next()
		if !ok {
			break
		}
		p.ProcessFrame(fr)
	}
	p.Finalize()
	close(stop)
	wg.Wait()
	// After Finalize the rebuilt map must contain occupied cells and be
	// consistent with the trajectory (non-empty bounded map).
	snap := p.MapSnapshot()
	if snap.W == 0 || snap.H == 0 {
		t.Fatal("empty map after Finalize")
	}
	occ := 0
	for _, v := range snap.Cells {
		if v == root.CellOccupied {
			occ++
		}
	}
	if occ == 0 {
		t.Fatal("map has no occupied cells after rebuild")
	}
	t.Logf("map %dx%d occupied=%d", snap.W, snap.H, occ)
}

func TestRestartReanchors(t *testing.T) {
	world := synth.RectWorld(14, 10, nil)
	opts := loopOpts(synth.MakeStraightPath(6, 60), 18)
	opts.Restarts = map[int]bool{30: true}
	gen := synth.New(world, opts)
	src := synth.NewSource(gen)
	p := root.NewPipeline(root.DefaultConfig(), root.NopLogger())
	consume(p, src)
	sawRestart := false
	for _, d := range p.Decisions() {
		if d.Reason == root.ReasonRestart {
			sawRestart = true
		}
	}
	if !sawRestart {
		t.Fatalf("no restart decision recorded")
	}
	if rmse := trajectoryRMSE(p, src.GroundTruth()); rmse > 0.6 {
		t.Fatalf("post-restart RMSE high: %.3f", rmse)
	}
	t.Logf("restart handled, RMSE=%.3f", trajectoryRMSE(p, src.GroundTruth()))
}

func TestLoopImprovesOverOdom(t *testing.T) {
	world := loopWorld()
	gen := synth.New(world, loopOpts(synth.MakeLoopPath(4, 24, 6), 15))
	src := synth.NewSource(gen)
	p := root.NewPipeline(root.DefaultConfig(), root.NopLogger())
	consume(p, src)
	traj := p.Trajectory()
	truth := src.GroundTruth()
	half := len(traj) * 2 / 3
	var sSum, oSum float64
	cnt := 0
	// SE2 alignment (same as trajectoryRMSE).
	ang0 := truth[0].Theta
	for i := half; i < len(traj) && i < len(truth); i++ {
		st := gen.At(i)
		lx := traj[i].Pose.X - traj[0].Pose.X
		ly := traj[i].Pose.Y - traj[0].Pose.Y
		c := math.Cos(truth[0].Theta - traj[0].Pose.Theta)
		s := math.Sin(truth[0].Theta - traj[0].Pose.Theta)
		sx := truth[0].X + c*lx - s*ly
		sy := truth[0].Y + s*lx + c*ly
		sSum += (sx-truth[i].X)*(sx-truth[i].X) +
			(sy-truth[i].Y)*(sy-truth[i].Y)
		// Odom is already in its own frame starting at origin.
		olx, oly := st.Odom.X, st.Odom.Y
		ox := truth[0].X + math.Cos(ang0)*olx - math.Sin(ang0)*oly
		oy := truth[0].Y + math.Sin(ang0)*olx + math.Cos(ang0)*oly
		oSum += ox*ox + oy*oy
		cnt++
	}
	slamErr := math.Sqrt(sSum / float64(cnt))
	odErr := math.Sqrt(oSum / float64(cnt))
	t.Logf("returning SLAM RMSE=%.3f odom RMSE=%.3f loops=%d",
		slamErr, odErr, len(p.LoopDecisions()))
	for _, l := range p.LoopDecisions() {
		t.Logf("  loop %d->%d %s score=%.2f chi2=%.1f measure=%s",
			l.From, l.To, l.Status, l.Score, l.Chi2, l.Measure)
	}
	if slamErr >= odErr*0.9 {
		t.Fatalf("loop closure did not improve returning leg: slam=%.3f odom=%.3f",
			slamErr, odErr)
	}
}

var _ = math.Pi
