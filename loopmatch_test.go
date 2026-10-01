package slam_test

import (
	"math"
	"testing"

	root "github.com/lechandonga/lidar-slam-pose-graph-go"
	"github.com/lechandonga/lidar-slam-pose-graph-go/synth"
)

// TestLoopLocalMatch generates two scans of the SAME place from identical
// poses and checks the loop matcher recovers near-identity relative motion.
func TestLoopLocalMatch(t *testing.T) {
	world := synth.RectWorld(14, 10, []struct{ X, Y, Ww, Hh float64 }{
		{-3.0, -3.0, 1.6, 0.4},
		{-3.5, -2.4, 0.4, 1.6},
		{0, 0, 2, 2},
	})
	gen := synth.New(world, synth.Options{
		Path: []synth.Pose{{X: -4, Y: -4, Theta: 0}},
		DT:   0.1, Nbeams: 360, Fov: 2 * math.Pi, MaxR: 20,
		BeamNoise: 0.03, Seed: 3,
	})
	a := gen.At(0)
	gen2 := synth.New(world, synth.Options{
		Path: []synth.Pose{{X: -4, Y: -4, Theta: 0}},
		DT:   0.1, Nbeams: 360, Fov: 2 * math.Pi, MaxR: 20,
		BeamNoise: 0.03, Seed: 99,
	})
	b := gen2.At(0)
	cfg := root.DefaultConfig()
	cfg.MatchWinXY = 1.0
	cfg.MatchWinTheta = math.Pi
	var ref, scan []root.Point
	for _, pt := range a.Points {
		ref = append(ref, root.Point{X: pt.X, Y: pt.Y})
	}
	for _, pt := range b.Points {
		scan = append(scan, root.Point{X: pt.X, Y: pt.Y})
	}
	mr := root.ExportMatchScan(cfg, ref, scan, root.Pose{})
	t.Logf("score=%.2f pose=%s eig=%.3f", mr.Score, mr.Pose, mr.EigRatio)
	if mr.Score < 0.7 {
		t.Fatalf("same-place match score too low: %.2f", mr.Score)
	}
	if d := math.Hypot(mr.Pose.X, mr.Pose.Y); d > 0.1 {
		t.Fatalf("relative motion not near zero: %s", mr.Pose)
	}

	// With an 0.4m wrong odom seed, the coarse+ICP must recover identity.
	cfg.MatchWinXY = 1.5
	mr2 := root.ExportMatchScan(cfg, ref, scan, root.Pose{X: 0.4, Y: -0.2, Theta: 0.1})
	t.Logf("seeded score=%.2f pose=%s", mr2.Score, mr2.Pose)
	if d := math.Hypot(mr2.Pose.X, mr2.Pose.Y); d > 0.15 {
		t.Fatalf("seeded recovery failed: %s", mr2.Pose)
	}
}

// TestLoopRevisit matches the first scan of a loop path against the last,
// which observe the same corner from (nearly) the same pose.
func TestLoopRevisit(t *testing.T) {
	world := synth.RectWorld(14, 10, []struct{ X, Y, Ww, Hh float64 }{
		{-3.0, -3.0, 1.6, 0.4},
		{-3.5, -2.4, 0.4, 1.6},
		{0, 0, 2, 2},
		{3.2, 3.0, 1.2, 0.8},
	})
	path := synth.MakeLoopPath(4.0, 24, 6)
	gen := synth.New(world, synth.Options{
		Path: path, DT: 0.1, Nbeams: 360, Fov: 2 * math.Pi, MaxR: 20,
		BeamNoise: 0.03, BearingJit: 0.004, Seed: 5,
	})
	first := gen.At(0)
	last := gen.At(gen.Len() - 1)
	var ref, scan []root.Point
	for _, pt := range first.Points {
		ref = append(ref, root.Point{X: pt.X, Y: pt.Y})
	}
	for _, pt := range last.Points {
		scan = append(scan, root.Point{X: pt.X, Y: pt.Y})
	}
	cfg := root.DefaultConfig()
	cfg.MatchWinXY = 2.0
	cfg.MatchWinTheta = math.Pi
	// Seed with zero relative motion (the truth for a closed loop).
	mr := root.ExportMatchScan(cfg, ref, scan, root.Pose{})
	t.Logf("revisit score=%.2f pose=%s firstTh=%.2f lastTh=%.2f",
		mr.Score, mr.Pose, first.Pose.Theta, last.Pose.Theta)
	if mr.Score < 0.45 {
		t.Fatalf("revisit score too low: %.2f", mr.Score)
	}
	if math.Abs(root.NormAngle(mr.Pose.Theta+math.Pi/2)) > 0.15 {
		t.Fatalf("revisit rotation wrong: %s", mr.Pose)
	}
}
