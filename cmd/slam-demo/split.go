package main

import (
	"fmt"
	"math"
	"os"

	root "github.com/lechandonga/lidar-slam-pose-graph-go"
	"github.com/lechandonga/lidar-slam-pose-graph-go/synth"
)

// runSplitDemo builds one trajectory, records it as two independently
// originated sessions sharing an overlap, and merges them: session A is
// saved, loaded, then session B is globally localized in A's map and
// anchored before its frames are consumed.
func runSplitDemo(outDir string, seed int64) {
	world := synth.RectWorld(16, 12, []struct{ X, Y, Ww, Hh float64 }{
		{-4.5, -4.0, 2.0, 0.5}, {0.5, 0.5, 2.2, 2.2},
		{-4.0, 3.2, 1.8, 1.0}, {3.8, 3.4, 1.4, 0.9},
		{5.0, -2.2, 0.7, 1.8}, {2.0, -4.2, 1.2, 0.5},
	})
	// Two straight legs joined by a turn; split with a 10-frame overlap.
	path := makeOverlapDemoPath()
	opt := synth.Options{
		DT: 0.1, Nbeams: 360, Fov: 2 * math.Pi, MaxR: 20,
		BeamNoise: 0.02, BearingJit: 0.003,
		OdomDrift: 0.02, OdomBias: 0.002, Seed: seed,
	}
	cut1 := 30
	cut2 := cut1 - 10
	sessA, sessB, _ := synth.SplitSessions(world, path, opt, cut1, cut2)

	logger := root.NewTextLogger(os.Stdout)
	p := root.NewPipeline(root.DefaultConfig(), logger)
	srcA := synth.SessionSource(sessA)
	for {
		fr, ok := srcA.Next()
		if !ok {
			break
		}
		p.ProcessFrame(fr)
	}
	p.Finalize()
	statePath := outDir + "/split-state.gob"
	if err := p.Save(statePath); err != nil {
		panic(err)
	}

	loaded, err := root.LoadState(statePath, logger)
	if err != nil {
		panic(err)
	}
	if err := loaded.PrepareNewSession(); err != nil {
		panic(err)
	}
	srcB := synth.SessionSource(sessB)
	first, _ := srcB.Next()
	lr := loaded.Localize(first.Scan, root.LocalizeHint{})
	fmt.Printf("\n=== cross-session localization ===\n")
	fmt.Printf("status=%s reason=%s score=%.3f residual=%.3f pose=%s candidates=%d\n",
		lr.Status, lr.Reason, lr.Score, lr.Residual, lr.Pose, len(lr.Candidates))
	if lr.Status == root.LocalizeUnique {
		errXY := math.Hypot(lr.Pose.X-sessB.StartWorld.X,
			lr.Pose.Y-sessB.StartWorld.Y)
		errTh := math.Abs(lr.Pose.Theta - sessB.StartWorld.Theta)
		fmt.Printf("annotated start=%v anchor error: dXY=%.3fm dTheta=%.4frad\n",
			sessB.StartWorld, errXY, errTh)
	}
	if lr.Status != root.LocalizeUnique {
		fmt.Printf("refusing to anchor (%s); old map left unchanged\n", lr.Status)
		return
	}
	if err := loaded.BeginSession(first, lr); err != nil {
		panic(err)
	}
	for {
		fr, ok := srcB.Next()
		if !ok {
			break
		}
		loaded.ProcessFrame(fr)
	}
	loaded.Finalize()

	n, e := loaded.GraphSummary()
	snap := loaded.MapSnapshot()
	occ := 0
	for _, v := range snap.Cells {
		if v == root.CellOccupied {
			occ++
		}
	}
	fmt.Printf("merged: nodes=%d edges=%d occupied=%d session=%d\n",
		n, e, occ, loaded.Session())
	mf, err := os.Create(outDir + "/split-map.pgm")
	if err != nil {
		panic(err)
	}
	defer mf.Close()
	if err := root.DumpMapPGM(mf, snap); err != nil {
		panic(err)
	}
	fmt.Println("wrote split-map.pgm and split-state.gob")
}

// makeOverlapDemoPath: east 6 m (30 frames), turn, north 6 m (30 frames).
func makeOverlapDemoPath() []synth.Pose {
	const n = 30
	pts := make([]synth.Pose, 0, 2*n+8)
	for i := 0; i < n; i++ {
		s := float64(i) / float64(n-1)
		pts = append(pts, synth.Pose{X: s * 6})
	}
	corner := pts[len(pts)-1]
	for i := 1; i <= 8; i++ {
		th := math.Pi / 2 * float64(i) / 8
		pts = append(pts, synth.Pose{X: corner.X, Y: corner.Y, Theta: th})
	}
	for i := 1; i <= n; i++ {
		s := float64(i) / float64(n)
		pts = append(pts, synth.Pose{X: corner.X, Y: corner.Y + s*6,
			Theta: math.Pi / 2})
	}
	return pts
}
