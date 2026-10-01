package slam

import "testing"

func TestOptimizeLoopPulls(t *testing.T) {
	g := &Graph{}
	// 3 nodes in a chain: 0 at 0, 1 at 1, 2 drifted to 2.2 instead of 2.
	cfg := DefaultConfig()
	info := diagInfo(0.05, 0.04)
	g.Nodes = []*Keyframe{
		{ID: 0, Pose: Pose{}},
		{ID: 1, Pose: Pose{X: 1}},
		{ID: 2, Pose: Pose{X: 2.2}},
	}
	g.Edges = []*Edge{
		{I: 0, J: 1, Kind: EdgeOdom, Measure: Pose{X: 1},
			Info: info, RawInfo: info, Status: LoopAccepted},
		{I: 1, J: 2, Kind: EdgeOdom, Measure: Pose{X: 1.1},
			Info: info, RawInfo: info, Status: LoopAccepted},
		{I: 0, J: 2, Kind: EdgeLoop, Measure: Pose{X: 2.0},
			Info: info, RawInfo: info, Status: LoopAccepted},
	}
	r := optimizeGraph(g, cfg)
	t.Logf("iters=%d converged=%v cost=%.3f poses=%s %s %s",
		r.Iterations, r.Converged, r.FinalCost,
		g.Nodes[0].Pose, g.Nodes[1].Pose, g.Nodes[2].Pose)
	if g.Nodes[2].Pose.X > 2.15 {
		t.Fatalf("loop edge did not pull node 2 back: %.3f", g.Nodes[2].Pose.X)
	}
}

func TestOptimizePCGStall(t *testing.T) {
	cfg := DefaultConfig()
	info := diagInfo(0.1, 0.08)
	n := 100
	g := &Graph{}
	for i := 0; i < n; i++ {
		g.Nodes = append(g.Nodes, &Keyframe{
			ID: i, Pose: Pose{X: float64(i) * 1.05},
		})
	}
	for i := 0; i+1 < n; i++ {
		g.Edges = append(g.Edges, &Edge{
			I: i, J: i + 1, Kind: EdgeOdom, Measure: Pose{X: 1.05},
			Info: info, RawInfo: info, Status: LoopAccepted,
		})
	}
	// Large contradictory loop.
	g.Edges = append(g.Edges, &Edge{
		I: 4, J: 76, Kind: EdgeLoop,
		Measure: Pose{X: 72.0 - 4.0},
		Info:    diagInfo(0.1, 0.08), RawInfo: diagInfo(0.1, 0.08),
		Status: LoopAccepted,
	})
	r := optimizeGraph(g, cfg)
	t.Logf("iters=%d conv=%v cost=%.3f", r.Iterations, r.Converged, r.FinalCost)
}

func TestOptimizeBigLoop(t *testing.T) {
	cfg := DefaultConfig()
	info := diagInfo(0.1, 0.08)
	n := 100
	g := &Graph{}
	for i := 0; i < n; i++ {
		g.Nodes = append(g.Nodes, &Keyframe{
			ID: i, Pose: Pose{X: float64(i) * 1.05},
		})
	}
	for i := 0; i+1 < n; i++ {
		g.Edges = append(g.Edges, &Edge{
			I: i, J: i + 1, Kind: EdgeOdom, Measure: Pose{X: 1.05},
			Info: info, RawInfo: info, Status: LoopAccepted,
		})
	}
	// Loop requiring a ~5m correction over 72 nodes (chi2 roughly (5/0.1)^2).
	g.Edges = append(g.Edges, &Edge{
		I: 4, J: 76, Kind: EdgeLoop,
		Measure: Pose{X: 71.0},
		Info:    info, RawInfo: info, Status: LoopAccepted,
	})
	r := optimizeGraph(g, cfg)
	t.Logf("iters=%d conv=%v cost=%.3f x76=%.3f",
		r.Iterations, r.Converged, r.FinalCost, g.Nodes[76].Pose.X)
	if g.Nodes[76].Pose.X > 78.5 {
		t.Fatalf("loop correction failed: %.3f", g.Nodes[76].Pose.X)
	}
}

func TestOptimizeSoftEdgeStable(t *testing.T) {
	cfg := DefaultConfig()
	info := diagInfo(0.1, 0.08)
	n := 90
	g := &Graph{}
	for i := 0; i < n; i++ {
		g.Nodes = append(g.Nodes, &Keyframe{
			ID: i, Pose: Pose{X: float64(i) * 1.05, Y: float64(i) * 0.01},
		})
	}
	for i := 0; i+1 < n; i++ {
		g.Edges = append(g.Edges, &Edge{
			I: i, J: i + 1, Kind: EdgeOdom, Measure: Pose{X: 1.05, Y: 0.01},
			Info: info, RawInfo: info, Status: LoopAccepted,
		})
	}
	// Soft edge, chi2 around 27: residual ~0.5m with scale 0.02.
	soft := scaleInfo(info, 0.02)
	g.Edges = append(g.Edges, &Edge{
		I: 0, J: 86, Kind: EdgeLoop,
		Measure: Pose{X: float64(86)*1.05 - 0.5},
		Info:    soft, RawInfo: info, Status: LoopSoft,
	})
	r := optimizeGraph(g, cfg)
	for i, n2 := range g.Nodes {
		if n2.Pose.X > 200 || n2.Pose.Y > 200 {
			t.Fatalf("node %d blew up: %s", i, n2.Pose)
		}
	}
	t.Logf("iters=%d conv=%v cost=%.3f", r.Iterations, r.Converged, r.FinalCost)
}

func TestOptimizeSquareLoop(t *testing.T) {
	// U-shape loop: chain of nodes, a single loop edge pulls the end back
	// to near the start. Initial poses are a drifted odom chain.
	cfg := DefaultConfig()
	info := diagInfo(0.05, 0.04)
	const perLeg = 25
	n := 2*perLeg + 1
	truth := make([]Pose, n)
	step := 0.2
	for i := 0; i < n; i++ {
		if i <= perLeg {
			truth[i] = Pose{X: float64(i) * step, Theta: 0}
		} else {
			k := i - perLeg
			truth[i] = Pose{X: float64(perLeg) * step, Y: float64(k) * step,
				Theta: 1.5708}
		}
	}
	g := &Graph{}
	for i, tp := range truth {
		pp := tp
		if i <= perLeg {
			pp.X *= 1.08
		}
		g.Nodes = append(g.Nodes, &Keyframe{ID: i, Pose: pp})
	}
	for i := 0; i+1 < n; i++ {
		m := Between(truth[i], truth[i+1])
		g.Edges = append(g.Edges, &Edge{
			I: i, J: i + 1, Kind: EdgeOdom, Measure: m,
			Info: info, RawInfo: info, Status: LoopAccepted,
		})
	}
	// Loop: end is 5m north of start (true geometry).
	g.Edges = append(g.Edges, &Edge{
		I: 0, J: n - 1, Kind: EdgeLoop,
		Measure: Between(truth[0], truth[n-1]),
		Info:    diagInfo(0.1, 0.08), RawInfo: diagInfo(0.1, 0.08),
		Status: LoopAccepted,
	})
	r := optimizeGraph(g, cfg)
	maxErr := 0.0
	for i := 1; i < n; i++ {
		if d := PoseDist(g.Nodes[i].Pose, truth[i]); d > maxErr {
			maxErr = d
		}
	}
	t.Logf("iters=%d conv=%v cost=%.3f maxErr=%.3f last=%s",
		r.Iterations, r.Converged, r.FinalCost, maxErr, g.Nodes[n-1].Pose)
	if maxErr > 0.15 {
		t.Fatalf("loop closure inaccurate: maxErr %.3f", maxErr)
	}
}
