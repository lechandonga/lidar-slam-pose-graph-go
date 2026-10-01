package posegraph

import (
	"math"
	"testing"

	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/geom"
)

func TestOptimizeTriangle(t *testing.T) {
	g := New()
	n0 := g.AddNode(0, geom.Pose{X: 0, Y: 0, Theta: 0})
	n1 := g.AddNode(1, geom.Pose{X: 1.1, Y: 0.05, Theta: 0})
	n2 := g.AddNode(2, geom.Pose{X: 1.0, Y: 1.1, Theta: 0})
	info := [3][3]float64{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}
	g.AddEdge(Edge{From: n0, To: n1, Type: EdgeOdom, Rel: geom.Pose{X: 1}, Info: info, Status: StatusAccepted})
	g.AddEdge(Edge{From: n1, To: n2, Type: EdgeOdom, Rel: geom.Pose{X: 0, Y: 1}, Info: info, Status: StatusAccepted})
	g.AddEdge(Edge{From: n0, To: n2, Type: EdgeLoop, Rel: geom.Pose{X: 1, Y: 1}, Info: info, Status: StatusAccepted})
	rep := g.Optimize(OptimizeParams{MaxIter: 30, ConvergeChi2: 1e-6, ConvergeStep: 1e-8, HuberDelta: 0})
	nodes := g.Nodes()
	for _, n := range nodes {
		t.Logf("node %d: %+v", n.ID, n.Pose)
	}
	t.Logf("rep %+v startChi2=%.4f", rep, rep.StartChi2)
	if math.Abs(nodes[2].Pose.X-1) > 0.05 || math.Abs(nodes[2].Pose.Y-1) > 0.05 {
		t.Fatalf("node2 not at (1,1): %+v", nodes[2].Pose)
	}
}

func TestOptimizeSquareLoop(t *testing.T) {
	g := New()
	truth := []geom.Pose{
		{X: 0, Y: 0, Theta: 0}, {X: 1, Y: 0, Theta: 0}, {X: 2, Y: 0, Theta: 0}, {X: 2, Y: 1, Theta: math.Pi / 2},
		{X: 2, Y: 2, Theta: math.Pi / 2}, {X: 1, Y: 2, Theta: math.Pi}, {X: 0, Y: 2, Theta: math.Pi},
		{X: 0, Y: 1, Theta: -math.Pi / 2}, {X: 0, Y: 0, Theta: -math.Pi / 2},
	}
	info := [3][3]float64{{4, 0, 0}, {0, 4, 0}, {0, 0, 2}}
	for i, q := range truth {
		id := g.AddNode(i, q)
		if i > 0 {
			rel := geom.Between(truth[i], truth[i-1])
			rel.X += 0.03 // small noisy edge
			g.AddEdge(Edge{From: id - 1, To: id, Type: EdgeOdom, Rel: rel, Info: info, Status: StatusAccepted})
		}
	}
	// loop: last pose equals first
	g.AddEdge(Edge{From: 0, To: len(truth) - 1, Type: EdgeLoop,
		Rel: geom.Between(truth[len(truth)-1], truth[0]), Info: info, Status: StatusAccepted})
	rep := g.Optimize(OptimizeParams{MaxIter: 50, ConvergeChi2: 1e-7, ConvergeStep: 1e-9, HuberDelta: 1})
	t.Logf("chi2 %.5f -> %.5f iters=%d converged=%v", rep.StartChi2, rep.FinalChi2, rep.Iterations, rep.Converged)
	for _, n := range g.Nodes() {
		q := truth[n.ID]
		err := math.Hypot(n.Pose.X-q.X, n.Pose.Y-q.Y)
		if err > 0.15 {
			t.Fatalf("node %d drifted: got %+v want %+v", n.ID, n.Pose, q)
		}
	}
}
