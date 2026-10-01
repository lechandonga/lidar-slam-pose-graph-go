package scanmatch_test

import (
	"math"
	"math/rand"
	"testing"

	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/geom"
	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/gridmap"
	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/scanmatch"
	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/sim"
)

func raster(m *gridmap.Map, pose geom.Pose, pts []geom.Point) {
	for _, q := range pts {
		w := pose.Transform(q)
		m.AddLine(pose.X, pose.Y, w.X, w.Y, true)
	}
}

func TestMatchConverges(t *testing.T) {
	world := sim.RectWorld(8)
	sp := sim.DefaultScanParams()
	rng := rand.New(rand.NewSource(7))
	truth := geom.Pose{X: 1.2, Y: -0.7, Theta: 0.3}
	ranges := sim.MakeScan(world, truth, sp, rng)
	sc := sensorScan(ranges, sp)
	pts := scanmatch.ValidPoints(sc)
	m := gridmap.NewMap(0.05, 10, 1, -1, -10, 10)
	raster(m, truth, pts)
	field := m.BuildField(truth.X, truth.Y, 8)

	init := geom.Pose{X: truth.X + 0.15, Y: truth.Y - 0.12, Theta: truth.Theta + 0.08}
	res := scanmatch.Match(pts, init, field, scanmatch.Params{
		MaxIter: 30, ConvergeStep: 1e-5, HuberSigma: 0.15,
		MinInfoEigenvalue: 0, MaxResidual: 0.15, MaxPosErr: 1, MaxAngErr: 0.5,
	}, 1, 0.5)
	err := math.Hypot(res.Pose.X-truth.X, res.Pose.Y-truth.Y)
	t.Logf("result=%+v resid=%.4f err=%.4f reason=%s converged=%v it=%d",
		res.Pose, res.MeanResid, err, res.Reason, res.Converged, res.Iterations)
	if err > 0.05 {
		t.Fatalf("match did not recover truth, err=%.3f", err)
	}
}

func TestMatchCorridorDegeneracy(t *testing.T) {
	// Two parallel walls: translation along the corridor is unobservable.
	m := gridmap.NewMap(0.05, 5, 1, -1, -10, 10)
	for x := -3.0; x <= 3.0; x += 0.05 {
		m.AddHit(x, 1.0)
		m.AddHit(x, -1.0)
	}
	f := m.BuildField(0, 0, 4)
	pts := []geom.Point{{X: 1.0, Y: 1.0}, {X: -0.5, Y: 1.0}, {X: 0.5, Y: -1.0}, {X: -1.2, Y: -1.0}}
	init := geom.Pose{X: 0.1, Y: 0, Theta: 0}
	res := scanmatch.Match(pts, init, f, scanmatch.Params{
		MaxIter: 20, ConvergeStep: 1e-5, HuberSigma: 0.15,
		MinInfoEigenvalue: 5.0, MaxResidual: 0.15, MaxPosErr: 1, MaxAngErr: 0.5,
	}, 1, 0.5)
	t.Logf("degenerate result reason=%s eigen=%.3f", res.Reason, res.MinEigen)
	if res.Reason != scanmatch.ReasonDegenerate {
		t.Fatalf("expected degeneracy, got %s", res.Reason)
	}
}
