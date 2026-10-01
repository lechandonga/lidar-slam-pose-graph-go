package slam_test

import (
	"fmt"
	"math"
	"math/rand"
	"testing"
	"time"

	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/geom"
	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/sensor"
	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/sim"
	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/slam"
)

func buildDataset(t *testing.T, seed int64, inject func(i int, sc *sensor.Scan, od *sensor.Odom)) ([]sensor.Frame, []geom.Pose) {
	t.Helper()
	world := sim.RectWorld(8)
	traj := sim.RectTrajectory(10, 14)
	rng := rand.New(rand.NewSource(seed))
	sp := sim.DefaultScanParams()
	op := sim.DefaultOdomParams()
	odoms := sim.MakeOdom(traj, op, rng)
	frames := make([]sensor.Frame, len(traj))
	ts := time.Unix(1000, 0)
	for i, pose := range traj {
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
	return frames, traj
}

func TestSmokeLoopClosure(t *testing.T) {
	frames, truthRaw := buildDataset(t, 42, nil)
	truth := normalizeFrame(truthRaw)
	p := slam.New(slam.DefaultConfig())
	for _, f := range frames {
		p.Process(f)
	}
	trajOut := p.Trajectory()
	rmseOdom, rmseSlam := 0.0, 0.0
	odomWorld := make([]geom.Pose, len(frames))
	for i, f := range frames {
		odomWorld[i] = geom.Pose{X: f.Odom.PoseX, Y: f.Odom.PoseY, Theta: f.Odom.PoseTheta}
	}
	odomWorld = alignToTruth(odomWorld, truth)
	trajOut = alignToTruth(trajOut, truth)
	for i := range truth {
		rmseOdom += sqPoseErrO(odomWorld[i], truth[i])
		rmseSlam += sqPoseErrO(trajOut[i], truth[i])
	}
	n := float64(len(truth))
	rmseOdom, rmseSlam = math.Sqrt(rmseOdom/n), math.Sqrt(rmseSlam/n)
	t.Logf("odom RMSE=%.3f m  slam RMSE=%.3f m  kfs=%d", rmseOdom, rmseSlam, len(p.Keyframes()))
	for _, e := range p.LoopEdges() {
		t.Logf("loop edge %d->%d status=%s chi2=%.2f reason=%s", e.From, e.To, e.Status, e.Chi2, e.Reason)
	}
	var badFused, badResid, maxErr int
	for i, r := range p.Records() {
		err := math.Hypot(trajOut[i].X-truth[i].X, trajOut[i].Y-truth[i].Y)
		if err > 0.3 {
			badFused++
		}
		if err > 0.3 && i < 50 {
			maxErr++
		}
		if r.MeanResidual > 0.1 {
			badResid++
		}
	}
	t.Logf("frames err>0.3m: %d (first half %d), frames resid>0.1: %d / %d", badFused, maxErr, badResid, len(p.Records()))
	if rmseSlam >= rmseOdom {
		t.Fatalf("loop closure did not improve trajectory: odom=%.3f slam=%.3f", rmseOdom, rmseSlam)
	}
	if len(p.LoopEdges()) == 0 {
		t.Fatal("no loop edges attempted")
	}
	fmt.Println("smoke ok")
}

func sqPoseErr(o sensor.Odom, truth geom.Pose) float64 {
	dx := o.PoseX - truth.X
	dy := o.PoseY - truth.Y
	return dx*dx + dy*dy
}

func normalizeFrame(truth []geom.Pose) []geom.Pose {
	out := make([]geom.Pose, len(truth))
	inv := geom.Inverse(truth[0])
	for i, q := range truth {
		out[i] = geom.Compose(q, inv)
	}
	return out
}

// alignToTruth finds the optimal rigid transform of path onto truth using
// centroids and singular angle search, then returns the aligned copy.
func alignToTruth(path, truth []geom.Pose) []geom.Pose {
	var cx, cy, tx, ty float64
	for i := range path {
		cx += path[i].X
		cy += path[i].Y
		tx += truth[i].X
		ty += truth[i].Y
	}
	n := float64(len(path))
	cx /= n
	cy /= n
	tx /= n
	ty /= n
	bestA, bestE := 0.0, math.Inf(1)
	for k := 0; k < 720; k++ {
		a := float64(k) * 2 * math.Pi / 720
		e := 0.0
		ca, sa := math.Cos(a), math.Sin(a)
		for i := range path {
			px := ca*(path[i].X-cx) - sa*(path[i].Y-cy) + tx
			py := sa*(path[i].X-cx) + ca*(path[i].Y-cy) + ty
			e += (px-truth[i].X)*(px-truth[i].X) + (py-truth[i].Y)*(py-truth[i].Y)
		}
		if e < bestE {
			bestE, bestA = e, a
		}
	}
	ca, sa := math.Cos(bestA), math.Sin(bestA)
	out := make([]geom.Pose, len(path))
	for i := range path {
		out[i].X = ca*(path[i].X-cx) - sa*(path[i].Y-cy) + tx
		out[i].Y = sa*(path[i].X-cx) + ca*(path[i].Y-cy) + ty
		out[i].Theta = geom.WrapAngle(path[i].Theta + bestA)
	}
	return out
}

func sqPoseErrO(a, b geom.Pose) float64 {
	dx := a.X - b.X
	dy := a.Y - b.Y
	return dx*dx + dy*dy
}
