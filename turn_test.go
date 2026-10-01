package slam_test

import (
	"math"
	"testing"

	root "github.com/lechandonga/lidar-slam-pose-graph-go"
	"github.com/lechandonga/lidar-slam-pose-graph-go/synth"
)

func TestTurnTracking(t *testing.T) {
	world := synth.RectWorld(14, 10, nil)
	path := synth.MakeTurnPath(4.0, 24, 10)
	gen := synth.New(world, synth.Options{
		Path: path, DT: 0.1, Nbeams: 360, Fov: 2 * math.Pi, MaxR: 20,
		BeamNoise: 0.03, BearingJit: 0.004,
		OdomDrift: 0.02, OdomBias: 0.003, Seed: 7,
	})
	src := synth.NewSource(gen)
	truth := src.GroundTruth()
	p := root.NewPipeline(root.DefaultConfig(), root.NopLogger())
	i := 0
	for {
		fr, ok := src.Next()
		if !ok {
			break
		}
		p.ProcessFrame(fr)
		if i%8 == 0 || i == len(truth)-1 {
			tp := p.Trajectory()
			est := tp[len(tp)-1].Pose
			dth := math.Abs(root.NormAngle(est.Theta - truth[i].Theta))
			t.Logf("i=%d truthTh=%.2f estTh=%.2f dth=%.3f xy=(%.2f,%.2f) truth=(%.2f,%.2f)",
				i, truth[i].Theta, est.Theta, dth,
				est.X, est.Y, truth[i].X, truth[i].Y)
		}
		i++
	}
}
