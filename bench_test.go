package slam_test

import (
	"testing"

	root "github.com/lechandonga/lidar-slam-pose-graph-go"
	"github.com/lechandonga/lidar-slam-pose-graph-go/synth"
)

func BenchmarkLoopRun(b *testing.B) {
	for n := 0; n < b.N; n++ {
		world := synth.RectWorld(14, 10, nil)
		path := synth.MakeLoopPath(4.0, 24, 6)
		gen := synth.New(world, synth.Options{
			Path: path, DT: 0.1, Nbeams: 360, Fov: 6.283, MaxR: 20,
			BeamNoise: 0.03, Seed: 42,
		})
		src := synth.NewSource(gen)
		p := root.NewPipeline(root.DefaultConfig(), root.NopLogger())
		for {
			fr, ok := src.Next()
			if !ok {
				break
			}
			p.ProcessFrame(fr)
		}
		p.Finalize()
	}
}

func BenchmarkNoLoop(b *testing.B) {
	for n := 0; n < b.N; n++ {
		world := synth.RectWorld(14, 10, nil)
		path := synth.MakeLoopPath(4.0, 24, 6)
		gen := synth.New(world, synth.Options{
			Path: path, DT: 0.1, Nbeams: 360, Fov: 6.283, MaxR: 20,
			BeamNoise: 0.03, Seed: 42,
		})
		src := synth.NewSource(gen)
		cfg := root.DefaultConfig()
		cfg.LoopMinFrames = 1 << 30
		p := root.NewPipeline(cfg, root.NopLogger())
		for {
			fr, ok := src.Next()
			if !ok {
				break
			}
			p.ProcessFrame(fr)
		}
		p.Finalize()
	}
}
