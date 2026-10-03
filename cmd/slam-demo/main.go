// slam-demo generates a local synthetic loop dataset, runs the full SLAM
// pipeline (scan match + odom fusion, pose graph with loop closure, grid
// map), prints key decisions and errors, and writes map.pgm plus a
// trajectory CSV. It uses no network and is fully deterministic.
package main

import (
	"flag"
	"fmt"
	"math"
	"os"

	root "github.com/lechandonga/lidar-slam-pose-graph-go"
	"github.com/lechandonga/lidar-slam-pose-graph-go/synth"
)

func main() {
	out := flag.String("out", ".", "output directory")
	seed := flag.Int64("seed", 42, "random seed")
	mode := flag.String("mode", "loop", "loop | split (two independent sessions)")
	flag.Parse()
	if *mode == "split" {
		runSplitDemo(*out, *seed)
		return
	}

	world := synth.RectWorld(14, 10, []struct{ X, Y, Ww, Hh float64 }{
		{0, 0, 2, 2},
		{-3, 2.5, 1.5, 1},
	})
	path := synth.MakeLoopPath(4.0, 24, 6)
	gen := synth.New(world, synth.Options{
		Path:       path,
		DT:         0.1,
		Nbeams:     360,
		Fov:        2 * math.Pi,
		MaxR:       20,
		BeamNoise:  0.03,
		BearingJit: 0.004,
		OdomDrift:  0.05,
		OdomBias:   0.008,
		Seed:       *seed,
	})
	src := synth.NewSource(gen)
	p := root.NewPipeline(root.DefaultConfig(), root.NewTextLogger(os.Stdout))
	if err := p.Run(src); err != nil {
		panic(err)
	}
	opt := p.Finalize()

	traj := p.Trajectory()
	truth := src.GroundTruth()
	odomAt := make(map[int64]root.Pose, len(traj))
	src2 := synth.NewSource(gen)
	for {
		fr, ok := src2.Next()
		if !ok {
			break
		}
		if fr.HasOdom {
			odomAt[fr.Scan.ID] = fr.Odom.Pose
		}
	}
	rmse := trajectoryRMSE(traj, truth)

	var acc, deg, rej int
	for _, d := range p.Decisions() {
		switch d.Outcome {
		case root.OutcomeAccepted:
			acc++
		case root.OutcomeDegraded:
			deg++
		case root.OutcomeRejected:
			rej++
		}
	}
	fmt.Printf("\nframes=%d accepted=%d degraded=%d rejected=%d\n",
		len(traj), acc, deg, rej)
	// Pure-odometry RMSE (SE2 aligned) demonstrates the correction gain.
	odomTraj := make([]root.TrajectoryPoint, 0, len(traj))
	for _, tp := range traj {
		odomTraj = append(odomTraj, root.TrajectoryPoint{
			FrameID: tp.FrameID, Time: tp.Time,
			Pose: odomAt[tp.FrameID],
		})
	}
	fmt.Printf("odom-only RMSE=%.3f m\n", trajectoryRMSE(odomTraj, truth))
	n, e := p.GraphSummary()
	fmt.Printf("graph nodes=%d edges=%d optIters=%d converged=%v cost=%.3f\n",
		n, e, opt.Iterations, opt.Converged, opt.FinalCost)
	fmt.Printf("trajectory RMSE vs ground truth: %.4f m\n", rmse)

	if err := os.WriteFile(*out+"/trajectory.csv", trajCSV(traj), 0o644); err != nil {
		panic(err)
	}
	mf, err := os.Create(*out + "/map.pgm")
	if err != nil {
		panic(err)
	}
	defer mf.Close()
	if err := root.DumpMapPGM(mf, p.MapSnapshot()); err != nil {
		panic(err)
	}
	fmt.Println("wrote map.pgm and trajectory.csv")
}

func trajectoryRMSE(traj []root.TrajectoryPoint, truth []root.Pose) float64 {
	if len(traj) == 0 {
		return 0
	}
	cosA := math.Cos(truth[0].Theta - traj[0].Pose.Theta)
	sinA := math.Sin(truth[0].Theta - traj[0].Pose.Theta)
	sum, n := 0.0, 0
	for i, tp := range traj {
		if i >= len(truth) {
			break
		}
		lx := tp.Pose.X - traj[0].Pose.X
		ly := tp.Pose.Y - traj[0].Pose.Y
		ax := truth[0].X + cosA*lx - sinA*ly
		ay := truth[0].Y + sinA*lx + cosA*ly
		sum += (ax-truth[i].X)*(ax-truth[i].X) +
			(ay-truth[i].Y)*(ay-truth[i].Y)
		n++
	}
	return math.Sqrt(sum / float64(n))
}

func trajCSV(traj []root.TrajectoryPoint) []byte {
	s := []byte("frame,time,x,y,theta\n")
	for _, t := range traj {
		s = append(s, fmt.Appendf(nil, "%d,%.3f,%.4f,%.4f,%.5f\n",
			t.FrameID, t.Time, t.Pose.X, t.Pose.Y, t.Pose.Theta)...)
	}
	return s
}
