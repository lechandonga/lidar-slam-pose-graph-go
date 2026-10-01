// slamdemo runs the full local SLAM pipeline on deterministic synthetic
// data, prints per-frame verdicts and error metrics, and writes the final
// grid map and trajectories to disk. No network or external services.
package main

import (
	"flag"
	"fmt"
	"log"
	"math"
	"math/rand"
	"os"
	"path/filepath"
	"time"

	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/geom"
	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/sensor"
	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/sim"
	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/slam"
)

func main() {
	var (
		outDir  = flag.String("out", "output", "output directory")
		seed    = flag.Int64("seed", 42, "deterministic RNG seed")
		verbose = flag.Bool("v", false, "print every frame verdict")
		saveAt  = flag.Int("save-at", -1, "save state at this frame, load and continue (0 = off)")
	)
	flag.Parse()
	if err := os.MkdirAll(*outDir, 0o755); err != nil {
		log.Fatal(err)
	}

	world := sim.RectWorld(8)
	truth0 := sim.RectTrajectory(10, 14)
	rng := rand.New(rand.NewSource(*seed))
	sp := sim.DefaultScanParams()
	op := sim.DefaultOdomParams()
	odoms := sim.MakeOdom(truth0, op, rng)
	truth := normalize(truth0)

	cfg := slam.DefaultConfig()
	p := slam.New(cfg)
	ts := time.Unix(1000, 0)

	statePath := filepath.Join(*outDir, "state.gob")
	for i := range truth0 {
		ranges := sim.MakeScan(world, truth0[i], sp, rng)
		sc := sensor.Scan{
			Time:     ts.Add(time.Duration(i) * 100 * time.Millisecond),
			AngleMin: sp.AngleMin, AngleInc: (sp.AngleMax - sp.AngleMin) / float64(sp.Beams-1),
			RangeMin: sp.RangeMin, RangeMax: sp.RangeMax, Ranges: ranges,
		}
		od := sensor.Odom{Time: sc.Time, PoseX: odoms[i].X, PoseY: odoms[i].Y, PoseTheta: odoms[i].Theta}
		rec := p.Process(sensor.Frame{Seq: i, Scan: sc, Odom: od})
		if *verbose {
			fmt.Printf("frame %3d mode=%-20s reason=%-28s resid=%5.3f kf=%-5v loop=%s\n",
				rec.Seq, rec.Mode, rec.Reason, rec.MeanResidual, rec.NewKeyframe, rec.LoopStatus)
		}
		if *saveAt >= 0 && i == *saveAt {
			if err := p.Save(statePath); err != nil {
				log.Fatalf("save: %v", err)
			}
			p2, err := slam.Load(statePath)
			if err != nil {
				log.Fatalf("load: %v", err)
			}
			p = p2
			fmt.Printf("state saved+reloaded at frame %d\n", i)
		}
	}

	out := align(p.Trajectory(), truth)
	odomAligned := align(odoms, truth)
	rmseSlam, rmseOdom := 0.0, 0.0
	for i := range truth {
		rmseSlam += sq(out[i], truth[i])
		rmseOdom += sq(odomAligned[i], truth[i])
	}
	n := float64(len(truth))
	rmseSlam, rmseOdom = math.Sqrt(rmseSlam/n), math.Sqrt(rmseOdom/n)

	writePath(filepath.Join(*outDir, "traj_slam.csv"), out)
	writePath(filepath.Join(*outDir, "traj_odom.csv"), odomAligned)
	writePath(filepath.Join(*outDir, "traj_truth.csv"), truth)
	if err := p.Map().WritePGM(filepath.Join(*outDir, "map.pgm")); err != nil {
		log.Fatal(err)
	}

	kfs, nodes, edges, active := p.Bounds()
	var accepted, soft, rejected int
	for _, e := range p.LoopEdges() {
		switch e.Status {
		case "accepted":
			accepted++
		case "soft_weighted":
			soft++
		case "rejected":
			rejected++
		}
	}
	fmt.Printf("\nframes=%d keyframes=%d nodes=%d edges=%d(active=%d)\n",
		len(p.Records()), kfs, nodes, edges, active)
	fmt.Printf("loop edges: accepted=%d soft=%d rejected=%d\n", accepted, soft, rejected)
	fmt.Printf("RMSE  odom=%.3f m   slam=%.3f m   improvement=%.1f%%\n",
		rmseOdom, rmseSlam, 100*(1-rmseSlam/rmseOdom))
	fmt.Printf("outputs written to %s/\n", *outDir)
}

func normalize(in []geom.Pose) []geom.Pose {
	out := make([]geom.Pose, len(in))
	inv := geom.Inverse(in[0])
	for i, q := range in {
		out[i] = geom.Compose(q, inv)
	}
	return out
}

func sq(a, b geom.Pose) float64 {
	dx, dy := a.X-b.X, a.Y-b.Y
	return dx*dx + dy*dy
}

func align(path, truth []geom.Pose) []geom.Pose {
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
		ca, sa := math.Cos(a), math.Sin(a)
		e := 0.0
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

func writePath(path string, traj []geom.Pose) {
	var b []byte
	for i, q := range traj {
		b = append(b, fmt.Appendf(nil, "%d,%.4f,%.4f,%.4f\n", i, q.X, q.Y, q.Theta)...)
	}
	if err := os.WriteFile(path, b, 0o644); err != nil {
		log.Fatal(err)
	}
}
