// Package sim generates deterministic synthetic 2D worlds, ground-truth
// trajectories, laser scans and noisy wheel odometry. Everything is local:
// no external services, no time/randomness beyond the seeded RNG.
package sim

import (
	"math"
	"math/rand"

	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/geom"
)

// World is a set of line-segment walls in meters.
type Segment struct{ A, B geom.Point }

type World struct {
	Segments []Segment
}

// RectWorld builds four outer walls plus a couple of interior walls so the
// environment has structure in every direction.
func RectWorld(half float64) World {
	w := World{}
	corners := []geom.Point{
		{X: -half, Y: -half}, {X: half, Y: -half}, {X: half, Y: half}, {X: -half, Y: half},
	}
	for i := 0; i < 4; i++ {
		w.Segments = append(w.Segments, Segment{A: corners[i], B: corners[(i+1)%4]})
	}
	// interior obstacles producing varied geometry (no long corridor only)
	w.Segments = append(w.Segments,
		Segment{A: geom.Point{X: -3, Y: -3}, B: geom.Point{X: -3, Y: -0.5}},
		Segment{A: geom.Point{X: -3, Y: -0.5}, B: geom.Point{X: -1.5, Y: -0.5}},
		Segment{A: geom.Point{X: 3.5, Y: 1}, B: geom.Point{X: 3.5, Y: 4}},
		Segment{A: geom.Point{X: 1, Y: 3.5}, B: geom.Point{X: 4, Y: 3.5}},
	)
	return w
}

// Raycast intersects one ray with all world segments; returns the nearest
// hit distance in (0,maxR] and whether a hit exists.
func (w World) Raycast(origin geom.Point, ang, maxR float64) (float64, bool) {
	dx, dy := math.Cos(ang), math.Sin(ang)
	best := maxR
	hit := false
	for _, s := range w.Segments {
		ex := s.B.X - s.A.X
		ey := s.B.Y - s.A.Y
		den := dx*ey - dy*ex
		if math.Abs(den) < 1e-12 {
			continue
		}
		tx := s.A.X - origin.X
		ty := s.A.Y - origin.Y
		t := (tx*ey - ty*ex) / den
		u := (tx*dy - ty*dx) / den
		if t > 1e-6 && t <= maxR && u >= 0 && u <= 1 && t < best {
			best = t
			hit = true
		}
	}
	return best, hit
}

// TrajPoint is a ground-truth robot pose with an associated time step.
type TrajPoint struct {
	Pose geom.Pose
}

// RectTrajectory creates a closed-loop square-ish trajectory with smooth
// corners: the robot revisits its start pose, enabling real loop closure.
func RectTrajectory(side float64, stepsPerSide int) []geom.Pose {
	var traj []geom.Pose
	s := side / 2
	centers := []geom.Point{{X: -s, Y: -s}, {X: s, Y: -s}, {X: s, Y: s}, {X: -s, Y: s}}
	headings := []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2}
	r := 1.5 // corner radius
	const cornerSteps = 12
	for sideIdx := 0; sideIdx < 4; sideIdx++ {
		c0 := centers[sideIdx]
		c1 := centers[(sideIdx+1)%4]
		h0 := headings[sideIdx]
		// straight portion: from c0 + heading*r to c1 - nextHeading*r
		h1 := headings[(sideIdx+1)%4]
		st := geom.Point{X: c0.X + math.Cos(h0)*r, Y: c0.Y + math.Sin(h0)*r}
		en := geom.Point{X: c1.X - math.Cos(h1)*r, Y: c1.Y - math.Sin(h1)*r}
		for i := 0; i < stepsPerSide; i++ {
			t := float64(i) / float64(stepsPerSide)
			traj = append(traj, geom.Pose{
				X: st.X + (en.X-st.X)*t, Y: st.Y + (en.Y-st.Y)*t, Theta: h0,
			})
		}
		// circular corner around c1 from h0 to h1
		cx := c1.X - math.Cos(h0)*r
		cy := c1.Y - math.Sin(h0)*r
		for i := 1; i <= cornerSteps; i++ {
			t := float64(i) / float64(cornerSteps)
			a := h0 + geom.WrapAngle(h1-h0)*t
			traj = append(traj, geom.Pose{
				X: cx + math.Cos(a)*r, Y: cy + math.Sin(a)*r, Theta: geom.WrapAngle(a),
			})
		}
	}
	return traj
}

// ScanParams describes the simulated laser.
type ScanParams struct {
	Beams    int
	AngleMin float64
	AngleMax float64
	RangeMax float64
	RangeMin float64
	Noise    float64 // gaussian sigma on range, meters
	MissProb float64 // probability a valid return is dropped
}

// DefaultScanParams is a 180-degree, 360 beam lidar.
func DefaultScanParams() ScanParams {
	return ScanParams{
		Beams: 540, AngleMin: -math.Pi, AngleMax: math.Pi,
		RangeMax: 20, RangeMin: 0.12, Noise: 0.02, MissProb: 0.01,
	}
}

// MakeScan raycasts from pose with seeded noise and dropped returns.
func MakeScan(w World, pose geom.Pose, p ScanParams, rng *rand.Rand) []float64 {
	out := make([]float64, p.Beams)
	inc := (p.AngleMax - p.AngleMin) / float64(p.Beams-1)
	for i := 0; i < p.Beams; i++ {
		a := pose.Theta + p.AngleMin + float64(i)*inc
		r, hit := w.Raycast(geom.Point{X: pose.X, Y: pose.Y}, a, p.RangeMax)
		switch {
		case !hit:
			out[i] = p.RangeMax + 1
		case rng.Float64() < p.MissProb:
			out[i] = p.RangeMax + 1
		default:
			n := rng.NormFloat64() * p.Noise
			v := r + n
			if v < p.RangeMin {
				v = p.RangeMin
			}
			out[i] = v
		}
	}
	return out
}

// OdomParams controls wheel odometry drift.
type OdomParams struct {
	ScaleXY float64 // multiplicative scale error (e.g. 0.98)
	ScaleTh float64
	NoiseXY float64 // additive gaussian sigma per frame, meters
	NoiseTh float64
}

// DefaultOdomParams gives a few percent scale drift plus jitter.
func DefaultOdomParams() OdomParams {
	return OdomParams{ScaleXY: 0.92, ScaleTh: 0.96, NoiseXY: 0.012, NoiseTh: 0.008}
}

// MakeOdom integrates noisy relative steps along the ground truth path.
// The returned poses are in the odometry frame; drift accumulates honestly.
func MakeOdom(truth []geom.Pose, p OdomParams, rng *rand.Rand) []geom.Pose {
	out := make([]geom.Pose, len(truth))
	out[0] = geom.Pose{}
	for i := 1; i < len(truth); i++ {
		rel := geom.Between(truth[i], truth[i-1])
		rel.X *= p.ScaleXY
		rel.Y *= p.ScaleXY
		rel.Theta *= p.ScaleTh
		rel.X += rng.NormFloat64() * p.NoiseXY
		rel.Y += rng.NormFloat64() * p.NoiseXY
		rel.Theta = geom.WrapAngle(rel.Theta + rng.NormFloat64()*p.NoiseTh)
		out[i] = geom.Compose(rel, out[i-1])
	}
	return out
}
