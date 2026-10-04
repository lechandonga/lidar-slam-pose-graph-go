// Package synth generates deterministic local 2-D SLAM datasets: a 2-D
// polygon world, a ground-truth trajectory, noisy planar laser scans and a
// drifting wheel odometry stream. Nothing here performs I/O or networking.
package synth

import (
	"math"
	"math/rand"
)

type Point struct {
	X, Y float64
}

type Pose struct {
	X, Y, Theta float64
}

// Segment is one world wall in metres.
type Segment struct {
	A, B Point
}

// Options configures a synthetic run.
type Options struct {
	// Waypoints are the ground-truth path; consecutive points are connected
	// by straight segments traversed at LinearSpeed.
	Waypoints []Point
	// Path, when non-nil, overrides Waypoints and supplies explicit
	// ground-truth poses (used for turn/loop scenarios).
	Path []Pose
	// ScanPath, when non-nil, supplies the poses at which laser scans are
	// ray-cast, while Path keeps supplying the poses wheel odometry is
	// integrated along. The two are the same length. This is required for a
	// session re-expressed in its own local frame: its odometry starts at
	// the local origin (Path), but its scans observe the SAME physical
	// world, so they must be cast at the world poses (ScanPath). When nil,
	// scans are cast at Path (the usual single-session case).
	ScanPath []Pose
	// Heading: 0 means align heading with travel direction; otherwise the
	// final heading is held (used for the turn scenario where heading
	// explicitly rotates in place — encode with duplicate waypoints).
	LinearSpeed  float64 // m/s
	AngularSpeed float64 // rad/s for in-place turns between collocated points
	DT           float64 // seconds per frame

	Nbeams int
	Fov    float64 // radians, centered on heading
	MaxR   float64

	// Noise (deterministic via Seed).
	BeamNoise  float64 // metres, gaussian sigma added to range
	BearingJit float64 // radians
	OdomDrift  float64 // fractional scale error per metre
	OdomBias   float64 // rad/s heading-rate bias

	Seed int64

	// BadFrames injects frames by index with the given fault:
	// "few_points", "nan", "burst".
	BadFrames map[int]Fault
	// BadOdoms marks odom samples invalid (missing/garbled).
	BadOdoms map[int]bool
	// Restarts inserts an odometry-frame reset after processing the frame.
	Restarts map[int]bool
}

type Fault struct {
	Kind string
}

// World is a fixed set of walls.
type World struct {
	Walls []Segment
}

// RectWorld builds a rectangular room with optional interior boxes.
func RectWorld(w, h float64, boxes []struct{ X, Y, Ww, Hh float64 }) World {
	walls := []Segment{
		{Point{-w / 2, -h / 2}, Point{w / 2, -h / 2}},
		{Point{w / 2, -h / 2}, Point{w / 2, h / 2}},
		{Point{w / 2, h / 2}, Point{-w / 2, h / 2}},
		{Point{-w / 2, h / 2}, Point{-w / 2, -h / 2}},
	}
	for _, b := range boxes {
		x0, y0 := b.X-b.Ww/2, b.Y-b.Hh/2
		x1, y1 := b.X+b.Ww/2, b.Y+b.Hh/2
		walls = append(walls,
			Segment{Point{x0, y0}, Point{x1, y0}},
			Segment{Point{x1, y0}, Point{x1, y1}},
			Segment{Point{x1, y1}, Point{x0, y1}},
			Segment{Point{x0, y1}, Point{x0, y0}},
		)
	}
	return World{Walls: walls}
}

type step struct {
	pose    Pose
	scan    []Point
	odom    Pose
	valid   bool
	restart bool
	id      int64
	t       float64
}

// Generator is a deterministic DataSource-compatible sequence.
type Generator struct {
	world World
	opt   Options
	rng   *rand.Rand
	steps []step
	idx   int
}

type generatedPath struct {
	poses []Pose
	scan  []Pose
	times []float64
}

// New builds the full sequence up front so GroundTruth is available.
func New(world World, opt Options) *Generator {
	if opt.DT <= 0 {
		opt.DT = 0.1
	}
	if opt.LinearSpeed <= 0 {
		opt.LinearSpeed = 0.5
	}
	if opt.Nbeams <= 0 {
		opt.Nbeams = 360
	}
	if opt.Fov <= 0 {
		opt.Fov = 2 * math.Pi
	}
	if opt.MaxR <= 0 {
		opt.MaxR = 20
	}
	g := &Generator{world: world, opt: opt, rng: rand.New(rand.NewSource(opt.Seed))}
	path := g.buildPath()
	g.buildFrames(path)
	return g
}

func (g *Generator) buildPath() generatedPath {
	var poses []Pose
	var times []float64
	if len(g.opt.Path) > 0 {
		for i, ps := range g.opt.Path {
			poses = append(poses, ps)
			times = append(times, float64(i)*g.opt.DT)
		}
		scan := poses
		if len(g.opt.ScanPath) == len(g.opt.Path) {
			scan = g.opt.ScanPath
		}
		return generatedPath{poses: poses, scan: scan, times: times}
	}
	t := 0.0
	for i := 0; i+1 < len(g.opt.Waypoints); i++ {
		a, b := g.opt.Waypoints[i], g.opt.Waypoints[i+1]
		dx, dy := b.X-a.X, b.Y-a.Y
		dist := math.Hypot(dx, dy)
		heading := math.Atan2(dy, dx)
		if dist < 1e-6 {
			// In-place turn: hold position for a short interval so callers
			// express a turn by repeating a waypoint is not needed; instead
			// we simply skip zero-length legs.
			continue
		}
		n := int(math.Ceil(dist / (g.opt.LinearSpeed * g.opt.DT)))
		for k := 0; k <= n; k++ {
			s := float64(k) / float64(n)
			if len(poses) > 0 && k == 0 {
				continue
			}
			poses = append(poses, Pose{
				X: a.X + s*dx, Y: a.Y + s*dy, Theta: heading,
			})
			times = append(times, t)
		}
	}
	return generatedPath{poses: poses, scan: poses, times: times}
}

func (g *Generator) buildFrames(path generatedPath) {
	odom := Pose{}
	prev := Pose{}
	for i, gt := range path.poses {
		if i == 0 {
			prev = gt
		}
		// Integrate noisy odometry relative motion from ground truth.
		dx, dy := gt.X-prev.X, gt.Y-prev.Y
		dth := normAngle(gt.Theta - prev.Theta)
		c, s := math.Cos(prev.Theta), math.Sin(prev.Theta)
		localX := c*dx + s*dy
		localY := -s*dx + c*dy
		moved := math.Hypot(dx, dy)
		scaledX := localX * (1 + g.opt.OdomDrift)
		biasedTh := dth + g.opt.OdomBias*g.opt.DT
		odom = compose(odom, Pose{X: scaledX, Y: localY, Theta: biasedTh})
		prev = gt

		scanAt := gt
		if i < len(path.scan) {
			scanAt = path.scan[i]
		}
		pts := g.castScan(scanAt)
		// Apply range/bearing noise.
		for j := range pts {
			if g.opt.BeamNoise > 0 {
				pts[j] = perturbPoint(g.rng, pts[j], g.opt.BeamNoise, g.opt.BearingJit)
			}
		}
		if f, ok := g.opt.BadFrames[i]; ok {
			pts = applyFault(g.rng, pts, f.Kind, g.opt.MaxR)
		}
		valid := true
		if g.opt.BadOdoms[i] {
			valid = false
		}
		restart := g.opt.Restarts[i]
		g.steps = append(g.steps, step{
			pose: gt, scan: pts, odom: odom, valid: valid,
			restart: restart, id: int64(i), t: float64(i) * g.opt.DT,
		})
		if restart {
			odom = Pose{} // odom frame origin restarts
		}
		_ = moved
	}
}

// castScan ray-casts nBeams around the pose.
func (g *Generator) castScan(p Pose) []Point {
	pts := make([]Point, 0, g.opt.Nbeams)
	for i := 0; i < g.opt.Nbeams; i++ {
		a := -g.opt.Fov/2 + g.opt.Fov*float64(i)/float64(g.opt.Nbeams-1)
		ang := p.Theta + a
		r := g.traceRay(p.X, p.Y, math.Cos(ang), math.Sin(ang))
		if r >= g.opt.MaxR {
			continue
		}
		pts = append(pts, Point{X: r * math.Cos(a), Y: r * math.Sin(a)})
	}
	return pts
}

func (g *Generator) traceRay(ox, oy, dx, dy float64) float64 {
	best := g.opt.MaxR
	for _, w := range g.world.Walls {
		r, ok := raySegment(ox, oy, dx, dy, w.A, w.B)
		if ok && r < best {
			best = r
		}
	}
	return best
}

func raySegment(ox, oy, dx, dy float64, a, b Point) (float64, bool) {
	ex, ey := b.X-a.X, b.Y-a.Y
	den := dx*ey - dy*ex
	if math.Abs(den) < 1e-12 {
		return 0, false
	}
	t := ((a.X-ox)*ey - (a.Y-oy)*ex) / den
	u := ((a.X-ox)*dy - (a.Y-oy)*dx) / den
	if t >= 0 && u >= 0 && u <= 1 {
		return t, true
	}
	return 0, false
}

func perturbPoint(rng *rand.Rand, p Point, rNoise, bJit float64) Point {
	r := math.Hypot(p.X, p.Y)
	a := math.Atan2(p.Y, p.X)
	r += rng.NormFloat64() * rNoise
	a += rng.NormFloat64() * bJit
	if r < 0 {
		r = 0
	}
	return Point{X: r * math.Cos(a), Y: r * math.Sin(a)}
}

func applyFault(rng *rand.Rand, pts []Point, kind string, maxR float64) []Point {
	switch kind {
	case "few_points":
		if len(pts) <= 5 {
			return pts
		}
		return pts[:5]
	case "nan":
		out := append([]Point(nil), pts...)
		for i := range out {
			if i%3 == 0 {
				out[i].X = math.NaN()
			}
		}
		return out
	case "burst":
		// Collapse most returns onto one bearing with random long ranges.
		out := make([]Point, len(pts))
		a := rng.Float64() * 2 * math.Pi
		for i := range out {
			if i%20 == 0 {
				out[i] = pts[i]
			} else {
				r := maxR * (0.5 + 0.5*rng.Float64())
				out[i] = Point{X: r * math.Cos(a), Y: r * math.Sin(a)}
			}
		}
		return out
	case "huge":
		out := append([]Point(nil), pts...)
		for i := range out {
			if i%4 == 0 {
				out[i].X = maxR * 5
			}
		}
		return out
	}
	return pts
}

func normAngle(a float64) float64 {
	for a > math.Pi {
		a -= 2 * math.Pi
	}
	for a < -math.Pi {
		a += 2 * math.Pi
	}
	return a
}

func compose(a, b Pose) Pose {
	c, s := math.Cos(a.Theta), math.Sin(a.Theta)
	return Pose{
		X:     a.X + c*b.X - s*b.Y,
		Y:     a.Y + s*b.X + c*b.Y,
		Theta: normAngle(a.Theta + b.Theta),
	}
}

// GroundTruth returns true poses in the same order as frames.
func (g *Generator) GroundTruthPoses() []Pose {
	out := make([]Pose, len(g.steps))
	for i, s := range g.steps {
		out[i] = s.pose
	}
	return out
}

// Len reports the number of frames.
func (g *Generator) Len() int { return len(g.steps) }

// Step is one converted frame in slam package types.
type Step struct {
	ID      int64
	Time    float64
	Pose    Pose
	Points  []Point
	Odom    Pose
	OdomOK  bool
	Restart bool
}

// At returns frame i.
func (g *Generator) At(i int) Step {
	s := g.steps[i]
	return Step{
		ID: s.id, Time: s.t, Pose: s.pose,
		Points: append([]Point(nil), s.scan...),
		Odom:   s.odom, OdomOK: s.valid, Restart: s.restart,
	}
}
