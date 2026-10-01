package geom

import "math"

// Pose is a rigid 2D transform (SE(2)) expressed in meters and radians.
// X points forward/east, Y points left/north, Theta is the CCW yaw.
type Pose struct {
	X     float64
	Y     float64
	Theta float64
}

// Point is a Cartesian point in meters, frame-dependent.
type Point struct {
	X float64
	Y float64
}

const twoPi = 2.0 * math.Pi

// WrapAngle normalizes an angle to (-pi, pi].
func WrapAngle(a float64) float64 {
	for a > math.Pi {
		a -= twoPi
	}
	for a <= -math.Pi {
		a += twoPi
	}
	return a
}

// Compose returns b followed by a (both expressed in the same outer frame).
func Compose(a, b Pose) Pose {
	st, ct := math.Sincos(b.Theta)
	return Pose{
		X:     b.X + ct*a.X - st*a.Y,
		Y:     b.Y + st*a.X + ct*a.Y,
		Theta: WrapAngle(b.Theta + a.Theta),
	}
}

// Inverse returns the inverse rigid transform.
func Inverse(p Pose) Pose {
	st, ct := math.Sincos(p.Theta)
	return Pose{
		X:     -ct*p.X - st*p.Y,
		Y:     st*p.X - ct*p.Y,
		Theta: WrapAngle(-p.Theta),
	}
}

// Between returns the relative transform that maps frame b to frame a:
// a = Between(a,b) composed with b, i.e. a * b^-1.
func Between(a, b Pose) Pose {
	return Compose(a, Inverse(b))
}

// Transform applies a pose to a point.
func (p Pose) Transform(pt Point) Point {
	st, ct := math.Sincos(p.Theta)
	return Point{
		X: p.X + ct*pt.X - st*pt.Y,
		Y: p.Y + st*pt.X + ct*pt.Y,
	}
}

// Dist is the Euclidean distance between two points.
func Dist(a, b Point) float64 {
	dx := a.X - b.X
	dy := a.Y - b.Y
	return math.Hypot(dx, dy)
}

// PoseDist returns the translational distance between two poses.
func PoseDist(a, b Pose) float64 {
	return math.Hypot(a.X-b.X, a.Y-b.Y)
}

// Interpolate linearly interpolates translation and angle between two poses, t in [0,1].
func Interpolate(a, b Pose, t float64) Pose {
	return Pose{
		X:     a.X + (b.X-a.X)*t,
		Y:     a.Y + (b.Y-a.Y)*t,
		Theta: WrapAngle(a.Theta + WrapAngle(b.Theta-a.Theta)*t),
	}
}
