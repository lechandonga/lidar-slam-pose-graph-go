package slam

import (
	"fmt"
	"math"
)

// Unit conventions (see docs/coordinates.md):
//   - positions x, y are in metres;
//   - angles theta are in radians, measured counter-clockwise from +x;
//   - SE2 pose composition follows p.Compose(q): apply q first, then p.

// Pose is world <- base: the base frame expressed in the world frame.
type Pose struct {
	X, Y, Theta float64
}

// Point is a point in the local (sensor) frame, metres.
type Point struct {
	X, Y float64
}

// Scan is one 2-D laser frame. Points are in the sensor frame, assumed
// coincident with the base frame (extrinsics can be applied in DataSource).
type Scan struct {
	ID     int64
	Time   float64 // seconds, monotonic source clock
	Points []Point
}

// Odom is one wheel-odometry sample expressed in the odometry frame.
type Odom struct {
	ID    int64
	Time  float64
	Pose  Pose
	Valid bool // false marks a dropped/garbled odom sample
}

// Frame bundles one synchronized scan/odom pair delivered by a DataSource.
type Frame struct {
	Scan    Scan
	Odom    Odom
	Restart bool
	HasOdom bool
}

// DataSource replays or generates deterministic local data.
type DataSource interface {
	Next() (Frame, bool)
	// GroundTruth returns the true trajectory for evaluation; synthetic
	// sources provide it, replay sources return nil.
	GroundTruth() []Pose
}

// Outcome of processing one frame.
type Outcome string

const (
	OutcomeAccepted Outcome = "accepted"
	OutcomeDegraded Outcome = "degraded"
	OutcomeRejected Outcome = "rejected"
)

// Distinct frame-level reason codes. Distinct failure causes get distinct
// codes so tests and logs can tell them apart.
const (
	ReasonOK              = "ok"
	ReasonTooFewPoints    = "too_few_points"
	ReasonNoOdom          = "odom_missing"
	ReasonDegenerate      = "scan_degenerate"
	ReasonMatchDivergence = "match_far_from_prediction"
	ReasonBadScan         = "bad_scan"
	ReasonRestart         = "odom_restart_reanchored"
	ReasonOdomInvalid     = "odom_invalid_sample"
)

// FrameDecision records what happened to a frame; it is what makes the
// pipeline explainable.
type FrameDecision struct {
	FrameID int64
	Time    float64
	Outcome Outcome
	Reason  string
	Detail  string
}

func (p Pose) String() string {
	return fmt.Sprintf("(%.4f %.4f %.4f)", p.X, p.Y, p.Theta)
}

// Inv returns the inverse pose.
func (p Pose) Inv() Pose {
	c, s := math.Cos(p.Theta), math.Sin(p.Theta)
	return Pose{X: -c*p.X - s*p.Y, Y: s*p.X - c*p.Y, Theta: normAngle(-p.Theta)}
}

// Compose returns p * q (apply q first, then p).
func (p Pose) Compose(q Pose) Pose {
	c, s := math.Cos(p.Theta), math.Sin(p.Theta)
	return Pose{
		X:     p.X + c*q.X - s*q.Y,
		Y:     p.Y + s*q.X + c*q.Y,
		Theta: normAngle(p.Theta + q.Theta),
	}
}

// Transform maps a local point into the world frame.
func (p Pose) Transform(pt Point) Point {
	c, s := math.Cos(p.Theta), math.Sin(p.Theta)
	return Point{X: p.X + c*pt.X - s*pt.Y, Y: p.Y + s*pt.X + c*pt.Y}
}

// Between returns the relative transform p^-1 * q.
func Between(p, q Pose) Pose {
	return p.Inv().Compose(q)
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

// NormAngle normalizes an angle to [-pi, pi].
func NormAngle(a float64) float64 { return normAngle(a) }

// PoseDist is the chord distance between two poses in metres.
func PoseDist(a, b Pose) float64 {
	return math.Hypot(a.X-b.X, a.Y-b.Y)
}
