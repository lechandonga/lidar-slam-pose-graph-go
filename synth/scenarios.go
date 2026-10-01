package synth

import "math"

// MakeRectPath builds explicit poses: straight out, a 90-degree in-place
// turn (inserted heading-only intermediate poses), then another leg.
// The 2-D path is parameterized by total length and number of frames.
func MakeStraightPath(length float64, n int) []Pose {
	pts := make([]Pose, n)
	for i := 0; i < n; i++ {
		s := float64(i) / float64(n-1)
		pts[i] = Pose{X: s * length, Y: 0, Theta: 0}
	}
	return pts
}

// MakeTurnPath goes straight, turns 90 deg in place, then goes straight.
func MakeTurnPath(leg float64, perLeg int, turnFrames int) []Pose {
	var pts []Pose
	for i := 0; i < perLeg; i++ {
		s := float64(i) / float64(perLeg)
		pts = append(pts, Pose{X: s * leg, Y: 0, Theta: 0})
	}
	start := pts[len(pts)-1]
	for i := 1; i <= turnFrames; i++ {
		th := math.Pi / 2 * float64(i) / float64(turnFrames)
		pts = append(pts, Pose{X: start.X, Y: start.Y, Theta: th})
	}
	for i := 1; i <= perLeg; i++ {
		s := float64(i) / float64(perLeg)
		pts = append(pts, Pose{
			X: start.X, Y: start.Y + s*leg, Theta: math.Pi / 2,
		})
	}
	return pts
}

// MakeLoopPath walks a rounded rectangle and returns to the start so loop
// closure can engage. side is the half-side length.
func MakeLoopPath(side float64, perSide int, turnFrames int) []Pose {
	type seg struct {
		dx, dy float64
		th     float64
	}
	segs := []seg{
		{2 * side, 0, 0},
		{0, 2 * side, math.Pi / 2},
		{-2 * side, 0, math.Pi},
		{0, -2 * side, 3 * math.Pi / 2},
	}
	cur := Pose{X: -side, Y: -side, Theta: 0}
	pts := []Pose{cur}
	turn := func(target float64) {
		dth := normAngle(target - cur.Theta)
		for i := 1; i <= turnFrames; i++ {
			pts = append(pts, Pose{
				X: cur.X, Y: cur.Y,
				Theta: cur.Theta + dth*float64(i)/float64(turnFrames),
			})
		}
		cur.Theta = target
	}
	for _, sg := range segs {
		turn(sg.th)
		for i := 1; i <= perSide; i++ {
			s := float64(i) / float64(perSide)
			pts = append(pts, Pose{
				X: cur.X + s*sg.dx, Y: cur.Y + s*sg.dy, Theta: sg.th,
			})
		}
		cur.X += sg.dx
		cur.Y += sg.dy
	}
	return pts
}

// CorridorWorld builds two long parallel walls (degenerate geometry test).
func CorridorWorld(length, halfWidth float64) World {
	return World{Walls: []Segment{
		{Point{-length / 2, -halfWidth}, Point{length / 2, -halfWidth}},
		{Point{-length / 2, halfWidth}, Point{length / 2, halfWidth}},
	}}
}
