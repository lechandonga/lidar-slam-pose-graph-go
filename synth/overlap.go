package synth

import "math"

// MakeOverlapPath builds a non-self-closing path for two-session tests:
// walk east along a corridor, turn north, then continue. Splitting it with
// an overlap gives two sessions that revisit a shared corner but never
// return to the start, so the area has asymmetric structure and a unique
// localization. leg is each straight segment length in metres.
func MakeOverlapPath(leg float64, perLeg, turnFrames int) []Pose {
	var pts []Pose
	for i := 0; i < perLeg; i++ {
		s := float64(i) / float64(perLeg-1)
		pts = append(pts, Pose{X: s * leg, Theta: 0})
	}
	start := pts[len(pts)-1]
	for i := 1; i <= turnFrames; i++ {
		th := math.Pi / 2 * float64(i) / float64(turnFrames)
		pts = append(pts, Pose{X: start.X, Y: start.Y, Theta: th})
	}
	for i := 1; i <= perLeg; i++ {
		s := float64(i) / float64(perLeg)
		pts = append(pts, Pose{X: start.X, Y: start.Y + s*leg, Theta: math.Pi / 2})
	}
	return pts
}
