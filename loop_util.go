package slam

import "math"

// pointsWithin returns reference points within radius of center.
func pointsWithin(ref []Point, center Pose, radius float64) []Point {
	out := make([]Point, 0, 256)
	for _, q := range ref {
		if math.Hypot(q.X-center.X, q.Y-center.Y) <= radius {
			out = append(out, q)
		}
	}
	return out
}

// relToWorld converts a scan-matched relative transform rel (expressing new
// scan in old scan's local frame) into the world-frame relative edge
// measurement Between(oldWorld, newWorld) expected by the graph optimizer.
// The two share theta; translation is rotated by old's world heading.
func relToWorld(old Pose, rel Pose) Pose {
	c, s := math.Cos(old.Theta), math.Sin(old.Theta)
	return Pose{
		X:     c*rel.X - s*rel.Y,
		Y:     s*rel.X + c*rel.Y,
		Theta: rel.Theta,
	}
}
