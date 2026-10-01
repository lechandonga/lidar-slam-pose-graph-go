package slam

import "math"

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
