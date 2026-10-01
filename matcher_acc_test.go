package slam

import (
	"math"
	"testing"
)

// TestMatchKnownShift puts scan points exactly on known walls and checks the
// matcher recovers a known small pose shift.
func TestMatchKnownShift(t *testing.T) {
	cfg := DefaultConfig()
	// World points: walls of a 6x4 room.
	var world []Point
	for x := -3.0; x <= 3.0; x += 0.05 {
		world = append(world, Point{x, -2}, Point{x, 2})
	}
	for y := -2.0; y <= 2.0; y += 0.05 {
		world = append(world, Point{-3, y}, Point{3, y})
	}
	// Simulated scan from pose (0.1,0.05,0.02) via ray casting on those
	// walls. We build the scan by intersecting beams with the rectangle.
	pose := Pose{X: 0.1, Y: 0.05, Theta: 0.02}
	var scan []Point
	for i := 0; i < 360; i++ {
		a := 2 * math.Pi * float64(i) / 360
		r := rayToRect(pose, a, 3, 2)
		if r <= 0 {
			continue
		}
		// point in sensor frame
		scan = append(scan, Point{X: r * math.Cos(a), Y: r * math.Sin(a)})
	}
	pred := Pose{} // predict origin; matcher should find true pose
	mr := matchScan(cfg, world, scan, pred)
	if !mr.CoarseOK {
		t.Fatalf("coarse match failed")
	}
	d := math.Hypot(mr.Pose.X-pose.X, mr.Pose.Y-pose.Y)
	if d > 0.05 || math.Abs(normAngle(mr.Pose.Theta-pose.Theta)) > 0.03 {
		t.Fatalf("match pose %v != truth %v (dist %v)", mr.Pose, pose, d)
	}
	t.Logf("matched %v score %.3f eig %.4f", mr.Pose, mr.Score, mr.EigRatio)
}

func rayToRect(p Pose, a, hx, hy float64) float64 {
	// ray in world frame; rectangle walls at x=+-hx, y=+-hy
	aw := p.Theta + a
	dx, dy := math.Cos(aw), math.Sin(aw)
	best := 20.0
	try := func(wx, wy float64) {
		t := 0.0
		if math.Abs(dx) > 1e-9 {
			t = (wx - p.X) / dx
			if t > 0 {
				y := p.Y + t*dy
				if math.Abs(y-wy) < 1e-9 && t < best {
					best = t
				}
			}
		}
		if math.Abs(dy) > 1e-9 {
			t = (wy - p.Y) / dy
			if t > 0 {
				x := p.X + t*dx
				if math.Abs(x-wx) < 1e-9 && t < best {
					best = t
				}
			}
		}
	}
	_ = try
	candidates := [][2]float64{
		{-hx, -hy}, {-hx, hy}, {hx, -hy}, {hx, hy},
	}
	for _, c := range candidates {
		wallX, wallY := c[0], c[1]
		if math.Abs(dx) > 1e-9 {
			tt := (wallX - p.X) / dx
			if tt > 1e-6 {
				yy := p.Y + tt*dy
				if yy >= -hy-1e-6 && yy <= hy+1e-6 && tt < best {
					best = tt
				}
			}
		}
		if math.Abs(dy) > 1e-9 {
			tt := (wallY - p.Y) / dy
			if tt > 1e-6 {
				xx := p.X + tt*dx
				if xx >= -hx-1e-6 && xx <= hx+1e-6 && tt < best {
					best = tt
				}
			}
		}
	}
	if best >= 20 {
		return -1
	}
	return best
}
