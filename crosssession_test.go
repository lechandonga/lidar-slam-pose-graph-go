package slam_test

import (
	"math"
	"testing"

	root "github.com/lechandonga/lidar-slam-pose-graph-go"
	"github.com/lechandonga/lidar-slam-pose-graph-go/synth"
)

// crossWorld is a feature-rich site: an outer rectangle plus interior
// boxes give a start pose inside the mapped area a unique alignment while
// wrong places stay distinguishably worse.
func crossWorld() synth.World {
	return synth.RectWorld(14, 10, []struct{ X, Y, Ww, Hh float64 }{
		{-3.0, -3.0, 1.6, 0.4},
		{-3.5, -2.4, 0.4, 1.6},
		{-2.2, -3.5, 0.5, 0.5},
		{0, 0, 2, 2},
		{-3, 2.5, 1.5, 1},
		{3.2, 3.0, 1.2, 0.8},
		{2.5, -2.5, 0.8, 0.8},
	})
}

type sessionData struct {
	frames []root.Frame
	truth  []root.Pose
}

// makeSession builds one independently-clocked session. Its Generator starts
// odometry at zero and frame IDs/times at zero, exactly as a rebooted robot
// would produce after being moved to a new location.
func makeSession(t *testing.T, w synth.World, path []synth.Pose, seed int64) sessionData {
	t.Helper()
	gen := synth.New(w, synth.Options{
		Path: path, DT: 0.1, Nbeams: 360, Fov: 2 * math.Pi, MaxR: 20,
		BeamNoise: 0.03, BearingJit: 0.004,
		OdomDrift: 0.02, OdomBias: 0.003, Seed: seed,
	})
	var d sessionData
	for i := 0; i < gen.Len(); i++ {
		st := gen.At(i)
		pts := make([]root.Point, len(st.Points))
		for j, q := range st.Points {
			pts[j] = root.Point{X: q.X, Y: q.Y}
		}
		d.frames = append(d.frames, root.Frame{
			Scan: root.Scan{ID: st.ID, Time: st.Time, Points: pts},
			Odom: root.Odom{
				ID: st.ID, Time: st.Time,
				Pose:  root.Pose{X: st.Odom.X, Y: st.Odom.Y, Theta: st.Odom.Theta},
				Valid: st.OdomOK,
			},
			HasOdom: true,
		})
		d.truth = append(d.truth, root.Pose{X: st.Pose.X, Y: st.Pose.Y, Theta: st.Pose.Theta})
	}
	return d
}

func consumeFrames(p *root.Pipeline, frames []root.Frame) {
	for _, fr := range frames {
		p.ProcessFrame(fr)
	}
}

// overlapSessions: session1 covers loop[0:cut1]; session2 starts at
// loop[startI] (inside session-1 coverage) and continues to the end.
func overlapSessions(t *testing.T, startFrac, cutFrac float64) (sessionData, sessionData, []synth.Pose) {
	w := crossWorld()
	loop := synth.MakeLoopPath(4.0, 24, 6)
	n := len(loop)
	cut1 := int(float64(n) * cutFrac)
	startI := int(float64(n) * startFrac)
	s1 := makeSession(t, w, loop[:cut1], 101)
	// Session 2 walks a continuous tail of the same loop, beginning inside
	// session-1 coverage; its trajectory is a contiguous loop subsequence.
	path2 := loop[startI:]
	s2 := makeSession(t, w, path2, 202)
	return s1, s2, loop
}

// mapAgreement compares two snapshots over common world bounds.
func mapAgreement(a, b *root.MapSnapshot) (same float64, occOnlyA, occOnlyB, compared int) {
	x0 := math.Max(a.OriginX, b.OriginX)
	y0 := math.Max(a.OriginY, b.OriginY)
	x1 := math.Min(a.OriginX+float64(a.W)*a.Resolution,
		b.OriginX+float64(b.W)*b.Resolution)
	y1 := math.Min(a.OriginY+float64(a.H)*a.Resolution,
		b.OriginY+float64(b.H)*b.Resolution)
	r := a.Resolution
	at := func(x, y float64, m *root.MapSnapshot) int8 {
		cx := int(math.Floor((x - m.OriginX) / r))
		cy := int(math.Floor((y - m.OriginY) / r))
		if cx < 0 || cy < 0 || cx >= m.W || cy >= m.H {
			return root.CellUnknown
		}
		return m.Cells[cy*m.W+cx]
	}
	sameCnt := 0
	for y := y0; y < y1; y += r {
		for x := x0; x < x1; x += r {
			ca, cb := at(x, y, a), at(x, y, b)
			if ca == root.CellUnknown && cb == root.CellUnknown {
				continue
			}
			compared++
			if ca == cb {
				sameCnt++
			} else if ca == root.CellOccupied {
				occOnlyA++
			} else if cb == root.CellOccupied {
				occOnlyB++
			}
		}
	}
	if compared > 0 {
		same = float64(sameCnt) / float64(compared)
	}
	return
}

// alignAndRMSE compares estimated poses against truth with best-fit initial
// SE2 alignment (both anchor their own origin), matched 1:1 in slice order.
func alignAndRMSE(est, truth []root.Pose) float64 {
	if len(est) == 0 || len(truth) == 0 {
		return 0
	}
	m := len(est)
	if len(truth) < m {
		m = len(truth)
	}
	return rmseAligned(est, truth, est[0])
}

// rmseAligned aligns "est" into truth coordinates given that the estimated
// pose estAnchor corresponds to truth[0]. Used for cross-session sessions,
// whose first estimated pose is the localization fix (not the world origin).
func rmseAligned(est, truth []root.Pose, estAnchor root.Pose) float64 {
	m := len(est)
	if len(truth) < m {
		m = len(truth)
	}
	c := math.Cos(truth[0].Theta - estAnchor.Theta)
	s := math.Sin(truth[0].Theta - estAnchor.Theta)
	sum := 0.0
	for i := 0; i < m; i++ {
		lx := est[i].X - estAnchor.X
		ly := est[i].Y - estAnchor.Y
		ax := truth[0].X + c*lx - s*ly
		ay := truth[0].Y + s*lx + c*ly
		sum += (ax-truth[i].X)*(ax-truth[i].X) + (ay-truth[i].Y)*(ay-truth[i].Y)
	}
	return math.Sqrt(sum / float64(m))
}

// rmseRigid fits the best rigid SE2 transform mapping a->b (centroid plus
// best-heading search) and returns RMSE. Compares two independently
// anchored trajectories describing the same physical path.
func rmseRigid(a, b []root.Pose) float64 {
	m := len(a)
	if len(b) < m {
		m = len(b)
	}
	if m == 0 {
		return 0
	}
	var ax, ay, bx, by float64
	for i := 0; i < m; i++ {
		ax += a[i].X
		ay += a[i].Y
		bx += b[i].X
		by += b[i].Y
	}
	ax /= float64(m)
	ay /= float64(m)
	bx /= float64(m)
	by /= float64(m)
	best := math.MaxFloat64
	for k := 0; k <= 3600; k++ {
		th := -math.Pi + float64(k)*2*math.Pi/3600
		c, s := math.Cos(th), math.Sin(th)
		sum := 0.0
		for i := 0; i < m; i++ {
			dx, dy := a[i].X-ax, a[i].Y-ay
			ux, uy := bx+c*dx-s*dy, by+s*dx+c*dy
			sum += (b[i].X-ux)*(b[i].X-ux) + (b[i].Y-uy)*(b[i].Y-uy)
		}
		if rms := math.Sqrt(sum / float64(m)); rms < best {
			best = rms
		}
	}
	return best
}

// occupiedAgreementRegion is occupiedAgreement restricted to a world box.
func occupiedAgreementRegion(base, split *root.MapSnapshot,
	minX, minY, maxX, maxY float64, tolCells int) (precision, recall, maxGap float64) {
	inBox := func(m *root.MapSnapshot) []root.Point {
		var out []root.Point
		for cy := 0; cy < m.H; cy++ {
			for cx := 0; cx < m.W; cx++ {
				if m.Cells[cy*m.W+cx] != root.CellOccupied {
					continue
				}
				p := root.Point{
					X: m.OriginX + (float64(cx)+0.5)*m.Resolution,
					Y: m.OriginY + (float64(cy)+0.5)*m.Resolution,
				}
				if p.X < minX || p.X > maxX || p.Y < minY || p.Y > maxY {
					continue
				}
				out = append(out, p)
			}
		}
		return out
	}
	bo, so := inBox(base), inBox(split)
	if len(so) == 0 || len(bo) == 0 {
		return 0, 0, 0
	}
	tol := float64(tolCells) * base.Resolution
	nearest := func(p root.Point, set []root.Point) (float64, bool) {
		best := math.MaxFloat64
		for _, q := range set {
			if d := math.Hypot(p.X-q.X, p.Y-q.Y); d < best {
				best = d
			}
			if best <= tol {
				return best, true
			}
		}
		return best, false
	}
	hitP := 0
	for _, p := range so {
		if d, ok := nearest(p, bo); ok {
			hitP++
		} else if d-tol > maxGap {
			maxGap = d - tol
		}
	}
	precision = float64(hitP) / float64(len(so))
	hitR := 0
	for _, p := range bo {
		if _, ok := nearest(p, so); ok {
			hitR++
		}
	}
	recall = float64(hitR) / float64(len(bo))
	return
}
