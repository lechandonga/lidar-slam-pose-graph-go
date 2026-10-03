package slam

import "math"

// Grid is a log-odds occupancy grid in the world frame. A zero-value Grid
// is not usable; construct with NewGrid. All exported reads are safe under
// the Pipeline mutex contract; Snapshot takes the lock internally.
type Grid struct {
	res     float64
	ox, oy  float64
	w, h    int
	logOdds []float64
	cfg     Config
}

// Cell values for querying.
const (
	CellUnknown  = 0
	CellFree     = -1
	CellOccupied = 1
)

func newGrid(cfg Config) *Grid {
	return &Grid{
		res:     cfg.MapResolution,
		cfg:     cfg,
		w:       0,
		h:       0,
		logOdds: nil,
	}
}

// MapSnapshot is an immutable, concurrency-safe copy used by queries while
// the SLAM thread keeps updating.
type MapSnapshot struct {
	Resolution float64
	OriginX    float64
	OriginY    float64
	W, H       int
	Cells      []int8 // -1 free, 0 unknown, 1 occupied
}

// Probability in [0,1] that cell (cx,cy) is occupied; 0.5 for out of bounds.
func (m *MapSnapshot) ProbAt(cx, cy int) float64 {
	if cx < 0 || cy < 0 || cx >= m.W || cy >= m.H {
		return 0.5
	}
	switch m.Cells[cy*m.W+cx] {
	case CellOccupied:
		return 0.9
	case CellFree:
		return 0.1
	default:
		return 0.5
	}
}

// IsOccupiedAtWorld answers a world-coordinate query.
func (m *MapSnapshot) IsOccupiedAtWorld(x, y float64) bool {
	cx := int(math.Floor((x - m.OriginX) / m.Resolution))
	cy := int(math.Floor((y - m.OriginY) / m.Resolution))
	if cx < 0 || cy < 0 || cx >= m.W || cy >= m.H {
		return false
	}
	return m.Cells[cy*m.W+cx] == CellOccupied
}

func (g *Grid) snapshot() *MapSnapshot {
	cells := make([]int8, g.w*g.h)
	for i, v := range g.logOdds {
		switch {
		case v > 0.4:
			cells[i] = CellOccupied
		case v < -0.4:
			cells[i] = CellFree
		default:
			cells[i] = CellUnknown
		}
	}
	return &MapSnapshot{
		Resolution: g.res,
		OriginX:    g.ox,
		OriginY:    g.oy,
		W:          g.w,
		H:          g.h,
		Cells:      cells,
	}
}

// ensureSize grows the grid to contain the given world bounds. Growth copies
// old cells preserving world alignment.
func (g *Grid) ensureSize(minX, minY, maxX, maxY float64) {
	pad := 1.0
	minX -= pad
	minY -= pad
	maxX += pad
	maxY += pad
	if g.w == 0 {
		// Allocate in block-aligned chunks so point-by-point updates do not
		// cause O(n) reallocations.
		const block = 50 // cells
		wc := int((maxX-minX)/g.res) + 2
		hc := int((maxY-minY)/g.res) + 2
		g.w = ((wc + block - 1) / block) * block
		g.h = ((hc + block - 1) / block) * block
		// Center the initial requested bounds inside the blocks.
		g.ox = minX - (float64(g.w)*g.res-(maxX-minX))/2
		g.oy = minY - (float64(g.h)*g.res-(maxY-minY))/2
		g.logOdds = make([]float64, g.w*g.h)
		return
	}
	nMinX, nMinY := g.ox, g.oy
	nMaxX := g.ox + float64(g.w)*g.res
	nMaxY := g.oy + float64(g.h)*g.res
	grow := false
	if minX < nMinX {
		nMinX = minX
		grow = true
	}
	if minY < nMinY {
		nMinY = minY
		grow = true
	}
	if maxX > nMaxX {
		nMaxX = maxX
		grow = true
	}
	if maxY > nMaxY {
		nMaxY = maxY
		grow = true
	}
	if !grow {
		return
	}
	const block = 50
	// Extend each side by a whole number of blocks so the allocation stays
	// bounded and old content keeps an EXACT integer-cell offset. Extending
	// only as far as needed (rather than recomputing a bounding origin)
	// avoids shifting the lattice and misplacing previously mapped walls.
	leftCells := 0
	if minX < g.ox {
		need := int((g.ox-minX)/g.res) + 1
		leftCells = ((need + block - 1) / block) * block
	}
	botCells := 0
	if minY < g.oy {
		need := int((g.oy-minY)/g.res) + 1
		botCells = ((need + block - 1) / block) * block
	}
	rightEdge := g.ox + float64(g.w)*g.res
	rightCells := 0
	if maxX > rightEdge {
		need := int((maxX-rightEdge)/g.res) + 1
		rightCells = ((need + block - 1) / block) * block
	}
	topEdge := g.oy + float64(g.h)*g.res
	topCells := 0
	if maxY > topEdge {
		need := int((maxY-topEdge)/g.res) + 1
		topCells = ((need + block - 1) / block) * block
	}
	nw := g.w + leftCells + rightCells
	nh := g.h + botCells + topCells
	newOX := g.ox - float64(leftCells)*g.res
	newOY := g.oy - float64(botCells)*g.res
	nl := make([]float64, nw*nh)
	offX := leftCells
	offY := botCells
	for y := 0; y < g.h; y++ {
		if y+offY < 0 {
			continue
		}
		copy(nl[(y+offY)*nw+offX:(y+offY)*nw+offX+g.w],
			g.logOdds[y*g.w:y*g.w+g.w])
	}
	g.logOdds, g.w, g.h, g.ox, g.oy = nl, nw, nh, newOX, newOY
}

// integrateScan ray-casts one scan observed at pose p and updates log odds.
// The grid lock is shared with Pipeline.mu: pipeline callers already hold
// the pipeline write lock and must pass locked=true to avoid self-deadlock;
// standalone callers pass locked=false.
func (g *Grid) integrateScanLocked(p Pose, scan []Point) {
	// Bounds of this scan in the world frame; grow the grid whenever it
	// falls outside the current extent. Growth must happen on EVERY scan,
	// not only the first one: a robot that keeps walking emits endpoints far
	// beyond the initial grid, and silently dropping out-of-range rays would
	// freeze the map at its first-frame footprint.
	minX, minY := p.X, p.Y
	maxX, maxY := minX, minY
	for _, pt := range scan {
		w := p.Transform(pt)
		if w.X < minX {
			minX = w.X
		}
		if w.Y < minY {
			minY = w.Y
		}
		if w.X > maxX {
			maxX = w.X
		}
		if w.Y > maxY {
			maxY = w.Y
		}
	}
	g.ensureSize(minX, minY, maxX, maxY)
	c := g.cfg
	for _, pt := range scan {
		w := p.Transform(pt)
		g.castRay(p.X, p.Y, w.X, w.Y, c)
	}
}

func (g *Grid) castRay(x0, y0, x1, y1 float64, c Config) {
	dx, dy := x1-x0, y1-y0
	length := math.Hypot(dx, dy)
	if length < c.MinRange || length > c.MaxRange {
		return
	}
	steps := int(length/g.res) + 1
	if steps > c.MapMaxRayStep {
		steps = c.MapMaxRayStep
	}
	ux, uy := dx/float64(steps), dy/float64(steps)
	x, y := x0, y0
	for s := 0; s < steps; s++ {
		cx := int(math.Floor((x - g.ox) / g.res))
		cy := int(math.Floor((y - g.oy) / g.res))
		if cx >= 0 && cy >= 0 && cx < g.w && cy < g.h {
			idx := cy*g.w + cx
			// Confirmed-occupied cells are never erased by free-space
			// crossings: a later session sweeping a ray across a wall the
			// old session saw must not turn the wall into free space.
			if g.logOdds[idx] < c.MapOccPersist {
				g.logOdds[idx] = math.Max(-c.MapLogOddMax,
					math.Min(c.MapLogOddMax, g.logOdds[idx]+c.MapLogOddFree))
			}
		}
		x += ux
		y += uy
	}
	// Endpoint: occupied.
	cx := int(math.Floor((x1 - g.ox) / g.res))
	cy := int(math.Floor((y1 - g.oy) / g.res))
	if cx >= 0 && cy >= 0 && cx < g.w && cy < g.h {
		idx := cy*g.w + cx
		g.logOdds[idx] = math.Max(-c.MapLogOddMax,
			math.Min(c.MapLogOddMax, g.logOdds[idx]+c.MapLogOddHit))
	}
}

// rebuildFromKeyframes clears the grid and re-integrates every keyframe scan
// at its optimized pose. This is what keeps map and trajectory aligned after
// loop closure.
func (g *Grid) rebuildFromKeyframes(kfs []*Keyframe) {
	g.w, g.h = 0, 0
	g.logOdds = nil
	// Determine the global bounds in one pass so the grid allocates once;
	// incremental growth per ray is O(n^2) for long runs.
	if len(kfs) > 0 {
		minX, minY := kfs[0].Pose.X, kfs[0].Pose.Y
		maxX, maxY := minX, minY
		for _, kf := range kfs {
			for _, pt := range kf.Scan {
				w := kf.Pose.Transform(pt)
				if w.X < minX {
					minX = w.X
				}
				if w.Y < minY {
					minY = w.Y
				}
				if w.X > maxX {
					maxX = w.X
				}
				if w.Y > maxY {
					maxY = w.Y
				}
			}
			if kf.Pose.X < minX {
				minX = kf.Pose.X
			}
			if kf.Pose.Y < minY {
				minY = kf.Pose.Y
			}
			if kf.Pose.X > maxX {
				maxX = kf.Pose.X
			}
			if kf.Pose.Y > maxY {
				maxY = kf.Pose.Y
			}
		}
		g.ensureSize(minX, minY, maxX, maxY)
	}
	for _, kf := range kfs {
		g.integrateScanLocked(kf.Pose, kf.Scan)
	}
}

func (g *Grid) raw() ([]float64, *MapSnapshot) {
	cp := make([]float64, len(g.logOdds))
	copy(cp, g.logOdds)
	return cp, g.snapshot()
}

// hitCellsWorld returns the world-frame center points of cells currently
// classified as occupied. It is the global reference used by cross-session
// localization: every old keyframe surface is already projected into the
// saved map, so this set never depends on the new session's odom frame.
func (g *Grid) hitCellsWorld() []Point {
	out := make([]Point, 0)
	for cy := 0; cy < g.h; cy++ {
		for cx := 0; cx < g.w; cx++ {
			if g.logOdds[cy*g.w+cx] > 0.4 {
				out = append(out, Point{
					X: g.ox + (float64(cx)+0.5)*g.res,
					Y: g.oy + (float64(cy)+0.5)*g.res,
				})
			}
		}
	}
	return out
}

// occupiedCount counts occupied cells (used by tests/diagnostics).
func (g *Grid) occupiedCount() int {
	n := 0
	for _, v := range g.logOdds {
		if v > 0.4 {
			n++
		}
	}
	return n
}

func (g *Grid) loadRaw(res, ox, oy float64, w, h int, vals []float64) {
	g.res, g.ox, g.oy = res, ox, oy
	g.w, g.h = w, h
	g.logOdds = append([]float64(nil), vals...)
}
