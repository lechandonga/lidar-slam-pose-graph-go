// Package gridmap provides a 2D log-odds occupancy map and local Euclidean
// distance-transform fields used by the scan matcher.
//
// World frame: meters, X east / forward, Y north / left, origin at the map
// center. Cell (0,0) covers the bottom-left corner of the covered area.
// The Map is safe for concurrent use: mutating methods take the write lock,
// readers must either hold RLock or work on a Snapshot.
package gridmap

import (
	"math"
	"sync"

	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/geom"
)

// Map is a fixed-size occupancy grid with log-odds cells.
type Map struct {
	mu       sync.RWMutex
	res      float64
	size     int     // cells per axis
	originX  float64 // world x of cell center 0
	originY  float64
	hit      float64
	free     float64
	minLO    float64
	maxLO    float64
	logOdds  []float64
	hitCount []int // diagnostics: how often a cell was marked occupied
}

// NewMap creates a square map covering [-half, half] meters per axis.
func NewMap(res, half, hit, free, minLO, maxLO float64) *Map {
	size := int(math.Ceil(2*half/res)) + 1
	return &Map{
		res:      res,
		size:     size,
		originX:  -half,
		originY:  -half,
		hit:      hit,
		free:     free,
		minLO:    minLO,
		maxLO:    maxLO,
		logOdds:  make([]float64, size*size),
		hitCount: make([]int, size*size),
	}
}

// Resolution returns meters per cell.
func (m *Map) Resolution() float64 { return m.res }

// Size returns cells per axis.
func (m *Map) Size() int { return m.size }

func (m *Map) index(cx, cy int) int { return cy*m.size + cx }

// WorldToCell converts world coordinates to cell indices (no bounds check).
func (m *Map) WorldToCell(x, y float64) (int, int) {
	cx := int(math.Floor((x - m.originX) / m.res))
	cy := int(math.Floor((y - m.originY) / m.res))
	return cx, cy
}

func (m *Map) inBounds(cx, cy int) bool {
	return cx >= 0 && cy >= 0 && cx < m.size && cy < m.size
}

// CellWorld returns the world coordinate of a cell center.
func (m *Map) CellWorld(cx, cy int) geom.Point {
	return geom.Point{X: m.originX + (float64(cx)+0.5)*m.res, Y: m.originY + (float64(cy)+0.5)*m.res}
}

// AddHit applies the occupied update to a world point.
func (m *Map) AddHit(x, y float64) {
	m.mu.Lock()
	defer m.mu.Unlock()
	cx, cy := m.WorldToCell(x, y)
	if !m.inBounds(cx, cy) {
		return
	}
	idx := m.index(cx, cy)
	v := m.logOdds[idx] + m.hit
	if v > m.maxLO {
		v = m.maxLO
	}
	m.logOdds[idx] = v
	m.hitCount[idx]++
}

// AddLine raycasts from (x0,y0) to (x1,y1): free cells along the way,
// and an occupied cell at the endpoint (Bresenham, deterministic).
func (m *Map) AddLine(x0, y0, x1, y1 float64, markEndpoint bool) {
	m.mu.Lock()
	defer m.mu.Unlock()
	cx0, cy0 := m.WorldToCell(x0, y0)
	cx1, cy1 := m.WorldToCell(x1, y1)
	dx := absInt(cx1 - cx0)
	dy := -absInt(cy1 - cy0)
	sx := sign(cx0, cx1)
	sy := sign(cy0, cy1)
	err := dx + dy
	for {
		atEnd := cx0 == cx1 && cy0 == cy1
		if m.inBounds(cx0, cy0) && !atEnd {
			idx := m.index(cx0, cy0)
			// A ray grazing a corner can pass through a cell that actually
			// contains a hit from another beam; never erase occupied
			// evidence with a free update.
			if m.logOdds[idx] > 0 {
				goto step
			}
			v := m.logOdds[idx] + m.free
			if v < m.minLO {
				v = m.minLO
			}
			m.logOdds[idx] = v
		}
	step:
		if cx0 == cx1 && cy0 == cy1 {
			break
		}
		e2 := 2 * err
		if e2 >= dy {
			err += dy
			cx0 += sx
		}
		if e2 <= dx {
			err += dx
			cy0 += sy
		}
	}
	// Endpoint occupied update at the exact hit cell.
	if markEndpoint {
		if m.inBounds(cx1, cy1) {
			idx := m.index(cx1, cy1)
			v := m.logOdds[idx] + m.hit
			if v > m.maxLO {
				v = m.maxLO
			}
			m.logOdds[idx] = v
			m.hitCount[idx]++
		}
	}
}

func absInt(v int) int {
	if v < 0 {
		return -v
	}
	return v
}

func sign(a, b int) int {
	if a < b {
		return 1
	}
	return -1
}

// Occupancy returns p(occupied) in [0,1] for a cell, plus bounds flag.
func (m *Map) Occupancy(cx, cy int) (float64, bool) {
	m.mu.RLock()
	defer m.mu.RUnlock()
	if !m.inBounds(cx, cy) {
		return 0, false
	}
	return logOddsToProb(m.logOdds[m.index(cx, cy)]), true
}

// IsOccupied reports whether a cell is above the 0.5 probability threshold.
func (m *Map) IsOccupied(cx, cy int) bool {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return m.inBounds(cx, cy) && m.logOdds[m.index(cx, cy)] > 0
}

func logOddsToProb(lo float64) float64 {
	return 1.0 / (1.0 + math.Exp(-lo))
}

// OccupiedCells returns world coordinates of all currently occupied cells
// (log odds > 0). The returned slice is a copy.
func (m *Map) OccupiedCells() []geom.Point {
	m.mu.RLock()
	defer m.mu.RUnlock()
	var out []geom.Point
	for cy := 0; cy < m.size; cy++ {
		for cx := 0; cx < m.size; cx++ {
			if m.logOdds[m.index(cx, cy)] > 0 {
				out = append(out, m.CellWorld(cx, cy))
			}
		}
	}
	return out
}

// Snapshot is an immutable copy of map data, safe to read without locking.
type Snapshot struct {
	Res     float64
	Size    int
	OriginX float64
	OriginY float64
	LogOdds []float64
}

// Snapshot copies the whole map (used by persistence and tests).
func (m *Map) Snapshot() Snapshot {
	m.mu.RLock()
	defer m.mu.RUnlock()
	lo := make([]float64, len(m.logOdds))
	copy(lo, m.logOdds)
	return Snapshot{Res: m.res, Size: m.size, OriginX: m.originX, OriginY: m.originY, LogOdds: lo}
}

// Restore replaces map contents from a snapshot. Parameters are re-derived
// from the snapshot so the caller cannot desync them.
func Restore(s Snapshot, hit, free, minLO, maxLO float64) *Map {
	m := &Map{
		res: s.Res, size: s.Size, originX: s.OriginX, originY: s.OriginY,
		hit: hit, free: free, minLO: minLO, maxLO: maxLO,
		logOdds:  append([]float64(nil), s.LogOdds...),
		hitCount: make([]int, s.Size*s.Size),
	}
	return m
}
