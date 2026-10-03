package slam

import (
	"math"
	"testing"
)

// TestOccupiedNotErasedByRepeatedSweep verifies the merge conflict policy at
// the grid level: once a cell is a confirmed wall (logOdds >= MapOccPersist),
// free-space ray crossings from later sessions must not decrease it.
func TestOccupiedNotErasedByRepeatedSweep(t *testing.T) {
	cfg := DefaultConfig()
	g := newGrid(cfg)
	g.ensureSize(-1, -1, 3, 1)
	cx := int(math.Floor((1.0 - g.ox) / g.res))
	cy := int(math.Floor((0.0 - g.oy) / g.res))
	idx := cy*g.w + cx

	// Confirm the wall cell directly (as repeated hit integration would).
	g.logOdds[idx] = cfg.MapOccPersist + 0.5
	confirmed := g.logOdds[idx]

	// Cast many rays whose FREE-space segment crosses the wall cell from
	// different directions. Endpoints stay beyond the cell so it is only a
	// pass-through, never the endpoint.
	starts := [][2]float64{{1.0, 2.0}, {2.0, 0.0}, {1.0, -2.0}, {-2.0, 0.0}}
	ends := [][2]float64{{1.0, -3.0}, {-3.0, 0.0}, {1.0, 3.0}, {3.0, 0.0}}
	for rep := 0; rep < 20; rep++ {
		for k, s := range starts {
			g.castRay(s[0], s[1], ends[k][0], ends[k][1], cfg)
		}
	}
	if g.logOdds[idx] < confirmed {
		t.Fatalf("confirmed wall erased by repeated crossing rays: %.3f -> %.3f",
			confirmed, g.logOdds[idx])
	}
	t.Logf("confirmed wall retained logOdds=%.3f after %d crossing rays",
		g.logOdds[idx], 20*4)
}

// TestUnconfirmedCellStillCleared ensures persistence does not freeze weak
// evidence: an unconfirmed cell must still be decremented by free rays.
func TestUnconfirmedCellStillCleared(t *testing.T) {
	cfg := DefaultConfig()
	g := newGrid(cfg)
	g.ensureSize(-1, -1, 3, 1)
	cx := int(math.Floor((0.5 - g.ox) / g.res))
	cy := int(math.Floor((0.0 - g.oy) / g.res))
	idx := cy*g.w + cx
	g.logOdds[idx] = cfg.MapOccPersist - 0.5 // occupied-looking but unconfirmed
	before := g.logOdds[idx]
	g.castRay(0, 0, 4.0, 0.004, cfg)
	if g.logOdds[idx] >= before {
		t.Fatalf("unconfirmed cell not decremented: %.3f -> %.3f",
			before, g.logOdds[idx])
	}
}
