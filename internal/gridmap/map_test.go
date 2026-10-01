package gridmap

import "testing"

func TestAddLineEndpoint(t *testing.T) {
	m := NewMap(0.1, 5, 1, -1, -10, 10)
	// horizontal ray from origin 3 m to the right
	m.AddLine(0, 0, 3.0, 0.0, true)
	ex, ey := m.WorldToCell(3.0, 0.0)
	if !m.IsOccupied(ex, ey) {
		t.Fatalf("endpoint cell (%d,%d) not occupied", ex, ey)
	}
	// free cells along the ray must be free
	fx, fy := m.WorldToCell(1.0, 0.0)
	if p, _ := m.Occupancy(fx, fy); p > 0.5 {
		t.Fatalf("mid-ray cell unexpectedly occupied p=%.2f", p)
	}
}

func TestBuildFieldDistance(t *testing.T) {
	m := NewMap(0.1, 5, 1, -1, -10, 10)
	m.AddHit(1.0, 0.0)
	f := m.BuildField(1.0, 0.0, 2.0)
	d, _, _, _, _, ok := f.Sample(1.3, 0.0)
	if !ok {
		t.Fatal("sample out of window")
	}
	if d < 0.2 || d > 0.4 {
		t.Fatalf("distance to hit ~0.3 expected, got %.3f", d)
	}
}
