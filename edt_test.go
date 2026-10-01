package slam

import (
	"math"
	"testing"
)

func TestEDTFinite(t *testing.T) {
	pts := []Point{}
	for i := 0; i < 200; i++ {
		a := float64(i) * 0.1
		pts = append(pts, Point{X: 5 * math.Cos(a), Y: 5 * math.Sin(a)})
	}
	lf := buildLikelihoodField(pts, 0.1, 0.3)
	for i, d := range lf.dist {
		if math.IsInf(d, 0) || math.IsNaN(d) || d > 1e5 {
			t.Fatalf("cell %d bad distance %v", i, d)
		}
	}
	r := lf.responseAt(5, 0)
	if r < 0.9 {
		t.Fatalf("expected high response at obstacle, got %v", r)
	}
	r = lf.responseAt(0, 0)
	if r > 0.01 {
		t.Fatalf("expected near-zero at center, got %v", r)
	}
}
