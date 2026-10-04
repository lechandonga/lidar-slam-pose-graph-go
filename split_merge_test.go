package slam_test

import (
	"math"
	"os"
	"path/filepath"
	"testing"

	root "github.com/lechandonga/lidar-slam-pose-graph-go"
	"github.com/lechandonga/lidar-slam-pose-graph-go/synth"
)

// splitWorld/path/opts/cuts are the documented two-session scenario: one
// continuous ground-truth route cut into two independently-originated
// sessions with an overlap, plus the same route run end-to-end as reference.
func splitScenario() (synth.World, []synth.Pose, synth.Options, int, int) {
	world := synth.RectWorld(16, 12, []struct{ X, Y, Ww, Hh float64 }{
		{-4.5, -4.0, 2.0, 0.5}, {-5.2, -3.0, 0.5, 2.0},
		{-2.8, -4.6, 0.7, 0.7}, {0.5, 0.5, 2.2, 2.2},
		{-4.0, 3.2, 1.8, 1.0}, {3.8, 3.4, 1.4, 0.9},
		{5.0, -2.2, 0.7, 1.8}, {2.0, -4.2, 1.2, 0.5},
	})
	const perLeg, turn = 30, 8
	path := make([]synth.Pose, 0, 2*perLeg+turn)
	for i := 0; i < perLeg; i++ {
		s := float64(i) / float64(perLeg-1)
		path = append(path, synth.Pose{X: s * 6})
	}
	corner := path[len(path)-1]
	for i := 1; i <= turn; i++ {
		th := math.Pi / 2 * float64(i) / float64(turn)
		path = append(path, synth.Pose{X: corner.X, Y: corner.Y, Theta: th})
	}
	for i := 1; i <= perLeg; i++ {
		s := float64(i) / float64(perLeg)
		path = append(path, synth.Pose{X: corner.X, Y: corner.Y + s*6,
			Theta: math.Pi / 2})
	}
	opt := synth.Options{
		DT: 0.1, Nbeams: 360, Fov: 2 * math.Pi, MaxR: 20,
		BeamNoise: 0.02, BearingJit: 0.003,
		OdomDrift: 0.02, OdomBias: 0.002, Seed: 42,
	}
	cut1 := 30
	cut2 := cut1 - 10
	return world, path, opt, cut1, cut2
}

func buildSplitA(t *testing.T) (*root.Pipeline, synth.SplitSession, synth.SplitSession) {
	t.Helper()
	world, path, opt, c1, c2 := splitScenario()
	a, b, _ := synth.SplitSessions(world, path, opt, c1, c2)
	p := root.NewPipeline(root.DefaultConfig(), root.NopLogger())
	runSession2(t, p, synth.SessionSource(a))
	return p, a, b
}

// TestSplitDataSelfConsistent: localizing a session with ITS OWN first scan
// against a map of ITS OWN route must return its annotated StartWorld.
func TestSplitDataSelfConsistent(t *testing.T) {
	world, path, opt, c1, c2 := splitScenario()
	a, b, _ := synth.SplitSessions(world, path, opt, c1, c2)
	check := func(name string, ss synth.SplitSession) {
		t.Helper()
		p := root.NewPipeline(root.DefaultConfig(), root.NopLogger())
		runSession2(t, p, synth.SessionSource(ss))
		first, _ := synth.SessionSource(ss).Next()
		lr := p.Localize(first.Scan, root.LocalizeHint{})
		// In the session's OWN map its first scan is the origin; that is
		// the self-consistency target. (StartWorld is the answer only in a
		// DIFFERENT, earlier session's world frame.)
		errXY := math.Hypot(lr.Pose.X, lr.Pose.Y)
		errTh := math.Abs(lr.Pose.Theta)
		t.Logf("%s self-localize status=%s score=%.3f rms=%.3f pose=%s "+
			"want=(0,0,0) errXY=%.3fm errTh=%.4frad cands=%d",
			name, lr.Status, lr.Score, lr.Residual, lr.Pose,
			errXY, errTh, len(lr.Candidates))
		if lr.Status != root.LocalizeUnique {
			t.Fatalf("%s self-localize not unique: %s %s", name, lr.Status, lr.Reason)
		}
		if errXY > 0.5 {
			t.Fatalf("%s errXY %.3f > 0.5m", name, errXY)
		}
		if errTh > 0.25 {
			t.Fatalf("%s errTh %.3f > 0.25rad", name, errTh)
		}
	}
	check("sessionA", a)
	check("sessionB", b)
}

// TestSplitLocalizeRecoversAnnotatedStart quantifies the anchor error
// against B's annotated StartWorld in the documented merge flow.
func TestSplitLocalizeRecoversAnnotatedStart(t *testing.T) {
	p, _, b := buildSplitA(t)
	if err := p.PrepareNewSession(); err != nil {
		t.Fatal(err)
	}
	first, _ := synth.SessionSource(b).Next()
	lr := p.Localize(first.Scan, root.LocalizeHint{})
	errXY := math.Hypot(lr.Pose.X-b.StartWorld.X, lr.Pose.Y-b.StartWorld.Y)
	errTh := math.Abs(root.NormAngle(lr.Pose.Theta - b.StartWorld.Theta))
	t.Logf("B anchor status=%s reason=%s score=%.3f rms=%.3f pose=%s "+
		"annotated=%v errXY=%.3fm errTh=%.4frad cands=%d",
		lr.Status, lr.Reason, lr.Score, lr.Residual, lr.Pose,
		b.StartWorld, errXY, errTh, len(lr.Candidates))
	if lr.Status != root.LocalizeUnique {
		t.Fatalf("want unique, got %s (%s)", lr.Status, lr.Reason)
	}
	if errXY > 0.5 {
		t.Fatalf("anchor errXY %.3f > 0.5m", errXY)
	}
	if errTh > 0.25 {
		t.Fatalf("anchor errTh %.3f > 0.25rad", errTh)
	}
}

func runMergedSplit(t *testing.T) *root.Pipeline {
	t.Helper()
	p, _, b := buildSplitA(t)
	statePath := filepath.Join(t.TempDir(), "a.gob")
	if err := p.Save(statePath); err != nil {
		t.Fatal(err)
	}
	loaded, err := root.LoadState(statePath, root.NopLogger())
	if err != nil {
		t.Fatal(err)
	}
	if err := loaded.PrepareNewSession(); err != nil {
		t.Fatal(err)
	}
	src := synth.SessionSource(b)
	first, _ := src.Next()
	lr := loaded.Localize(first.Scan, root.LocalizeHint{})
	if lr.Status != root.LocalizeUnique {
		t.Fatalf("merge localize: %s %s", lr.Status, lr.Reason)
	}
	if err := loaded.BeginSession(first, lr); err != nil {
		t.Fatal(err)
	}
	runSession2(t, loaded, src)
	return loaded
}

func runBaseline(t *testing.T) *root.Pipeline {
	t.Helper()
	world, path, opt, _, _ := splitScenario()
	o := opt
	o.Path = path
	o.Seed = opt.Seed*7 + 1
	p := root.NewPipeline(root.DefaultConfig(), root.NopLogger())
	runSession2(t, p, synth.NewSource(synth.New(world, o)))
	return p
}

func occupiedKeys(s *root.MapSnapshot) map[[2]int]bool {
	m := map[[2]int]bool{}
	for cy := 0; cy < s.H; cy++ {
		for cx := 0; cx < s.W; cx++ {
			if s.Cells[cy*s.W+cx] == root.CellOccupied {
				m[[2]int{cx, cy}] = true
			}
		}
	}
	return m
}

// TestSplitMergeMatchesSingleRun: merged B trajectory RMSE vs truth and
// overlap-region occupied-cell agreement vs the single-run baseline map.
func TestSplitMergeMatchesSingleRun(t *testing.T) {
	merged := runMergedSplit(t)
	baseline := runBaseline(t)
	_, _, b := buildSplitA(t)

	_, idBase := merged.SessionInfo()
	sum, mx, n := 0.0, 0.0, 0
	for _, tp := range merged.Trajectory() {
		if tp.FrameID < idBase {
			continue
		}
		idx := int(tp.FrameID - idBase)
		if idx >= len(b.TruthWorld) {
			continue
		}
		want := b.TruthWorld[idx]
		d := math.Hypot(tp.Pose.X-want.X, tp.Pose.Y-want.Y)
		sum += d * d
		n++
		if d > mx {
			mx = d
		}
	}
	rmse := math.Sqrt(sum / float64(n))
	t.Logf("merged B trajectory RMSE=%.3fm max=%.3fm n=%d", rmse, mx, n)
	if rmse > 0.6 {
		t.Fatalf("merged B RMSE %.3f too large", rmse)
	}

	ms, mb := merged.MapSnapshot(), baseline.MapSnapshot()
	minX, minY := math.Inf(1), math.Inf(1)
	maxX, maxY := math.Inf(-1), math.Inf(-1)
	for _, w := range b.TruthWorld[:12] {
		minX, minY = math.Min(minX, w.X), math.Min(minY, w.Y)
		maxX, maxY = math.Max(maxX, w.X), math.Max(maxY, w.Y)
	}
	pad := 8.0 // overlap scans see far walls (MaxR=20m)
	// Symmetric occupied-cell agreement in the overlap field of view,
	// aligned in WORLD coordinates (independent of each grid's origin).
	near := func(snap *root.MapSnapshot, x, y float64) bool {
		for ddy := -2; ddy <= 2; ddy++ {
			for ddx := -2; ddx <= 2; ddx++ {
				if snap.IsOccupiedAtWorld(
					x+float64(ddx)*snap.Resolution,
					y+float64(ddy)*snap.Resolution) {
					return true
				}
			}
		}
		return false
	}
	inBBox := func(x, y float64) bool {
		return x >= minX-pad && x <= maxX+pad && y >= minY-pad && y <= maxY+pad
	}
	agree, tot := 0, 0
	for c := range occupiedKeys(ms) {
		wx := ms.OriginX + (float64(c[0])+0.5)*ms.Resolution
		wy := ms.OriginY + (float64(c[1])+0.5)*ms.Resolution
		if !inBBox(wx, wy) {
			continue
		}
		tot++
		if near(mb, wx, wy) {
			agree++
		}
	}
	bAgree, bTot := 0, 0
	for c := range occupiedKeys(mb) {
		wx := mb.OriginX + (float64(c[0])+0.5)*mb.Resolution
		wy := mb.OriginY + (float64(c[1])+0.5)*mb.Resolution
		if !inBBox(wx, wy) {
			continue
		}
		bTot++
		if near(ms, wx, wy) {
			bAgree++
		}
	}
	if tot == 0 || bTot == 0 {
		t.Fatal("no overlap cells to compare")
	}
	fwd := float64(agree) / float64(tot)
	rev := float64(bAgree) / float64(bTot)
	overlap := (fwd + rev) / 2
	t.Logf("overlap map agreement merged<->baseline fwd=%.3f(%d/%d) "+
		"rev=%.3f(%d/%d) mean=%.3f", fwd, agree, tot, rev, bAgree, bTot, overlap)
	if overlap < 0.8 {
		t.Fatalf("overlap map agreement %.3f below 0.8", overlap)
	}
}

// TestSplitRelocalizeFailureKeepsOldState: a scan outside the mapped area
// must not yield a high-score unique mismatch, and the saved state file must
// be byte-for-byte unchanged after the failed attempt.
func TestSplitRelocalizeFailureKeepsOldState(t *testing.T) {
	p, _, _ := buildSplitA(t)
	statePath := filepath.Join(t.TempDir(), "a.gob")
	if err := p.Save(statePath); err != nil {
		t.Fatal(err)
	}
	before, err := os.ReadFile(statePath)
	if err != nil {
		t.Fatal(err)
	}
	n0, e0 := p.GraphSummary()
	occ0 := countOccupied(p.MapSnapshot())

	world, _, _, _, _ := splitScenario()
	gen := synth.New(world, synth.Options{
		Path: []synth.Pose{{X: 60, Y: 60}}, DT: 0.1, Nbeams: 360,
		Fov: 2 * math.Pi, MaxR: 20, BeamNoise: 0.02, Seed: 555,
	})
	fr, _ := synth.NewSource(gen).Next()
	if err := p.PrepareNewSession(); err != nil {
		t.Fatal(err)
	}
	lr := p.Localize(fr.Scan, root.LocalizeHint{})
	t.Logf("outside-scan status=%s reason=%s score=%.3f rms=%.3f cands=%d",
		lr.Status, lr.Reason, lr.Score, lr.Residual, len(lr.Candidates))
	if lr.Status == root.LocalizeUnique {
		t.Fatal("outside scan must not produce a unique high-score anchor")
	}
	if err := p.BeginSession(fr, lr); err == nil {
		t.Fatal("BeginSession must reject non-unique")
	}
	n1, e1 := p.GraphSummary()
	if n1 != n0 || e1 != e0 || countOccupied(p.MapSnapshot()) != occ0 {
		t.Fatal("failed localization mutated in-memory state")
	}
	after, err := os.ReadFile(statePath)
	if err != nil {
		t.Fatal(err)
	}
	if len(before) != len(after) {
		t.Fatal("failed localization changed the saved file length")
	}
	for i := range before {
		if before[i] != after[i] {
			t.Fatalf("saved state byte %d changed after failed localize", i)
		}
	}
	t.Logf("old state untouched: nodes=%d edges=%d occupied=%d bytes=%d",
		n0, e0, occ0, len(before))
}
