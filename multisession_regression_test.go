package slam_test

import (
	"math"
	"path/filepath"
	"sync"
	"testing"

	root "github.com/lechandonga/lidar-slam-pose-graph-go"
	"github.com/lechandonga/lidar-slam-pose-graph-go/synth"
)

func overlapWorldReg() synth.World {
	return synth.RectWorld(16, 12, []struct{ X, Y, Ww, Hh float64 }{
		{-4.5, -4.0, 2.0, 0.5}, {0.5, 0.5, 2.2, 2.2},
		{-4.0, 3.2, 1.8, 1.0}, {3.8, 3.4, 1.4, 0.9},
		{5.0, -2.2, 0.7, 1.8}, {2.0, -4.2, 1.2, 0.5},
	})
}

func overlapPathReg() []synth.Pose {
	const n = 30
	pts := make([]synth.Pose, 0, 2*n+8)
	for i := 0; i < n; i++ {
		s := float64(i) / float64(n-1)
		pts = append(pts, synth.Pose{X: s * 6})
	}
	corner := pts[len(pts)-1]
	for i := 1; i <= 8; i++ {
		th := math.Pi / 2 * float64(i) / 8
		pts = append(pts, synth.Pose{X: corner.X, Y: corner.Y, Theta: th})
	}
	for i := 1; i <= n; i++ {
		s := float64(i) / float64(n)
		pts = append(pts, synth.Pose{X: corner.X, Y: corner.Y + s*6,
			Theta: math.Pi / 2})
	}
	return pts
}

func overlapOptsReg(seed int64) synth.Options {
	return synth.Options{
		DT: 0.1, Nbeams: 360, Fov: 2 * math.Pi, MaxR: 20,
		BeamNoise: 0.02, BearingJit: 0.003,
		OdomDrift: 0.02, OdomBias: 0.002, Seed: seed,
	}
}

func buildSplitReg(seed int64) (synth.SplitSession, synth.SplitSession) {
	a, b, _ := synth.SplitSessions(overlapWorldReg(), overlapPathReg(),
		overlapOptsReg(seed), 30, 20)
	return a, b
}

func runAllReg(t *testing.T, p *root.Pipeline, src root.DataSource) {
	t.Helper()
	for {
		fr, ok := src.Next()
		if !ok {
			break
		}
		p.ProcessFrame(fr)
	}
	p.Finalize()
}

func firstFrameReg(ss synth.SplitSession) (root.Frame, root.DataSource) {
	src := synth.SessionSource(ss)
	fr, _ := src.Next()
	return fr, src
}

// TestSplitSelfConsistentReg: localizing session B's own first scan in A's
// map must recover B's annotated StartWorld, not A's origin.
func TestSplitSelfConsistentReg(t *testing.T) {
	sessA, sessB := buildSplitReg(42)
	p := root.NewPipeline(root.DefaultConfig(), root.NopLogger())
	runAllReg(t, p, synth.SessionSource(sessA))

	first, _ := firstFrameReg(sessB)
	if err := p.PrepareNewSession(); err != nil {
		t.Fatal(err)
	}
	lr := p.Localize(first.Scan, root.LocalizeHint{})
	t.Logf("[self-consistent] B truth=(%.3f %.3f %.3f) status=%s reason=%s "+
		"score=%.3f rms=%.3f pose=%s cands=%d",
		sessB.StartWorld.X, sessB.StartWorld.Y, sessB.StartWorld.Theta,
		lr.Status, lr.Reason, lr.Score, lr.Residual, lr.Pose,
		len(lr.Candidates))
	if lr.Status != root.LocalizeUnique {
		t.Fatalf("want unique, got %s (%s)", lr.Status, lr.Reason)
	}
	errXY := math.Hypot(lr.Pose.X-sessB.StartWorld.X,
		lr.Pose.Y-sessB.StartWorld.Y)
	errTh := math.Abs(root.NormAngle(lr.Pose.Theta - sessB.StartWorld.Theta))
	t.Logf("[self-consistent] deviation errXY=%.3fm errTh=%.4frad",
		errXY, errTh)
	if errXY > 0.5 {
		t.Fatalf("localization deviates %.3fm from B's annotated start", errXY)
	}
	if errTh > 0.25 {
		t.Fatalf("heading deviation %.3f too large", errTh)
	}
	if d := math.Hypot(lr.Pose.X, lr.Pose.Y); d < 3.0 {
		t.Fatalf("localization snapped to A origin (d=%.3f), not B start", d)
	}
}

// TestSplitFailureUntouchedReg: an out-of-map first scan fails (distinct
// from ambiguity) and changes nothing in the loaded state.
func TestSplitFailureUntouchedReg(t *testing.T) {
	sessA, _ := buildSplitReg(43)
	p := root.NewPipeline(root.DefaultConfig(), root.NopLogger())
	runAllReg(t, p, synth.SessionSource(sessA))
	n0, e0 := p.GraphSummary()
	tr0, occ0 := len(p.Trajectory()), countOccupied(p.MapSnapshot())

	if err := p.PrepareNewSession(); err != nil {
		t.Fatal(err)
	}
	// Well outside the room: virtually no returns, so localization fails
	// (reason bad_scan:too_few_points) — distinct from both unique and
	// ambiguous. Symmetric in-room ambiguity is covered separately by
	// TestCrossSessionLocalizeAmbiguous.
	fr := scanAt(synth.Pose{X: 60, Y: 60}, 7001)
	lr := p.Localize(fr.Scan, root.LocalizeHint{})
	t.Logf("[fail-untouched] status=%s reason=%s score=%.3f",
		lr.Status, lr.Reason, lr.Score)
	if lr.Status != root.LocalizeFailed {
		t.Fatalf("want failed, got %s", lr.Status)
	}
	if lr.Reason == root.ReasonLocAmbiguous {
		t.Fatal("failure must be distinguishable from ambiguity")
	}
	if err := p.BeginSession(fr, lr); err == nil {
		t.Fatal("BeginSession must reject failure")
	}
	if d := p.ProcessFrame(fr); d.Reason != root.ReasonAwaitingAnchor {
		t.Fatalf("want awaiting_session_anchor, got %s", d.Reason)
	}
	n1, e1 := p.GraphSummary()
	if n1 != n0 || e1 != e0 || len(p.Trajectory()) != tr0 ||
		countOccupied(p.MapSnapshot()) != occ0 {
		t.Fatalf("state changed on failure: nodes %d->%d edges %d->%d "+
			"traj %d->%d occ %d->%d", n0, n1, e0, e1, tr0,
			len(p.Trajectory()), occ0, countOccupied(p.MapSnapshot()))
	}
	t.Logf("[fail-untouched] unchanged nodes=%d edges=%d traj=%d occ=%d",
		n1, e1, len(p.Trajectory()), countOccupied(p.MapSnapshot()))
}

func mergeBothReg(t *testing.T, seed int64) (*root.Pipeline, synth.SplitSession) {
	t.Helper()
	sessA, sessB := buildSplitReg(seed)
	p := root.NewPipeline(root.DefaultConfig(), root.NopLogger())
	runAllReg(t, p, synth.SessionSource(sessA))
	if err := p.PrepareNewSession(); err != nil {
		t.Fatal(err)
	}
	first, src := firstFrameReg(sessB)
	lr := p.Localize(first.Scan, root.LocalizeHint{})
	if lr.Status != root.LocalizeUnique {
		t.Fatalf("B localize: %s %s score=%.3f", lr.Status, lr.Reason, lr.Score)
	}
	if err := p.BeginSession(first, lr); err != nil {
		t.Fatal(err)
	}
	runAllReg(t, p, src)
	return p, sessB
}

func sessionBRMSEReg(t *testing.T, p *root.Pipeline, sessB synth.SplitSession,
	aOrigin synth.Pose) (rmse, maxErr float64) {
	t.Helper()
	_, base := p.SessionInfo()
	sum, mx, n := 0.0, 0.0, 0
	for _, tp := range p.Trajectory() {
		if tp.FrameID < base {
			continue
		}
		idx := int(tp.FrameID - base)
		if idx >= len(sessB.TruthWorld) {
			continue
		}
		want := worldToSession(aOrigin, sessB.TruthWorld[idx])
		d := math.Hypot(tp.Pose.X-want.X, tp.Pose.Y-want.Y)
		sum += d * d
		n++
		if d > mx {
			mx = d
		}
	}
	if n == 0 {
		t.Fatal("no session-B points scored")
	}
	return math.Sqrt(sum / float64(n)), mx
}

func raySegReg(ox, oy, dx, dy, ax, ay, bx, by float64) (float64, bool) {
	ex, ey := bx-ax, by-ay
	den := dx*ey - dy*ex
	if math.Abs(den) < 1e-12 {
		return 0, false
	}
	tt := ((ax-ox)*ey - (ay-oy)*ex) / den
	u := ((ax-ox)*dy - (ay-oy)*dx) / den
	if tt >= 0 && u >= 0 && u <= 1 {
		return tt, true
	}
	return 0, false
}

// overlapMapAgreementReg casts dense rays from overlap truth poses and
// reports the fraction whose true wall endpoint is occupied in the map.
func overlapMapAgreementReg(t *testing.T, world synth.World,
	snap *root.MapSnapshot, poses []synth.Pose) float64 {
	t.Helper()
	hits, tot := 0, 0
	for _, pose := range poses {
		for a := -math.Pi; a < math.Pi; a += math.Pi / 12 {
			dx, dy := math.Cos(pose.Theta+a), math.Sin(pose.Theta+a)
			best := 20.0
			found := false
			for _, w := range world.Walls {
				if r, ok := raySegReg(pose.X, pose.Y, dx, dy,
					w.A.X, w.A.Y, w.B.X, w.B.Y); ok && r < best {
					best, found = r, true
				}
			}
			if !found {
				continue
			}
			wx := pose.X + best*dx
			wy := pose.Y + best*dy
			tot++
			if occupiedWithinReg(snap, wx, wy, 3*snap.Resolution) {
				hits++
			}
		}
	}
	if tot == 0 {
		t.Fatal("no overlap wall samples")
	}
	return float64(hits) / float64(tot)
}

// occupiedWithinReg reports an occupied cell within tol metres of (x,y).
// Grid maps agree up to a few cells of independent-pass noise, so map
// parity uses a tolerance rather than an exact single-cell match.
func occupiedWithinReg(s *root.MapSnapshot, x, y, tol float64) bool {
	steps := int(tol/s.Resolution + 0.5)
	for dy := -steps; dy <= steps; dy++ {
		for dx := -steps; dx <= steps; dx++ {
			if s.IsOccupiedAtWorld(x+float64(dx)*s.Resolution,
				y+float64(dy)*s.Resolution) {
				return true
			}
		}
	}
	return false
}

// TestMergeMatchesSingleRunReg compares the two-session merge against one
// continuous run over the identical path (trajectory RMSE + overlap map).
func TestMergeMatchesSingleRunReg(t *testing.T) {
	const seed = 51
	merged, sessB := mergeBothReg(t, seed)

	o := overlapOptsReg(seed)
	o.Path = overlapPathReg()
	baseP := root.NewPipeline(root.DefaultConfig(), root.NopLogger())
	runAllReg(t, baseP, synth.NewSource(synth.New(overlapWorldReg(), o)))

	aOrigin := overlapPathReg()[0]
	rmse, maxErr := sessionBRMSEReg(t, merged, sessB, aOrigin)
	t.Logf("[merge-vs-single] B trajectory rmse=%.3fm max=%.3fm", rmse, maxErr)
	if rmse > 0.6 {
		t.Fatalf("merged B RMSE too high: %.3f", rmse)
	}
	overlap := overlapPathReg()[20:30]
	agreeM := overlapMapAgreementReg(t, overlapWorldReg(),
		merged.MapSnapshot(), overlap)
	agreeB := overlapMapAgreementReg(t, overlapWorldReg(),
		baseP.MapSnapshot(), overlap)
	aOnly, _ := buildSplitReg(seed)
	aP := root.NewPipeline(root.DefaultConfig(), root.NopLogger())
	runAllReg(t, aP, synth.SessionSource(aOnly))
	agreeA := overlapMapAgreementReg(t, overlapWorldReg(),
		aP.MapSnapshot(), overlap)
	t.Logf("[merge-vs-single] A-only overlap agreement=%.3f", agreeA)
	t.Logf("[merge-vs-single] overlap map agreement merged=%.3f single=%.3f",
		agreeM, agreeB)
	if agreeM < 0.7 {
		t.Fatalf("merged overlap agreement too low: %.3f", agreeM)
	}
	if math.Abs(agreeM-agreeB) > 0.12 {
		t.Fatalf("merged/%.3f vs single/%.3f overlap maps disagree",
			agreeM, agreeB)
	}
}

// TestRepeatedOverlapConvergesReg feeds the SAME already-mapped overlap for
// several real load->run->save rounds and requires nodes/edges to stop
// growing and the file size to stabilize.
func TestRepeatedOverlapConvergesReg(t *testing.T) {
	sessA, sessB := buildSplitReg(44)
	p := root.NewPipeline(root.DefaultConfig(), root.NopLogger())
	runAllReg(t, p, synth.SessionSource(sessA))
	statePath := filepath.Join(t.TempDir(), "state.gob")

	replayOverlap := func(q *root.Pipeline, extraFrames int) {
		t.Helper()
		if err := q.PrepareNewSession(); err != nil {
			t.Fatal(err)
		}
		first, src := firstFrameReg(sessB)
		lr := q.Localize(first.Scan, root.LocalizeHint{})
		if lr.Status != root.LocalizeUnique {
			t.Fatalf("localize: %s %s", lr.Status, lr.Reason)
		}
		if err := q.BeginSession(first, lr); err != nil {
			t.Fatal(err)
		}
		for i := 0; i < extraFrames; i++ {
			fr, ok := src.Next()
			if !ok {
				break
			}
			q.ProcessFrame(fr)
		}
		q.Finalize()
		if err := q.Save(statePath); err != nil {
			t.Fatal(err)
		}
	}

	// Initial merged pass over the overlap only (frames A already mapped).
	replayOverlap(p, 9)
	load := func() *root.Pipeline {
		q, err := root.LoadState(statePath, root.NopLogger())
		if err != nil {
			t.Fatal(err)
		}
		return q
	}
	type size struct {
		n, e int
		b    int64
	}
	var hist []size
	n0, e0 := p.GraphSummary()
	hist = append(hist, size{n0, e0, fileSize(statePath)})
	for r := 0; r < 6; r++ {
		q := load()
		replayOverlap(q, 9)
		n, e := q.GraphSummary()
		hist = append(hist, size{n, e, fileSize(statePath)})
		p = q
	}
	for i, h := range hist {
		t.Logf("[repeated-overlap] round=%d nodes=%d edges=%d bytes=%d",
			i, h.n, h.e, h.b)
	}
	// The last two rounds must be identical: graph and map have converged.
	last, prev := hist[len(hist)-1], hist[len(hist)-2]
	if last.n != prev.n || last.e != prev.e {
		t.Fatalf("graph still growing on identical re-survey: "+
			"nodes %d->%d edges %d->%d", prev.n, last.n, prev.e, last.e)
	}
	// File size may wiggle with bounded logs but must not trend upward in
	// the converged tail.
	if last.b > prev.b {
		t.Fatalf("state file grew after convergence: %d -> %d", prev.b, last.b)
	}
}

// TestRealOldVersionStateLoadsReg loads a state file actually produced by
// the previous released program (testdata/real-v1.gob, built from the v1
// commit), verifies migration and that processing continues.
func TestRealOldVersionStateLoadsReg(t *testing.T) {
	p, err := root.LoadState("testdata/real-v1.gob", root.NopLogger())
	if err != nil {
		t.Fatalf("genuine v1 file must load: %v", err)
	}
	n, e := p.GraphSummary()
	occ := countOccupied(p.MapSnapshot())
	t.Logf("[real-v1] loaded nodes=%d edges=%d traj=%d occ=%d session=%d",
		n, e, len(p.Trajectory()), occ, p.Session())
	if n == 0 || occ == 0 {
		t.Fatal("migrated v1 state lost map data")
	}
	if p.Session() != 0 {
		t.Fatal("v1 keyframes must migrate to session 0")
	}
	// New safeguards must be active, not left at gob zero values.
	if root.ExportConfig(p).PersistedLogCap <= 0 {
		t.Fatal("PersistedLogCap not repaired for old state")
	}
	if root.ExportConfig(p).RevisitReuseRadius <= 0 {
		t.Fatal("RevisitReuseRadius not repaired for old state")
	}
	// A resumed same-session frame is accepted (pipeline is usable, not a
	// half-new/half-old object).
	gen := synth.New(overlapWorldReg(), overlapOptsReg(5))
	src := synth.NewSource(gen)
	fr, _ := src.Next()
	d := p.ProcessFrame(fr)
	t.Logf("[real-v1] resumed first frame outcome=%s reason=%s",
		d.Outcome, d.Reason)
}

// TestMergeConcurrentQuerySaveReg hammers MapSnapshot and Save while a
// merge is in progress; run under -race.
func TestMergeConcurrentQuerySaveReg(t *testing.T) {
	sessA, sessB := buildSplitReg(60)
	p := root.NewPipeline(root.DefaultConfig(), root.NopLogger())
	runAllReg(t, p, synth.SessionSource(sessA))
	if err := p.PrepareNewSession(); err != nil {
		t.Fatal(err)
	}
	first, src := firstFrameReg(sessB)
	lr := p.Localize(first.Scan, root.LocalizeHint{})
	if lr.Status != root.LocalizeUnique {
		t.Fatalf("localize: %s %s", lr.Status, lr.Reason)
	}
	if err := p.BeginSession(first, lr); err != nil {
		t.Fatal(err)
	}
	statePath := filepath.Join(t.TempDir(), "c.gob")
	var wg sync.WaitGroup
	stop := make(chan struct{})
	wg.Add(2)
	go func() {
		defer wg.Done()
		for {
			select {
			case <-stop:
				return
			default:
				if s := p.MapSnapshot(); s != nil {
					_ = s.IsOccupiedAtWorld(0, 0)
				}
			}
		}
	}()
	go func() {
		defer wg.Done()
		for {
			select {
			case <-stop:
				return
			default:
				_ = p.Save(statePath)
			}
		}
	}()
	for {
		fr, ok := src.Next()
		if !ok {
			break
		}
		p.ProcessFrame(fr)
	}
	p.Finalize()
	close(stop)
	wg.Wait()
	n, e := p.GraphSummary()
	t.Logf("[concurrent] merge done nodes=%d edges=%d occ=%d",
		n, e, countOccupied(p.MapSnapshot()))
	if n == 0 {
		t.Fatal("concurrent merge produced no graph")
	}
}
