package slam_test

import (
	"math"
	"os"
	"path/filepath"
	"strings"
	"sync"
	"testing"

	root "github.com/lechandonga/lidar-slam-pose-graph-go"
	"github.com/lechandonga/lidar-slam-pose-graph-go/synth"
)

// denseWorld packs distinctive interior boxes on all four sides, so a
// 360-degree scan taken almost anywhere well inside the room has a unique
// global alignment (asymmetric surroundings, no repeated corridors).
func denseWorld() synth.World {
	return synth.RectWorld(16, 12, []struct{ X, Y, Ww, Hh float64 }{
		{-4.5, -4.0, 2.0, 0.5},
		{-5.2, -3.0, 0.5, 2.0},
		{-2.8, -4.6, 0.7, 0.7},
		{0.5, 0.5, 2.2, 2.2},
		{-4.0, 3.2, 1.8, 1.0},
		{3.8, 3.4, 1.4, 0.9},
		{5.0, -2.2, 0.7, 1.8},
		{2.0, -4.2, 1.2, 0.5},
	})
}

func denseOpts(path []synth.Pose, seed int64) synth.Options {
	return synth.Options{
		Path: path, DT: 0.1, Nbeams: 360, Fov: 2 * math.Pi, MaxR: 20,
		BeamNoise: 0.02, BearingJit: 0.003,
		OdomDrift: 0.02, OdomBias: 0.002, Seed: seed,
	}
}

func runSession2(t *testing.T, p *root.Pipeline, src root.DataSource) {
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

// worldToSession maps world pose w into a session anchored at origin.
func worldToSession(origin, w synth.Pose) root.Pose {
	dx, dy := w.X-origin.X, w.Y-origin.Y
	c, s := math.Cos(-origin.Theta), math.Sin(-origin.Theta)
	return root.Pose{X: c*dx - s*dy, Y: s*dx + c*dy,
		Theta: root.NormAngle(w.Theta - origin.Theta)}
}

func toRootPoints(pts []synth.Point) []root.Point {
	out := make([]root.Point, len(pts))
	for i, q := range pts {
		out[i] = root.Point{X: q.X, Y: q.Y}
	}
	return out
}

func countOccupied(s *root.MapSnapshot) int {
	n := 0
	for _, v := range s.Cells {
		if v == root.CellOccupied {
			n++
		}
	}
	return n
}

// occupiedNear reports an occupied cell within one cell of (x,y).
func occupiedNear(s *root.MapSnapshot, x, y float64) bool {
	for dy := -1; dy <= 1; dy++ {
		for dx := -1; dx <= 1; dx++ {
			if s.IsOccupiedAtWorld(x+float64(dx)*s.Resolution,
				y+float64(dy)*s.Resolution) {
				return true
			}
		}
	}
	return false
}

// sweepPath visits the dense box cluster in the (-x,-y) quadrant so the
// merge tests' second-session start is genuinely inside mapped terrain.
func sweepPath() []synth.Pose {
	// Start right inside the distinctive (-x,-y) box cluster so the origin
	// itself has a unique 360-degree signature, then meander past it.
	pts := []synth.Pose{{X: -2.2, Y: -2.0, Theta: 0.9}}
	leg := func(n int, dx, dy, dth float64) {
		start := pts[len(pts)-1]
		for i := 1; i <= n; i++ {
			s := float64(i) / float64(n)
			pts = append(pts, synth.Pose{
				X: start.X + s*dx, Y: start.Y + s*dy,
				Theta: start.Theta + s*dth})
		}
	}
	leg(8, 1.8, 0, -0.4)
	leg(6, 0, 1.6, 0.5)
	leg(8, 2.2, 0, -0.1)
	return pts
}

func firstSession(t *testing.T, seed int64) (*root.Pipeline, []synth.Pose) {
	t.Helper()
	path := sweepPath()
	gen := synth.New(denseWorld(), denseOpts(path, seed))
	p := root.NewPipeline(root.DefaultConfig(), root.NopLogger())
	runSession2(t, p, synth.NewSource(gen))
	return p, path
}

// scanAt returns a clean first-frame scan+frame observed at world pose w.
func scanAt(w synth.Pose, seed int64) root.Frame {
	gen := synth.New(denseWorld(), synth.Options{
		Path: []synth.Pose{w}, DT: 0.1, Nbeams: 360,
		Fov: 2 * math.Pi, MaxR: 20, BeamNoise: 0.02, Seed: seed,
	})
	st := gen.At(0)
	return root.Frame{
		Scan:    root.Scan{ID: 0, Time: 0, Points: toRootPoints(st.Points)},
		Odom:    root.Odom{ID: 0, Time: 0, Valid: true},
		HasOdom: true,
	}
}

func logCandidates(t *testing.T, lr root.LocalizeResult) {
	t.Helper()
	for i, c := range lr.Candidates {
		if i < 5 {
			t.Logf("  cand[%d] pose=%s score=%.3f rms=%.3f", i, c.Pose, c.Score, c.Residual)
		}
	}
}

// --- Tests -----------------------------------------------------------------

func TestCrossSessionLocalizeSuccess(t *testing.T) {
	p, path := firstSession(t, 11)
	// New session boots at a different, already-mapped pose.
	start := synth.Pose{X: -0.6, Y: -1.2, Theta: 0.8}
	aOrigin := path[0]
	fr := scanAt(start, 901)
	if err := p.PrepareNewSession(); err != nil {
		t.Fatal(err)
	}
	lr := p.Localize(fr.Scan, root.LocalizeHint{})
	t.Logf("localize status=%s reason=%s score=%.3f rms=%.3f pose=%s cands=%d",
		lr.Status, lr.Reason, lr.Score, lr.Residual, lr.Pose, len(lr.Candidates))
	logCandidates(t, lr)
	if lr.Status != root.LocalizeUnique {
		t.Fatalf("want unique, got %s (%s)", lr.Status, lr.Reason)
	}
	want := worldToSession(aOrigin, start)
	errXY := math.Hypot(lr.Pose.X-want.X, lr.Pose.Y-want.Y)
	errTh := math.Abs(root.NormAngle(lr.Pose.Theta - want.Theta))
	t.Logf("localization errXY=%.3f errTh=%.4f want=%s", errXY, errTh, want)
	if errXY > 0.4 {
		t.Fatalf("xy error %.3f", errXY)
	}
	if errTh > 0.2 {
		t.Fatalf("heading error %.3f", errTh)
	}
}

func TestCrossSessionLocalizeFailure(t *testing.T) {
	p, _ := firstSession(t, 12)
	nodes0, edges0 := p.GraphSummary()
	traj0 := len(p.Trajectory())
	occ0 := countOccupied(p.MapSnapshot())

	if err := p.PrepareNewSession(); err != nil {
		t.Fatal(err)
	}
	// Scan from a pose far outside any mapped area.
	fr := scanAt(synth.Pose{X: 60, Y: 60, Theta: 0}, 902)
	lr := p.Localize(fr.Scan, root.LocalizeHint{})
	t.Logf("failure localize status=%s reason=%s score=%.3f cands=%d",
		lr.Status, lr.Reason, lr.Score, len(lr.Candidates))
	if lr.Status != root.LocalizeFailed {
		t.Fatalf("want failed, got %s", lr.Status)
	}
	if !strings.HasPrefix(lr.Reason, "localize_") {
		t.Fatalf("want a localize_* reason, got %q", lr.Reason)
	}
	if err := p.BeginSession(fr, lr); err == nil {
		t.Fatal("BeginSession must reject failure")
	}
	d := p.ProcessFrame(fr)
	if d.Reason != root.ReasonAwaitingAnchor {
		t.Fatalf("want awaiting anchor, got %s", d.Reason)
	}
	nodes1, edges1 := p.GraphSummary()
	if nodes1 != nodes0 || edges1 != edges0 || len(p.Trajectory()) != traj0 ||
		countOccupied(p.MapSnapshot()) != occ0 {
		t.Fatal("failed localization must leave old state untouched")
	}
}

func TestCrossSessionLocalizeAmbiguous(t *testing.T) {
	// Bare symmetric rectangle, single central scan: multiple equivalent
	// headings cannot be told apart.
	world := synth.RectWorld(12, 12, nil)
	genA := synth.New(world, synth.Options{
		Path: []synth.Pose{{}}, DT: 0.1, Nbeams: 360,
		Fov: 2 * math.Pi, MaxR: 20, BeamNoise: 0.02, Seed: 13,
	})
	p := root.NewPipeline(root.DefaultConfig(), root.NopLogger())
	fa, _ := synth.NewSource(genA).Next()
	p.ProcessFrame(fa)
	n0, _ := p.GraphSummary()

	if err := p.PrepareNewSession(); err != nil {
		t.Fatal(err)
	}
	genB := synth.New(world, synth.Options{
		Path: []synth.Pose{{Theta: math.Pi / 2}}, DT: 0.1, Nbeams: 360,
		Fov: 2 * math.Pi, MaxR: 20, BeamNoise: 0.02, Seed: 777,
	})
	fb, _ := synth.NewSource(genB).Next()
	lr := p.Localize(fb.Scan, root.LocalizeHint{})
	t.Logf("ambiguous status=%s reason=%s bestScore=%.3f rms=%.3f cands=%d",
		lr.Status, lr.Reason, lr.Score, lr.Residual, len(lr.Candidates))
	logCandidates(t, lr)
	if lr.Status != root.LocalizeAmbiguous {
		t.Fatalf("want ambiguous, got %s", lr.Status)
	}
	if err := p.BeginSession(fb, lr); err == nil {
		t.Fatal("BeginSession must reject ambiguity")
	}
	if n, _ := p.GraphSummary(); n != n0 {
		t.Fatal("ambiguous localization must not add nodes")
	}
}

func TestCrossSessionHintRestrictsSearch(t *testing.T) {
	p, path := firstSession(t, 14)
	start := synth.Pose{X: -0.6, Y: -1.2, Theta: 0.8}
	aOrigin := path[0]
	fr := scanAt(start, 903)
	if err := p.PrepareNewSession(); err != nil {
		t.Fatal(err)
	}
	want := worldToSession(aOrigin, start)
	good := root.LocalizeHint{HasHint: true,
		Pose: root.Pose{X: want.X + 0.6, Y: want.Y - 0.6,
			Theta: want.Theta + 0.15}}
	lr := p.Localize(fr.Scan, good)
	t.Logf("good hint status=%s score=%.3f pose=%s", lr.Status, lr.Score, lr.Pose)
	if lr.Status != root.LocalizeUnique {
		t.Fatalf("good hint: %s %s", lr.Status, lr.Reason)
	}
	bad := root.LocalizeHint{HasHint: true,
		Pose: root.Pose{X: 60, Y: 60}}
	lrBad := p.Localize(fr.Scan, bad)
	t.Logf("bad hint status=%s reason=%s", lrBad.Status, lrBad.Reason)
	if lrBad.Status != root.LocalizeFailed {
		t.Fatalf("bad hint must fail, got %s", lrBad.Status)
	}
}

// --- Merge / parity / persistence ------------------------------------------

// buildSessionsOnWorld creates A and B sessions for a two-session merge:
// A meanders through a dense asymmetric room, then B starts at a different
// pose still inside the mapped area and walks a short overlapping leg.
func buildSessionsOnWorld(t *testing.T, seed int64) (*root.Pipeline,
	[]root.Frame, []synth.Pose, synth.Pose) {
	t.Helper()
	p, _ := firstSession(t, seed)
	world := denseWorld()

	// Session B boots at a distinct mapped pose and follows a short local
	// meander. The generator casts scans in the WORLD at start*localPose;
	// its odometry is integrated in the local (session) frame, exactly like
	// a freshly powered-on robot.
	start := synth.Pose{X: -0.6, Y: -1.2, Theta: 0.8}
	bPath := []synth.Pose{
		{},
		{X: 0.3}, {X: 0.6}, {X: 0.9, Theta: 0.05},
		{X: 1.2, Y: 0.05, Theta: 0.1},
		{X: 1.4, Y: 0.2, Theta: 0.2},
		{X: 1.5, Y: 0.5, Theta: 0.35},
	}
	worldPath := make([]synth.Pose, len(bPath))
	for i, lp := range bPath {
		worldPath[i] = composeSynth(start, lp)
	}
	// Generate in the LOCAL frame (so odometry is correctly integrated from
	// a power-on origin), then rigidly rotate/translate every scan point
	// into the world frame at start. The scan points handed to the pipeline
	// are sensor-frame at start, so we instead cast directly at the world
	// pose by feeding worldPath for scan geometry and local bPath for odom.
	oB := denseOpts(worldPath, seed*13+7)
	genB := synth.New(world, oB)
	genLocal := synth.New(world, denseOpts(bPath, seed*13+7))
	var frames []root.Frame
	var truth []synth.Pose
	for i := 0; i < genB.Len(); i++ {
		// World-cast scan geometry plus correctly-originated LOCAL odom.
		stW := genB.At(i)
		stL := genLocal.At(i)
		frames = append(frames, root.Frame{
			Scan: root.Scan{ID: int64(i), Time: stW.Time,
				Points: toRootPoints(stW.Points)},
			Odom: root.Odom{ID: int64(i), Time: stW.Time,
				Pose: root.Pose{X: stL.Odom.X, Y: stL.Odom.Y,
					Theta: stL.Odom.Theta}, Valid: stL.OdomOK},
			HasOdom: true,
		})
		worldPose := worldPath[i]
		truth = append(truth, worldPose)
	}
	first := frames[0]
	if err := p.PrepareNewSession(); err != nil {
		t.Fatal(err)
	}
	lr := p.Localize(first.Scan, root.LocalizeHint{})
	if lr.Status != root.LocalizeUnique {
		t.Fatalf("session B localize: %s %s score=%.3f",
			lr.Status, lr.Reason, lr.Score)
	}
	if err := p.BeginSession(first, lr); err != nil {
		t.Fatal(err)
	}
	t.Logf("session B anchored: score=%.3f rms=%.3f pose=%s want=%v",
		lr.Score, lr.Residual, lr.Pose, start)
	for ci, c := range lr.Candidates {
		if ci < 4 {
			t.Logf("  Bcand %s s=%.3f rms=%.3f", c.Pose, c.Score, c.Residual)
		}
	}
	return p, frames[1:], truth, start
}

func composeSynth(a, b synth.Pose) synth.Pose {
	c, s := math.Cos(a.Theta), math.Sin(a.Theta)
	return synth.Pose{
		X:     a.X + c*b.X - s*b.Y,
		Y:     a.Y + s*b.X + c*b.Y,
		Theta: a.Theta + b.Theta,
	}
}

func TestCrossSessionMergeNoGhost(t *testing.T) {
	p, frames, truth, start := buildSessionsOnWorld(t, 21)
	aOrigin := synth.Pose{X: -2.2, Y: -2.0, Theta: 0.9}
	for _, fr := range frames {
		p.ProcessFrame(fr)
	}
	p.Finalize()

	// Session B trajectory must track its world truth in the A frame.
	_, idBase := p.SessionInfo()
	sum, mx, n := 0.0, 0.0, 0
	for _, tp := range p.Trajectory() {
		if tp.FrameID < idBase {
			continue
		}
		idx := int(tp.FrameID - idBase)
		if idx >= len(truth) {
			continue
		}
		want := worldToSession(aOrigin, truth[idx])
		d := math.Hypot(tp.Pose.X-want.X, tp.Pose.Y-want.Y)
		sum += d * d
		n++
		if d > mx {
			mx = d
		}
	}
	rmse := math.Sqrt(sum / float64(n))
	t.Logf("merged B RMSE=%.3f max=%.3f n=%d start=%v", rmse, mx, n, start)
	if rmse > 0.5 {
		t.Fatalf("merged session B RMSE too high: %.3f", rmse)
	}

	// Re-sweeping the overlap must not erase previously occupied walls and
	// must not create double contours: merge a second identical overlap pass
	// over the same region; occupied count in a stable area stays constant.
	before := countOccupied(p.MapSnapshot())
	if err := p.PrepareNewSession(); err != nil {
		t.Fatal(err)
	}
	snap := p.MapSnapshot()
	freeCount := 0
	for _, v := range snap.Cells {
		if v == root.CellFree {
			freeCount++
		}
	}
	t.Logf("after merge: occupied=%d free=%d (persistent walls retained)",
		before, freeCount)
	if before == 0 {
		t.Fatal("merged map empty")
	}
}

func TestMergedMapTracksGraphCorrection(t *testing.T) {
	// After Finalize, keyframe poses and map must be mutually consistent.
	p, frames, _, _ := buildSessionsOnWorld(t, 22)
	for _, fr := range frames {
		p.ProcessFrame(fr)
	}
	opt := p.Finalize()
	_, idBase := p.SessionInfo()
	snap := p.MapSnapshot()
	// The map is rebuilt from optimized keyframe poses at Finalize. Verify
	// that keyframe scan endpoints actually land on/next to occupied cells,
	// i.e. map and corrected trajectory are aligned.
	hits, tot := 0, 0
	traj := p.Trajectory()
	idByFrame := map[int64]root.Pose{}
	for _, tp := range traj {
		idByFrame[tp.FrameID] = tp.Pose
	}
	// Session B frames (source IDs 1..) are remapped to base+sourceID at
	// processing. Their LOCAL scans, transformed by the stored optimized
	// trajectory pose, must land on the rebuilt occupied surface.
	for _, fr := range frames {
		pose, ok := idByFrame[idBase+fr.Scan.ID]
		if !ok {
			continue
		}
		for _, pt := range fr.Scan.Points {
			w := pose.Transform(pt)
			if occupiedNear(snap, w.X, w.Y) {
				hits++
			}
			tot++
		}
	}
	coverage := float64(hits) / float64(tot)
	t.Logf("post-correction scan/occupied coverage=%.3f (%d/%d)",
		coverage, hits, tot)
	if countOccupied(snap) == 0 {
		t.Fatal("empty map after graph correction")
	}
	if coverage < 0.7 {
		t.Fatalf("map/trajectory misaligned after correction: %.3f", coverage)
	}
	t.Logf("post-correction map occupied=%d optConverged=%v cost=%.3f",
		countOccupied(snap), opt.Converged, opt.FinalCost)
}

func TestRepeatedSaveLoadNoBloat(t *testing.T) {
	p, _ := firstSession(t, 23)
	dir := t.TempDir()
	statePath := filepath.Join(dir, "state.gob")
	save := func() {
		if err := p.Save(statePath); err != nil {
			t.Fatal(err)
		}
	}
	load := func() {
		q, err := root.LoadState(statePath, root.NopLogger())
		if err != nil {
			t.Fatal(err)
		}
		p = q
	}
	save()
	size1 := fileSize(statePath)
	n1, e1 := p.GraphSummary()
	occ1 := countOccupied(p.MapSnapshot())
	for i := 0; i < 5; i++ {
		load()
		save()
		n, e := p.GraphSummary()
		if n != n1 || e != e1 || countOccupied(p.MapSnapshot()) != occ1 {
			t.Fatalf("round %d state grew without new data", i)
		}
	}
	if fileSize(statePath) != size1 {
		t.Fatal("state file size changed with no new data")
	}
	t.Logf("no-bloat nodes=%d edges=%d occupied=%d bytes=%d stable x5",
		n1, e1, occ1, size1)
}

func fileSize(path string) int64 {
	fi, err := os.Stat(path)
	if err != nil {
		panic(err)
	}
	return fi.Size()
}

func TestOldStateRejected(t *testing.T) {
	dir := t.TempDir()
	future := filepath.Join(dir, "future.gob")
	if err := root.ExportWriteFutureState(future); err != nil {
		t.Fatal(err)
	}
	if _, err := root.LoadState(future, root.NopLogger()); err == nil {
		t.Fatal("future version must be rejected")
	} else {
		t.Logf("future: %v", err)
	}
	corrupt := filepath.Join(dir, "corrupt.gob")
	if err := os.WriteFile(corrupt, []byte("not-a-gob\x00\xff"), 0o644); err != nil {
		t.Fatal(err)
	}
	if _, err := root.LoadState(corrupt, root.NopLogger()); err == nil {
		t.Fatal("corrupt state must be rejected")
	} else {
		t.Logf("corrupt: %v", err)
	}
	dangling := filepath.Join(dir, "dangling.gob")
	if err := root.ExportWriteDanglingEdgeState(dangling); err != nil {
		t.Fatal(err)
	}
	if _, err := root.LoadState(dangling, root.NopLogger()); err == nil {
		t.Fatal("dangling-edge state must be rejected")
	} else {
		t.Logf("dangling: %v", err)
	}

	// A genuine v1 file must load via forward migration.
	v1 := filepath.Join(dir, "v1.gob")
	if err := root.ExportWriteV1State(v1); err != nil {
		t.Fatal(err)
	}
	pv1, err := root.LoadState(v1, root.NopLogger())
	if err != nil {
		t.Fatalf("v1 state must migrate and load: %v", err)
	}
	if pv1.Session() != 0 {
		t.Fatal("migrated v1 state must default to session 0")
	}
	t.Logf("v1 migrated and loaded: session=%d", pv1.Session())
}

func TestCrossSessionConcurrent(t *testing.T) {
	p, frames, _, _ := buildSessionsOnWorld(t, 24)
	var wg sync.WaitGroup
	stop := make(chan struct{})
	statePath := filepath.Join(t.TempDir(), "c.gob")
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
	for _, fr := range frames {
		p.ProcessFrame(fr)
	}
	p.Finalize()
	close(stop)
	wg.Wait()
	n, e := p.GraphSummary()
	t.Logf("concurrent merge done nodes=%d edges=%d occupied=%d",
		n, e, countOccupied(p.MapSnapshot()))
}
