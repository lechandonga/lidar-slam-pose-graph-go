package slam_test

import (
	"fmt"
	"math"
	"os"
	"path/filepath"
	"sync"
	"testing"

	root "github.com/lechandonga/lidar-slam-pose-graph-go"
	"github.com/lechandonga/lidar-slam-pose-graph-go/synth"
)

var _ = math.Pi

func TestCrossSessionLocalizeSuccess(t *testing.T) {
	s1, s2, _ := overlapSessions(t, 0.15, 0.55)
	p := root.NewPipeline(root.DefaultConfig(), root.NopLogger())
	consumeFrames(p, s1.frames)
	p.Finalize()

	res := p.BeginSessionLocalize(s2.frames[0], nil)
	for _, h := range res.Hypotheses {
		t.Logf("hyp %s score=%.3f rms=%.3f", h.Pose, h.Score, h.ResidualRMS)
	}
	if res.Status != root.LocalizeUnique {
		t.Fatalf("expected unique fix, got %s (%s)", res.Status, res.Reason)
	}
	if res.Best.Score < 0.78 {
		t.Fatalf("fix score too weak: %.3f", res.Best.Score)
	}
	if res.Best.ResidualRMS > 0.12 {
		t.Fatalf("fix residual too large: %.3f m", res.Best.ResidualRMS)
	}
	if res.ScoreMargin < 0.15 {
		t.Fatalf("fix not well separated: margin=%.3f", res.ScoreMargin)
	}
	if res.AppliedEdge < 0 {
		t.Fatal("no relocation edge anchored")
	}
	t.Logf("LOCALIZE session=1 status=%s reason=%s score=%.3f rms=%.4f m margin=%.3f fix=%s",
		res.Status, res.Reason, res.Best.Score, res.Best.ResidualRMS,
		res.ScoreMargin, res.Applied)

	consumeFrames(p, s2.frames[1:])
	p.Finalize()
	nodes, edges := p.GraphSummary()
	t.Logf("merged graph nodes=%d edges=%d", nodes, edges)

	var s2est []root.Pose
	for _, tp := range p.Trajectory() {
		if tp.Session == 1 {
			s2est = append(s2est, tp.Pose)
		}
	}
	if len(s2est) == 0 {
		t.Fatal("no session-1 trajectory points")
	}
	rmse := alignAndRMSE(s2est, s2.truth)
	t.Logf("s2 first est=%s first truth=%s last est=%s last truth=%s n=%d",
		s2est[0], s2.truth[0], s2est[len(s2est)-1],
		s2.truth[len(s2.truth)-1], len(s2est))
	for i := 0; i < len(s2est) && i < len(s2.truth); i += 12 {
		t.Logf("i=%d est=%s truth=%s", i, s2est[i], s2.truth[i])
	}
	t.Logf("session2 post-localize RMSE=%.3f m", rmse)
	if rmse > 0.8 {
		t.Fatalf("session2 RMSE too high: %.3f", rmse)
	}
}

func TestCrossSessionLocalizeHint(t *testing.T) {
	s1, s2, _ := overlapSessions(t, 0.15, 0.55)
	p := root.NewPipeline(root.DefaultConfig(), root.NopLogger())
	consumeFrames(p, s1.frames)
	p.Finalize()
	// Hints are given in the OLD (drifted) world frame. Take the old
	// session's pose at the overlap index and perturb it.
	startI := 18 // loop index of the overlap start (0.15 of 121 frames)
	var near root.Pose
	tr0 := p.Trajectory()
	want := s2.truth[0]
	near = tr0[startI].Pose
	hint := &root.LocalizeHint{
		Pose: root.Pose{
			X: near.X + 0.6, Y: near.Y - 0.5,
			Theta: root.NormAngle(near.Theta + 0.2),
		},
		HasPose: true, WinXY: 1.5, WinTheta: 0.6,
	}
	res := p.BeginSessionLocalize(s2.frames[0], hint)
	t.Logf("LOCALIZE(hint) status=%s reason=%s score=%.3f rms=%.3f near=%s want~%s",
		res.Status, res.Reason, res.Best.Score, res.Best.ResidualRMS, near, want)
	if res.Status != root.LocalizeUnique {
		t.Fatalf("hint near truth should localize, got %s (%s)",
			res.Status, res.Reason)
	}
}

func TestCrossSessionLocalizeFailure(t *testing.T) {
	w := crossWorld()
	loop := synth.MakeLoopPath(4.0, 24, 6)
	n := len(loop)
	s1 := makeSession(t, w, loop[:int(float64(n)*0.3)], 101)
	// Start on the far top-left side never covered by the short session 1.
	s2 := makeSession(t, w, []synth.Pose{loop[int(float64(n)*0.90)]}, 202)

	p := root.NewPipeline(root.DefaultConfig(), root.NopLogger())
	consumeFrames(p, s1.frames)
	p.Finalize()
	bN, bE := p.GraphSummary()
	bT := len(p.Trajectory())
	bMap := p.MapSnapshot()

	res := p.BeginSessionLocalize(s2.frames[0], nil)
	t.Logf("LOCALIZE(off-map) status=%s reason=%s bestScore=%.3f rms=%.3f margin=%.3f",
		res.Status, res.Reason, res.Best.Score, res.Best.ResidualRMS, res.ScoreMargin)
	if res.Status == root.LocalizeUnique {
		t.Fatalf("must not localize off-map, got %s", res.Applied)
	}
	if res.Status != root.LocalizeFailed {
		t.Fatalf("expected failed, got %s", res.Status)
	}
	aN, aE := p.GraphSummary()
	if aN != bN || aE != bE || len(p.Trajectory()) != bT {
		t.Fatalf("failed localization mutated state: nodes %d->%d edges %d->%d traj %d->%d",
			bN, aN, bE, aE, bT, len(p.Trajectory()))
	}
	same, oa, ob, _ := mapAgreement(bMap, p.MapSnapshot())
	if oa+ob != 0 || same < 0.999 {
		t.Fatalf("failed localization changed the map: occOnlyA=%d occOnlyB=%d", oa, ob)
	}
	// Un-localized frames are rejected, not consumed.
	d := p.ProcessFrame(s2.frames[0])
	if d.Outcome != root.OutcomeRejected || d.Reason != root.ReasonSessionNotLocalized {
		t.Fatalf("un-localized frame: got %s/%s", d.Outcome, d.Reason)
	}
	if len(p.Trajectory()) != bT {
		t.Fatal("rejected frame appended to trajectory")
	}
	// Retry at a genuinely mapped spot succeeds; the old session resumes.
	s3 := makeSession(t, w, []synth.Pose{loop[int(float64(n)*0.15)]}, 303)
	res2 := p.BeginSessionLocalize(s3.frames[0], nil)
	t.Logf("LOCALIZE(retry) status=%s reason=%s score=%.3f",
		res2.Status, res2.Reason, res2.Best.Score)
	if res2.Status != root.LocalizeUnique {
		t.Fatalf("retry on mapped spot should succeed, got %s (%s)",
			res2.Status, res2.Reason)
	}
}

func TestCrossSessionAmbiguous(t *testing.T) {
	// A periodic corridor with repeated identical bays. Starting in one bay
	// admits several equally-scoring alignments at the period spacing ->
	// ambiguous.
	w := periodicCorridorWorld()
	path := corridorPath()
	s1 := makeSession(t, w, path, 101)
	// New session starts in the same kind of bay, but with no cue which one.
	s2 := makeSession(t, w, []synth.Pose{{X: 0, Y: 0, Theta: 0}}, 202)

	p := root.NewPipeline(root.DefaultConfig(), root.NopLogger())
	consumeFrames(p, s1.frames)
	p.Finalize()
	bN, bE := p.GraphSummary()
	res := p.BeginSessionLocalize(s2.frames[0], nil)
	t.Logf("LOCALIZE(periodic) status=%s reason=%s best=%.3f second=%.3f margin=%.3f nhyp=%d",
		res.Status, res.Reason, res.Best.Score, res.Second.Score,
		res.ScoreMargin, len(res.Hypotheses))
	if res.Status != root.LocalizeAmbiguous {
		t.Fatalf("periodic corridor should be ambiguous, got %s (%s)",
			res.Status, res.Reason)
	}
	if res.AppliedEdge >= 0 {
		t.Fatal("ambiguous fix must not be applied")
	}
	aN, aE := p.GraphSummary()
	if aN != bN || aE != bE {
		t.Fatalf("ambiguous localization mutated graph: %d,%d -> %d,%d",
			bN, bE, aN, aE)
	}
}

// periodicCorridorWorld builds a long corridor with repeated identical
// bays (transverse wall stubs) every 4 m.
func periodicCorridorWorld() synth.World {
	var walls []synth.Segment
	// Long corridor walls.
	walls = append(walls,
		synth.Segment{A: synth.Point{X: -20, Y: -3}, B: synth.Point{X: 20, Y: -3}},
		synth.Segment{A: synth.Point{X: -20, Y: 3}, B: synth.Point{X: 20, Y: 3}},
	)
	// Identical transverse stubs at x = -12,-8,-4,0,4,8,12.
	for _, x := range []float64{-12, -8, -4, 0, 4, 8, 12} {
		walls = append(walls,
			synth.Segment{A: synth.Point{X: x, Y: -3}, B: synth.Point{X: x, Y: -1.5}},
			synth.Segment{A: synth.Point{X: x, Y: 1.5}, B: synth.Point{X: x, Y: 3}},
		)
	}
	return synth.World{Walls: walls}
}

func corridorPath() []synth.Pose {
	var path []synth.Pose
	for x := -16.0; x <= 16.0; x += 0.5 {
		path = append(path, synth.Pose{X: x, Y: 0, Theta: 0})
	}
	return path
}

// TestCrossSessionEquivalentToOneShot: split into two independent sessions,
// merge, and compare with the same data consumed in one continuous run.
func TestCrossSessionEquivalentToOneShot(t *testing.T) {
	w := crossWorld()
	loop := synth.MakeLoopPath(4.0, 24, 6)
	n := len(loop)
	cut1 := int(float64(n) * 0.75)
	startI := int(float64(n) * 0.15)

	// Baseline: one continuous run over the whole loop (same seeds per leg
	// used for each session so noise realizations match).
	full1 := makeSession(t, w, loop[:cut1], 101)
	full2 := makeSession(t, w, loop[cut1:], 102)
	base := root.NewPipeline(root.DefaultConfig(), root.NopLogger())
	consumeFrames(base, full1.frames)
	consumeFrames(base, full2.frames)
	base.Finalize()
	baseMap := base.MapSnapshot()

	// Split: session1 then an independently rebooted session2.
	s1 := makeSession(t, w, loop[:cut1], 101)
	p2gen := makeSession(t, w, loop[startI:], 202)
	split := root.NewPipeline(root.DefaultConfig(), root.NopLogger())
	consumeFrames(split, s1.frames)
	split.Finalize()
	res := split.BeginSessionLocalize(p2gen.frames[0], nil)
	if res.Status != root.LocalizeUnique {
		t.Fatalf("split localize: %s (%s)", res.Status, res.Reason)
	}
	t.Logf("split localize score=%.3f rms=%.4f margin=%.3f",
		res.Best.Score, res.Best.ResidualRMS, res.ScoreMargin)
	consumeFrames(split, p2gen.frames[1:])
	split.Finalize()
	splitMap := split.MapSnapshot()
	n2, e2 := split.GraphSummary()
	t.Logf("split final nodes=%d edges=%d", n2, e2)
	var nSess0, nSess1 int
	for _, kf := range root.ExportPipelineNodes(split) {
		if kf.Session == 0 {
			nSess0++
		} else {
			nSess1++
		}
	}
	t.Logf("post-compact session0 nodes=%d session1 nodes=%d", nSess0, nSess1)
	for _, ld := range split.LoopDecisions() {
		t.Logf("LOOP %d->%d %s score=%.2f reason=%s",
			ld.From, ld.To, ld.Status, ld.Score, ld.Reason)
	}
	for _, ld := range split.LocalizeDecisions() {
		t.Logf("LOC session=%d %s %s", ld.Session, ld.Status, ld.Reason)
	}

	// Trajectory agreement in the common region loop[startI:]. Both runs
	// estimate the same physical path from independently anchored frames,
	// so compare after a best rigid SE2 fit (shape agreement, not frame).
	var baseSeg, splitSeg []root.Pose
	for i, tp := range base.Trajectory() {
		if i >= startI {
			baseSeg = append(baseSeg, tp.Pose)
		}
	}
	for _, tp := range split.Trajectory() {
		if tp.Session == 1 {
			splitSeg = append(splitSeg, tp.Pose)
		}
	}
	shapeRMSE := rmseRigid(splitSeg, baseSeg)
	t.Logf("one-shot vs split rigid-shape RMSE=%.3f m (base=%d split=%d pts)",
		shapeRMSE, len(baseSeg), len(splitSeg))
	if shapeRMSE > 0.5 {
		t.Fatalf("split trajectory shape diverges from one-shot baseline: %.3f m",
			shapeRMSE)
	}
	same, occA, occB, compared := mapAgreement(baseMap, splitMap)
	t.Logf("map agreement same=%.4f occOnlyBase=%d occOnlySplit=%d compared=%d",
		same, occA, occB, compared)
	bt := base.Trajectory()
	// The decisive metric is occupied-cell agreement (wall contours).
	// Free/unknown differences at scan-shadow boundaries are expected
	// across two independently noisy runs; doubled walls are not.
	// Restrict the contour comparison to the OVERLAP region: ground-truth
	// loop indices [startI:cut1], expressed in each run's own map frame
	// via its trajectory bounds there.
	cut1i := cut1
	var minX, minY, maxX, maxY float64
	first := true
	for i, tp := range bt {
		if i < startI || i > cut1i {
			continue
		}
		q := tp.Pose
		if first {
			minX, minY, maxX, maxY = q.X, q.Y, q.X, q.Y
			first = false
		} else {
			minX = math.Min(minX, q.X)
			minY = math.Min(minY, q.Y)
			maxX = math.Max(maxX, q.X)
			maxY = math.Max(maxY, q.Y)
		}
	}
	pad := 4.0
	prec, rec, gap := occupiedAgreementRegion(baseMap, splitMap,
		minX-pad, minY-pad, maxX+pad, maxY+pad, 5)
	t.Logf("OVERLAP occupied precision=%.4f recall=%.4f maxGap=%.3f m box=[%.1f,%.1f,%.1f,%.1f]",
		prec, rec, gap, minX, minY, maxX, maxY)
	if prec < 0.90 || rec < 0.90 {
		t.Fatalf("overlap contours diverge from baseline: prec=%.3f rec=%.3f",
			prec, rec)
	}
}

// TestRepeatedSaveLoadNoBloat exercises many save->load->run-a-little cycles
// re-scanning the same overlap. Keyframes/edges/grid must not grow round over
// round once the area is fully mapped.
func TestRepeatedSaveLoadNoBloat(t *testing.T) {
	w := crossWorld()
	loop := synth.MakeLoopPath(4.0, 24, 6)
	s1 := makeSession(t, w, loop[:len(loop)/2], 101)
	p := root.NewPipeline(root.DefaultConfig(), root.NopLogger())
	consumeFrames(p, s1.frames)
	p.Finalize()
	dir := t.TempDir()
	statePath := filepath.Join(dir, "state.gob")
	if err := p.Save(statePath); err != nil {
		t.Fatal(err)
	}
	// Repeatedly reload, re-localize over the same overlap, run a few
	// frames, and save again.
	overlap := makeSession(t, w, loop[:20], 202)
	var sizes []int64
	var counts [][3]int
	for round := 0; round < 4; round++ {
		q, err := root.LoadState(statePath, root.NopLogger())
		if err != nil {
			t.Fatalf("round %d load: %v", round, err)
		}
		lr := q.BeginSessionLocalize(overlap.frames[0], nil)
		if lr.Status != root.LocalizeUnique {
			t.Fatalf("round %d localize: %s (%s)", round, lr.Status, lr.Reason)
		}
		consumeFrames(q, overlap.frames[1:15])
		if err := q.Save(statePath); err != nil {
			t.Fatal(err)
		}
		fi, _ := os.Stat(statePath)
		n, e := q.GraphSummary()
		sizes = append(sizes, fi.Size())
		counts = append(counts, [3]int{n, e, q.MapSnapshot().W * q.MapSnapshot().H})
	}
	for i, s := range sizes {
		t.Logf("round %d file=%d bytes nodes=%d edges=%d cells=%d",
			i, s, counts[i][0], counts[i][1], counts[i][2])
	}
	// The last two rounds must be essentially identical: the re-scanned
	// overlap compacts into existing keyframes and adds no new state.
	if sizes[len(sizes)-1] > sizes[len(sizes)-2]+256 {
		t.Fatalf("state file keeps growing: %d -> %d bytes",
			sizes[len(sizes)-2], sizes[len(sizes)-1])
	}
	if counts[len(counts)-1][0] > counts[len(counts)-2][0] {
		t.Fatalf("keyframes grow on repeat save/load: %d -> %d",
			counts[len(counts)-2][0], counts[len(counts)-1][0])
	}
}

// TestStateVersionCompat: v1 files migrate, corrupt/unknown versions are
// rejected with a clear error and never produce a half-loaded pipeline.
func TestStateVersionCompat(t *testing.T) {
	dir := t.TempDir()
	// A current-version file loads cleanly.
	w := crossWorld()
	s1 := makeSession(t, w, synth.MakeLoopPath(4.0, 24, 6)[:40], 101)
	p := root.NewPipeline(root.DefaultConfig(), root.NopLogger())
	consumeFrames(p, s1.frames)
	p.Finalize()
	good := filepath.Join(dir, "good.gob")
	if err := p.Save(good); err != nil {
		t.Fatal(err)
	}
	if _, err := root.LoadState(good, root.NopLogger()); err != nil {
		t.Fatalf("current state should load: %v", err)
	}

	// Hand-build a v1-ish file with the same gob layout but Version=1. It
	// must be accepted via migration (safe zero session fields).
	v1path := filepath.Join(dir, "v1.gob")
	if err := root.ExportWriteStateFileVersion(v1path, 1); err != nil {
		t.Fatal(err)
	}
	pv1, err := root.LoadState(v1path, root.NopLogger())
	if err != nil {
		t.Fatalf("v1 state should migrate: %v", err)
	}
	n, _ := pv1.GraphSummary()
	if n == 0 {
		t.Fatal("migrated v1 state lost keyframes")
	}
	if pv1.SessionID() != 0 {
		t.Fatalf("migrated v1 session should be 0, got %d", pv1.SessionID())
	}

	// A future/unknown version must be rejected explicitly.
	bad := filepath.Join(dir, "future.gob")
	if err := os.WriteFile(bad, []byte("garbage-not-gob"), 0o644); err != nil {
		t.Fatal(err)
	}
	if _, err := root.LoadState(bad, root.NopLogger()); err == nil {
		t.Fatal("corrupt file must be rejected")
	} else {
		t.Logf("corrupt file rejected: %v", err)
	}
	fut := filepath.Join(dir, "future.gob2")
	if err := root.ExportWriteStateFileVersion(fut, 999); err != nil {
		t.Fatal(err)
	}
	if _, err := root.LoadState(fut, root.NopLogger()); err == nil {
		t.Fatal("future version must be rejected")
	} else {
		t.Logf("future version rejected: %v", err)
	}
}

// TestConcurrentSessionSaveAndQuery runs localization/merging while
// goroutines hammer map queries and saves; passes cleanly under -race.
func TestConcurrentSessionSaveAndQuery(t *testing.T) {
	s1, s2, _ := overlapSessions(t, 0.15, 0.55)
	p := root.NewPipeline(root.DefaultConfig(), root.NopLogger())
	consumeFrames(p, s1.frames)
	p.Finalize()
	dir := t.TempDir()
	var wg sync.WaitGroup
	stop := make(chan struct{})
	for g := 0; g < 3; g++ {
		wg.Add(1)
		go func(g int) {
			defer wg.Done()
			i := 0
			for {
				select {
				case <-stop:
					return
				default:
				}
				snap := p.MapSnapshot()
				_ = snap.IsOccupiedAtWorld(0, 0)
				if i%7 == 0 {
					_ = p.Save(filepath.Join(dir,
						fmt.Sprintf("s%d-%d.gob", g, i)))
				}
				i++
			}
		}(g)
	}
	res := p.BeginSessionLocalize(s2.frames[0], nil)
	if res.Status != root.LocalizeUnique {
		t.Fatalf("localize under concurrency: %s (%s)", res.Status, res.Reason)
	}
	consumeFrames(p, s2.frames[1:])
	p.Finalize()
	close(stop)
	wg.Wait()
}
