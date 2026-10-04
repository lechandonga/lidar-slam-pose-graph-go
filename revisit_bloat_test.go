package slam_test

import (
	"math"
	"os"
	"path/filepath"
	"sync"
	"testing"

	root "github.com/lechandonga/lidar-slam-pose-graph-go"
	"github.com/lechandonga/lidar-slam-pose-graph-go/synth"
)

func TestRepeatedLoadRunSaveConverges(t *testing.T) {
	world := synth.RectWorld(16, 12, []struct{ X, Y, Ww, Hh float64 }{
		{-4.5, -4.0, 2.0, 0.5}, {0.5, 0.5, 2.2, 2.2},
		{3.8, 3.4, 1.4, 0.9}, {5.0, -2.2, 0.7, 1.8},
	})
	// A maps a route starting near (-3,-2).
	aPath := []synth.Pose{
		{-3.0, -2.0, 0.2}, {-2.5, -1.9, 0.2}, {-2.0, -1.7, 0.2},
		{-1.5, -1.5, 0.2}, {-1.0, -1.4, 0.15}, {-0.5, -1.3, 0.1},
		{0.0, -1.2, 0.05}, {0.5, -1.1, 0.0},
	}
	opts := func(seed int64) synth.Options {
		return synth.Options{DT: 0.1, Nbeams: 360, Fov: 2 * math.Pi,
			MaxR: 20, BeamNoise: 0.02, BearingJit: 0.003, Seed: seed}
	}
	pa := root.NewPipeline(root.DefaultConfig(), root.NopLogger())
	oA := opts(11)
	oA.Path = aPath
	runSession2(t, pa, synth.NewSource(synth.New(world, oA)))
	statePath := filepath.Join(t.TempDir(), "site.gob")
	if err := pa.Save(statePath); err != nil {
		t.Fatal(err)
	}

	// B boots at A's first pose and walks the SAME leg in its own local
	// frame (odom/clock zeroed). Scans are cast at world poses via ScanPath.
	origin := aPath[0]
	bLocal := []synth.Pose{
		{}, {X: 0.5}, {X: 1.0}, {X: 1.5}, {X: 2.0},
		{X: 2.5}, {X: 3.0}, {X: 3.5},
	}
	bWorld := make([]synth.Pose, len(bLocal))
	for i, lp := range bLocal {
		bWorld[i] = composeSynth(origin, lp)
	}
	buildB := func(seed int64) []root.Frame {
		oWorld := opts(seed)
		oWorld.Path = bWorld
		oWorld.ScanPath = bWorld
		oLocal := opts(seed)
		oLocal.Path = bLocal
		gW := synth.New(world, oWorld)
		gL := synth.New(world, oLocal)
		frames := make([]root.Frame, gW.Len())
		for i := 0; i < gW.Len(); i++ {
			stW, stL := gW.At(i), gL.At(i)
			frames[i] = root.Frame{
				Scan: root.Scan{ID: int64(i), Time: stW.Time,
					Points: toRootPoints(stW.Points)},
				Odom: root.Odom{ID: int64(i), Time: stL.Time,
					Pose: root.Pose{X: stL.Odom.X, Y: stL.Odom.Y,
						Theta: stL.Odom.Theta}, Valid: stL.OdomOK},
				HasOdom: true,
			}
		}
		return frames
	}

	n0, e0 := pa.GraphSummary()
	t.Logf("A map nodes=%d edges=%d occ=%d", n0, e0, countOccupied(pa.MapSnapshot()))
	occ0 := countOccupied(pa.MapSnapshot())
	var sizes []int64
	// A's first frame is the pipeline world origin, so B's boot pose maps
	// to (0,0,0) in A's world frame, not the generator's numeric origin.
	anchorErr := func(pose root.Pose) float64 { return math.Hypot(pose.X, pose.Y) }
	for round := 0; round < 4; round++ {
		p, err := root.LoadState(statePath, root.NopLogger())
		if err != nil {
			t.Fatal(err)
		}
		if err := p.PrepareNewSession(); err != nil {
			t.Fatal(err)
		}
		frames := buildB(int64(900 + round))
		lr := p.Localize(frames[0].Scan, root.LocalizeHint{})
		if lr.Status != root.LocalizeUnique {
			t.Fatalf("round %d localize %s: %s", round, lr.Status, lr.Reason)
		}
		if d := anchorErr(lr.Pose); d > 0.5 {
			t.Fatalf("round %d anchor err %.3fm vs A origin", round, d)
		}
		if err := p.BeginSession(frames[0], lr); err != nil {
			t.Fatal(err)
		}
		for _, fr := range frames[1:] {
			p.ProcessFrame(fr)
		}
		p.Finalize()
		if err := p.Save(statePath); err != nil {
			t.Fatal(err)
		}
		n, e := p.GraphSummary()
		occ := countOccupied(p.MapSnapshot())
		fi, _ := os.Stat(statePath)
		sizes = append(sizes, fi.Size())
		t.Logf("round %d: nodes=%d edges=%d occupied=%d bytes=%d "+
			"localizeScore=%.3f anchorErrXY=%.3f",
			round+1, n, e, occ, fi.Size(), lr.Score,
			anchorErr(lr.Pose))
		// Pure overlap: no new terrain, so graph and map must not grow.
		if n != n0 {
			t.Fatalf("round %d nodes grew %d -> %d on unchanged area", round, n0, n)
		}
		if e != e0 {
			t.Fatalf("round %d edges grew %d -> %d on unchanged area", round, e0, e)
		}
		if occ != occ0 {
			t.Fatalf("round %d occupied grew %d -> %d on unchanged area", round, occ0, occ)
		}
	}
	// File size must be stable, not monotonically climbing.
	for i := 1; i < len(sizes); i++ {
		if d := sizes[i] - sizes[i-1]; d > 64 {
			t.Fatalf("state file grew %d bytes between round %d and %d (%d->%d)",
				d, i, i+1, sizes[i-1], sizes[i])
		}
	}
	t.Logf("converged sizes over 4 rounds: %v (nodes=%d edges=%d)", sizes, n0, e0)
}

// TestNewTerrainStillGrows guards the anti-bloat change in the other
// direction: exploring genuinely NEW area after a revisit must add nodes,
// edges and map cells (suppression must not delete or freeze real growth).
func TestNewTerrainStillGrows(t *testing.T) {
	world := synth.RectWorld(16, 12, []struct{ X, Y, Ww, Hh float64 }{
		{-4.5, -4.0, 2.0, 0.5}, {0.5, 0.5, 2.2, 2.2},
		{3.8, 3.4, 1.4, 0.9},
	})
	aPath := []synth.Pose{{-3, -2, 0.2}, {-2.5, -1.9, 0.2},
		{-2, -1.7, 0.2}, {-1.5, -1.5, 0.2}}
	o := synth.Options{DT: 0.1, Nbeams: 360, Fov: 2 * math.Pi, MaxR: 20,
		BeamNoise: 0.02, Seed: 7}
	o.Path = aPath
	p := root.NewPipeline(root.DefaultConfig(), root.NopLogger())
	runSession2(t, p, synth.NewSource(synth.New(world, o)))
	n0, e0 := p.GraphSummary()
	occ0 := countOccupied(p.MapSnapshot())

	// Continue into brand-new terrain (far north) within the same session
	// would add keyframes; emulate by processing extra frames.
	newPath := []synth.Pose{
		{-1.5, -1.5, 0.2}, {-1.0, -0.5, 0.5}, {-0.5, 0.8, 1.0},
		{0.0, 2.2, 1.4}, {0.5, 3.5, 1.5},
	}
	o2 := synth.Options{DT: 0.1, Nbeams: 360, Fov: 2 * math.Pi, MaxR: 20,
		BeamNoise: 0.02, Seed: 8}
	o2.Path = newPath
	src := synth.NewSource(synth.New(world, o2))
	next, _ := src.Next() // first frame coincides with last A pose-ish
	p.ProcessFrame(next)
	for {
		fr, ok := src.Next()
		if !ok {
			break
		}
		p.ProcessFrame(fr)
	}
	p.Finalize()
	n1, e1 := p.GraphSummary()
	occ1 := countOccupied(p.MapSnapshot())
	t.Logf("new terrain: nodes %d->%d edges %d->%d occupied %d->%d",
		n0, n1, e0, e1, occ0, occ1)
	if n1 <= n0 || e1 <= e0 || occ1 <= occ0 {
		t.Fatal("exploring new terrain must grow the graph and map")
	}
}

// TestResumeConcurrentQueryAndSave exercises map queries and state saves
// racing a continued merge run (cross-session frames being processed and
// Finalize), under -race.
func TestResumeConcurrentQueryAndSave(t *testing.T) {
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
		t.Fatal(lr.Reason)
	}
	if err := loaded.BeginSession(first, lr); err != nil {
		t.Fatal(err)
	}

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
				snap := loaded.MapSnapshot()
				_ = snap.IsOccupiedAtWorld(0, 0)
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
				if err := loaded.Save(filepath.Join(t.TempDir(), "x.gob")); err != nil {
					t.Error(err)
					return
				}
			}
		}
	}()
	for {
		fr, ok := src.Next()
		if !ok {
			break
		}
		loaded.ProcessFrame(fr)
	}
	loaded.Finalize()
	close(stop)
	wg.Wait()
	n, e := loaded.GraphSummary()
	t.Logf("concurrent resume+query+save done nodes=%d edges=%d occupied=%d",
		n, e, countOccupied(loaded.MapSnapshot()))
	// Final saved file must be loadable (no torn write under concurrency).
	final := filepath.Join(t.TempDir(), "final.gob")
	if err := loaded.Save(final); err != nil {
		t.Fatal(err)
	}
	if _, err := root.LoadState(final, root.NopLogger()); err != nil {
		t.Fatalf("concurrently written final state unreadable: %v", err)
	}
}
