package slam

import (
	"encoding/gob"
	"errors"
	"fmt"
	"io"
	"math"
	"os"
	"path/filepath"
)

// ErrUnsupportedState is returned when a state file declares a version this
// build cannot read.
var ErrUnsupportedState = errors.New("slam: unsupported state file version")

// stateVersion must be bumped when the on-disk layout changes.
//
//	v1: initial layout (no Session / FrameIDBase / relocation state);
//	v2: multi-session layout. v1 files are migrated on load.
const (
	stateVersion    = 2
	stateVersionV1  = 1
	minStateVersion = 1
)

// savedState is the complete resumable state. Everything needed to continue
// a run identically is captured here.
type savedState struct {
	Version int
	Config  Config

	Nodes []*Keyframe
	Edges []*Edge
	Traj  []TrajectoryPoint

	FrameCount int64
	LastPose   Pose
	LastOdom   Pose
	OdomBase   Pose
	Started    bool
	LastKF     int
	LastKFOdom Pose
	LastScan   []Point

	Decisions []FrameDecision
	LoopLog   []LoopDecision
	OptLog    []OptLogEntry

	GridRes    float64
	GridOX     float64
	GridOY     float64
	GridW      int
	GridH      int
	GridValues []float64

	// v2 multi-session bookkeeping.
	Session     int
	FrameIDBase int64
	AwaitAnchor bool
}

func (p *Pipeline) snapshotStateLocked() savedState {
	g := p.grid
	vals := make([]float64, len(g.logOdds))
	copy(vals, g.logOdds)
	gridRes, ox, oy := g.res, g.ox, g.oy
	gw, gh := g.w, g.h

	st := savedState{
		Version: stateVersion, Config: p.cfg,
		Nodes: p.graph.Nodes, Edges: p.graph.Edges,
		Traj:       p.traj,
		FrameCount: p.frameCount,
		LastPose:   p.lastPose, LastOdom: p.lastOdom,
		OdomBase: p.odomBase,
		Started:  p.started, LastKF: p.lastKF, LastKFOdom: p.lastKFOdom,
		LastScan:  p.lastScan,
		Decisions: p.decisions, LoopLog: p.loopLog, OptLog: p.optLog,
		GridRes: gridRes, GridOX: ox, GridOY: oy,
		GridW: gw, GridH: gh, GridValues: vals,
		Session: p.session, FrameIDBase: p.frameIDBase,
		AwaitAnchor: p.awaitAnchor,
	}
	st.Decisions = capTail(st.Decisions, p.cfg.PersistedLogCap)
	st.LoopLog = capTail(st.LoopLog, p.cfg.PersistedLogCap)
	st.OptLog = capTail(st.OptLog, p.cfg.PersistedLogCap)
	return st
}

// capTail keeps at most cap elements (the most recent) of s. A non-positive
// cap means unbounded.
func capTail[S ~[]E, E any](s S, capn int) S {
	if capn <= 0 || len(s) <= capn {
		return s
	}
	return s[len(s)-capn:]
}

// Save atomically writes the full state to path. It writes to a sibling
// temporary file and renames it into place, so a crash mid-save can never
// leave a half-written trajectory or map.
func (p *Pipeline) Save(path string) error {
	p.mu.Lock()
	st := p.snapshotStateLocked()
	// Deep-copy every mutable slice while still holding the lock: Nodes
	// carry pointer Keyframes whose poses the optimizer mutates, and Edges
	// are pointers too. Encoding outside the lock over shallow copies would
	// race a concurrent Finalize/ProcessFrame. Gob encoding itself needs no
	// lock, but it must read only this private snapshot.
	st = p.deepCopyStateLocked(st)
	p.mu.Unlock()
	return writeStateAtomic(path, st)
}

func (p *Pipeline) deepCopyStateLocked(st savedState) savedState {
	nodes := make([]*Keyframe, len(st.Nodes))
	for i, kf := range st.Nodes {
		k := *kf
		k.Scan = append([]Point(nil), kf.Scan...)
		k.Surface = append([]Point(nil), kf.Surface...)
		nodes[i] = &k
	}
	st.Nodes = nodes
	edges := make([]*Edge, len(st.Edges))
	for i, e := range st.Edges {
		ee := *e
		edges[i] = &ee
	}
	st.Edges = edges
	st.Traj = append([]TrajectoryPoint(nil), st.Traj...)
	st.LastScan = append([]Point(nil), st.LastScan...)
	st.Decisions = append([]FrameDecision(nil), st.Decisions...)
	st.LoopLog = append([]LoopDecision(nil), st.LoopLog...)
	st.OptLog = append([]OptLogEntry(nil), st.OptLog...)
	return st
}

func writeStateAtomic(path string, st savedState) (err error) {
	if err := st.checkFinite(); err != nil {
		return fmt.Errorf("slam: refusing to persist non-finite state: %w", err)
	}
	return writeGobAtomic(path, &st)
}

// writeGobAtomic gob-encodes v to a sibling temp file and atomically renames
// it into place.
func writeGobAtomic(path string, v any) (err error) {
	dir := filepath.Dir(path)
	tmp, err := os.CreateTemp(dir, ".slam-state-*")
	if err != nil {
		return err
	}
	tmpName := tmp.Name()
	defer func() {
		if err != nil {
			tmp.Close()
			os.Remove(tmpName)
		}
	}()
	enc := gob.NewEncoder(tmp)
	if err = enc.Encode(v); err != nil {
		return err
	}
	if err = tmp.Sync(); err != nil {
		return err
	}
	if err = tmp.Close(); err != nil {
		return err
	}
	return os.Rename(tmpName, path)
}

// LoadState reads a snapshot saved by Save.
func LoadState(path string, logger Logger) (*Pipeline, error) {
	f, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer f.Close()
	var st savedState
	dec := gob.NewDecoder(f)
	if err := dec.Decode(&st); err != nil {
		return nil, fmt.Errorf("slam: state file truncated or corrupt: %w", err)
	}
	switch st.Version {
	case stateVersionV1:
		migrateV1ToV2(&st)
	case stateVersion:
		// current layout
	default:
		return nil, fmt.Errorf("%w: got %d, this build supports versions %d..%d",
			ErrUnsupportedState, st.Version, minStateVersion, stateVersion)
	}
	// Repair config fields added after this file was written (gob decodes
	// them as zero). Done before validation so an older, otherwise-valid
	// file resumes with every safeguard active rather than zeroed values.
	st.Config.fillLegacyDefaults()
	// A constructible pipeline is created only after full validation, so a
	// malformed file can never produce a half-new/half-old pipeline.
	if err := st.validate(); err != nil {
		return nil, fmt.Errorf("slam: refusing to load invalid state: %w", err)
	}
	if logger == nil {
		logger = NopLogger()
	}
	p := NewPipeline(st.Config, logger)
	p.restoreStateLocked(st)
	return p, nil
}

// migrateV1ToV2 fills fields introduced in v2. All v1 keyframes belong to
// session 0; frame IDs were already globally unique in v1 because there was
// only one session.
func migrateV1ToV2(st *savedState) {
	for _, kf := range st.Nodes {
		if kf.Session == 0 {
			kf.Session = 0
		}
	}
	st.Session = 0
	st.FrameIDBase = 0
	st.Version = stateVersion
}

// validate checks the structural integrity of a decoded state. Any error
// means the file must be rejected outright rather than partially applied.
func (s *savedState) validate() error {
	if s.GridW < 0 || s.GridH < 0 || s.GridRes <= 0 {
		return fmt.Errorf("bad grid geometry %dx%d res=%v",
			s.GridW, s.GridH, s.GridRes)
	}
	if len(s.GridValues) != s.GridW*s.GridH {
		return fmt.Errorf("grid data length %d does not match %dx%d",
			len(s.GridValues), s.GridW, s.GridH)
	}
	for i, e := range s.Edges {
		if e.I < 0 || e.J < 0 || e.I >= len(s.Nodes) || e.J >= len(s.Nodes) {
			return fmt.Errorf("edge %d references missing node (%d,%d)",
				i, e.I, e.J)
		}
	}
	for i, kf := range s.Nodes {
		if kf == nil {
			return fmt.Errorf("node %d is nil", i)
		}
	}
	return s.checkFinite()
}

func (p *Pipeline) restoreStateLocked(st savedState) {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.graph.Nodes = st.Nodes
	p.graph.Edges = st.Edges
	p.traj = st.Traj
	p.frameCount = st.FrameCount
	p.lastPose = st.LastPose
	p.lastOdom = st.LastOdom
	p.odomBase = st.OdomBase
	p.started = st.Started
	p.lastKF = st.LastKF
	p.lastKFOdom = st.LastKFOdom
	p.lastScan = append([]Point(nil), st.LastScan...)
	p.decisions = st.Decisions
	p.loopLog = st.LoopLog
	p.optLog = st.OptLog
	p.session = st.Session
	p.frameIDBase = st.FrameIDBase
	p.grid.loadRaw(st.GridRes, st.GridOX, st.GridOY,
		st.GridW, st.GridH, st.GridValues)
	// Same-session resume: a normal Save happens mid-session with
	// awaitAnchor=false, so frames continue immediately. A cross-session
	// start (Load then Localize+BeginSession) sets awaitAnchor explicitly
	// via PrepareNewSession.
	p.awaitAnchor = st.AwaitAnchor
}

func (s *savedState) checkFinite() error {
	check := func(name string, poses ...Pose) error {
		for i, ps := range poses {
			if math.IsNaN(ps.X) || math.IsNaN(ps.Y) || math.IsNaN(ps.Theta) ||
				math.IsInf(ps.X, 0) || math.IsInf(ps.Y, 0) || math.IsInf(ps.Theta, 0) {
				return fmt.Errorf("%s[%d] non-finite", name, i)
			}
		}
		return nil
	}
	if err := check("last", s.LastPose, s.LastOdom, s.LastKFOdom); err != nil {
		return err
	}
	for _, kf := range s.Nodes {
		if err := check("node", kf.Pose); err != nil {
			return err
		}
	}
	for _, v := range s.GridValues {
		if math.IsNaN(v) || math.IsInf(v, 0) {
			return errors.New("grid contains non-finite cells")
		}
	}
	return nil
}

// DumpMapPGM writes a human-viewable PGM of a snapshot (for local checks).
func DumpMapPGM(w io.Writer, snap *MapSnapshot) error {
	if _, err := fmt.Fprintf(w, "P5\n%d %d\n255\n", snap.W, snap.H); err != nil {
		return err
	}
	buf := make([]byte, snap.W)
	for y := snap.H - 1; y >= 0; y-- {
		for x := 0; x < snap.W; x++ {
			v := byte(200)
			switch snap.Cells[y*snap.W+x] {
			case CellOccupied:
				v = 0
			case CellFree:
				v = 254
			}
			buf[x] = v
		}
		if _, err := w.Write(buf); err != nil {
			return err
		}
	}
	return nil
}
