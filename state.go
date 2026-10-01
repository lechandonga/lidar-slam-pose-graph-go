package slam

import (
	"bytes"
	"encoding/gob"
	"errors"
	"fmt"
	"io"
	"math"
	"os"
	"path/filepath"
)

// stateVersion must be bumped when the on-disk layout changes.
const stateVersion = 1

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
}

// keyframeGob is the serialization form of a Keyframe. surfaceBase is
// unexported but required to reconstruct matching surfaces after a reload.
type keyframeGob struct {
	ID                          int
	FrameID                     int64
	Time                        float64
	Pose, PoseOdom, SurfaceBase Pose
	Scan, Surface               []Point
	UpdatedAt                   int64
}

// GobEncode ensures surfaceBase (unexported) survives persistence.
func (k *Keyframe) GobEncode() ([]byte, error) {
	var buf bytes.Buffer
	if err := gob.NewEncoder(&buf).Encode(keyframeGob{
		ID: k.ID, FrameID: k.FrameID, Time: k.Time,
		Pose: k.Pose, PoseOdom: k.PoseOdom, SurfaceBase: k.surfaceBase,
		Scan: k.Scan, Surface: k.Surface, UpdatedAt: k.UpdatedAt,
	}); err != nil {
		return nil, err
	}
	return buf.Bytes(), nil
}

// GobDecode restores all keyframe fields including surfaceBase.
func (k *Keyframe) GobDecode(data []byte) error {
	var kg keyframeGob
	if err := gob.NewDecoder(bytes.NewReader(data)).Decode(&kg); err != nil {
		return err
	}
	*k = Keyframe{
		ID: kg.ID, FrameID: kg.FrameID, Time: kg.Time,
		Pose: kg.Pose, PoseOdom: kg.PoseOdom, surfaceBase: kg.SurfaceBase,
		Scan: kg.Scan, Surface: kg.Surface, UpdatedAt: kg.UpdatedAt,
	}
	return nil
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
		Started: p.started, LastKF: p.lastKF, LastKFOdom: p.lastKFOdom,
		LastScan:  p.lastScan,
		Decisions: p.decisions, LoopLog: p.loopLog, OptLog: p.optLog,
		GridRes: gridRes, GridOX: ox, GridOY: oy,
		GridW: gw, GridH: gh, GridValues: vals,
	}
	return st
}

// Save atomically writes the full state to path. It writes to a sibling
// temporary file and renames it into place, so a crash mid-save can never
// leave a half-written trajectory or map.
func (p *Pipeline) Save(path string) error {
	p.mu.Lock()
	st := p.snapshotStateLocked()
	p.mu.Unlock()
	return writeStateAtomic(path, st)
}

func writeStateAtomic(path string, st savedState) (err error) {
	if err := st.checkFinite(); err != nil {
		return fmt.Errorf("slam: refusing to persist non-finite state: %w", err)
	}
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
	if err = enc.Encode(&st); err != nil {
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
		return nil, err
	}
	if st.Version != stateVersion {
		return nil, fmt.Errorf("slam: unsupported state version %d (want %d)",
			st.Version, stateVersion)
	}
	if logger == nil {
		logger = NopLogger()
	}
	p := NewPipeline(st.Config, logger)
	p.restoreStateLocked(st)
	return p, nil
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
	p.started = st.Started
	p.lastKF = st.LastKF
	p.lastKFOdom = st.LastKFOdom
	p.lastScan = append([]Point(nil), st.LastScan...)
	p.decisions = st.Decisions
	p.loopLog = st.LoopLog
	p.optLog = st.OptLog
	p.grid.loadRaw(st.GridRes, st.GridOX, st.GridOY,
		st.GridW, st.GridH, st.GridValues)
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
