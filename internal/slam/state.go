package slam

import (
	"encoding/gob"
	"fmt"
	"io"
	"os"
	"path/filepath"

	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/geom"
	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/gridmap"
	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/posegraph"
)

// stateV1 is the on-disk representation. Fields are plain types so the
// encoding is stable across Go versions.
type stateV1 struct {
	Version int
	Cfg     Config
	Graph   posegraph.Snapshot
	MapSnap gridmap.Snapshot

	Keyframes []Keyframe
	Records   []FrameRecord

	LastFramePose geom.Pose
	LastOdomX     float64
	LastOdomY     float64
	LastOdomTh    float64
	LastRestart   int
	HaveOdom      bool
	LastKeyNode   int
	LoopAttempts  int
	FieldDirty    bool
}

// Save writes the complete state to path atomically: a temp file is fsynced
// and renamed over the target, so a crash can never leave a half-written
// state that Load would accept.
func (p *Pipeline) Save(path string) error {
	p.mu.RLock()
	st := stateV1{
		Version:       1,
		Cfg:           p.cfg,
		Graph:         p.graph.Snapshot(),
		MapSnap:       p.mapRef.Snapshot(),
		Keyframes:     append([]Keyframe(nil), p.keyframes...),
		Records:       append([]FrameRecord(nil), p.records...),
		LastFramePose: p.lastFramePose,
		LastOdomX:     p.lastOdom.X,
		LastOdomY:     p.lastOdom.Y,
		LastOdomTh:    p.lastOdom.Theta,
		LastRestart:   p.lastRestart,
		HaveOdom:      p.haveOdom,
		LastKeyNode:   p.lastKeyNode,
		LoopAttempts:  p.loopAttempts,
		FieldDirty:    p.localField == nil,
	}
	p.mu.RUnlock()

	dir := filepath.Dir(path)
	tmp, err := os.CreateTemp(dir, ".slam-state-*")
	if err != nil {
		return err
	}
	tmpName := tmp.Name()
	enc := gob.NewEncoder(tmp)
	if err := enc.Encode(&st); err != nil {
		tmp.Close()
		os.Remove(tmpName)
		return err
	}
	if err := tmp.Sync(); err != nil {
		tmp.Close()
		os.Remove(tmpName)
		return err
	}
	if err := tmp.Close(); err != nil {
		os.Remove(tmpName)
		return err
	}
	return os.Rename(tmpName, path)
}

// Load reads a state file and reconstructs an equivalent pipeline.
func Load(path string) (*Pipeline, error) {
	f, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer f.Close()
	var st stateV1
	dec := gob.NewDecoder(f)
	if err := dec.Decode(&st); err != nil && err != io.EOF {
		return nil, fmt.Errorf("decode state: %w", err)
	}
	if st.Version != 1 {
		return nil, fmt.Errorf("unsupported state version %d", st.Version)
	}
	p := New(st.Cfg)
	p.graph = posegraph.Restore(st.Graph)
	p.mapRef = gridmap.Restore(st.MapSnap,
		st.Cfg.MapHitLogOdds, st.Cfg.MapFreeLogOdds, st.Cfg.MapMinLogOdds, st.Cfg.MapMaxLogOdds)
	p.keyframes = st.Keyframes
	p.records = st.Records
	p.lastFramePose = st.LastFramePose
	p.lastOdom = geom.Pose{X: st.LastOdomX, Y: st.LastOdomY, Theta: st.LastOdomTh}
	p.lastRestart = st.LastRestart
	p.haveOdom = st.HaveOdom
	p.lastKeyNode = st.LastKeyNode
	p.loopAttempts = st.LoopAttempts
	if st.FieldDirty {
		p.localField = nil
	} else {
		// Rebuild deterministically from the latest keyframe set; the field is
		// a pure function of keyframes + center, so results are reproducible.
		if len(p.keyframes) > 0 {
			p.localField = p.localFieldLocked(p.lastFramePose)
		}
	}
	return p, nil
}
