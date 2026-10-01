package slam

import (
	"sort"

	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/geom"
	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/gridmap"
	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/posegraph"
)

// Records returns a copy of every per-frame verdict so far.
func (p *Pipeline) Records() []FrameRecord {
	p.mu.RLock()
	defer p.mu.RUnlock()
	out := make([]FrameRecord, len(p.records))
	copy(out, p.records)
	return out
}

// Keyframes returns a copy of the current keyframes (optimized poses).
func (p *Pipeline) Keyframes() []Keyframe {
	p.mu.RLock()
	defer p.mu.RUnlock()
	out := make([]Keyframe, len(p.keyframes))
	copy(out, p.keyframes)
	return out
}

// Map returns the live map pointer; callers must only use its locked API
// (Occupancy / IsOccupied / Snapshot), which is concurrency safe.
func (p *Pipeline) Map() *gridmap.Map {
	p.mu.RLock()
	defer p.mu.RUnlock()
	return p.mapRef
}

// GraphSnapshot returns a detached copy of nodes and edges.
func (p *Pipeline) GraphSnapshot() posegraph.Snapshot {
	p.mu.RLock()
	defer p.mu.RUnlock()
	return p.graph.Snapshot()
}

// Trajectory returns one pose per processed frame, aligned with the current
// keyframe poses. Non-keyframe poses are interpolated between the surrounding
// keyframes, so loop corrections propagate smoothly to the whole path.
func (p *Pipeline) Trajectory() []geom.Pose {
	p.mu.RLock()
	defer p.mu.RUnlock()
	out := make([]geom.Pose, len(p.records))
	if len(p.keyframes) == 0 {
		for i := range out {
			out[i] = p.records[i].FusedPose
		}
		return out
	}
	ordered := make([]Keyframe, len(p.keyframes))
	copy(ordered, p.keyframes)
	sort.Slice(ordered, func(i, j int) bool { return ordered[i].Seq < ordered[j].Seq })

	ki := 0
	for i, r := range p.records {
		for ki+1 < len(ordered) && ordered[ki+1].Seq <= r.Seq {
			ki++
		}
		switch {
		case r.Seq <= ordered[0].Seq:
			out[i] = ordered[0].Pose
		case ki+1 >= len(ordered):
			out[i] = ordered[len(ordered)-1].Pose
		default:
			a, b := ordered[ki], ordered[ki+1]
			span := float64(b.Seq - a.Seq)
			t := float64(r.Seq-a.Seq) / span
			out[i] = geom.Interpolate(a.Pose, b.Pose, t)
		}
	}
	return out
}

// LoopEdges returns only loop-closure edges with their verdicts.
func (p *Pipeline) LoopEdges() []posegraph.Edge {
	snap := p.GraphSnapshot()
	var out []posegraph.Edge
	for _, e := range snap.Edges {
		if e.Type == posegraph.EdgeLoop {
			out = append(out, e)
		}
	}
	return out
}

// Bounds reports graph size for resource-growth verification.
func (p *Pipeline) Bounds() (keyframes, nodes, edges, activeEdges int) {
	p.mu.RLock()
	defer p.mu.RUnlock()
	total, active := p.graph.EdgeCount()
	return len(p.keyframes), p.graph.NodeCount(), total, active
}
