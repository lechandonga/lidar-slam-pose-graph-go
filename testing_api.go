package slam

import "math"

// This file exposes a small, stable internal API used by the test-suite
// (and usable by embedders building regression tests). It deliberately does
// not expose anything mutable beyond what the optimizer needs.

// EdgeKindOdom / EdgeKindLoop selectors for ExportAddEdge.
const (
	ExportEdgeOdom  = EdgeOdom
	ExportEdgeLoop  = EdgeLoop
	ExportEdgeReloc = EdgeReloc
)

// ExportDiagInfo returns a diagonal SE2 information matrix.
func ExportDiagInfo(sigXY, sigTh float64) [3][3]float64 {
	return diagInfo(sigXY, sigTh)
}

// ExportNewGraph returns an empty pose graph.
func ExportNewGraph() *Graph { return &Graph{} }

// ExportAddNode appends a node at the given pose.
func ExportAddNode(g *Graph, pose Pose) {
	g.Nodes = append(g.Nodes, &Keyframe{ID: len(g.Nodes), Pose: pose})
}

// ExportAddEdge appends an edge with a diagonal information matrix.
func ExportAddEdge(g *Graph, kind EdgeKind, i, j int, measure Pose,
	info [3][3]float64) {
	g.Edges = append(g.Edges, &Edge{
		I: i, J: j, Kind: kind, Measure: measure,
		Info: info, RawInfo: info, Status: LoopAccepted,
	})
}

// ExportNodes returns the graph's nodes.
func ExportNodes(g *Graph) []*Keyframe { return g.Nodes }

// ExportPipelineNodes returns the pipeline's current keyframes.
func ExportPipelineNodes(p *Pipeline) []*Keyframe {
	p.mu.RLock()
	defer p.mu.RUnlock()
	return p.graph.Nodes
}

// ExportOptimize runs the pose-graph optimizer.
func ExportOptimize(g *Graph, cfg Config) OptimizeResult {
	return optimizeGraph(g, cfg)
}

// ExportDiagInfoFromEdge exposes an edge's raw information matrix.
func ExportDiagInfoFromEdge(e *Edge) [3][3]float64 { return e.RawInfo }

// ExportMatchScan exposes scan matching for diagnostic tests.
func ExportMatchScan(cfg Config, ref []Point, scan []Point, pred Pose) MatchResult {
	return matchScan(cfg, ref, scan, pred)
}

// ExportWriteStateFileVersion writes a snapshot tagged with an arbitrary
// version number, used by tests to exercise version migration/rejection.
func ExportWriteStateFileVersion(path string, version int) error {
	p := NewPipeline(DefaultConfig(), NopLogger())
	p.ProcessFrame(Frame{
		Scan:    Scan{ID: 1, Time: 0.1, Points: exportSquareScan()},
		Odom:    Odom{ID: 1, Time: 0.1, Valid: true},
		HasOdom: true,
	})
	p.Finalize()
	p.mu.Lock()
	st := p.snapshotStateLocked()
	p.mu.Unlock()
	st.Version = version
	return writeStateAtomic(path, st)
}

func exportSquareScan() []Point {
	var pts []Point
	for i := 0; i < 40; i++ {
		a := float64(i) / 40 * 2 * 3.141592653589793
		pts = append(pts, Point{X: 2 * math.Cos(a), Y: 2 * math.Sin(a)})
	}
	return pts
}
