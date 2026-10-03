package slam

// This file exposes a small, stable internal API used by the test-suite
// (and usable by embedders building regression tests). It deliberately does
// not expose anything mutable beyond what the optimizer needs.

// EdgeKindOdom / EdgeKindLoop selectors for ExportAddEdge.
const (
	ExportEdgeOdom = EdgeOdom
	ExportEdgeLoop = EdgeLoop
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

// The following helpers back the state-compatibility unit tests. They write
// deliberately malformed/versioned gob states through the same atomic path
// used by Save, so LoadState's validation/version handling can be exercised
// without reaching into unexported types from the external test package.

// ExportWriteFutureState writes a state claiming a newer on-disk version.
func ExportWriteFutureState(path string) error {
	return writeTestState(path, 999, false)
}

// ExportWriteDanglingEdgeState writes a structurally invalid v2 state whose
// single edge references a missing node.
func ExportWriteDanglingEdgeState(path string) error {
	return writeTestState(path, stateVersion, true)
}
