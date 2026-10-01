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
