// Package posegraph stores keyframe nodes and their SE(2) relative-pose
// constraints, and runs a deterministic Gauss-Newton optimizer with Huber
// robust kernels. Loop edges carry an explicit acceptance status so that
// contradictory closures are distinguishable in the logs.
package posegraph

import (
	"sync"

	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/geom"
)

// EdgeType identifies how a constraint was produced.
type EdgeType string

const (
	EdgeOdom EdgeType = "odom"
	EdgeLoop EdgeType = "loop"
)

// EdgeStatus records the robustness verdict for a loop edge.
type EdgeStatus string

const (
	StatusAccepted EdgeStatus = "accepted"
	StatusSoft     EdgeStatus = "soft_weighted"
	StatusRejected EdgeStatus = "rejected"
)

// Node is one keyframe.
type Node struct {
	ID   int
	Seq  int // processing sequence of the frame that created it
	Pose geom.Pose
}

// Edge is a relative-pose constraint T_from^-1 * T_to = Rel.
type Edge struct {
	From   int
	To     int
	Type   EdgeType
	Rel    geom.Pose
	Info   [3][3]float64
	Status EdgeStatus
	Reason string
	Chi2   float64 // chi2 measured at insertion time (loop edges)
}

// Active reports whether the optimizer must use this edge.
func (e Edge) Active() bool { return e.Status != StatusRejected }

// Graph is the in-memory pose graph. All operations are serialized by mu.
type Graph struct {
	mu    sync.RWMutex
	nodes []Node
	edges []Edge
}

// New creates an empty graph.
func New() *Graph { return &Graph{} }

// AddNode appends a node and returns its assigned id.
func (g *Graph) AddNode(seq int, pose geom.Pose) int {
	g.mu.Lock()
	defer g.mu.Unlock()
	id := len(g.nodes)
	g.nodes = append(g.nodes, Node{ID: id, Seq: seq, Pose: pose})
	return id
}

// AddEdge appends a constraint.
func (g *Graph) AddEdge(e Edge) {
	g.mu.Lock()
	defer g.mu.Unlock()
	g.edges = append(g.edges, e)
}

// Nodes returns a copy of all nodes.
func (g *Graph) Nodes() []Node {
	g.mu.RLock()
	defer g.mu.RUnlock()
	out := make([]Node, len(g.nodes))
	copy(out, g.nodes)
	return out
}

// NodePose returns the current estimate of node id.
func (g *Graph) NodePose(id int) (geom.Pose, bool) {
	g.mu.RLock()
	defer g.mu.RUnlock()
	if id < 0 || id >= len(g.nodes) {
		return geom.Pose{}, false
	}
	return g.nodes[id].Pose, true
}

// Edges returns a copy of all edges, including rejected ones.
func (g *Graph) Edges() []Edge {
	g.mu.RLock()
	defer g.mu.RUnlock()
	out := make([]Edge, len(g.edges))
	copy(out, g.edges)
	return out
}

// NodeCount / EdgeCount report graph size.
func (g *Graph) NodeCount() int {
	g.mu.RLock()
	defer g.mu.RUnlock()
	return len(g.nodes)
}

// EdgeCount returns total and active edge counts.
func (g *Graph) EdgeCount() (total, active int) {
	g.mu.RLock()
	defer g.mu.RUnlock()
	for _, e := range g.edges {
		total++
		if e.Active() {
			active++
		}
	}
	return total, active
}

// Snapshot serializes the graph for persistence.
type Snapshot struct {
	Nodes []Node
	Edges []Edge
}

// Snapshot returns a deep copy of graph contents.
func (g *Graph) Snapshot() Snapshot {
	g.mu.RLock()
	defer g.mu.RUnlock()
	nodes := make([]Node, len(g.nodes))
	copy(nodes, g.nodes)
	edges := make([]Edge, len(g.edges))
	copy(edges, g.edges)
	return Snapshot{Nodes: nodes, Edges: edges}
}

// Restore builds a graph from a snapshot.
func Restore(s Snapshot) *Graph {
	g := New()
	g.nodes = append([]Node(nil), s.Nodes...)
	g.edges = append([]Edge(nil), s.Edges...)
	return g
}
