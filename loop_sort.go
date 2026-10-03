package slam

import "sort"

// sortEdges keeps on-disk and in-memory edge order deterministic after
// compaction (map iteration order is randomized in Go).
func sortEdges(edges []*Edge) {
	sort.SliceStable(edges, func(i, j int) bool {
		if edges[i].Kind != edges[j].Kind {
			return edges[i].Kind < edges[j].Kind
		}
		if edges[i].I != edges[j].I {
			return edges[i].I < edges[j].I
		}
		return edges[i].J < edges[j].J
	})
}
