package slam

import (
	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/geom"
	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/posegraph"
)

// InjectLoopEdgeForTest inserts a loop candidate with an obviously wrong
// relative transform and forces the same accept/soft/reject pipeline used by
// real closures, so robustness against contradictory constraints is testable.
// chi2Hint overrides the measured chi2 gate to simulate a confident-but-wrong
// detector result.
func (p *Pipeline) InjectLoopEdgeForTest(from, to int, rel geom.Pose, chi2Hint float64) {
	p.mu.Lock()
	defer p.mu.Unlock()
	cfg := p.cfg
	status := posegraph.StatusAccepted
	reason := "injected_for_test"
	switch {
	case chi2Hint >= cfg.LoopRejectChi2:
		status = posegraph.StatusRejected
		reason = "injected_loop_chi2_above_reject_threshold"
	case chi2Hint > cfg.LoopSoftChi2:
		status = posegraph.StatusSoft
		reason = "injected_loop_chi2_in_soft_band"
	}
	p.graph.AddEdge(posegraph.Edge{
		From: from, To: to, Type: posegraph.EdgeLoop,
		Rel: rel, Info: diagInfo(cfg.ScanWeight),
		Status: status, Reason: reason, Chi2: chi2Hint,
	})
	if status == posegraph.StatusRejected {
		return
	}
	rep := p.graph.Optimize(posegraph.OptimizeParams{
		MaxIter: cfg.OptMaxIter, ConvergeChi2: cfg.OptConvergeChi2,
		ConvergeStep: cfg.OptConvergeStep, HuberDelta: cfg.HuberDelta,
	})
	for i := range p.keyframes {
		if np, ok := p.graph.NodePose(p.keyframes[i].NodeID); ok {
			p.keyframes[i].Pose = np
		}
	}
	if len(p.keyframes) > 0 {
		p.lastFramePose = p.keyframes[len(p.keyframes)-1].Pose
	}
	var rec FrameRecord
	rec.OptConverged = rep.Converged
	rec.OptIter = rep.Iterations
	p.rebuildMapLocked(&rec)
}
