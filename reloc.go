// Cross-session global localization.
//
// A new session starts in an unknown place with its wheel odometry and clock
// reset to zero. After loading a previously saved pipeline, the first scan(s)
// must be localized inside the OLD world map instead of being appended to
// the end of the old trajectory. This file performs a global scan-over-map
// search and classifies the outcome as unique / ambiguous / failed. State is
// only mutated on an unambiguous result.
package slam

import (
	"math"
	"sort"
)

// LocalizeStatus classifies a cross-session localization attempt.
type LocalizeStatus string

const (
	LocalizeUnique    LocalizeStatus = "unique"
	LocalizeAmbiguous LocalizeStatus = "ambiguous"
	LocalizeFailed    LocalizeStatus = "failed"
)

// LocalizeCandidate is one scored alignment hypothesis in the old WORLD
// frame (world <- new-session base).
type LocalizeCandidate struct {
	Pose     Pose
	Score    float64 // fine inlier fraction in [0,1]
	Residual float64 // RMS point-to-surface distance, metres
	Overlap  int     // scan points landing within the mapped area
	Matches  int     // old keyframes contributing reference surface
}

// LocalizeResult is the explainable output of a localization attempt. It is
// returned whether or not localization was accepted, so callers can judge
// alignment quality from Score/Residual and tell the failure modes apart.
type LocalizeResult struct {
	Status     LocalizeStatus
	Reason     string // distinct reason code (see ReasonLoc* constants)
	Pose       Pose   // best candidate, world frame (zero when failed)
	Score      float64
	Residual   float64
	HintUsed   bool
	Candidates []LocalizeCandidate // ranked distinct candidates (best first)
}

// Distinct localization reason codes.
const (
	ReasonLocOK           = "localized"
	ReasonLocEmptyMap     = "localize_empty_prior_map"
	ReasonLocBadScan      = "localize_bad_scan"
	ReasonLocNoOverlap    = "localize_no_overlap"
	ReasonLocScoreLow     = "localize_score_low"
	ReasonLocResidualHigh = "localize_residual_high"
	ReasonLocAmbiguous    = "localize_ambiguous_multiple_modes"
)

// LocalizeHint optionally restricts the global search to a rough
// pose/heading clue in the old world frame.
type LocalizeHint struct {
	HasHint bool
	Pose    Pose
}

// Localize scans the saved map for where scan was taken. It never mutates
// pipeline state: call BeginSession to commit an accepted hypothesis.
func (p *Pipeline) Localize(scan Scan, hint LocalizeHint) LocalizeResult {
	p.mu.RLock()
	defer p.mu.RUnlock()
	return p.localizeLocked(scan, hint)
}

// localReferenceLocked collects the rasterized union of surfaces of keyframes
// whose poses lie within radius of center: dense, local, and independent of
// the new session's odom frame.
func (p *Pipeline) localReferenceLocked(center Pose, radius float64) []Point {
	var pts []Point
	for _, kf := range p.graph.Nodes {
		if PoseDist(kf.Pose, center) > radius {
			continue
		}
		pts = append(pts, kf.SurfaceWorld()...)
	}
	return rasterizeHits(pts, p.cfg.MapResolution)
}

type locSeed struct {
	pose  Pose
	score float64
}

func (p *Pipeline) localizeLocked(scan Scan, hint LocalizeHint) LocalizeResult {
	res := LocalizeResult{Status: LocalizeFailed}
	if len(p.graph.Nodes) == 0 || p.grid == nil || p.grid.w == 0 {
		res.Reason = ReasonLocEmptyMap
		return res
	}
	clean, reason := validateScan(p.cfg, scan)
	if reason != ReasonOK {
		res.Reason = ReasonLocBadScan + ":" + reason
		return res
	}

	// Global coarse reference: occupied hit cells of the whole saved map,
	// dilated through an EDT field for cheap overlap scoring.
	worldRef := p.grid.hitCellsWorld()
	if len(worldRef) == 0 {
		res.Reason = ReasonLocEmptyMap
		return res
	}
	coarse := subsample(clean, p.cfg.LocalizeCoarseScanPts)
	field := buildLikelihoodField(worldRef, p.cfg.LocalizeCoarseStepXY,
		p.cfg.FieldInflate)

	minX, minY, maxX, maxY := field.bounds()
	stepXY := p.cfg.LocalizeCoarseStepXY
	stepTh := p.cfg.LocalizeCoarseStepTheta

	var seeds []locSeed
	if hint.HasHint {
		res.HintUsed = true
		rxy, rth := p.cfg.LocalizeHintRadiusXY, p.cfg.LocalizeHintRadiusTheta
		nxy := int(rxy/stepXY) + 1
		nth := int(rth/stepTh) + 1
		for ti := -nth; ti <= nth; ti++ {
			for yi := -nxy; yi <= nxy; yi++ {
				for xi := -nxy; xi <= nxy; xi++ {
					pose := Pose{
						X:     hint.Pose.X + float64(xi)*stepXY,
						Y:     hint.Pose.Y + float64(yi)*stepXY,
						Theta: normAngle(hint.Pose.Theta + float64(ti)*stepTh),
					}
					if s := field.overlapFraction(pose, coarse); s > 0 {
						seeds = append(seeds, locSeed{pose, s})
					}
				}
			}
		}
	} else {
		// Full-map sweep on the coarse grid; theta is fully periodic.
		nth := int(math.Pi/stepTh + 0.5)
		for x := minX; x <= maxX; x += stepXY {
			for y := minY; y <= maxY; y += stepXY {
				for ti := 0; ti < 2*nth; ti++ {
					pose := Pose{X: x, Y: y,
						Theta: float64(ti)*stepTh - math.Pi}
					if s := field.overlapFraction(pose, coarse); s > 0 {
						seeds = append(seeds, locSeed{pose, s})
					}
				}
			}
		}
	}
	if len(seeds) == 0 {
		res.Reason = ReasonLocNoOverlap
		return res
	}
	sort.Slice(seeds, func(i, j int) bool {
		if seeds[i].score != seeds[j].score {
			return seeds[i].score > seeds[j].score
		}
		return lessPose(seeds[i].pose, seeds[j].pose)
	})
	// Drop clearly dead coarse hypotheses, then spread so each spatial /
	// angular mode keeps a representative seed. Spreading BEFORE truncation
	// matters: in a symmetric room the wrong-heading mode has the same high
	// coarse overlap as the right one and must survive to refinement, where
	// its score decides whether the localization is genuinely ambiguous.
	cutoff := 0.25
	if best := seeds[0].score; best > 0 {
		if c := best - 0.2; c > cutoff {
			cutoff = c
		}
	}
	filtered := seeds[:0]
	for _, sd := range seeds {
		if sd.score >= cutoff {
			filtered = append(filtered, sd)
		}
	}
	seeds = spreadSeeds(filtered, 2*stepXY, 3*stepTh)
	if len(seeds) > 60 {
		seeds = seeds[:60]
	}

	lcfg := p.cfg
	lcfg.MatchWinXY = p.cfg.LocalizeRefineWinXY
	lcfg.MatchWinTheta = p.cfg.LocalizeRefineWinTheta
	lcfg.CoarseStepXY = stepXY
	lcfg.CoarseStepTheta = stepTh
	lcfg.RefineIter = 12

	var cands []LocalizeCandidate
	globalField := buildLikelihoodField(worldRef, p.cfg.FieldResolution,
		p.cfg.FieldInflate)
	for _, sd := range seeds {
		radius := p.cfg.LocalizeKFRefRadius
		ref := p.localReferenceLocked(sd.pose, radius)
		// The fine reference is the dense union of nearby keyframe
		// surfaces (small after per-cell dedup). Global walls feed the
		// coarse search; pulling the entire map into every ICP would be
		// far too expensive. If no keyframe surface is nearby, fall back to
		// a bounded window of the global walls rather than the whole map.
		if len(ref) < 8 {
			ref = pointsWithin(worldRef, sd.pose, radius)
		}
		if len(ref) > 1200 {
			ref = ref[:1200]
		}
		mr := matchScan(lcfg, ref, clean, sd.pose)
		if !mr.CoarseOK {
			continue
		}
		// Guard against ICP local minima in symmetric/degenerate rooms:
		// score both the refined pose and the raw coarse seed against the
		// GLOBAL likelihood field, keeping the better-scoring pose. The
		// coarse seed is a high-overlap grid hypothesis; point-to-line ICP
		// can be dragged off a correct heading mode by raster normals even
		// when the seed itself aligns nearly perfectly.
		if s, _ := relocWeightedScore(globalField, sd.pose, clean); s > mr.Score {
			mr.Score = s
			mr.Pose = sd.pose
		}
		if s, _ := relocWeightedScore(globalField, mr.Pose, clean); s > mr.Score {
			mr.Score = s
		}
		rms, overlap := surfaceRMS(ref, mr.Pose, clean, p.cfg.FieldInflate)
		cands = append(cands, LocalizeCandidate{
			Pose: mr.Pose, Score: mr.Score, Residual: rms,
			Overlap: overlap,
			Matches: countKeyframesNearLocked(p, mr.Pose, radius),
		})
	}
	if len(cands) == 0 {
		res.Reason = ReasonLocNoOverlap
		return res
	}
	sort.Slice(cands, func(i, j int) bool {
		if cands[i].Score != cands[j].Score {
			return cands[i].Score > cands[j].Score
		}
		return lessPose(cands[i].Pose, cands[j].Pose)
	})
	// Merge same-mode hypotheses before testing for a second distinct mode.
	distinct := clusterCandidates(cands, 3*stepXY, 3*stepTh)
	res.Candidates = distinct
	best := distinct[0]
	res.Pose, res.Score, res.Residual = best.Pose, best.Score, best.Residual

	enoughOverlap := float64(best.Overlap)/float64(len(clean)) >= 0.5
	if best.Score < p.cfg.LocalizeMinScore || !enoughOverlap {
		res.Reason = ReasonLocScoreLow
		return res
	}
	if best.Residual > p.cfg.LocalizeMaxResidual {
		res.Reason = ReasonLocResidualHigh
		return res
	}
	if len(distinct) > 1 {
		second := distinct[1]
		secondViable := second.Score >= p.cfg.LocalizeMinScore &&
			second.Residual <= p.cfg.LocalizeMaxResidual
		gap := best.Score - second.Score
		if secondViable && gap < p.cfg.LocalizeScoreMargin {
			res.Status = LocalizeAmbiguous
			res.Reason = ReasonLocAmbiguous
			return res
		}
	}
	res.Status = LocalizeUnique
	res.Reason = ReasonLocOK
	p.log.Localizef("status=%s reason=%s score=%.3f rms=%.3f pose=%s hint=%v cands=%d",
		res.Status, res.Reason, best.Score, best.Residual, best.Pose,
		hint.HasHint, len(distinct))
	return res
}

// spreadSeeds greedily keeps seeds separated by at least (dxy, dth), in
// score order, removing redundant coarse hypotheses before refinement.
func spreadSeeds(in []locSeed, dxy, dth float64) []locSeed {
	out := make([]locSeed, 0, len(in))
	for _, s := range in {
		keep := true
		for _, q := range out {
			if PoseDist(q.pose, s.pose) < dxy &&
				math.Abs(normAngle(q.pose.Theta-s.pose.Theta)) < dth {
				keep = false
				break
			}
		}
		if keep {
			out = append(out, s)
		}
	}
	return out
}

// clusterCandidates merges candidates describing the same alignment mode.
func clusterCandidates(in []LocalizeCandidate, dxy, dth float64) []LocalizeCandidate {
	var out []LocalizeCandidate
	for _, c := range in {
		merged := false
		for _, q := range out {
			if PoseDist(q.Pose, c.Pose) < dxy &&
				math.Abs(normAngle(q.Pose.Theta-c.Pose.Theta)) < dth {
				merged = true
				break
			}
		}
		if !merged {
			out = append(out, c)
		}
	}
	return out
}

func countKeyframesNearLocked(p *Pipeline, pose Pose, radius float64) int {
	n := 0
	for _, kf := range p.graph.Nodes {
		if PoseDist(kf.Pose, pose) <= radius {
			n++
		}
	}
	return n
}

// surfaceRMS reports the RMS nearest-surface distance of scan transformed by
// pose, counting only points landing inside the reference bounds.
func surfaceRMS(ref []Point, pose Pose, scan []Point, maxd float64) (rms float64, inBounds int) {
	ri := buildRefIndex(ref, maxd)
	sum := 0.0
	used := 0
	for _, pt := range scan {
		w := pose.Transform(pt)
		j, d := ri.nearest(w.X, w.Y, maxd)
		if j < 0 {
			continue
		}
		inBounds++
		sum += d * d
		used++
	}
	if used == 0 {
		return math.Inf(1), inBounds
	}
	return math.Sqrt(sum / float64(used)), inBounds
}

func minInt(a, b int) int {
	if a < b {
		return a
	}
	return b
}

// relocWeightedScore scores a localization hypothesis with emphasis on
// NEAR-FIELD returns. In a large open room the far outer walls look nearly
// identical from many poses and dominate an unweighted inlier fraction,
// letting a translated hypothesis masquerade as correct; nearby objects
// (furniture, the wall beside the robot) constrain the pose far more
// tightly, so each point is weighted inversely by its range.
func relocWeightedScore(f *likelihoodField, p Pose, scan []Point) (float64, int) {
	var wsum, ssum float64
	n := 0
	for _, pt := range scan {
		r := math.Hypot(pt.X, pt.Y)
		if r < 0.05 {
			r = 0.05
		}
		// Near points (<2 m) count up to 6x; far points (>12 m) ~1x.
		w := 1.0 + 10.0/(r+1.0)
		wp := p.Transform(pt)
		// Every scan point votes over the FULL weight: a point that lands
		// far from every mapped surface (or outside the mapped area)
		// contributes ZERO support, not a free full mark. Counting such
		// points as matches let a translated/rotated hypothesis in a
		// wall-rich room collect a near-perfect score from the points that
		// happened to rest near unrelated walls.
		wsum += w
		n++
		cx, cy := f.cell(wp.X, wp.Y)
		if cx < 0 || cy < 0 || cx >= f.w || cy >= f.h {
			continue
		}
		d := f.dist[cy*f.w+cx]
		if d > 3*f.inflate {
			continue
		}
		ssum += w * math.Exp(-(d*d)/(2*f.sigma2))
	}
	if n == 0 || wsum == 0 {
		return 0, 0
	}
	return ssum / wsum, n
}
