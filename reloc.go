package slam

import (
	"math"
	"sort"
)

// localReferenceSurfaceLocked builds one world-frame matching surface from
// ALL previously mapped keyframes (global localization may start anywhere
// in the old map, unlike frame-to-frame tracking which only needs the
// immediate neighbourhood).
func (p *Pipeline) localReferenceSurfaceLocked() []Point {
	var pts []Point
	for _, kf := range p.graph.Nodes {
		pts = append(pts, kf.SurfaceWorld()...)
	}
	pts = rasterizeHits(pts, p.cfg.MapResolution)
	// Prefer the accumulated occupancy map itself when it has content: it
	// already fuses every prior session's observations at map resolution
	// and directly represents "what the new scan must line up with".
	snap := p.grid.snapshot()
	if snap.W > 0 {
		var gpts []Point
		for cy := 0; cy < snap.H; cy++ {
			for cx := 0; cx < snap.W; cx++ {
				if snap.Cells[cy*snap.W+cx] == CellOccupied {
					gpts = append(gpts, Point{
						X: snap.OriginX + (float64(cx)+0.5)*snap.Resolution,
						Y: snap.OriginY + (float64(cy)+0.5)*snap.Resolution,
					})
				}
			}
		}
		if len(gpts) > len(pts)/2 {
			return gpts
		}
	}
	return pts
}

// localizeSeeds enumerates deterministic coarse-search seeds. With a hint
// the window is small and centered on it; without one the search covers the
// full heading range at every old keyframe position (the robot must be
// somewhere previously traversed).
func (p *Pipeline) localizeSeeds(scan []Point, hint *LocalizeHint,
	ref []Point) []Pose {
	cfg := p.cfg
	type sp struct {
		pose  Pose
		score float64
	}
	var centers []Pose
	winXY, winTh := cfg.LocalizeHintXYWin, cfg.LocalizeHintThWin
	stepXY := cfg.LocalizeCoarseXY
	stepTh := cfg.LocalizeCoarseTheta
	if hint != nil && hint.HasPose {
		if hint.WinXY > 0 {
			winXY = hint.WinXY
		}
		if hint.WinTheta > 0 {
			winTh = hint.WinTheta
		}
		centers = []Pose{hint.Pose}
	} else {
		// Even centered on a visited keyframe we sweep a small XY radius:
		// the new start pose need not coincide exactly with a keyframe.
		winXY = stepXY
		// Heading is the hardest lattice to cover (a 0.1 rad error moves a
		// 10 m-range beam by a metre); use a finer angular sampling.
		stepTh = 0.15
		// One search center per old keyframe position covers the mapped area
		// without a dense global XY lattice (the coarse radius reaches
		// roughly halfway between keyframes).
		// Spatial dedup of centers: consecutive keyframes along the same
		// wall are 0.3 m apart and produce identical coarse scores; keep one
		// center per ~1 m cell.
		for _, kf := range p.graph.Nodes {
			c := Pose{X: kf.Pose.X, Y: kf.Pose.Y}
			dup := false
			for _, q := range centers {
				if PoseDist(c, q) < 2.0 {
					dup = true
					break
				}
			}
			if !dup {
				centers = append(centers, c)
			}
		}
	}
	// The reference is a map-resolution surface; building the coarse field
	// at a coarser resolution would create spurious zero-distance cells and
	// destroy the overlap score. Use the map resolution.
	field := buildLikelihoodField(ref, cfg.MapResolution, cfg.FieldInflate)
	coarse := scan
	if len(scan) > 120 {
		coarse = make([]Point, 0, len(scan)/2)
		for i := 0; i < len(scan); i += 6 {
			coarse = append(coarse, scan[i])
		}
	}
	eval := func(c Pose, nXY, nTh int, dXY, dTh float64) []sp {
		var out []sp
		for ti := -nTh; ti <= nTh; ti++ {
			for yi := -nXY; yi <= nXY; yi++ {
				for xi := -nXY; xi <= nXY; xi++ {
					pose := Pose{
						X:     c.X + float64(xi)*dXY,
						Y:     c.Y + float64(yi)*dXY,
						Theta: normAngle(c.Theta + float64(ti)*dTh),
					}
					if s := field.overlapFraction(pose, coarse); s > 0 {
						out = append(out, sp{pose, s})
					}
				}
			}
		}
		return out
	}
	var stage1 []sp
	if hint != nil && hint.HasPose {
		// Hint: one small dense window.
		stage1 = eval(centers[0], int(winXY/stepXY)+1,
			int(winTh/stepTh), stepXY, stepTh)
	} else {
		// No hint: two-stage search to keep the expensive per-pose trig
		// bounded. Stage 1 evaluates a handful of coarse headings at every
		// mapped center (XY on the center only).
		const coarseAngStep = 0.9
		nAng := int(math.Floor(math.Pi/coarseAngStep + 0.5))
		for _, c := range centers {
			stage1 = append(stage1, eval(c, 0, nAng, stepXY, coarseAngStep)...)
		}
	}
	sort.Slice(stage1, func(i, j int) bool {
		if stage1[i].score != stage1[j].score {
			return stage1[i].score > stage1[j].score
		}
		return lessPose(stage1[i].pose, stage1[j].pose)
	})
	// Stage 2: refine only the top distinct stage-1 basins with the fine XY
	// + heading lattice.
	var out []sp
	refineCount := 10
	if hint != nil && hint.HasPose {
		refineCount = len(stage1)
	}
	for _, cand := range stage1 {
		if refineCount == 0 {
			break
		}
		dup := false
		for _, q := range out {
			if PoseDist(cand.pose, q.pose) < 2*stepXY {
				dup = true
				break
			}
		}
		if dup {
			continue
		}
		refineCount--
		// Add a few immediately neighboring heading/XY offsets so ICP's
		// capture radius is never wider than the coarse angular gap.
		for _, dth := range []float64{-2 * stepTh, -stepTh, 0, stepTh, 2 * stepTh} {
			for _, dx := range []float64{-stepXY, 0, stepXY} {
				out = append(out, sp{
					pose: Pose{X: cand.pose.X + dx, Y: cand.pose.Y,
						Theta: normAngle(cand.pose.Theta + dth)},
					score: cand.score,
				})
			}
		}
		out = append(out, cand)
	}
	sort.Slice(out, func(i, j int) bool {
		if out[i].score != out[j].score {
			return out[i].score > out[j].score
		}
		return lessPose(out[i].pose, out[j].pose)
	})
	// Greedily keep spatially/rotationally distinct top seeds only: a dense
	// coarse lattice produces dozens of near-duplicates that ICP would
	// refine to the same basin.
	minSepXY := stepXY * 1.5
	minSepTh := stepTh * 1.5
	var seeds []Pose
	for _, cand := range out {
		dup := false
		for _, q := range seeds {
			if PoseDist(cand.pose, q) < minSepXY &&
				math.Abs(normAngle(cand.pose.Theta-q.Theta)) < minSepTh {
				dup = true
				break
			}
		}
		if !dup {
			seeds = append(seeds, cand.pose)
		}
		if len(seeds) >= 12 {
			break
		}
	}
	return seeds
}

// globalLocalize aligns a new session's first scan against the old map and
// classifies the result. It mutates nothing; the caller applies the fix.
func (p *Pipeline) globalLocalize(scan []Point, hint *LocalizeHint) LocalizeResult {
	res := LocalizeResult{Status: LocalizeFailed, AppliedEdge: -1}
	if len(p.graph.Nodes) == 0 {
		res.Reason = "empty_map"
		return res
	}
	ref := p.localReferenceSurfaceLocked()
	if len(ref) < 10 {
		res.Reason = "reference_too_sparse"
		return res
	}
	seeds := p.localizeSeeds(scan, hint, ref)
	lcfg := p.cfg
	// Global alignment starts from coarse lattices, not from a tight
	// tracking window: allow ICP enough capture range per seed, and a
	// wider point-to-line influence than frame tracking (old surfaces were
	// rasterized at capture time and only corrected rigidly).
	lcfg.MatchWinXY = 0.3
	lcfg.MatchWinTheta = 0.25
	lcfg.CoarseStepXY = p.cfg.LocalizeCoarseXY
	lcfg.RefineIter = 20
	lcfg.FieldInflate = p.cfg.FieldInflate * 1.5
	fine := buildLikelihoodField(ref, p.cfg.FieldResolution, p.cfg.FieldInflate)
	ri := buildRefIndex(ref, p.cfg.MapResolution*1.5)
	// PCA normals are identical for every seed; matchICP rebuilds them per
	// call, which dominates global-localization cost. Build once and use
	// the cached variant.
	icpCache := buildICPCache(lcfg, ri)
	type hyp struct {
		LocalizeHypothesis
	}
	var hyps []hyp
	for _, seed := range seeds {
		pose, inlier, ratio, ncorr, weak := matchICPCached(lcfg, ri, icpCache, scan, seed)
		if ncorr < 3 {
			continue
		}
		score, _ := fine.score(pose, scan)
		rms := residualRMS(ri, pose, scan)
		// Blend the ICP inlier fraction with the likelihood-field response:
		// ICP alone under-scores long, clean walls once any residual >2
		// cells exists, while the field response rewards all near-surface
		// points continuously.
		score = math.Max(inlier, score)
		hyps = append(hyps, hyp{LocalizeHypothesis{
			Pose: pose, Score: score, ResidualRMS: rms,
			EigRatio: ratio, WeakAxes: weak,
		}})
	}
	// Cluster distinct basins: two refined solutions within a small SE2
	// radius are the same hypothesis; keep the best member of each cluster.
	sort.Slice(hyps, func(i, j int) bool {
		if hyps[i].Score != hyps[j].Score {
			return hyps[i].Score > hyps[j].Score
		}
		return lessPose(hyps[i].Pose, hyps[j].Pose)
	})
	const sepXY = 0.5
	const sepTh = 0.35
	var clustered []LocalizeHypothesis
	for _, h := range hyps {
		duplicate := false
		for k := range clustered {
			if PoseDist(h.Pose, clustered[k].Pose) < sepXY &&
				math.Abs(normAngle(h.Pose.Theta-clustered[k].Pose.Theta)) < sepTh {
				duplicate = true
				break
			}
		}
		if !duplicate {
			clustered = append(clustered, h.LocalizeHypothesis)
		}
	}
	for _, h := range clustered {
		res.Hypotheses = append(res.Hypotheses, h)
	}
	if len(clustered) == 0 {
		res.Reason = "no_coarse_match"
		return res
	}
	res.Best = clustered[0]
	if len(clustered) > 1 {
		res.Second = clustered[1]
	}
	res.ScoreMargin = res.Best.Score - res.Second.Score

	if res.Best.Score < p.cfg.LocalizeMinScore {
		res.Status = LocalizeFailed
		res.Reason = "score_below_min"
		return res
	}
	if res.Best.ResidualRMS > p.cfg.LocalizeMaxRMS {
		res.Status = LocalizeFailed
		res.Reason = "residual_too_large"
		return res
	}
	if res.Best.WeakAxes[0] && res.Best.WeakAxes[1] {
		res.Status = LocalizeAmbiguous
		res.Reason = "degenerate_translation"
		return res
	}
	if res.Best.Score < p.cfg.LocalizeStrongScore ||
		(len(clustered) > 1 && res.ScoreMargin < p.cfg.LocalizeAmbiguousMargin) {
		res.Status = LocalizeAmbiguous
		if res.Best.Score < p.cfg.LocalizeStrongScore {
			res.Reason = "best_not_strong"
		} else {
			res.Reason = "multiple_plausible_poses"
		}
		return res
	}
	res.Status = LocalizeUnique
	res.Reason = "unique_strong_fix"
	res.Applied = res.Best.Pose
	return res
}

// residualRMS computes the mean normal-surface distance of scan points at
// pose against the reference, in metres. It quantifies alignment quality in
// physical units for logs and tests.
func residualRMS(ri *refIndex, pose Pose, scan []Point) float64 {
	sum, n := 0.0, 0
	for _, pt := range scan {
		w := pose.Transform(pt)
		_, d := ri.nearest(w.X, w.Y, 0.5)
		if d >= 0.5 {
			continue
		}
		sum += d * d
		n++
	}
	if n == 0 {
		return math.Inf(1)
	}
	return math.Sqrt(sum / float64(n))
}
