package slam

import (
	"math"
	"sort"
)

// MatchResult is the raw scan-matching output before quality gating.
type MatchResult struct {
	Pose       Pose
	Score      float64 // mean field response in [0,1]
	CoarseOK   bool
	EigRatio   float64 // min/max eigenvalue of the 3x3 information matrix
	UsedPoints int
	WeakAxes   [3]bool // poorly observed x,y,theta update directions
	Info       [3][3]float64
}

// matchScan registers scan against reference world points near the predicted
// pose. Deterministic: fixed grids, deterministic tie-break order.
func matchScan(cfg Config, ref []Point, scan []Point, pred Pose) MatchResult {
	res := MatchResult{Pose: pred}
	// Coarse stage: count scan points landing within inflate-distance of a
	// rasterized hit cell (fast integer lookup, no exp). ICP provides the
	// precise inlier-fraction score afterwards.
	field := buildLikelihoodField(ref, cfg.CoarseStepXY*2, cfg.FieldInflate)
	// Seed ICP from the prediction and a small deterministic set of nearby
	// seeds; keep the solution with the highest inlier fraction. The
	// likelihood field still provides the initial coarse score.
	type cand struct {
		pose  Pose
		score float64
	}
	seeds := []cand{{pred, -1}}
	coarseXY := cfg.CoarseStepXY
	coarseTh := cfg.CoarseStepTheta
	stepsT := int(cfg.MatchWinTheta/coarseTh) + 1
	stepsXY := int(cfg.MatchWinXY/coarseXY) + 1
	coarseScan := scan
	if len(scan) > 120 {
		coarseScan = make([]Point, 0, len(scan)/2)
		for i := 0; i < len(scan); i += 2 {
			coarseScan = append(coarseScan, scan[i])
		}
	}
	for ti := -stepsT; ti <= stepsT; ti++ {
		th := pred.Theta + float64(ti)*coarseTh
		for yi := -stepsXY; yi <= stepsXY; yi++ {
			for xi := -stepsXY; xi <= stepsXY; xi++ {
				if xi == 0 && yi == 0 && ti == 0 {
					continue
				}
				p2 := Pose{
					X:     pred.X + float64(xi)*coarseXY,
					Y:     pred.Y + float64(yi)*coarseXY,
					Theta: normAngle(th),
				}
				s := field.overlapFraction(p2, coarseScan)
				if s > 0 {
					seeds = append(seeds, cand{p2, s})
				}
			}
		}
	}
	// Keep only prediction plus the best few coarse candidates.
	sort.Slice(seeds[1:], func(i, j int) bool {
		return seeds[1+i].score > seeds[1+j].score
	})
	if len(seeds) > 5 {
		seeds = seeds[:4]
	}
	var bestPose Pose
	bestInlier := -1.0
	bestRatio := 0.0
	nCorr := 0
	var bestWeak [3]bool
	var bestInfo [3][3]float64
	for _, sd := range seeds {
		pose, inlier, ratio, nc, weak := matchICP(cfg, ref, scan, sd.pose)
		lastInfo := icpDebugH
		if inlier > bestInlier+1e-12 ||
			(math.Abs(inlier-bestInlier) <= 1e-12 && lessPose(pose, bestPose)) {
			bestInlier = inlier
			bestPose = pose
			bestRatio = ratio
			nCorr = nc
			bestWeak = weak
			bestInfo = lastInfo
		}
	}
	if nCorr < 3 || bestInlier < 0 {
		return res
	}
	pose := bestPose
	meanRes := bestInlier
	ratio := bestRatio
	// Consistency check: nearest-surface distance per scan point. A match
	// that looks good on inlier fraction but slides along a periodic wall
	// leaves a tail of points with large distances; detect that here.
	ri := buildRefIndex(ref, cfg.MapResolution*1.5)
	near := 0
	for _, pt := range scan {
		w := pose.Transform(pt)
		j, d := ri.nearest(w.X, w.Y, 2*cfg.FieldInflate)
		if j >= 0 && d < cfg.FieldInflate {
			near++
		}
	}
	nearFrac := float64(near) / float64(len(scan))
	score := meanRes
	if nearFrac < score-0.1 {
		score = nearFrac
	}
	res.Pose = pose
	res.Score = score
	res.CoarseOK = true
	res.EigRatio = ratio
	res.WeakAxes = bestWeak
	res.Info = bestInfo
	res.UsedPoints = len(scan)
	return res
}

func lessPose(a, b Pose) bool {
	if a.X != b.X {
		return a.X < b.X
	}
	if a.Y != b.Y {
		return a.Y < b.Y
	}
	return a.Theta < b.Theta
}

const refineEps = 1e-5

// refineGN maximizes mean field response with damped Gauss-Newton using
// numerical Jacobians. Returns pose, degeneracy ratio and final score.
func refineGN(cfg Config, f *likelihoodField, scan []Point, start Pose) (Pose, float64, float64) {
	p := start
	h := cfg.RefineStep
	var lastH [3][3]float64
	lastScore, _ := f.score(p, scan)
	for it := 0; it < cfg.RefineIter; it++ {
		var H [3][3]float64
		var g [3]float64
		used := 0
		for _, pt := range scan {
			w := p.Transform(pt)
			r0 := f.responseAt(w.X, w.Y)
			if r0 <= 0 {
				continue
			}
			rx := f.responseAt(w.X+h, w.Y)
			ry := f.responseAt(w.X, w.Y+h)
			wp := Pose{X: p.X, Y: p.Y, Theta: p.Theta + h}.Transform(pt)
			wm := Pose{X: p.X, Y: p.Y, Theta: p.Theta - h}.Transform(pt)
			jx := (rx - r0) / h
			jy := (ry - r0) / h
			jth := (f.responseAt(wp.X, wp.Y) - f.responseAt(wm.X, wm.Y)) / (2 * h)
			e := 1 - r0
			g[0] += jx * e
			g[1] += jy * e
			g[2] += jth * e
			H[0][0] += jx * jx
			H[0][1] += jx * jy
			H[0][2] += jx * jth
			H[1][1] += jy * jy
			H[1][2] += jy * jth
			H[2][2] += jth * jth
			used++
		}
		if used < 3 {
			return p, 0, lastScore
		}
		H[1][0] = H[0][1]
		H[2][0] = H[0][2]
		H[2][1] = H[1][2]
		for a := 0; a < 3; a++ {
			H[a][a] += 1e-6
		}
		lastH = H
		dx, ok := solve3(H, g)
		if !ok {
			return p, eigRatio(H), lastScore
		}
		step := 1.0
		var s1 float64
		for tries := 0; tries < 6; tries++ {
			cand := Pose{
				X:     p.X + step*dx[0],
				Y:     p.Y + step*dx[1],
				Theta: normAngle(p.Theta + step*dx[2]),
			}
			s1, _ = f.score(cand, scan)
			if s1 >= lastScore {
				p = cand
				break
			}
			step *= 0.5
		}
		lastScore = s1
		if math.Abs(step*dx[0]) < refineEps &&
			math.Abs(step*dx[1]) < refineEps &&
			math.Abs(step*dx[2]) < refineEps {
			break
		}
	}
	return p, eigRatio(lastH), lastScore
}

func solve3(A [3][3]float64, b [3]float64) ([3]float64, bool) {
	var m [3][4]float64
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			m[i][j] = A[i][j]
		}
		m[i][3] = b[i]
	}
	for col := 0; col < 3; col++ {
		piv := col
		for r := col + 1; r < 3; r++ {
			if math.Abs(m[r][col]) > math.Abs(m[piv][col]) {
				piv = r
			}
		}
		if math.Abs(m[piv][col]) < 1e-12 {
			return [3]float64{}, false
		}
		m[piv], m[col] = m[col], m[piv]
		for r := 0; r < 3; r++ {
			if r == col {
				continue
			}
			f := m[r][col] / m[col][col]
			for k := col; k < 4; k++ {
				m[r][k] -= f * m[col][k]
			}
		}
	}
	var x [3]float64
	for i := 0; i < 3; i++ {
		x[i] = m[i][3] / m[i][i]
	}
	return x, true
}

// eigRatio returns minEig/maxEig of a symmetric 3x3 via Jacobi rotations.
func eigRatio(A [3][3]float64) float64 {
	a := A
	for sweep := 0; sweep < 12; sweep++ {
		off := math.Abs(a[0][1]) + math.Abs(a[0][2]) + math.Abs(a[1][2])
		if off < 1e-14 {
			break
		}
		for p := 0; p < 2; p++ {
			for qq := p + 1; qq < 3; qq++ {
				if math.Abs(a[p][qq]) < 1e-14 {
					continue
				}
				tau := (a[qq][qq] - a[p][p]) / (2 * a[p][qq])
				t := 0.0
				if tau >= 0 {
					t = 1 / (tau + math.Sqrt(1+tau*tau))
				} else {
					t = -1 / (-tau + math.Sqrt(1+tau*tau))
				}
				c := 1 / math.Sqrt(1+t*t)
				s := t * c
				for k := 0; k < 3; k++ {
					g1 := c*a[p][k] - s*a[qq][k]
					g2 := s*a[p][k] + c*a[qq][k]
					a[p][k], a[qq][k] = g1, g2
				}
				for k := 0; k < 3; k++ {
					g1 := c*a[k][p] - s*a[k][qq]
					g2 := s*a[k][p] + c*a[k][qq]
					a[k][p], a[k][qq] = g1, g2
				}
			}
		}
	}
	e0, e1, e2 := a[0][0], a[1][1], a[2][2]
	mx := math.Max(e0, math.Max(e1, e2))
	mn := math.Min(e0, math.Min(e1, e2))
	if mx <= 0 || mn <= 0 {
		return 0
	}
	return mn / mx
}
