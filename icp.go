package slam

import "math"

// refSeg is a local line segment built from two neighboring reference
// points, with its unit normal.
type refSeg struct {
	p1, p2 Point
	nx, ny float64
	valid  bool
}

// buildReferenceSegments takes world-frame reference points ordered by
// proximity (they come from keyframe scans, themselves angle-ordered) and
// builds short segments between close neighbors. A simple spatial grid
// supplies nearest-neighbor queries.
type refIndex struct {
	res              float64
	originX, originY float64
	w, h             int
	cells            [][]int
	pts              []Point
}

// rasterizeHits maps points onto a fixed-resolution grid and returns the
// centers of occupied cells (each cell appears at most once).
func rasterizeHits(pts []Point, res float64) []Point {
	if len(pts) == 0 {
		return nil
	}
	minX, minY := pts[0].X, pts[0].Y
	for _, p := range pts[1:] {
		if p.X < minX {
			minX = p.X
		}
		if p.Y < minY {
			minY = p.Y
		}
	}
	type cell struct{ x, y int }
	seen := make(map[cell]struct{}, len(pts))
	out := make([]Point, 0, len(pts))
	for _, p := range pts {
		cx := int(math.Floor((p.X - minX) / res))
		cy := int(math.Floor((p.Y - minY) / res))
		if _, ok := seen[cell{cx, cy}]; ok {
			continue
		}
		seen[cell{cx, cy}] = struct{}{}
		out = append(out, Point{
			X: minX + (float64(cx)+0.5)*res,
			Y: minY + (float64(cy)+0.5)*res,
		})
	}
	return out
}

func buildRefIndex(pts []Point, res float64) *refIndex {
	if len(pts) == 0 {
		return &refIndex{res: res}
	}
	minX, minY := pts[0].X, pts[0].Y
	maxX, maxY := minX, minY
	for _, p := range pts[1:] {
		if p.X < minX {
			minX = p.X
		}
		if p.Y < minY {
			minY = p.Y
		}
		if p.X > maxX {
			maxX = p.X
		}
		if p.Y > maxY {
			maxY = p.Y
		}
	}
	ri := &refIndex{
		res: res, originX: minX - res, originY: minY - res,
		w:   int((maxX-minX)/res) + 3,
		h:   int((maxY-minY)/res) + 3,
		pts: pts,
	}
	ri.cells = make([][]int, ri.w*ri.h)
	for i, p := range pts {
		cx := int((p.X - ri.originX) / res)
		cy := int((p.Y - ri.originY) / res)
		if cx >= 0 && cy >= 0 && cx < ri.w && cy < ri.h {
			ri.cells[cy*ri.w+cx] = append(ri.cells[cy*ri.w+cx], i)
		}
	}
	return ri
}

func (ri *refIndex) nearest(x, y float64, maxd float64) (int, float64) {
	best := -1
	bestD := maxd
	cx0 := int((x - ri.originX) / ri.res)
	cy0 := int((y - ri.originY) / ri.res)
	r := int(maxd/ri.res) + 1
	for dy := -r; dy <= r; dy++ {
		for dx := -r; dx <= r; dx++ {
			cx, cy := cx0+dx, cy0+dy
			if cx < 0 || cy < 0 || cx >= ri.w || cy >= ri.h {
				continue
			}
			for _, idx := range ri.cells[cy*ri.w+cx] {
				p := ri.pts[idx]
				d := math.Hypot(p.X-x, p.Y-y)
				if d < bestD {
					bestD = d
					best = idx
				}
			}
		}
	}
	return best, bestD
}

// segmentAt estimates a local surface normal at nearest point idx by local
// PCA over points within segMax: the normal is the eigenvector of the
// scatter matrix with the smallest eigenvalue. This avoids relying on point
// ordering across scans captured at different poses.
func (ri *refIndex) segmentAt(idx int, segMax float64) refSeg {
	p := ri.pts[idx]
	cx := int((p.X - ri.originX) / ri.res)
	cy := int((p.Y - ri.originY) / ri.res)
	r := int(segMax/ri.res) + 1
	var mx, my, mxx, myy, mxy float64
	n := 0
	for dy := -r; dy <= r; dy++ {
		for dx := -r; dx <= r; dx++ {
			xx, yy := cx+dx, cy+dy
			if xx < 0 || yy < 0 || xx >= ri.w || yy >= ri.h {
				continue
			}
			for _, j := range ri.cells[yy*ri.w+xx] {
				q := ri.pts[j]
				if math.Hypot(q.X-p.X, q.Y-p.Y) > segMax {
					continue
				}
				mx += q.X
				my += q.Y
				mxx += q.X * q.X
				myy += q.Y * q.Y
				mxy += q.X * q.Y
				n++
			}
		}
	}
	if n < 3 {
		return refSeg{valid: false}
	}
	mx /= float64(n)
	my /= float64(n)
	sxx := mxx/float64(n) - mx*mx
	syy := myy/float64(n) - my*my
	sxy := mxy/float64(n) - mx*my
	// Smallest-eigenvalue eigenvector of [[sxx,sxy],[sxy,syy]].
	tr := sxx + syy
	disc := math.Sqrt(math.Max(0, (sxx-syy)*(sxx-syy)/4+sxy*sxy))
	evMin := tr/2 - disc
	var nx, ny float64
	if math.Abs(sxy) > 1e-12 {
		nx, ny = -sxy, sxx-evMin
	} else if sxx <= syy {
		nx, ny = 1, 0
	} else {
		nx, ny = 0, 1
	}
	l := math.Hypot(nx, ny)
	if l < 1e-12 {
		return refSeg{valid: false}
	}
	return refSeg{p1: Point{mx, my}, valid: true, nx: nx / l, ny: ny / l}
}

// matchICP refines an initial pose with point-to-line ICP using analytic
// Gauss-Newton. Returns pose, mean |normal distance| in metres, information
// eigenvalue ratio, and number of correspondences.
func matchICP(cfg Config, ref []Point, scan []Point, start Pose) (pose2 Pose, score2 float64, ratio2 float64, inlier2 int, weak [3]bool) {
	// References are already rasterized surfaces; thin them once more in
	// case multiple keyframes overlap (duplicate cells are harmless to PCA).
	ri := buildRefIndex(ref, cfg.MapResolution*1.5)
	return matchICPWithIndex(cfg, ri, scan, start)
}

// icpCache holds the per-reference structures (spatial index + precomputed
// PCA normals) reused across many global-localization seeds.
type icpCache struct {
	segMax float64
	surf   []struct {
		x, y, nx, ny float64
		hasNormal    bool
	}
}

func buildICPCache(cfg Config, ri *refIndex) *icpCache {
	segMax := cfg.MapResolution * 3
	c := &icpCache{segMax: segMax}
	c.surf = make([]struct {
		x, y, nx, ny float64
		hasNormal    bool
	}, len(ri.pts))
	for idx := range ri.pts {
		s := ri.segmentAt(idx, segMax)
		c.surf[idx].x = s.p1.X
		c.surf[idx].y = s.p1.Y
		c.surf[idx].nx = s.nx
		c.surf[idx].ny = s.ny
		c.surf[idx].hasNormal = s.valid
	}
	return c
}

func matchICPWithIndex(cfg Config, ri *refIndex, scan []Point, start Pose) (Pose, float64, float64, int, [3]bool) {
	return matchICPCached(cfg, ri, buildICPCache(cfg, ri), scan, start)
}

// matchICPCached is matchICP with caller-supplied reference normals.
func matchICPCached(cfg Config, ri *refIndex, cache *icpCache, scan []Point, start Pose) (pose2 Pose, score2 float64, ratio2 float64, inlier2 int, weak [3]bool) {
	pose := start
	var H [3][3]float64
	nCorr := 0
	meanRes := 0.0
	inlierFrac := 0.0
	// Precompute local surface normals once: PCA over the neighborhood is
	// the most expensive step and the reference does not move between ICP
	// iterations.
	for it := 0; it < cfg.RefineIter; it++ {
		H = [3][3]float64{}
		var g [3]float64
		nCorr = 0
		meanRes = 0
		inliers := 0
		// Huber cutoff for correspondences.
		kHuber := 0.15
		for _, pt := range scan {
			w := pose.Transform(pt)
			j, ndist := ri.nearest(w.X, w.Y, cfg.FieldInflate)
			if j < 0 {
				continue
			}
			seg := cache.surf[j]
			var nx, ny, qx, qy float64
			if seg.hasNormal {
				nx, ny, qx, qy = seg.nx, seg.ny, seg.x, seg.y
			} else {
				// Point-to-point along the approach direction.
				rp := ri.pts[j]
				dx, dy := w.X-rp.X, w.Y-rp.Y
				l := math.Hypot(dx, dy)
				if l < 1e-9 {
					continue
				}
				nx, ny, qx, qy = dx/l, dy/l, rp.X, rp.Y
			}
			r := nx*(w.X-qx) + ny*(w.Y-qy)
			wgt := 1.0
			if math.Abs(r) > kHuber {
				wgt = kHuber / math.Abs(r)
			}
			// Jacobian of the transformed point:
			// dw/d(dx)=R e_x ; dw/d(dy)=R e_y ;
			// dw/d(dtheta) = R [-sin th*x - cos th*y,
			//                    cos th*x - sin th*y]
			c, s := math.Cos(pose.Theta), math.Sin(pose.Theta)
			jx := nx*c + ny*s
			jy := -nx*s + ny*c
			jth := nx*(-s*pt.X-c*pt.Y) + ny*(c*pt.X-s*pt.Y)
			H[0][0] += wgt * jx * jx
			H[0][1] += wgt * jx * jy
			H[0][2] += wgt * jx * jth
			H[1][1] += wgt * jy * jy
			H[1][2] += wgt * jy * jth
			H[2][2] += wgt * jth * jth
			g[0] += wgt * jx * r
			g[1] += wgt * jy * r
			g[2] += wgt * jth * r
			meanRes += math.Abs(r)
			nCorr++
			if math.Abs(r) < 2*cfg.MapResolution && ndist < cfg.FieldInflate {
				inliers++
			}
		}
		if nCorr < 3 {
			return pose, 0, 0, nCorr, [3]bool{true, true, true}
		}
		H[1][0] = H[0][1]
		H[2][0] = H[0][2]
		H[2][1] = H[1][2]
		meanRes /= float64(nCorr)
		inlierFrac = float64(inliers) / float64(len(scan))
		for a := 0; a < 3; a++ {
			H[a][a] += 1e-6
		}
		dx, ok := solve3(H, g)
		if !ok {
			return pose, meanRes, eigRatio(H), nCorr, weakAxes(H)
		}
		pose.X -= dx[0]
		pose.Y -= dx[1]
		pose.Theta = normAngle(pose.Theta - dx[2])
		if math.Abs(dx[0]) < 1e-5 && math.Abs(dx[1]) < 1e-5 && math.Abs(dx[2]) < 1e-5 {
			break
		}
	}
	weak = weakAxes(H)
	icpDebugH = H
	return pose, inlierFrac, eigRatio(H), nCorr, weak
}

var icpDebugH [3][3]float64

// weakAxes marks SE2 update directions whose information is less than 2%
// of the strongest direction. In a straight corridor the along-corridor
// translation is typically weak even when theta and lateral translation
// are well constrained; the overall min/max eigenvalue ratio alone cannot
// express that, so fusion treats weak directions as odom-only.
func weakAxes(H [3][3]float64) [3]bool {
	var w [3]bool
	mx := 0.0
	for i := 0; i < 3; i++ {
		if H[i][i] > mx {
			mx = H[i][i]
		}
	}
	if mx <= 0 {
		return [3]bool{true, true, true}
	}
	for i := 0; i < 3; i++ {
		if H[i][i] < 0.02*mx {
			w[i] = true
		}
	}
	return w
}
