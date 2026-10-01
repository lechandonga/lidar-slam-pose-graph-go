package slam

import "math"

// likelihoodField is a Gaussian-blurred obstacle grid built from reference
// (keyframe) points in world coordinates.
type likelihoodField struct {
	res              float64
	inflate          float64
	originX, originY float64
	w, h             int
	dist             []float64 // distance to nearest reference point, m
	sigma2           float64
}

func buildLikelihoodField(points []Point, res, inflate float64) *likelihoodField {
	lf := &likelihoodField{res: res, inflate: inflate, sigma2: inflate * inflate}
	if len(points) == 0 {
		return lf
	}
	minX, minY := points[0].X, points[0].Y
	maxX, maxY := minX, minY
	for _, p := range points[1:] {
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
	pad := inflate + 0.2
	minX -= pad
	minY -= pad
	maxX += pad
	maxY += pad
	lf.originX, lf.originY = minX, minY
	lf.w = int((maxX-minX)/res) + 2
	lf.h = int((maxY-minY)/res) + 2
	if lf.w < 4 {
		lf.w = 4
	}
	if lf.h < 4 {
		lf.h = 4
	}
	big := math.MaxFloat32
	lf.dist = make([]float64, lf.w*lf.h)
	for i := range lf.dist {
		lf.dist[i] = big
	}
	for _, p := range points {
		cx := int(math.Floor((p.X - minX) / res))
		cy := int(math.Floor((p.Y - minY) / res))
		if cx >= 0 && cy >= 0 && cx < lf.w && cy < lf.h {
			lf.dist[cy*lf.w+cx] = 0
		}
	}
	lf.edt()
	return lf
}

func (f *likelihoodField) cell(x, y float64) (int, int) {
	return int(math.Floor((x - f.originX) / f.res)), int(math.Floor((y - f.originY) / f.res))
}

// response is exp(-d^2/(2 sigma^2)), in [0,1].
func (f *likelihoodField) responseAt(x, y float64) float64 {
	cx, cy := f.cell(x, y)
	if cx < 0 || cy < 0 || cx >= f.w || cy >= f.h {
		return 0
	}
	d := f.dist[cy*f.w+cx]
	if d > 3*f.inflate {
		return 0
	}
	return math.Exp(-(d * d) / (2 * f.sigma2))
}

func (f *likelihoodField) score(p Pose, scan []Point) (float64, int) {
	sum, n := 0.0, 0
	for _, pt := range scan {
		w := p.Transform(pt)
		cx, cy := f.cell(w.X, w.Y)
		if cx < 0 || cy < 0 || cx >= f.w || cy >= f.h {
			continue
		}
		n++
		d := f.dist[cy*f.w+cx]
		sum += math.Exp(-(d * d) / (2 * f.sigma2))
	}
	if n == 0 {
		return 0, 0
	}
	return sum / float64(len(scan)), n
}

// overlapFraction is a cheap coarse score: fraction of scan points whose
// rasterized cell is within one cell of a reference hit cell (Chebyshev
// distance <= 1). No transcendental functions.
func (f *likelihoodField) overlapFraction(p Pose, scan []Point) float64 {
	hit := 0
	for _, pt := range scan {
		w := p.Transform(pt)
		cx, cy := f.cell(w.X, w.Y)
		found := false
		for dy := -1; dy <= 1 && !found; dy++ {
			for dx := -1; dx <= 1; dx++ {
				xx, yy := cx+dx, cy+dy
				if xx < 0 || yy < 0 || xx >= f.w || yy >= f.h {
					continue
				}
				if f.dist[yy*f.w+xx] <= f.inflate {
					found = true
					break
				}
			}
		}
		if found {
			hit++
		}
	}
	if len(scan) == 0 {
		return 0
	}
	return float64(hit) / float64(len(scan))
}

// edt computes the exact squared Euclidean distance transform in grid units
// using the two-pass Felzenszwalb-Huttenlocher parabola-envelope method,
// then converts back to metres. Deterministic and allocation-bounded.
func (f *likelihoodField) edt() {
	w, h, d := f.w, f.h, f.dist
	vert := make([]float64, w*h)
	tmp := make([]float64, maxInt(w, h))
	v := make([]int, maxInt(w, h))
	z := make([]float64, maxInt(w, h)+1)
	for x := 0; x < w; x++ {
		for y := 0; y < h; y++ {
			v := d[y*w+x]
			if v == 0 {
				tmp[y] = 0
			} else {
				tmp[y] = math.MaxFloat32
			}
		}
		edtSquared1d(tmp[:h], v, z)
		for y := 0; y < h; y++ {
			vert[y*w+x] = tmp[y]
		}
	}
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			tmp[x] = vert[y*w+x]
		}
		edtSquared1d(tmp[:w], v, z)
		for x := 0; x < w; x++ {
			s := tmp[x]
			if s >= math.MaxFloat32/2 {
				d[y*w+x] = math.MaxFloat32
			} else {
				d[y*w+x] = math.Sqrt(s) * f.res
			}
		}
	}
}

func edtSquared1d(d []float64, v []int, z []float64) {
	n := len(d)
	kk := 0
	v[0] = 0
	z[0] = math.Inf(-1)
	z[1] = math.Inf(1)
	for q := 1; q < n; q++ {
		s := 0.0
		for {
			i := v[kk]
			s = ((d[q] + float64(q*q)) - (d[i] + float64(i*i))) / (2 * float64(q-i))
			if s > z[kk] {
				break
			}
			kk--
			if kk < 0 {
				kk = 0
				break
			}
		}
		kk++
		v[kk] = q
		z[kk] = s
		z[kk+1] = math.Inf(1)
	}
	k := 0
	for q := 0; q < n; q++ {
		for z[k+1] < float64(q) {
			k++
		}
		dd := float64(q - v[k])
		val := d[v[k]]
		if val >= math.MaxFloat32/2 {
			d[q] = math.MaxFloat32
		} else {
			d[q] = val + dd*dd
		}
	}
}

func maxInt(a, b int) int {
	if a > b {
		return a
	}
	return b
}
