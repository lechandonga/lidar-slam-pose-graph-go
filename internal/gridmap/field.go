package gridmap

import "math"

// Field is a local Euclidean distance-transform field sampled at the map
// resolution. Every cell also stores the world coordinates of its nearest
// occupied cell, enabling exact point-to-point residuals at wall corners
// (the raw distance surface rounds convex corners outward). Cells without
// any seed in the window carry no observation.
type Field struct {
	res              float64
	w, h             int
	originX, originY float64
	dist             []float64
	nnX, nnY         []float64
	has              []bool
	maxDist          float64
}

// BuildField computes distance + nearest-seed labels over the square window
// [cx-half, cx+half]^2. Deterministic: ties resolve to the lowest indices.
func (m *Map) BuildField(cx, cy, half float64) *Field {
	m.mu.RLock()
	res := m.res
	x0 := cx - half
	y0 := cy - half
	n := int(math.Ceil(2*half/res)) + 1
	w, h := n, n
	cxMin, cyMin := m.WorldToCell(x0, y0)
	d := make([]float64, w*h)
	lx := make([]int, w*h)
	ly := make([]int, w*h)
	has := make([]bool, w*h)
	inf := math.Inf(1)
	for i := range d {
		d[i] = inf
	}
	for gy := 0; gy < h; gy++ {
		for gx := 0; gx < w; gx++ {
			sx, sy := cxMin+gx, cyMin+gy
			if m.inBounds(sx, sy) && m.logOdds[m.index(sx, sy)] > 0 {
				idx := gy*w + gx
				d[idx] = 0
				lx[idx] = gx
				ly[idx] = gy
				has[idx] = true
			}
		}
	}
	m.mu.RUnlock()

	edtLabels(d, lx, ly, has, w, h)

	dist := make([]float64, w*h)
	nnX := make([]float64, w*h)
	nnY := make([]float64, w*h)
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			idx := y*w + x
			if math.IsInf(d[idx], 1) {
				continue
			}
			dist[idx] = math.Sqrt(d[idx]) * res
			nnX[idx] = x0 + (float64(lx[idx])+0.5)*res
			nnY[idx] = y0 + (float64(ly[idx])+0.5)*res
			has[idx] = true
		}
	}
	return &Field{
		res: res, w: w, h: h,
		originX: x0 + res/2, originY: y0 + res/2,
		dist: dist, nnX: nnX, nnY: nnY, has: has,
		maxDist: float64(w+h) * res,
	}
}

// edtBuf holds reusable scratch space for the separable label EDT.
type edtBuf struct {
	d, seedD, z []float64
	lx, ly, v   []int
	has         []bool
}

func newEDTBuf(w, h int) *edtBuf {
	n := w
	if h > n {
		n = h
	}
	return &edtBuf{
		d: make([]float64, n), seedD: make([]float64, n), z: make([]float64, n+1),
		lx: make([]int, n), ly: make([]int, n), v: make([]int, n),
		has: make([]bool, n),
	}
}

// edtLabels runs the separable Felzenszwalb squared-distance transform while
// carrying nearest-seed labels, using a single reusable scratch buffer.
func edtLabels(d []float64, lx, ly []int, has []bool, w, h int) {
	b := newEDTBuf(w, h)
	// columns
	for x := 0; x < w; x++ {
		for y := 0; y < h; y++ {
			i := y*w + x
			b.d[y], b.lx[y], b.ly[y], b.has[y] = d[i], lx[i], ly[i], has[i]
		}
		edt1D(b.d[:h], b.lx[:h], b.ly[:h], b.has[:h], b.seedD[:h], b.v[:h], b.z[:h+1])
		for y := 0; y < h; y++ {
			i := y*w + x
			d[i], lx[i], ly[i], has[i] = b.d[y], b.lx[y], b.ly[y], b.has[y]
		}
	}
	// rows
	for y := 0; y < h; y++ {
		base := y * w
		for x := 0; x < w; x++ {
			b.d[x], b.lx[x], b.ly[x], b.has[x] = d[base+x], lx[base+x], ly[base+x], has[base+x]
		}
		edt1D(b.d[:w], b.lx[:w], b.ly[:w], b.has[:w], b.seedD[:w], b.v[:w], b.z[:w+1])
		for x := 0; x < w; x++ {
			d[base+x], lx[base+x], ly[base+x], has[base+x] = b.d[x], b.lx[x], b.ly[x], b.has[x]
		}
	}
}

// edt1D performs one lower-envelope pass. d holds accumulated squared
// distance from the orthogonal pass (0 at seeds); labels propagate seed
// coordinates. v/z are caller-provided envelope scratch arrays.
func edt1D(d []float64, lx, ly []int, has []bool, seedD []float64, v []int, z []float64) {
	n := len(d)
	first := -1
	for q := 0; q < n; q++ {
		if has[q] {
			first = q
			break
		}
	}
	if first < 0 {
		return
	}
	for q := 0; q < n; q++ {
		if has[q] {
			seedD[q] = d[q]
		}
	}
	k := 0
	v[0] = first
	z[0] = math.Inf(-1)
	z[1] = math.Inf(1)
	for q := first + 1; q < n; q++ {
		if !has[q] {
			continue
		}
		s := ((float64(q*q) + seedD[q]) - (float64(v[k]*v[k]) + seedD[v[k]])) /
			(2 * float64(q-v[k]))
		for s <= z[k] {
			k--
			s = ((float64(q*q) + seedD[q]) - (float64(v[k]*v[k]) + seedD[v[k]])) /
				(2 * float64(q-v[k]))
		}
		k++
		v[k] = q
		z[k] = s
		z[k+1] = math.Inf(1)
	}
	j := 0
	for q := 0; q < n; q++ {
		for z[j+1] < float64(q) {
			j++
		}
		src := v[j]
		dd := float64(q - src)
		d[q] = dd*dd + seedD[src]
		lx[q] = lx[src]
		ly[q] = ly[src]
		has[q] = true
	}
}

// Sample returns bilinear distance, gradient, nearest occupied point and
// observation flag at a world position.
func (f *Field) Sample(x, y float64) (d, gx, gy, nnx, nny float64, ok bool) {
	fx := (x - f.originX) / f.res
	fy := (y - f.originY) / f.res
	if fx < 0 || fy < 0 || fx > float64(f.w-1)-1e-9 || fy > float64(f.h-1)-1e-9 {
		return f.maxDist, 0, 0, 0, 0, false
	}
	x0 := int(math.Floor(fx))
	y0 := int(math.Floor(fy))
	tx := fx - float64(x0)
	ty := fy - float64(y0)
	i00 := y0*f.w + x0
	i10 := i00 + 1
	i01 := i00 + f.w
	i11 := i01 + 1
	if !f.has[i00] || !f.has[i10] || !f.has[i01] || !f.has[i11] {
		return f.maxDist, 0, 0, 0, 0, false
	}
	d00, d10 := f.dist[i00], f.dist[i10]
	d01, d11 := f.dist[i01], f.dist[i11]
	d = (1-tx)*(1-ty)*d00 + tx*(1-ty)*d10 + (1-tx)*ty*d01 + tx*ty*d11
	gx = ((1-ty)*(d10-d00) + ty*(d11-d01)) / f.res
	gy = ((1-tx)*(d01-d00) + tx*(d11-d10)) / f.res
	nnx = (1-tx)*(1-ty)*f.nnX[i00] + tx*(1-ty)*f.nnX[i10] + (1-tx)*ty*f.nnX[i01] + tx*ty*f.nnX[i11]
	nny = (1-tx)*(1-ty)*f.nnY[i00] + tx*(1-ty)*f.nnY[i10] + (1-tx)*ty*f.nnY[i01] + tx*ty*f.nnY[i11]
	return d, gx, gy, nnx, nny, true
}

// WindowHalf reports the half-side of the built field window in meters.
func (f *Field) WindowHalf() float64 {
	return float64(f.w) * f.res / 2
}
