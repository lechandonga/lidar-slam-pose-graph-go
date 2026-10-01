package scanmatch

import (
	"math"

	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/geom"
	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/gridmap"
)

// Result is the matcher verdict for one frame.
type Result struct {
	Pose       geom.Pose // refined pose
	MeanResid  float64   // mean point-to-surface distance in meters
	UsedPoints int
	InBounds   int // points sampled inside the field window
	Iterations int
	MinEigen   float64 // smallest eigenvalue of the 3x3 information matrix
	Converged  bool
	Reason     Reason
}

// Params controls the Gauss-Newton refinement.
type Params struct {
	MaxIter           int
	ConvergeStep      float64
	HuberSigma        float64 // meters; residual kernel width
	MinInfoEigenvalue float64
	MaxResidual       float64
	MaxPosErr         float64
	MaxAngErr         float64
}

// Match refines init against the local distance field with a coarse-to-fine
// schedule: the residual is first minimized against a smoothed (scaled)
// distance surface with relaxed convergence, then refined at full scale.
// This keeps corners and slightly stale predictions in the convergence basin.
// It never mutates inputs and always returns a fully populated Result.
func Match(points []geom.Point, init geom.Pose, field *gridmap.Field, p Params, maxPosErr, maxAngErr float64) Result {
	// Two-level coarse-to-fine: distance surface x4 (wide basin), then x1.
	coarse := p
	coarse.MaxIter = p.MaxIter / 2
	if coarse.MaxIter < 5 {
		coarse.MaxIter = 5
	}
	first := matchScaled(points, init, field, coarse, maxPosErr+0.5, maxAngErr+0.3, 4.0)
	return matchScaled(points, first.Pose, field, p, maxPosErr, maxAngErr, 1.0)
}

func matchScaled(points []geom.Point, init geom.Pose, field *gridmap.Field, p Params, maxPosErr, maxAngErr, distScale float64) Result {
	res := Result{Pose: init, Reason: ReasonOK}
	res.UsedPoints = len(points)
	if len(points) == 0 {
		res.Reason = ReasonRejectFewPoints
		return res
	}
	if field == nil {
		res.Reason = ReasonNoField
		return res
	}
	if distScale > 1.0 {
		// Coarse phase with a softened Huber cutoff; its verdict is never
		// returned to the caller, only the refined pose is.
		coarse := p
		coarse.MaxIter = coarse.MaxIter / 2
		if coarse.MaxIter < 5 {
			coarse.MaxIter = 5
		}
		r := matchScaled(points, init, field, coarse, maxPosErr+0.3, maxAngErr+0.2, distScale/2)
		init = r.Pose
	}
	cur := init
	sigma := p.HuberSigma
	var (
		Hxx, Hxy, Hxt, Hyy, Hyt, Htt float64
		bx, by, bt                   float64
	)
	for it := 0; it < p.MaxIter; it++ {
		res.Iterations = it + 1
		Hxx, Hxy, Hxt, Hyy, Hyt, Htt = 0, 0, 0, 0, 0, 0
		bx, by, bt = 0, 0, 0
		sumR, nR, nIn := 0.0, 0, 0
		st, ct := math.Sincos(cur.Theta)
		for _, q := range points {
			wx := cur.X + ct*q.X - st*q.Y
			wy := cur.Y + st*q.X + ct*q.Y
			d0, gx0, gy0, nnx, nny, ok := field.Sample(wx, wy)
			if !ok {
				continue
			}
			// Blended residual: point-to-surface (distance field) dominates,
			// plus a point-to-nearest-hit term that fixes the outward bias of
			// the rounded distance field at convex corners.
			rx := wx - nnx
			ry := wy - nny
			planeW, pointW := 0.7, 0.3
			d := (planeW*d0 + pointW*math.Hypot(rx, ry)) / distScale
			gx := (planeW*gx0 + pointW*rx/(math.Hypot(rx, ry)+1e-9)) / distScale
			gy := (planeW*gy0 + pointW*ry/(math.Hypot(rx, ry)+1e-9)) / distScale
			nIn++
			sumR += math.Abs(d)
			nR++
			// Jacobian of residual wrt (x, y, theta).
			dxTheta := -st*q.X - ct*q.Y
			dyTheta := ct*q.X - st*q.Y
			jx := gx
			jy := gy
			jt := gx*dxTheta + gy*dyTheta
			w := 1.0
			if sigma > 0 && math.Abs(d) > sigma {
				w = sigma / math.Abs(d) // Huber weight
			}
			r := d * w
			Hxx += w * jx * jx
			Hxy += w * jx * jy
			Hxt += w * jx * jt
			Hyy += w * jy * jy
			Hyt += w * jy * jt
			Htt += w * jt * jt
			bx += jx * r
			by += jy * r
			bt += jt * r
		}
		res.InBounds = nIn
		if nIn < 3 {
			res.Reason = ReasonRejectHighResidual
			return res
		}
		res.MeanResid = sumR / float64(nR) * distScale
		res.MinEigen = minEigen3(Hxx, Hxy, Hxt, Hyy, Hyt, Htt) / float64(nIn)
		dx, dy, dt, solved := solve3(Hxx, Hxy, Hxt, Hyy, Hyt, Htt, -bx, -by, -bt)
		if !solved {
			res.Reason = ReasonDegenerate
			return res
		}
		// Damped update for numerical safety on poorly conditioned frames.
		step := math.Hypot(dx, dy)
		capStep := 0.25
		if step > capStep {
			s := capStep / step
			dx, dy, dt = dx*s, dy*s, dt*s
		}
		cur = geom.Pose{X: cur.X + dx, Y: cur.Y + dy, Theta: geom.WrapAngle(cur.Theta + dt)}
		if math.Hypot(dx, dy) < p.ConvergeStep && math.Abs(dt) < p.ConvergeStep {
			res.Converged = true
			break
		}
	}
	res.Pose = cur

	posErr := math.Hypot(cur.X-init.X, cur.Y-init.Y)
	angErr := math.Abs(geom.WrapAngle(cur.Theta - init.Theta))
	switch {
	case res.MeanResid > p.MaxResidual:
		res.Reason = ReasonRejectHighResidual
	case posErr > maxPosErr || angErr > maxAngErr:
		res.Reason = ReasonRejectDeviation
	case res.MinEigen < p.MinInfoEigenvalue:
		res.Reason = ReasonDegenerate
	}
	return res
}

// solve3 solves a symmetric 3x3 positive (semi)definite system by explicit
// inversion; returns ok=false when the determinant is too small.
func solve3(a, b, c, d, e, f, gx, gy, gz float64) (x, y, z float64, ok bool) {
	// Matrix: [a b c; b d e; c e f]
	A := a*(d*f-e*e) - b*(b*f-e*c) + c*(b*e-d*c)
	if math.Abs(A) < 1e-12 {
		return 0, 0, 0, false
	}
	ia := (d*f - e*e) / A
	ib := -(b*f - e*c) / A
	ic := (b*e - d*c) / A
	id := (a*f - c*c) / A
	ie := -(a*e - b*c) / A
	iff := (a*d - b*b) / A
	x = ia*gx + ib*gy + ic*gz
	y = ib*gx + id*gy + ie*gz
	z = ic*gx + ie*gy + iff*gz
	return x, y, z, true
}

// minEigen3 returns the smallest eigenvalue of a symmetric 3x3 matrix using
// the analytic eigenvalue formula of the characteristic polynomial.
func minEigen3(a, b, c, d, e, f float64) float64 {
	m11, m22, m33 := a, d, f
	p1 := b*b + c*c + e*e
	q := (m11 + m22 + m33) / 3
	p2 := (m11-q)*(m11-q) + (m22-q)*(m22-q) + (m33-q)*(m33-q) + 2*p1
	if p2 <= 0 {
		return q
	}
	p := math.Sqrt(p2 / 6)
	// B = (M - qI)/p, det constrained to [-1,1]
	detB := (((m11-q)/p)*((m22-q)/p)*(m33-q)/p +
		2*(b/p)*(c/p)*(e/p) -
		((m11-q)/p)*(e/p)*(e/p) -
		((m22-q)/p)*(c/p)*(c/p) -
		((m33-q)/p)*(b/p)*(b/p))
	r := 0.5 * detB
	if r > 1 {
		r = 1
	}
	if r < -1 {
		r = -1
	}
	phi := math.Acos(r) / 3
	// Eigenvalues are q+2p cos(phi + k*2pi/3); the smallest uses k=1.
	return q + 2*p*math.Cos(phi+2*math.Pi/3)
}
