package posegraph

import (
	"math"

	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/geom"
)

// OptimizeParams bounds the Gauss-Newton run and defines convergence.
type OptimizeParams struct {
	MaxIter      int
	ConvergeChi2 float64 // stop on relative chi2 decrease below this
	ConvergeStep float64 // stop when max pose update is below (m / rad)
	HuberDelta   float64 // robust kernel applied per residual component
}

// OptimizeReport describes one optimization run.
type OptimizeReport struct {
	Iterations  int
	Converged   bool
	StartChi2   float64
	FinalChi2   float64
	MaxStep     float64
	ActiveEdges int
}

// Optimize runs Gauss-Newton on active edges. Node 0 is fixed by prior to
// gauge-freedom. The result is deterministic: same graph + params + node
// order always yields identical poses (pure arithmetic, no goroutines).
func (g *Graph) Optimize(p OptimizeParams) OptimizeReport {
	g.mu.Lock()
	defer g.mu.Unlock()

	active := make([]int, 0, len(g.edges))
	for i, e := range g.edges {
		if e.Active() {
			active = append(active, i)
		}
	}
	rep := OptimizeReport{ActiveEdges: len(active), Converged: false}
	if len(g.nodes) <= 1 || len(active) == 0 {
		rep.Converged = true
		rep.StartChi2, rep.FinalChi2 = g.chi2Locked(active), g.chi2Locked(active)
		return rep
	}
	rep.StartChi2 = g.chi2Locked(active)
	prevChi2 := rep.StartChi2

	n := len(g.nodes)
	dim := 3 * n
	H := make([]float64, dim*dim)
	b := make([]float64, dim)
	lambda := 1e-3

	for it := 0; it < p.MaxIter; it++ {
		rep.Iterations = it + 1
		for i := range H {
			H[i] = 0
		}
		for i := range b {
			b[i] = 0
		}

		for _, ei := range active {
			e := g.edges[ei]
			err, Ja, Jb := edgeResidual(g.nodes[e.From].Pose, g.nodes[e.To].Pose, e.Rel)
			r := [3]float64{err.X, err.Y, err.Theta}
			weight := 1.0
			if p.HuberDelta > 0 {
				norm := math.Sqrt(e.Info[0][0]*r[0]*r[0] + e.Info[1][1]*r[1]*r[1] + e.Info[2][2]*r[2]*r[2])
				if norm > p.HuberDelta {
					weight = p.HuberDelta / norm
				}
			}
			if e.Status == StatusSoft {
				weight *= 0.25
			}
			addEdgeContribution(H, b, e, Ja, Jb, r, weight)
		}

		// Fix node 0 with a strong diagonal prior (gauge freedom).
		for k := 0; k < 3; k++ {
			H[k*dim+k] += 1e6
		}
		// Levenberg-Marquardt damping; steps that raise chi2 are rolled back.
		for i := 0; i < dim; i++ {
			if H[i*dim+i] < 1e-9 {
				H[i*dim+i] = 1e-9
			}
			H[i*dim+i] *= 1 + lambda
		}

		delta, ok := solveLinear(H, b, dim)
		if !ok {
			rep.Converged = false
			lambda *= 10
			if lambda > 1e8 {
				break
			}
			continue
		}
		saved := append([]Node(nil), g.nodes...)
		maxStep := 0.0
		for i := range g.nodes {
			if i == 0 {
				continue
			}
			dx, dy, dt := delta[3*i], delta[3*i+1], delta[3*i+2]
			g.nodes[i].Pose.X += dx
			g.nodes[i].Pose.Y += dy
			g.nodes[i].Pose.Theta = geom.WrapAngle(g.nodes[i].Pose.Theta + dt)
			if s := math.Hypot(dx, dy); s > maxStep {
				maxStep = s
			}
			if a := math.Abs(dt); a > maxStep {
				maxStep = a
			}
		}
		chi2 := g.chi2Locked(active)
		if chi2 > prevChi2*(1+1e-9) && lambda < 1e6 {
			g.nodes = saved
			lambda *= 10
			continue
		}
		lambda *= 0.5
		if lambda < 1e-9 {
			lambda = 1e-9
		}
		rep.MaxStep = maxStep
		rel := 0.0
		if prevChi2 > 1e-12 {
			rel = (prevChi2 - chi2) / prevChi2
		}
		prevChi2 = chi2
		rep.FinalChi2 = chi2
		if maxStep < p.ConvergeStep {
			rep.Converged = true
			break
		}
		if it > 0 && math.Abs(rel) < p.ConvergeChi2 {
			rep.Converged = true
			break
		}
	}
	if rep.FinalChi2 == 0 {
		rep.FinalChi2 = g.chi2Locked(active)
	}
	return rep
}

// edgeResidual returns e = z^-1 * a^-1 * b and analytic Jacobians for
// world-frame (left) perturbations, matching the additive pose updates used
// by Optimize. Zero error means b * a^-1 = z (b follows a via z).
func edgeResidual(a, b, z geom.Pose) (err geom.Pose, Ja, Jb [3][3]float64) {
	err = geom.Between(b, a)
	err = geom.Between(err, z)
	ang := -a.Theta - z.Theta
	sa, ca := math.Sincos(ang)
	// Left perturb b' = eps_b * b: de_xy = R(-ta-tz) eps, de_t = eps_t.
	Jb = [3][3]float64{
		{ca, -sa, 0},
		{sa, ca, 0},
		{0, 0, 1},
	}
	// de_xy = R(-ta-tz)(tb - ta) under left perturb of a.
	sx := ca*(b.X-a.X) - sa*(b.Y-a.Y)
	sy := sa*(b.X-a.X) + ca*(b.Y-a.Y)
	Ja = [3][3]float64{
		{-ca, sa, sy},
		{-sa, -ca, -sx},
		{0, 0, -1},
	}
	return err, Ja, Jb
}

func residualVec(a, b, z geom.Pose) [3]float64 {
	rel := geom.Between(b, a)
	e := geom.Between(rel, z)
	return [3]float64{e.X, e.Y, e.Theta}
}

func addEdgeContribution(H []float64, b []float64, e Edge, Ja, Jb [3][3]float64, r [3]float64, weight float64) {
	info := e.Info
	dim := int(math.Sqrt(float64(len(H))))
	add := func(node, other int, J, Jo [3][3]float64) {
		off := 3 * node
		offO := 3 * other
		// Diagonal block H += weight * J^T I J, gradient b += J^T I W r.
		for i := 0; i < 3; i++ {
			for j := 0; j < 3; j++ {
				val := 0.0
				for k := 0; k < 3; k++ {
					for rr := 0; rr < 3; rr++ {
						val += J[rr][i] * weightedInfo(info, rr, k, weight) * J[rr][j]
					}
				}
				H[(off+i)*dim+off+j] += val
			}
			g := 0.0
			for k := 0; k < 3; k++ {
				for rr := 0; rr < 3; rr++ {
					g += J[rr][i] * weightedInfo(info, rr, k, weight) * r[k]
				}
			}
			b[off+i] += g
		}
		if other >= 0 {
			for i := 0; i < 3; i++ {
				for j := 0; j < 3; j++ {
					val := 0.0
					for rr := 0; rr < 3; rr++ {
						for k := 0; k < 3; k++ {
							val += J[rr][i] * weightedInfo(info, rr, k, weight) * Jo[k][j]
						}
					}
					H[(off+i)*dim+offO+j] += val
				}
			}
		}
	}
	add(e.To, e.From, Jb, Ja)
	add(e.From, e.To, Ja, Jb)
}

func weightedInfo(info [3][3]float64, r, c int, w float64) float64 {
	return w * info[r][c]
}

// Chi2 returns the weighted squared residual sum over active edges.
func (g *Graph) Chi2() float64 {
	g.mu.RLock()
	defer g.mu.RUnlock()
	idx := make([]int, 0, len(g.edges))
	for i, e := range g.edges {
		if e.Active() {
			idx = append(idx, i)
		}
	}
	return g.chi2Locked(idx)
}

func (g *Graph) chi2Locked(active []int) float64 {
	s := 0.0
	for _, ei := range active {
		e := g.edges[ei]
		r := residualVec(g.nodes[e.From].Pose, g.nodes[e.To].Pose, e.Rel)
		w := 1.0
		if e.Status == StatusSoft {
			w = 0.25
		}
		for i := 0; i < 3; i++ {
			s += w * e.Info[i][i] * r[i] * r[i]
		}
	}
	return s
}
