package slam

import "math"

// EdgeKind distinguishes odometry chain edges from loop edges.
type EdgeKind int

const (
	EdgeOdom EdgeKind = iota
	EdgeLoop
	EdgeReloc // cross-session global-localization anchor constraint
)

// LoopStatus values make contradictory constraints distinguishable.
type LoopStatus string

const (
	LoopAccepted LoopStatus = "accepted"
	LoopSoft     LoopStatus = "soft_downweighted"
	LoopRejected LoopStatus = "rejected"
)

// Edge is a relative SE2 constraint between two keyframes.
type Edge struct {
	I, J    int
	Kind    EdgeKind
	Measure Pose
	// Info is the 3x3 information matrix (order x, y, theta) already scaled
	// by any robust down-weighting; RawInfo is the unscaled original.
	Info    [3][3]float64
	RawInfo [3][3]float64
	Status  LoopStatus
	Chi2    float64 // residual energy evaluated at insertion time
	Reason  string  // rejection / downweight reason for loops
}

// Keyframe stores an optimized pose and a bounded, subsampled scan.
type Keyframe struct {
	ID       int
	FrameID  int64
	Seq      int64 // pipeline-wide unique frame sequence (FrameID/Time restart per session)
	Session  int   // session that created this keyframe
	Time     float64
	Pose     Pose // optimized, world frame
	PoseOdom Pose // odom-frame pose at capture
	Scan     []Point
	// surfaceBase is the pose at which Surface was rasterized; after graph
	// optimization the surface is corrected to the current Pose.
	surfaceBase Pose
	// Surface is the dense local wall surface (rasterized hit cells in the
	// world frame at capture time) used for scan-matching references.
	Surface   []Point
	UpdatedAt int64
}

// Graph is the pose graph. Access in Pipeline is serialized by a mutex; the
// type itself is deterministic so the optimizer is unit testable.
type Graph struct {
	Nodes []*Keyframe
	Edges []*Edge
}

// OptimizeResult reports convergence diagnostics.
type OptimizeResult struct {
	Iterations  int
	Converged   bool
	FinalDelta  float64
	InitialCost float64
	FinalCost   float64
}

// edgeError returns the standard SE2 pose-graph error for edge with measure z:
//
//	e_ij = [ R_i^T (t_j - t_i) ; norm(theta_j-theta_i-theta_z) ]
//	                         minus the translational part of z
//
// i.e. translation components expressed in node i's frame.
func edgeError(xi, xj, z Pose) [3]float64 {
	dx, dy := xj.X-xi.X, xj.Y-xi.Y
	c, s := math.Cos(xi.Theta), math.Sin(xi.Theta)
	ex := c*dx + s*dy - z.X
	ey := -s*dx + c*dy - z.Y
	eth := normAngle(xj.Theta - xi.Theta - z.Theta)
	return [3]float64{ex, ey, eth}
}

// optimizeGraph runs damped Gauss-Newton with Huber weighting on loop edges.
// Node 0 is fixed. Iteration order is fixed; repeated runs on the same graph
// produce the same result.
func optimizeGraph(g *Graph, cfg Config) OptimizeResult {
	out := OptimizeResult{}
	n := len(g.Nodes)
	out.InitialCost = graphCost(g)
	if n <= 1 {
		out.Converged = true
		out.FinalCost = out.InitialCost
		return out
	}
	for it := 0; it < cfg.OptMaxIter; it++ {
		prevCost := graphCost(g)
		savedPoses := make([]Pose, n)
		for i, nd := range g.Nodes {
			savedPoses[i] = nd.Pose
		}
		// Block-sparse normal equations: one 3x3 diagonal block per node
		// and one 3x3 off-diagonal block per edge. This keeps each GN
		// iteration O(nodes+edges) regardless of run length.
		diag := make([][3][3]float64, n)
		off := make(map[[2]int][3][3]float64, len(g.Edges))
		b := make([]float64, 3*n)
		for _, e := range g.Edges {
			xi, xj := g.Nodes[e.I].Pose, g.Nodes[e.J].Pose
			err := edgeError(xi, xj, e.Measure)
			rw := 1.0
			if e.Kind == EdgeLoop || e.Kind == EdgeReloc {
				rw = huberWeight(err, e.Info, cfg.OptHuberDelta)
			}
			Ai, Aj := edgeJacobians(xi, xj)
			// Weighted Jacobians.
			var WA, WB [3][3]float64
			for r := 0; r < 3; r++ {
				for c := 0; c < 3; c++ {
					s := 0.0
					for k := 0; k < 3; k++ {
						s += rw * e.Info[r][k] * Ai[k][c]
					}
					WA[r][c] = s
					s = 0
					for k := 0; k < 3; k++ {
						s += rw * e.Info[r][k] * Aj[k][c]
					}
					WB[r][c] = s
				}
			}
			if e.I != 0 {
				diag[e.I] = add33(diag[e.I], mulAtB(Ai, WA))
			}
			if e.J != 0 {
				diag[e.J] = add33(diag[e.J], mulAtB(Aj, WB))
			}
			if e.I != 0 && e.J != 0 {
				blk := mulAtB(Ai, WB) // H_ij = Ai^T W Aj
				key := [2]int{e.I, e.J}
				off[key] = add33(off[key], blk)
			}
			var gI, gJ [3]float64
			for r := 0; r < 3; r++ {
				for k := 0; k < 3; k++ {
					gI[r] += WA[k][r] * err[k]
					gJ[r] += WB[k][r] * err[k]
				}
				// gI/gJ are the positive gradient components; the normal
				// equations are H dx = -grad.
				if e.I != 0 {
					b[3*e.I+r] -= gI[r]
				}
				if e.J != 0 {
					b[3*e.J+r] -= gJ[r]
				}
			}
		}
		diag[0] = [3][3]float64{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}
		// Node 0 is fixed at identity: its b is zero, its diagonal is set to
		// identity and coupling blocks are zeroed. To keep H symmetric, the
		// diagonal blocks of other nodes must also drop contributions that
		// came from edges incident on node 0. Rebuild those diagonal blocks
		// is avoided by constructing a reduced system (nodes 1..n-1).
		// Levenberg-Marquardt damping ladder: the first loop-closure step
		// can temporarily raise chain residuals, so a single rejected step
		// must not terminate optimization. Retry with stronger damping;
		// accept only a cost-decreasing step. A genuinely contradictory
		// constraint never gets a decreasing step and therefore can never
		// move the graph (Huber additionally bounds loop influence).
		lambdas := []float64{1e-4, 1e-3, 1e-2, 0.1, 1.0, 10.0}
		accepted := false
		var maxd float64
		for _, lam := range lambdas {
			Hd := make([][3][3]float64, n)
			for i := 1; i < n; i++ {
				Hd[i][0][0] = diag[i][0][0] * (1 + lam)
				Hd[i][1][1] = diag[i][1][1] * (1 + lam)
				Hd[i][2][2] = diag[i][2][2] * (1 + lam)
				Hd[i][0][1] = diag[i][0][1]
				Hd[i][1][0] = diag[i][1][0]
				Hd[i][0][2] = diag[i][0][2]
				Hd[i][2][0] = diag[i][2][0]
				Hd[i][1][2] = diag[i][1][2]
				Hd[i][2][1] = diag[i][2][1]
			}
			Hd[0] = diag[0]
			dx := solveBlockPCG(Hd, off, b, n)
			bad := false
			for _, v := range dx {
				if math.IsNaN(v) || math.IsInf(v, 0) {
					bad = true
					break
				}
			}
			if bad {
				continue
			}
			const maxStepTrans = 5.0
			const maxStepRot = 2.0
			for i := 1; i < n; i++ {
				mt := math.Max(math.Abs(dx[3*i]), math.Abs(dx[3*i+1]))
				if mt > maxStepTrans {
					s := maxStepTrans / mt
					dx[3*i] *= s
					dx[3*i+1] *= s
					dx[3*i+2] *= s
				}
				if t := math.Abs(dx[3*i+2]); t > maxStepRot {
					dx[3*i+2] *= maxStepRot / t
				}
			}
			for i := 1; i < n; i++ {
				g.Nodes[i].Pose = Pose{
					X:     savedPoses[i].X + dx[3*i],
					Y:     savedPoses[i].Y + dx[3*i+1],
					Theta: normAngle(savedPoses[i].Theta + dx[3*i+2]),
				}
			}
			if graphCost(g) < prevCost {
				accepted = true
				for i := 1; i < n; i++ {
					for k := 0; k < 3; k++ {
						if d := math.Abs(dx[3*i+k]); d > maxd {
							maxd = d
						}
					}
				}
				break
			}
		}
		if !accepted {
			for i := range g.Nodes {
				g.Nodes[i].Pose = savedPoses[i]
			}
			// No cost-decreasing step even at maximum damping: the graph is
			// at a local minimum under current robust weights; stop.
			out.Converged = true
			break
		}
		out.Iterations = it + 1
		out.FinalDelta = maxd
		if maxd < cfg.OptConvergeDelta {
			out.Converged = true
			break
		}
	}
	out.FinalCost = graphCost(g)
	return out
}

// mulAtB returns A^T B.
func mulAtB(A, B [3][3]float64) [3][3]float64 {
	var R [3][3]float64
	for r := 0; r < 3; r++ {
		for c := 0; c < 3; c++ {
			s := 0.0
			for k := 0; k < 3; k++ {
				s += A[k][r] * B[k][c]
			}
			R[r][c] = s
		}
	}
	return R
}

func add33(a, b [3][3]float64) [3][3]float64 {
	for r := 0; r < 3; r++ {
		for c := 0; c < 3; c++ {
			a[r][c] += b[r][c]
		}
	}
	return a
}

// edgeJacobians returns A_i, A_j of edgeError w.r.t. WORLD-frame
// perturbations (the optimizer adds dx directly to X/Y/Theta).
func edgeJacobians(xi, xj Pose) (Ai, Aj [3][3]float64) {
	dx, dy := xj.X-xi.X, xj.Y-xi.Y
	ci, si := math.Cos(xi.Theta), math.Sin(xi.Theta)
	Ai = [3][3]float64{
		{-ci, -si, si*dx - ci*dy},
		{si, -ci, ci*dx + si*dy},
		{0, 0, -1},
	}
	Aj = [3][3]float64{
		{ci, si, 0},
		{-si, ci, 0},
		{0, 0, 1},
	}
	return Ai, Aj
}

// edgeJacobiansNumeric computes Jacobians by central differences in world
// frame, used to validate the analytic derivatives.
func edgeJacobiansNumeric(xi, xj, z Pose) (Ai, Aj [3][3]float64) {
	h := 1e-6
	e0 := edgeError(xi, xj, z)
	pe := func(p Pose, d [3]float64) Pose {
		return Pose{X: p.X + d[0], Y: p.Y + d[1], Theta: normAngle(p.Theta + d[2])}
	}
	for c := 0; c < 3; c++ {
		var dp [3]float64
		dp[c] = h
		ep := edgeError(pe(xi, dp), xj, z)
		em := edgeError(pe(xi, [3]float64{-dp[0], -dp[1], -dp[2]}), xj, z)
		for r := 0; r < 3; r++ {
			Ai[r][c] = (ep[r] - em[r]) / (2 * h)
		}
		ep = edgeError(xi, pe(xj, dp), z)
		em = edgeError(xi, pe(xj, [3]float64{-dp[0], -dp[1], -dp[2]}), z)
		for r := 0; r < 3; r++ {
			Aj[r][c] = (ep[r] - em[r]) / (2 * h)
		}
	}
	_ = e0
	return Ai, Aj
}

func graphCost(g *Graph) float64 {
	c := 0.0
	for _, e := range g.Edges {
		err := edgeError(g.Nodes[e.I].Pose, g.Nodes[e.J].Pose, e.Measure)
		for i := 0; i < 3; i++ {
			for j := 0; j < 3; j++ {
				c += err[i] * e.Info[i][j] * err[j]
			}
		}
	}
	return c
}

func huberWeight(e [3]float64, info [3][3]float64, delta float64) float64 {
	q := 0.0
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			q += e[i] * info[i][j] * e[j]
		}
	}
	if q <= delta*delta {
		return 1
	}
	return delta / math.Sqrt(q)
}

// solveBlockPCG solves H x = b on a pose graph with 3x3 diagonal blocks and
// sparse symmetric off-diagonal blocks, using preconditioned conjugate
// gradients with block-Jacobi preconditioning. Deterministic fixed order.
// solvePoseGraph builds the dense symmetric positive-(semi)definite normal
// system from block data and solves it by Gauss elimination with partial
// pivoting. Keyframe counts are bounded (MaxKeyframes), so the O(n^3) cost
// stays small; node 0 rows/columns are the identity constraint.
func solveBlockPCG(diag [][3][3]float64, off map[[2]int][3][3]float64,
	b []float64, n int) []float64 {
	nn := 3 * n
	A := make([]float64, nn*nn)
	for i := 0; i < n; i++ {
		for r := 0; r < 3; r++ {
			for c := 0; c < 3; c++ {
				A[(3*i+r)*nn+3*i+c] = diag[i][r][c]
			}
		}
	}
	for key, blk := range off {
		i, j := key[0], key[1]
		for r := 0; r < 3; r++ {
			for c := 0; c < 3; c++ {
				A[(3*i+r)*nn+3*j+c] = blk[r][c]
				A[(3*j+c)*nn+3*i+r] = blk[r][c]
			}
		}
	}
	x, ok := denseSolve(A, append([]float64(nil), b...), nn)
	if !ok {
		return make([]float64, nn)
	}
	return x
}

// denseSolve Gaussian-eliminates A x = b (A row-major n*n) with pivoting.
func denseSolve(A, b []float64, n int) ([]float64, bool) {
	for col := 0; col < n; col++ {
		piv := col
		for r := col + 1; r < n; r++ {
			if math.Abs(A[r*n+col]) > math.Abs(A[piv*n+col]) {
				piv = r
			}
		}
		if math.Abs(A[piv*n+col]) < 1e-14 {
			return nil, false
		}
		if piv != col {
			A[piv*n+col], A[col*n+col] = A[col*n+col], A[piv*n+col]
			for k := col + 1; k < n; k++ {
				A[piv*n+k], A[col*n+k] = A[col*n+k], A[piv*n+k]
			}
			b[piv], b[col] = b[col], b[piv]
		}
		for r := 0; r < n; r++ {
			if r == col {
				continue
			}
			f := A[r*n+col] / A[col*n+col]
			if f == 0 {
				continue
			}
			for k := col; k < n; k++ {
				A[r*n+k] -= f * A[col*n+k]
			}
			b[r] -= f * b[col]
		}
	}
	x := make([]float64, n)
	for i := 0; i < n; i++ {
		x[i] = b[i] / A[i*n+i]
	}
	return x, true
}

func inv3(A [3][3]float64) [3][3]float64 {
	// Cofactor matrix / determinant.
	var c [3][3]float64
	c[0][0] = A[1][1]*A[2][2] - A[1][2]*A[2][1]
	c[0][1] = A[1][2]*A[2][0] - A[1][0]*A[2][2]
	c[0][2] = A[1][0]*A[2][1] - A[1][1]*A[2][0]
	c[1][0] = A[0][2]*A[2][1] - A[0][1]*A[2][2]
	c[1][1] = A[0][0]*A[2][2] - A[0][2]*A[2][0]
	c[1][2] = A[0][1]*A[2][0] - A[0][0]*A[2][1]
	c[2][0] = A[0][1]*A[1][2] - A[0][2]*A[1][1]
	c[2][1] = A[0][2]*A[1][0] - A[0][0]*A[1][2]
	c[2][2] = A[0][0]*A[1][1] - A[0][1]*A[1][0]
	det := A[0][0]*c[0][0] + A[0][1]*c[1][0] + A[0][2]*c[2][0]
	if math.Abs(det) < 1e-18 {
		return [3][3]float64{{1e6, 0, 0}, {0, 1e6, 0}, {0, 0, 1e6}}
	}
	// Inverse is the TRANSPOSE of the cofactor matrix / det.
	var inv [3][3]float64
	for r := 0; r < 3; r++ {
		for k := 0; k < 3; k++ {
			inv[k][r] = c[r][k] / det
		}
	}
	return inv
}
