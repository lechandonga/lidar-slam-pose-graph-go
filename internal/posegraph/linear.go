package posegraph

import "math"

// solveLinear solves H x = -b for symmetric positive (semi)definite H using
// Gauss elimination with partial pivoting. Deterministic; returns ok=false
// when the matrix is singular. Works in place on copies.
func solveLinear(Hin, bin []float64, n int) ([]float64, bool) {
	A := make([]float64, n*n)
	copy(A, Hin)
	rhs := make([]float64, n)
	for i := range rhs {
		rhs[i] = -bin[i]
	}
	// Enforce symmetry numerically (guards against round-off noise).
	for i := 0; i < n; i++ {
		for j := i + 1; j < n; j++ {
			v := 0.5 * (A[i*n+j] + A[j*n+i])
			A[i*n+j] = v
			A[j*n+i] = v
		}
	}
	for k := 0; k < n; k++ {
		piv := k
		for i := k + 1; i < n; i++ {
			if math.Abs(A[i*n+k]) > math.Abs(A[piv*n+k]) {
				piv = i
			}
		}
		if math.Abs(A[piv*n+k]) < 1e-14 {
			return nil, false
		}
		if piv != k {
			for j := 0; j < n; j++ {
				A[k*n+j], A[piv*n+j] = A[piv*n+j], A[k*n+j]
			}
			rhs[k], rhs[piv] = rhs[piv], rhs[k]
		}
		for i := k + 1; i < n; i++ {
			f := A[i*n+k] / A[k*n+k]
			for j := k; j < n; j++ {
				A[i*n+j] -= f * A[k*n+j]
			}
			rhs[i] -= f * rhs[k]
		}
	}
	x := make([]float64, n)
	for i := n - 1; i >= 0; i-- {
		s := rhs[i]
		for j := i + 1; j < n; j++ {
			s -= A[i*n+j] * x[j]
		}
		x[i] = s / A[i*n+i]
	}
	return x, true
}
