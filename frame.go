package slam

import "math"

// validateScan applies the point-level quality gate. It returns the cleaned
// point set plus a distinct reason code when the frame is unusable.
func validateScan(cfg Config, s Scan) ([]Point, string) {
	if len(s.Points) < cfg.MinPoints {
		return nil, ReasonTooFewPoints
	}
	good := make([]Point, 0, len(s.Points))
	bad := 0
	// Angular diversity: bad frames here are bursts of near-identical rays
	// (rain/glass reflections collapsing onto one bearing).
	bearingBuckets := make(map[int]struct{})
	bucketScale := 0.05 // radians per bucket
	for _, p := range s.Points {
		if math.IsNaN(p.X) || math.IsInf(p.X, 0) ||
			math.IsNaN(p.Y) || math.IsInf(p.Y, 0) {
			bad++
			continue
		}
		r := math.Hypot(p.X, p.Y)
		if r > cfg.MaxRange || r < cfg.MinRange {
			bad++
			continue
		}
		good = append(good, p)
		b := int(math.Atan2(p.Y, p.X) / bucketScale)
		bearingBuckets[b] = struct{}{}
	}
	if len(good) < cfg.MinPoints {
		return nil, ReasonTooFewPoints
	}
	if float64(bad)/float64(len(s.Points)) > cfg.BadPointFraction {
		return nil, ReasonBadScan
	}
	if len(bearingBuckets) < cfg.MinUniqueRays {
		return nil, ReasonBadScan
	}
	return good, ReasonOK
}

// fuseDelta blends a scan-matched relative motion with the wheel-odometry
// relative motion. wxy/wth in [0,1] are the scan-match weights; zero means
// pure odometry (degraded corridor mode).
func fuseDelta(odomDelta Pose, matchDelta Pose, wxy, wth float64) Pose {
	return Pose{
		X:     (1-wxy)*odomDelta.X + wxy*matchDelta.X,
		Y:     (1-wxy)*odomDelta.Y + wxy*matchDelta.Y,
		Theta: normAngle((1-wth)*odomDelta.Theta + wth*matchDelta.Theta),
	}
}

// gaussianWeights turns sigmas into a normalized weight pair [0,1] using the
// inverse-variance blend between two estimates.
func gaussianWeights(sigMatch, sigOdom float64) float64 {
	wm := 1 / (sigMatch * sigMatch)
	wo := 1 / (sigOdom * sigOdom)
	return wm / (wm + wo)
}
