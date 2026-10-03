package slam

import "math"

// Config holds every tunable parameter of the pipeline. Defaults returned by
// DefaultConfig are the values documented in docs/parameters.md.
type Config struct {
	// --- Scan quality gate ---
	MinPoints        int     // frames with fewer valid points are rejected
	MaxRange         float64 // metres; points farther are dropped
	MinRange         float64 // metres; points closer are dropped
	BadPointFraction float64 // fraction of NaN/Inf/out-of-range points that flags a bad frame
	MinUniqueRays    int     // distinct angular samples needed (anti-flash noise)

	// --- Scan matching ---
	FieldResolution float64 // likelihood-field cell size, metres
	FieldInflate    float64 // obstacle influence radius, metres
	MatchWinXY      float64 // coarse search half-window, metres
	MatchWinTheta   float64 // coarse search half-window, radians
	CoarseStepXY    float64
	CoarseStepTheta float64
	RefineIter      int     // Gauss-Newton refinement iterations
	RefineStep      float64 // numerical Jacobian step
	// Gate: match vs prediction. Translation metres, rotation radians.
	MaxMatchTransErr float64
	MaxMatchRotErr   float64
	// MaxDeltaConflict bounds how far the scan-match per-frame delta may
	// diverge from the odometry delta before the match is distrusted.
	MaxDeltaConflict float64
	// Degeneracy: smallest covariance eigenvalue / largest (dimensionless
	// anisotropy); below MinEigRatio the match is geometrically degenerate.
	MinEigRatio float64

	// --- Fusion (match pose delta vs wheel odom delta) ---
	SigmaMatchXY    float64
	SigmaMatchTheta float64
	SigmaOdomXY     float64
	SigmaOdomTheta  float64
	// When a match is degenerate, trust odometry this much (0 = pure odom).
	DegradedMatchWeightXY    float64
	DegradedMatchWeightTheta float64
	// Restart handling: odom delta larger than this vs the SLAM motion is
	// treated as an odometry re-initialization, not real motion.
	RestartJumpXY    float64
	RestartJumpTheta float64

	// --- Keyframes ---
	KeyframeMinDist  float64 // metres of translation since last keyframe
	KeyframeMinTheta float64 // radians of rotation since last keyframe
	KeyframeMaxGap   float64 // metres, force a keyframe at least this far
	MaxScanPointsKF  int     // subsample cap per keyframe (bounded memory)
	OdomEdgeSigmaXY  float64
	OdomEdgeSigmaTh  float64

	// --- Loop closure ---
	LoopSearchRadius float64 // metres, candidate keyframes within this
	LoopOdomRadius   float64 // metres, odom-frame revisit radius
	LoopMinFrames    int     // index separation, avoids adjacency
	LoopCheckEvery   int     // keyframe cadence for loop attempts
	LoopSigmaXY      float64
	LoopSigmaTh      float64
	// Robust gating: residual Mahalanobis energy above this is rejected;
	// between LoopAcceptChi2 and LoopSoftChi2 it is down-weighted.
	LoopAcceptChi2 float64
	LoopSoftChi2   float64
	// LoopHardRejectChi2 rejects high-confidence-looking but geometrically
	// impossible revisits (mismatched places with repeated structure).
	LoopHardRejectChi2 float64
	// LoopOdomMaxDiff* bound how far a scan loop measurement may disagree
	// with the drift-free odometry-chain relative motion.
	LoopOdomMaxDiffXY float64
	LoopOdomMaxDiffTh float64
	LoopMinScore      float64 // match score (mean field response) required
	LoopStrongScore   float64

	// --- Cross-session global localization ---
	// LocalizeMinScore is the minimum fine alignment score (inlier fraction
	// / likelihood response in [0,1]) for a start pose to be considered at
	// all. LocalizeStrongScore is required for a unique, accepted fix.
	LocalizeMinScore    float64
	LocalizeStrongScore float64
	// LocalizeAmbiguousMargin: when the best and second-best distinct
	// hypotheses score within this margin, the fix is declared ambiguous.
	LocalizeAmbiguousMargin float64
	// LocalizeMaxRMS bounds the mean point-to-surface residual (metres) of
	// an accepted fix; larger residuals are rejected as weak alignment.
	LocalizeMaxRMS float64
	// Global coarse search sampling (no hint: full 2*pi heading sweep over
	// every previously mapped keyframe position).
	LocalizeCoarseXY    float64
	LocalizeCoarseTheta float64
	// Default search windows around a user-supplied pose hint.
	LocalizeHintXYWin float64
	LocalizeHintThWin float64
	// CompactDist*: cross-session keyframes whose FINALIZED poses coincide
	// within these limits describe the same physical spot and are merged at
	// save/Finalize time so repeated sessions do not accumulate duplicate
	// nodes, edges or map cells.
	CompactDistXY float64
	CompactDistTh float64
	// MapFreezeOccupied prevents rays from clearing a cell already
	// classified occupied: re-observing a mapped wall can never erase it,
	// which is what keeps repeated sweeps stable across sessions.
	MapFreezeOccupied bool

	// --- Pose graph optimization ---
	OptMaxIter       int
	OptConvergeDelta float64 // max |dx| between iterations
	OptHuberDelta    float64 // robust kernel k (SE2 residual units mixed)

	// --- Map ---
	MapResolution float64
	MapLogOddHit  float64
	MapLogOddFree float64
	MapLogOddMax  float64
	MapMaxRayStep int // ray samples per ray cap (bounded cost)

	// --- Runtime ---
	OptEveryNKeyframes int // incremental optimization cadence
	RNGSeed            int64
}

// DefaultConfig returns validated defaults for a small indoor platform.
func DefaultConfig() Config {
	return Config{
		MinPoints:        25,
		MaxRange:         20.0,
		MinRange:         0.05,
		BadPointFraction: 0.25,
		MinUniqueRays:    20,

		FieldResolution: 0.05,
		FieldInflate:    0.30,
		MatchWinXY:      0.30,
		MatchWinTheta:   0.25,
		CoarseStepXY:    0.05,
		CoarseStepTheta: 0.04,
		RefineIter:      8,
		RefineStep:      0.01,

		MaxMatchTransErr: 0.60,
		MaxMatchRotErr:   0.45,
		MaxDeltaConflict: 0.25,
		MinEigRatio:      1e-3,

		SigmaMatchXY:             0.015,
		SigmaMatchTheta:          0.01,
		SigmaOdomXY:              0.15,
		SigmaOdomTheta:           0.1,
		DegradedMatchWeightXY:    0.2,
		DegradedMatchWeightTheta: 0.2,
		RestartJumpXY:            1.0,
		RestartJumpTheta:         1.0,

		KeyframeMinDist:  0.3,
		KeyframeMinTheta: 0.15,
		KeyframeMaxGap:   2.0,
		MaxScanPointsKF:  360,
		OdomEdgeSigmaXY:  0.05,
		OdomEdgeSigmaTh:  0.04,

		LoopSearchRadius:   1.2,
		LoopOdomRadius:     1.2,
		LoopMinFrames:      15,
		LoopCheckEvery:     2,
		LoopSigmaXY:        0.10,
		LoopSigmaTh:        0.08,
		LoopAcceptChi2:     9.0,
		LoopSoftChi2:       25.0,
		LoopHardRejectChi2: 100.0,
		LoopOdomMaxDiffXY:  2.0,
		LoopOdomMaxDiffTh:  0.3,
		LoopMinScore:       0.45,
		LoopStrongScore:    0.75,

		LocalizeMinScore:        0.55,
		LocalizeStrongScore:     0.78,
		LocalizeAmbiguousMargin: 0.08,
		LocalizeMaxRMS:          0.12,
		LocalizeCoarseXY:        0.5,
		LocalizeCoarseTheta:     0.20,
		LocalizeHintXYWin:       1.5,
		LocalizeHintThWin:       0.6,
		CompactDistXY:           0.25,
		CompactDistTh:           0.15,
		MapFreezeOccupied:       true,

		OptMaxIter:       30,
		OptConvergeDelta: 1e-4,
		OptHuberDelta:    4.0,

		MapResolution: 0.05,
		MapLogOddHit:  0.55,
		MapLogOddFree: -0.30,
		MapLogOddMax:  4.0,
		MapMaxRayStep: 1000,

		OptEveryNKeyframes: 5,
		RNGSeed:            1,
	}
}

func (c *Config) validate() {
	if c.MinPoints < 3 {
		panic("slam: MinPoints too small")
	}
	if c.FieldResolution <= 0 || c.MapResolution <= 0 {
		panic("slam: resolutions must be positive")
	}
	if math.IsNaN(c.FieldInflate) {
		panic("slam: invalid config")
	}
}
