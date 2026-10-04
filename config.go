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
	// RevisitSuppressionXY/Theta: when a new keyframe candidate lands this
	// close to an EXISTING keyframe (other than the immediate odom-chain
	// predecessor), the existing node is reused instead of adding a new
	// one. This keeps re-sweeping an already mapped area (load -> run a
	// short overlap leg -> save, repeated) from growing the graph and the
	// state file round after round. Genuinely new terrain stays farther
	// than this and adds nodes normally.
	RevisitSuppressionXY    float64
	RevisitSuppressionTheta float64

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
	// MapOccPersist is the log-odds level above which a cell counts as
	// confirmed occupied: free-space ray crossings never decrease it, so a
	// wall confirmed in one session cannot be erased by a later session
	// sweeping a ray across the same cell (repeated traversals stay sharp
	// instead of smearing into unknown/free).
	MapOccPersist float64

	// --- Cross-session global localization ---
	// Coarse global scan over the old map: raster spacing, angle spacing
	// and how many scan points the cheap overlap stage uses.
	LocalizeCoarseStepXY    float64
	LocalizeCoarseStepTheta float64
	LocalizeCoarseScanPts   int
	// Search radius around a user-supplied pose/heading hint.
	LocalizeHintRadiusXY    float64
	LocalizeHintRadiusTheta float64
	// Refinement windows around each surviving coarse candidate.
	LocalizeRefineWinXY    float64
	LocalizeRefineWinTheta float64
	LocalizeKFRefRadius    float64 // old keyframes within this feed the local ref
	// Decision thresholds:
	//   unique   = best score >= MinScore, best RMS <= MaxResidual and
	//              best-second score gap >= ScoreMargin;
	//   ambiguous= a spatially distinct second candidate also clears
	//              MinScore/MaxResidual within the margin;
	//   failed   = anything else (nothing is committed).
	LocalizeMinScore    float64
	LocalizeMaxResidual float64
	LocalizeScoreMargin float64
	// Information of the anchor constraint(s) inserted at session start.
	LocalizeEdgeSigmaXY float64
	LocalizeEdgeSigmaTh float64
	LocalizeMaxAnchors  int
	// CrossSessionRadius is the world-frame radius for detecting revisits
	// of old-session areas while a new session runs (odom frames differ
	// across sessions so odom-proximity loop search cannot find them).
	LocalizeCrossSessionRadius float64
	LocalizeCrossMinScore      float64 // accept only strong cross-session matches

	// --- Runtime ---
	OptEveryNKeyframes int // incremental optimization cadence
	RNGSeed            int64
	// MaxDecisionLog / MaxLoopLog / MaxOptLog bound how many per-frame
	// decision, loop-attempt and optimization log entries are retained
	// (newest kept). The logs are diagnostic and must not make a repeatedly
	// saved/loaded state grow without bound when the graph itself has
	// converged. 0 keeps every entry (used only if an embedder opts in).
	MaxDecisionLog int
	MaxLoopLog     int
	MaxOptLog      int
	// PersistLogs controls whether diagnostic logs (per-frame decisions,
	// loop attempts, optimization trace) are written into the state file.
	// They are NOT required to resume a run (the graph/trajectory/grid
	// are), so they default to off: keeping them off is what lets a
	// repeatedly loaded/re-swept/saved state over an unchanged area have a
	// byte-stable file once the graph itself has converged. In-process,
	// Decisions()/LoopDecisions()/OptLog() remain fully populated.
	PersistLogs bool
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

		KeyframeMinDist:         0.3,
		KeyframeMinTheta:        0.15,
		KeyframeMaxGap:          2.0,
		MaxScanPointsKF:         360,
		OdomEdgeSigmaXY:         0.05,
		OdomEdgeSigmaTh:         0.04,
		RevisitSuppressionXY:    0.40,
		RevisitSuppressionTheta: 0.30,

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

		OptMaxIter:       30,
		OptConvergeDelta: 1e-4,
		OptHuberDelta:    4.0,

		MapResolution: 0.05,
		MapLogOddHit:  0.55,
		MapLogOddFree: -0.30,
		MapLogOddMax:  4.0,
		MapMaxRayStep: 1000,
		MapOccPersist: 2.0,

		LocalizeCoarseStepXY:       0.5,
		LocalizeCoarseStepTheta:    0.19635, // ~11.25 degrees, 32 heading bins
		LocalizeCoarseScanPts:      90,
		LocalizeHintRadiusXY:       2.0,
		LocalizeHintRadiusTheta:    0.6,
		LocalizeRefineWinXY:        0.8,
		LocalizeRefineWinTheta:     0.35,
		LocalizeKFRefRadius:        6.0,
		LocalizeMinScore:           0.55,
		LocalizeMaxResidual:        0.20,
		LocalizeScoreMargin:        0.15,
		LocalizeEdgeSigmaXY:        0.08,
		LocalizeEdgeSigmaTh:        0.05,
		LocalizeMaxAnchors:         2,
		LocalizeCrossSessionRadius: 1.5,
		LocalizeCrossMinScore:      0.70,

		OptEveryNKeyframes: 5,
		RNGSeed:            1,
		MaxDecisionLog:     500,
		MaxLoopLog:         300,
		MaxOptLog:          200,
		PersistLogs:        false,
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
