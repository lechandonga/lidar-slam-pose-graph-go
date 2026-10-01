package slam

// Config collects every tunable parameter of the pipeline.
// Distances are meters, angles radians, weights are information-like scales.
type Config struct {
	// Scan geometry / preprocessing
	MinValidPoints int     // frames with fewer valid returns are rejected
	MaxRange       float64 // beams beyond this are treated as no-return
	MinRange       float64 // beams closer than this are treated as noise

	// Scan matching (Gauss-Newton on a local distance-transform field)
	MatchMaxIter      int
	MatchConvergeStep float64 // stop when |delta pose| (m, rad) drops below
	MatchMaxError     float64 // mean residual above this rejects the match
	MatchMaxPosErr    float64 // |scan vs prediction| translation limit (m)
	MatchMaxAngErr    float64 // |scan vs prediction| angle limit (rad)
	MinInfoEigenvalue float64 // smallest JtJ eigenvalue: degeneracy gate
	MaxResidualSigma  float64 // per-point Huber cutoff in cells

	// Fusion
	ScanWeight  float64 // information weight of a healthy scan match
	OdomWeight  float64 // information weight of wheel odometry
	RestartJump float64 // odom jump larger than this between frames = restart

	// Keyframes / local map
	KeyframeDist    float64 // translational travel to spawn a keyframe (m)
	KeyframeAngle   float64 // rotational travel to spawn a keyframe (rad)
	LocalKeyframes  int     // number of recent keyframes in the local field
	SeqInfoXY       float64 // information of sequential keyframe links (x,y)
	SeqInfoTh       float64 // information of sequential keyframe links (theta)
	LocalFieldRange float64 // effective sensor radius used for matching (m)

	// Loop closure
	LoopMinFrames   int     // do not close against very recent nodes
	LoopSearchDist  float64 // candidate radius in the current estimate (m)
	LoopMaxHeadDiff float64 // heading must match at a true revisit (rad)
	LoopMaxError    float64 // matched mean residual required to accept (m)
	LoopMaxPosErr   float64 // match-vs-hypothesis translation limit (m)
	LoopMaxAngErr   float64 // match-vs-hypothesis angle limit (rad)
	LoopSoftChi2    float64 // chi2 under this: full weight
	LoopRejectChi2  float64 // chi2 above this: constraint rejected
	LoopTryEvery    int     // attempt detection every N processed frames

	// Graph optimization
	OptMaxIter      int
	OptConvergeChi2 float64 // relative chi2 decrease to declare convergence
	OptConvergeStep float64 // max pose update below which we stop
	HuberDelta      float64 // robust-kernel delta for every edge

	// Grid map
	MapResolution  float64 // meters per cell
	MapHalfSize    float64 // map covers [-half, half] meters on each axis
	MapHitLogOdds  float64
	MapFreeLogOdds float64
	MapMinLogOdds  float64
	MapMaxLogOdds  float64

	// Deterministic RNG seed (data generation / jitter).
	RNGSeed int64
}

// DefaultConfig returns conservative values used by the demo and tests.
func DefaultConfig() Config {
	return Config{
		MinValidPoints: 30,
		MaxRange:       20.0,
		MinRange:       0.15,

		MatchMaxIter:      20,
		MatchConvergeStep: 1e-4,
		MatchMaxError:     0.25,
		MatchMaxPosErr:    0.6,
		MatchMaxAngErr:    0.35,
		MinInfoEigenvalue: 0.08,
		MaxResidualSigma:  3.0,

		ScanWeight:  9.0,
		OdomWeight:  1.0,
		RestartJump: 3.0,

		KeyframeDist:    0.5,
		KeyframeAngle:   0.35,
		LocalKeyframes:  8,
		SeqInfoXY:       2.0,
		SeqInfoTh:       4.0,
		LocalFieldRange: 20.0,

		LoopMinFrames:   25,
		LoopSearchDist:  1.5,
		LoopMaxHeadDiff: 0.8,
		LoopMaxError:    0.12,
		LoopMaxPosErr:   1.0,
		LoopMaxAngErr:   0.6,
		LoopSoftChi2:    4.0,
		LoopRejectChi2:  25.0,
		LoopTryEvery:    1,

		OptMaxIter:      30,
		OptConvergeChi2: 1e-4,
		OptConvergeStep: 1e-5,
		HuberDelta:      1.0,

		MapResolution:  0.05,
		MapHalfSize:    20.0,
		MapHitLogOdds:  0.9,
		MapFreeLogOdds: -0.4,
		MapMinLogOdds:  -4.0,
		MapMaxLogOdds:  4.0,

		RNGSeed: 20261002,
	}
}
