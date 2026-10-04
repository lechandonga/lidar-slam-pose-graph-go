package synth

import "math"

// SplitSession is one independently recorded session covering part of a
// shared world. Its wheel odometry and clock start at zero even though the
// ground-truth start pose (StartWorld) lies somewhere inside the area an
// earlier session already mapped.
type SplitSession struct {
	// Frames in the session's own local sensor/odom frame: scan points and
	// odometry poses are expressed relative to StartWorld.
	Frames []Step
	// StartWorld is the session origin in the common world frame, i.e. the
	// ground-truth answer the global localizer has to recover.
	StartWorld Pose
	// TruthWorld is the per-frame ground-truth pose in the common world.
	TruthWorld []Pose
}

// SplitOptions configures SplitSessions.
type SplitOptions struct {
	Options Options
	// Cut1/Cut2 split full[0:Cut1] and full[Cut2:] into two sessions with a
	// required overlap (Cut2 < Cut1 means overlap). Each segment's local
	// data is re-expressed with odometry and clock reset to zero.
	Cut1, Cut2 int
}

// SplitSessions builds one ground-truth trajectory, cuts it into two
// independently-originated sessions that share an overlap, and returns both
// sessions plus the full ground-truth poses. Noise is sampled independently
// per session (distinct seeds derived from opt.Seed), as it would be across
// real power cycles.
func SplitSessions(world World, path []Pose, opt Options, cut1, cut2 int) (
	sessA, sessB SplitSession, fullTruth []Pose,
) {
	if cut2 < 0 {
		cut2 = 0
	}
	if cut1 > len(path) {
		cut1 = len(path)
	}
	build := func(seg []Pose, seed int64, origin Pose) SplitSession {
		o := opt
		o.Seed = seed
		// Re-express poses in the segment-local frame: translate and rotate
		// so the first pose is (0,0,0); scans/odom generated from this path
		// are then exactly what a freshly booted robot at origin sees.
		local := make([]Pose, len(seg))
		inv := invert(origin)
		truthWorld := make([]Pose, len(seg))
		for i, ps := range seg {
			// Full SE2 re-expression inv*p: translate to the origin AND
			// rotate so the session starts heading 0 in its own frame. The
			// generator's odometry is integrated in this local frame, so
			// the rotation is mandatory, not just the translation.
			local[i] = compose(inv, ps)
			truthWorld[i] = ps
		}
		o.Path = local
		// Scans observe the physical world: cast them at the original
		// WORLD poses while odometry integrates along the re-expressed
		// local path. Casting at local poses would ray-trace the world
		// walls from positions the robot never occupied, producing scans
		// that do not correspond to the annotated StartWorld.
		o.ScanPath = seg
		g := New(world, o)
		ss := SplitSession{StartWorld: origin, TruthWorld: truthWorld}
		for i := 0; i < g.Len(); i++ {
			st := g.At(i)
			// Re-zero times per session: Generator already numbers from 0.
			ss.Frames = append(ss.Frames, st)
		}
		return ss
	}
	fullTruth = append(fullTruth, path...)
	sessA = build(path[0:cut1], opt.Seed*7+1, path[0])
	if cut2 < len(path) {
		sessB = build(path[cut2:], opt.Seed*13+5, path[cut2])
	}
	return sessA, sessB, fullTruth
}

func invert(p Pose) Pose {
	c, s := math.Cos(p.Theta), math.Sin(p.Theta)
	return Pose{X: -c*p.X - s*p.Y, Y: s*p.X - c*p.Y,
		Theta: normAngle(-p.Theta)}
}

// SessionSource turns a SplitSession into a replayable frame source with
// independent zero-based IDs/times/odom.
func SessionSource(ss SplitSession) *Source {
	steps := make([]step, len(ss.Frames))
	for i, st := range ss.Frames {
		steps[i] = step{
			pose:  Pose{X: st.Pose.X, Y: st.Pose.Y, Theta: st.Pose.Theta},
			scan:  []Point{},
			odom:  Pose{X: st.Odom.X, Y: st.Odom.Y, Theta: st.Odom.Theta},
			valid: st.OdomOK, restart: st.Restart,
			id: int64(i), t: float64(i) * 0.1,
		}
		for _, q := range st.Points {
			steps[i].scan = append(steps[i].scan, Point{X: q.X, Y: q.Y})
		}
	}
	return NewSource(&Generator{steps: steps})
}
