package slam

// writeTestState builds a minimal version-controlled (or deliberately
// broken) savedState and writes it atomically.
func writeTestState(path string, version int, dangling bool) error {
	st := savedState{Version: version, Config: DefaultConfig()}
	st.Nodes = []*Keyframe{{
		ID: 0, FrameID: 0, Time: 0, Pose: Pose{}, PoseOdom: Pose{},
		SurfaceBase: Pose{}, Scan: []Point{}, Surface: []Point{},
	}}
	if dangling {
		st.Edges = []*Edge{{I: 0, J: 7}}
	}
	st.GridRes = st.Config.MapResolution
	st.GridW, st.GridH = 2, 2
	st.GridValues = make([]float64, 4)
	return writeStateAtomic(path, st)
}

// savedStateV1 is the exact historical wire layout: the same field names as
// savedState minus the v2-only Session/FrameIDBase/AwaitAnchor, and the same
// Keyframe wire type (gob fills missing fields with zero values on decode).
type savedStateV1 struct {
	Version    int
	Config     Config
	Nodes      []*Keyframe
	Edges      []*Edge
	Traj       []TrajectoryPoint
	FrameCount int64
	LastPose   Pose
	LastOdom   Pose
	Started    bool
	LastKF     int
	LastKFOdom Pose
	LastScan   []Point
	Decisions  []FrameDecision
	LoopLog    []LoopDecision
	OptLog     []OptLogEntry
	GridRes    float64
	GridOX     float64
	GridOY     float64
	GridW      int
	GridH      int
	GridValues []float64
}

// ExportWriteV1State writes a genuine v1-layout state (no v2 fields) to
// exercise forward migration on load.
func ExportWriteV1State(path string) error {
	st := savedStateV1{
		Version: 1, Config: DefaultConfig(),
		Nodes:   []*Keyframe{{ID: 0, FrameID: 0}},
		GridRes: DefaultConfig().MapResolution,
		GridW:   2, GridH: 2, GridValues: make([]float64, 4),
	}
	return writeGobAtomic(path, &st)
}
