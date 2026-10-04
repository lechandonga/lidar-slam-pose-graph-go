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
