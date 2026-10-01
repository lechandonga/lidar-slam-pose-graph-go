package synth

import slampkg "github.com/lechandonga/lidar-slam-pose-graph-go"

// Source turns a Generator into a replayable slampkg.DataSource.
type Source struct {
	g   *Generator
	idx int
}

func NewSource(g *Generator) *Source { return &Source{g: g} }

func (s *Source) Next() (slampkg.Frame, bool) {
	if s.idx >= s.g.Len() {
		return slampkg.Frame{}, false
	}
	st := s.g.At(s.idx)
	s.idx++
	fr := slampkg.Frame{
		Scan: slampkg.Scan{
			ID:   st.ID,
			Time: st.Time,
			Points: func() []slampkg.Point {
				out := make([]slampkg.Point, len(st.Points))
				for i, p := range st.Points {
					out[i] = slampkg.Point{X: p.X, Y: p.Y}
				}
				return out
			}(),
		},
		Odom: slampkg.Odom{
			ID: st.ID, Time: st.Time,
			Pose:  slampkg.Pose{X: st.Odom.X, Y: st.Odom.Y, Theta: st.Odom.Theta},
			Valid: st.OdomOK,
		},
		HasOdom: true,
		Restart: st.Restart,
	}
	return fr, true
}

func (s *Source) GroundTruth() []slampkg.Pose {
	src := s.g.GroundTruthPoses()
	out := make([]slampkg.Pose, len(src))
	for i, p := range src {
		out[i] = slampkg.Pose{X: p.X, Y: p.Y, Theta: p.Theta}
	}
	return out
}

// Rewind restarts replay from frame 0.
func (s *Source) Rewind() { s.idx = 0 }
