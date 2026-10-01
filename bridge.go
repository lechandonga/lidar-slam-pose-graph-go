package slam

// This file adapts the local data-generation helper package to the DataSource
// interface without making the core package import synth (tests and the demo
// program wire the two together; replay sources only need Frame).

// FrameSource is a simple in-memory DataSource for replay: feed frames in
// order and optionally ground-truth poses.
type FrameSource struct {
	Frames []Frame
	Truth  []Pose
	idx    int
}

// NewFrameSource wraps prebuilt frames.
func NewFrameSource(frames []Frame, truth []Pose) *FrameSource {
	return &FrameSource{Frames: frames, Truth: truth}
}

func (s *FrameSource) Next() (Frame, bool) {
	if s.idx >= len(s.Frames) {
		return Frame{}, false
	}
	f := s.Frames[s.idx]
	s.idx++
	return f, true
}

func (s *FrameSource) GroundTruth() []Pose { return s.Truth }

// Rewind resets the source so the same input can be replayed again.
func (s *FrameSource) Rewind() { s.idx = 0 }
