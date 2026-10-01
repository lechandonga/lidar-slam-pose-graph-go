package slam

import (
	"time"

	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/fusion"
	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/geom"
	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/scanmatch"
)

// Keyframe is one node's stored observation. Only keyframe scans enter the
// global map, so a rebuilt map always agrees with optimized poses.
type Keyframe struct {
	NodeID int
	Seq    int
	Pose   geom.Pose
	Points []geom.Point
}

// FrameRecord is the auditable per-frame verdict appended to the log.
type FrameRecord struct {
	Seq            int
	Time           time.Time
	Mode           fusion.Mode
	Reason         scanmatch.Reason
	PredictedPose  geom.Pose
	FusedPose      geom.Pose
	ScanPose       geom.Pose
	MeanResidual   float64
	MinEigen       float64
	MatchIter      int
	ValidPoints    int
	InBounds       int
	ScanWeight     float64
	OdomWeight     float64
	Restart        bool
	NewKeyframe    bool
	NodeID         int
	LoopTried      bool
	LoopCandidates int
	LoopStatus     string // "" | accepted | soft_weighted | rejected
	LoopReason     string
	LoopChi2       float64
	OptConverged   bool
	OptIter        int
	MapRebuilt     bool
}
