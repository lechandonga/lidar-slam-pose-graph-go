// Package fusion combines scan-matched increments with wheel odometry in the
// tangent space of SE(2), using fixed information weights. It also detects
// odometry frame restarts and distinct scan-matching failure modes.
package fusion

import (
	"math"

	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/geom"
	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/scanmatch"
)

// Mode records how the pose of a frame was produced.
type Mode string

const (
	ModeFused    Mode = "fused_scan_odom"
	ModeOdomOnly Mode = "odom_only"
	ModeDegraded Mode = "degraded_weak_scan"
	ModeRestart  Mode = "odom_restart_reinitialized"
)

// Weights are information-style scales (dimensionless).
type Weights struct {
	Scan float64
	Odom float64
}

// Decision is the auditable per-frame fusion verdict.
type Decision struct {
	Mode      Mode
	Reason    scanmatch.Reason
	ScanPose  geom.Pose
	Predicted geom.Pose
	Fused     geom.Pose
	ScanW     float64
	OdomW     float64
}

// Fuse produces the frame pose.
//
//	predicted: pose propagated purely by wheel odometry from the last state
//	matched:   scan-matcher result (may carry a rejection/degrade reason)
//	restart:   true when the odometry frame restarted since last frame
func Fuse(predicted geom.Pose, matched scanmatch.Result, restart bool, w Weights) Decision {
	d := Decision{
		Reason:    matched.Reason,
		ScanPose:  matched.Pose,
		Predicted: predicted,
	}
	if restart {
		d.Mode = ModeRestart
		d.Fused = matched.Pose
		if matched.Reason == scanmatch.ReasonRejectBadFrame ||
			matched.Reason == scanmatch.ReasonRejectFewPoints ||
			matched.Reason == scanmatch.ReasonRejectHighResidual ||
			matched.Reason == scanmatch.ReasonRejectDeviation {
			// No trustworthy scan: keep the predicted pose but mark reset.
			d.Fused = predicted
		}
		return d
	}
	switch matched.Reason {
	case scanmatch.ReasonRejectBadFrame,
		scanmatch.ReasonRejectFewPoints,
		scanmatch.ReasonRejectHighResidual,
		scanmatch.ReasonRejectDeviation:
		d.Mode = ModeOdomOnly
		d.Fused = predicted
		d.OdomW = w.Odom
		return d
	}
	sw := w.Scan
	mode := ModeFused
	if matched.Reason == scanmatch.ReasonDegenerate {
		sw *= 0.25
		mode = ModeDegraded
	}
	if matched.Reason == scanmatch.ReasonNoField {
		sw = 0
		mode = ModeOdomOnly
	}
	d.Mode = mode
	d.ScanW = sw
	d.OdomW = w.Odom
	d.Fused = tangentBlend(predicted, matched.Pose, w.Odom, sw)
	return d
}

// tangentBlend computes (wO * log(odom) + wS * log(scan)) relative to the
// odom prediction, i.e. a weighted average in the local tangent space.
func tangentBlend(odomPose, scanPose geom.Pose, wO, wS float64) geom.Pose {
	if wS <= 0 {
		return odomPose
	}
	delta := geom.Between(scanPose, odomPose) // odom^-1 scan
	k := wS / (wO + wS)
	part := geom.Pose{
		X:     k * delta.X,
		Y:     k * delta.Y,
		Theta: geom.WrapAngle(k * delta.Theta),
	}
	return geom.Compose(part, odomPose)
}

// IsRestart decides whether consecutive odometry samples belong to a new
// odometry frame: either the explicit counter changed or a physically
// impossible jump (larger than jumpMeters) appeared between frames.
func IsRestart(prevRestart, curRestart int, prevOdom, curOdom geom.Pose, jumpMeters float64) bool {
	if curRestart != prevRestart {
		return true
	}
	if math.Hypot(curOdom.X-prevOdom.X, curOdom.Y-prevOdom.Y) > jumpMeters {
		return true
	}
	return false
}
