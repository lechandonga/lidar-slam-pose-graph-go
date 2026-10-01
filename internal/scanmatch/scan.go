package scanmatch

import (
	"math"

	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/geom"
	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/sensor"
)

// ValidPoints converts a raw scan to body-frame Cartesian points, dropping
// no-return, too-close and out-of-range beams. Pure function, deterministic.
func ValidPoints(s sensor.Scan) []geom.Point {
	pts := make([]geom.Point, 0, len(s.Ranges))
	for i, r := range s.Ranges {
		if math.IsNaN(r) || r <= s.RangeMin || r >= s.RangeMax {
			continue
		}
		a := s.AngleMin + float64(i)*s.AngleInc
		pts = append(pts, geom.Point{X: r * math.Cos(a), Y: r * math.Sin(a)})
	}
	return pts
}

// Reason explains why a frame's scan outcome was accepted, rejected or fused
// in a degraded way. Values are part of the persisted log contract.
type Reason string

const (
	ReasonOK                 Reason = "scan_match_ok"
	ReasonRejectBadFrame     Reason = "reject_bad_frame"
	ReasonRejectFewPoints    Reason = "reject_few_points"
	ReasonRejectHighResidual Reason = "reject_high_residual"
	ReasonRejectDeviation    Reason = "reject_deviation_too_large"
	ReasonDegenerate         Reason = "degrade_degenerate_geometry"
	ReasonNoField            Reason = "degrade_no_local_map"
)
