package sensor

import "time"

// Scan is one planar laser sweep in the robot body frame.
// Ranges are ordered counter-clockwise starting at AngleMin, one range per
// beam; RangeMax marks an invalid / no-return beam. Units: meters, radians.
type Scan struct {
	Time      time.Time
	AngleMin  float64
	AngleInc  float64
	RangeMin  float64
	RangeMax  float64
	Ranges    []float64
	Bad       bool   // replay layer flag: the whole frame is corrupt
	BadReason string // human readable reason carried by the data source
}

// Count returns the number of beams.
func (s Scan) Count() int { return len(s.Ranges) }

// Odom is a wheel-odometry sample: absolute pose in the odometry frame plus
// a monotonically increasing Restart counter. A restart means the odometry
// frame origin jumped (power cycle / relocalization reset).
type Odom struct {
	Time      time.Time
	PoseX     float64
	PoseY     float64
	PoseTheta float64
	Restart   int
}

// Frame is one synchronized processing unit.
type Frame struct {
	Seq  int
	Scan Scan
	Odom Odom
}
