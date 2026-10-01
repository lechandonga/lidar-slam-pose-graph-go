package slam_test

import (
	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/sensor"
	"github.com/lechandonga/lidar-slam-pose-graph-go/internal/sim"
)

func sensorScan(ranges []float64, sp sim.ScanParams) sensor.Scan {
	return sensor.Scan{
		AngleMin: sp.AngleMin,
		AngleInc: (sp.AngleMax - sp.AngleMin) / float64(sp.Beams-1),
		RangeMin: sp.RangeMin,
		RangeMax: sp.RangeMax,
		Ranges:   ranges,
	}
}
