package extended_kalman_filter

import (
	"math"

	"../kalman_filter"
	"../measurement_package"
	"../tools"
	"gonum.org/v1/gonum/mat"
)

type FusionEKF struct {
	isInitialized     bool
	previousTimeStamp float64
	Ekf               kalman_filter.KalmanFilter
}

func (filter *FusionEKF) updateQ(dt float64) {
	noiseAx := 9.0
	noiseAy := 9.0
	filter.Ekf.F_.Set(0, 2, dt)
	filter.Ekf.F_.Set(1, 3, dt)
	filter.Ekf.Q_ = mat.NewDense(4, 4, []float64{(math.Pow(dt, 4) / 4) * noiseAx, 0, (math.Pow(dt, 3) / 2) * noiseAx, 0,
		0, (math.Pow(dt, 4) / 4) * noiseAy, 0, (math.Pow(dt, 3) / 2) * noiseAy,
		(math.Pow(dt, 3) / 2) * noiseAx, 0, math.Pow(dt, 2) * noiseAx, 0,
		0, (math.Pow(dt, 3) / 2) * noiseAy, 0, math.Pow(dt, 2) * noiseAy})
}

func New() *FusionEKF {
	newEKF := FusionEKF{false, 0, *kalman_filter.New()}
	return &newEKF
}

func (filter *FusionEKF) ProcessMeasurement(measurement_pack *measurement_package.MeasurementPackage) {
	if !filter.isInitialized {
		if measurement_pack.SensorType == measurement_package.LASER {
			filter.Ekf.X_ = mat.NewDense(4, 1, []float64{measurement_pack.RawMeasurements.At(0, 0), measurement_pack.RawMeasurements.At(1, 0), 4, 1})
		} else {
			filter.Ekf.X_ = tools.ToPolar(measurement_pack)
		}
		filter.isInitialized = true
		filter.previousTimeStamp = measurement_pack.TimeStamp
		return
	}
	dt := (measurement_pack.TimeStamp - filter.previousTimeStamp) / 1000000.0
	filter.previousTimeStamp = measurement_pack.TimeStamp
	filter.updateQ(dt)
	filter.Ekf.Predict()
	if measurement_pack.SensorType == measurement_package.LASER {
		filter.Ekf.Update(measurement_pack.RawMeasurements)
	} else {
		filter.Ekf.Hj_ = tools.CalculateJacobian(filter.Ekf.X_)
		filter.Ekf.UpdateEKF(measurement_pack.RawMeasurements)
	}
}
