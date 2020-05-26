package measurement_package

import "gonum.org/v1/gonum/mat"

const (
	LASER int = 0
	RADAR int = 1
)

type MeasurementPackage struct {
	SensorType      int
	TimeStamp       float64
	RawMeasurements *mat.Dense
}

// New MeasurementPackage
func New() *MeasurementPackage {
	return &MeasurementPackage{0, 0, mat.NewDense(1, 2, nil)}
}

// New calculator
func NewMeasurementPackage(sensorType int, timeStamp float64, rawMeasurements *mat.Dense) *MeasurementPackage {
	return &MeasurementPackage{sensorType, timeStamp, rawMeasurements}
}
