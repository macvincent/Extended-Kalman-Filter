package tools

import (
	"fmt"
	"math"

	"../measurement_package"
	"gonum.org/v1/gonum/mat"
)

func CalculateJacobian(x_state *mat.Dense) *mat.Dense {
	Hj := *mat.NewDense(3, 4, nil)
	px := x_state.At(0, 0)
	py := x_state.At(1, 0)
	vx := x_state.At(2, 0)
	vy := x_state.At(3, 0)
	var px2, py2 float64 = math.Pow(px, 2), math.Pow(py, 2)
	if math.Abs(py2+px2) == 0 {
		return &Hj
	}
	Hj.Set(0, 0, px/(math.Pow((px2+py2), 0.5)))
	Hj.Set(0, 1, py/(math.Pow((px2+py2), 0.5)))
	Hj.Set(1, 0, -py/(px2+py2))
	Hj.Set(1, 1, px/(px2+py2))
	Hj.Set(2, 0, (py*(vx*py-vy*px))/math.Pow((px2+py2), 1.5))
	Hj.Set(2, 1, (px*(vy*px-vx*py))/math.Pow((px2+py2), 1.5))
	Hj.Set(2, 2, px/(math.Pow((px2+py2), 0.5)))
	Hj.Set(2, 3, py/(math.Pow((px2+py2), 0.5)))
	return &Hj
}

func CalculateRMSE(estimations [][]float64, ground_truth [][]float64) []float64 {
	rmse := []float64{0, 0, 0, 0}

	if len(estimations) != len(ground_truth) || len(estimations) == 0 {
		fmt.Println("Invalid estimation or ground_truth data")
		return rmse
	}

	for i := range estimations[0] {
		for j := range estimations {
			rmse[i] += math.Pow(estimations[j][i]-ground_truth[j][i], 2)
		}
		rmse[i] /= float64(len(estimations))
		rmse[i] = math.Sqrt(rmse[i])
	}

	return rmse
}

func ToPolar(measurement_pack *measurement_package.MeasurementPackage) *mat.Dense {
	px := measurement_pack.RawMeasurements.At(0, 0) * math.Cos(measurement_pack.RawMeasurements.At(0, 1))
	py := measurement_pack.RawMeasurements.At(0, 0) * math.Sin(measurement_pack.RawMeasurements.At(0, 1))
	vx := measurement_pack.RawMeasurements.At(0, 2) * math.Cos(measurement_pack.RawMeasurements.At(0, 1))
	vy := measurement_pack.RawMeasurements.At(0, 2) * math.Sin(measurement_pack.RawMeasurements.At(0, 1))
	polar := *mat.NewDense(4, 1, []float64{px, py, vx, vy})
	return &polar
}

func ToCartesian(polar *mat.Dense) *mat.Dense {
	px := polar.At(0, 0)
	py := polar.At(1, 0)
	vx := polar.At(2, 0)
	vy := polar.At(3, 0)
	var px2, py2 float64 = math.Pow(px, 2), math.Pow(py, 2)
	cart := *mat.NewDense(3, 1, []float64{math.Sqrt(px2 + py2), math.Atan2(py, px), (px*vx + py*vy) / math.Sqrt(px2+py2)})
	return &cart
}

func Identity(size int) *mat.Dense {
	count := 0
	var s []float64
	for i := 0; i < size; i++ {
		for j := 0; j < size; j++ {
			if count == i {
				s = append(s, float64(1))
			} else {
				s = append(s, float64(0))
			}
			count++
		}
		count = 0
	}
	retValue := mat.NewDense(size, size, s)
	return retValue
}
