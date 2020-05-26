package kalman_filter

import (
	"math"

	"../tools"
	"gonum.org/v1/gonum/mat"
)

type KalmanFilter struct {
	X_       *mat.Dense
	P_       *mat.Dense
	F_       *mat.Dense
	Q_       *mat.Dense
	H_       *mat.Dense
	Hj_      *mat.Dense
	R_       *mat.Dense
	R_radar_ *mat.Dense
}

func New() *KalmanFilter {
	x := *mat.NewDense(4, 1, []float64{1, 1, 1, 1})
	R := *mat.NewDense(2, 2, []float64{0.01, 0,
		0, 0.01})
	P := *mat.NewDense(4, 4, []float64{1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1000, 0,
		0, 0, 0, 1000})
	R_radar := *mat.NewDense(3, 3, []float64{0.01, 0, 0,
		0, 0.0001, 0,
		0, 0, 0.01})
	H := *mat.NewDense(2, 4, []float64{1, 0, 0, 0,
		0, 1, 0, 0})
	F := *mat.NewDense(4, 4, []float64{1, 0, 1, 0,
		0, 1, 0, 1,
		0, 0, 1, 0,
		0, 0, 0, 1})
	Hj := *mat.NewDense(3, 4, []float64{0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0})
	newFilter := KalmanFilter{&x, &P, &F, mat.NewDense(4, 4, nil), &H, &Hj, &R, &R_radar}
	return &newFilter
}

func (filter *KalmanFilter) Predict() {
	filter.X_.Mul(filter.F_, filter.X_)
	filter.P_.Mul(filter.F_, filter.P_)
	filter.P_.Mul(filter.P_, filter.F_.T())
	filter.P_.Add(filter.P_, filter.Q_)
}

func (filter *KalmanFilter) Update(z *mat.Dense) {
	y := mat.NewDense(2, 1, nil)
	S := mat.NewDense(2, 2, nil)
	si := mat.NewDense(2, 2, nil)
	pht := mat.NewDense(4, 2, nil)
	K := mat.NewDense(4, 2, nil)
	z_pred := mat.NewDense(2, 1, nil)
	hp := mat.NewDense(2, 4, nil)
	ky := mat.NewDense(4, 1, nil)
	kh := mat.NewDense(4, 4, nil)

	z_pred.Mul(filter.H_, filter.X_)
	y.Sub(z, z_pred)
	hp.Mul(filter.H_, filter.P_)
	S.Mul(hp, filter.H_.T())
	S.Add(S, filter.R_)
	si.Inverse(S)
	pht.Mul(filter.P_, filter.H_.T())
	K.Mul(pht, si)

	//new estimate
	ky.Mul(K, y)
	filter.X_.Add(ky, filter.X_)
	I := tools.Identity(4)
	kh.Mul(K, filter.H_)
	I.Sub(I, kh)
	filter.P_.Mul(I, filter.P_)
}

func (filter *KalmanFilter) UpdateEKF(z *mat.Dense) {
	y := mat.NewDense(3, 1, nil)
	S := mat.NewDense(3, 3, nil)
	K := mat.NewDense(4, 3, nil)

	si := mat.NewDense(3, 3, nil)
	pht := mat.NewDense(4, 3, nil)
	z_pred := tools.ToCartesian(filter.X_)
	hp := mat.NewDense(3, 4, nil)
	ky := mat.NewDense(4, 1, nil)
	kh := mat.NewDense(4, 4, nil)

	y.Sub(z, z_pred)
	hp.Mul(filter.Hj_, filter.P_)
	S.Mul(hp, filter.Hj_.T())
	S.Add(S, filter.R_radar_)
	si.Inverse(S)
	pht.Mul(filter.P_, filter.Hj_.T())
	K.Mul(pht, si)
	y.Set(1, 0, math.Remainder(y.At(1, 0), 2*math.Pi))

	//new estimate
	ky.Mul(K, y)
	filter.X_.Add(ky, filter.X_)
	I := tools.Identity(4)
	kh.Mul(K, filter.Hj_)
	I.Sub(I, kh)
	filter.P_.Mul(I, filter.P_)
}
