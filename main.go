package main

import (
	"bufio"
	"fmt"
	"log"
	"os"
	"strconv"
	"strings"

	"./extended_kalman_filter"
	"./kalman_filter"
	"./measurement_package"
	"gonum.org/v1/gonum/mat"
)

func check(err error, fName string) {
	if err != nil {
		fmt.Println("Could not open file", fName)
		panic(err)
	}
}

func main() {
	// Open file with sensor data
	fusionEKF := extended_kalman_filter.New()
	inFileName := "./data/obj_pose-laser-radar-synthetic-input.txt"
	inF, err := os.Open(inFileName)
	check(err, inFileName)
	defer inF.Close()

	scanner := bufio.NewScanner(inF)
	for scanner.Scan() {
		inputValues := strings.Split(scanner.Text(), "\t")
		sensorType := inputValues[0]
		inputLength := len(inputValues)
		if sensorType == "L" {
			sensorType_ := measurement_package.LASER
			x, _ := strconv.ParseFloat(inputValues[1], 64)
			y, _ := strconv.ParseFloat(inputValues[2], 64)
			rawMeasurements := mat.NewDense(2, 1, []float64{x, y})
			timeStamp, _ := strconv.ParseFloat(inputValues[3], 64)
			measPackage := measurement_package.NewMeasurementPackage(sensorType_, timeStamp, rawMeasurements)
			fusionEKF.ProcessMeasurement(measPackage)
		} else if sensorType == "R" {
			sensorType_ := measurement_package.RADAR
			rho_measured, _ := strconv.ParseFloat(inputValues[1], 64)
			phi_measured, _ := strconv.ParseFloat(inputValues[2], 64)
			rhodot_measured, _ := strconv.ParseFloat(inputValues[3], 64)
			rawMeasurements := mat.NewDense(3, 1, []float64{rho_measured, phi_measured, rhodot_measured})
			timeStamp, _ := strconv.ParseFloat(inputValues[4], 64)
			measPackage := measurement_package.NewMeasurementPackage(sensorType_, timeStamp, rawMeasurements)
			fusionEKF.ProcessMeasurement(measPackage)
		}
		// fusionEKF.ProcessMeasurement(meas_package);
		x_gt, _ := strconv.ParseFloat(inputValues[inputLength-4], 64)
		y_gt, _ := strconv.ParseFloat(inputValues[inputLength-3], 64)
		vx_gt, _ := strconv.ParseFloat(inputValues[inputLength-2], 64)
		vy_gt, _ := strconv.ParseFloat(inputValues[inputLength-1], 64)
		gtValues := []float64{x_gt, y_gt, vx_gt, vy_gt}
		fmt.Println(gtValues)
	}

	// Check for error in file reading
	if err := scanner.Err(); err != nil {
		log.Fatal(err)
	}
	test := kalman_filter.New()
	test.Predict()
}
