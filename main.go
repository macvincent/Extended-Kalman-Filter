package main

import (
	"bufio"
	"fmt"
	"log"
	"os"
	"strconv"
	"strings"

	"./extended_kalman_filter"
	"./measurement_package"
	"./tools"
	"gonum.org/v1/gonum/mat"
)

func main() {
	// Open file with sensor data
	fusionEKF := extended_kalman_filter.New()
	inFileName := "./data/obj_pose-laser-radar-synthetic-input.txt"
	inF, err := os.Open(inFileName)
	check(err, inFileName)
	defer inF.Close()

	gTruthValues := [][]float64{}
	predictedValues := [][]float64{}

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
		x_gt, _ := strconv.ParseFloat(inputValues[inputLength-4], 64)
		y_gt, _ := strconv.ParseFloat(inputValues[inputLength-3], 64)
		vx_gt, _ := strconv.ParseFloat(inputValues[inputLength-2], 64)
		vy_gt, _ := strconv.ParseFloat(inputValues[inputLength-1], 64)
		gtValues := []float64{x_gt, y_gt, vx_gt, vy_gt}
		gTruthValues = append(gTruthValues, gtValues)
		predictedValues = append(predictedValues, []float64{fusionEKF.Ekf.X_.At(0, 0), fusionEKF.Ekf.X_.At(1, 0), fusionEKF.Ekf.X_.At(2, 0), fusionEKF.Ekf.X_.At(3, 0)})
	}

	// Check for error in file reading
	if err := scanner.Err(); err != nil {
		log.Fatal(err)
	}
	RMSE_values := tools.CalculateRMSE(predictedValues, gTruthValues)
	storeStates(gTruthValues, predictedValues)
	storeRMSE(RMSE_values)
	fmt.Println("EKF: Position states stored in file '../data/filter_output.txt' in format (px, py).")
}

func check(err error, fName string) {
	if err != nil {
		fmt.Println("Could not open file", fName)
		panic(err)
	}
}

func storeStates(gTruthValues [][]float64, predictedValues [][]float64) {
	// Write filter output to file
	simulationData, err := os.Create("./data/filter_output.txt")
	check(err, "./data/filter_output.txt")
	defer simulationData.Close()
	for _, state := range predictedValues {
		tempString := ""
		for _, value := range state {
			tempString += strconv.FormatFloat(value, 'E', -1, 64) + "\t"
		}
		tempString += "\n"
		simulationData.WriteString(tempString)
	}

}

func storeRMSE(RMSE_values []float64) {
	// Write RMSE output to file
	RMSEData, err := os.Create("./data/rmse_data.txt")
	check(err, "./data/rmse_data.txt")
	defer RMSEData.Close()
	for _, state := range RMSE_values {
		tempString := strconv.FormatFloat(state, 'E', -1, 64) + "\n"
		RMSEData.WriteString(tempString)
	}
}
