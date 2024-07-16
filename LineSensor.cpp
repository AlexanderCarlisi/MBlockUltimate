#include "LineSensor.h"

// Check Ports
#if (LINESENSOR_PORT == NC)
#error "Invalid Line Sensor Port"
#endif


LineSensor::LineSensor() :
  lineSensor(LINESENSOR_PORT) {
    // Constructor
  }


void LineSensor::checkLine() {
  onLeft = lineSensor.readSensor1() == lineColor; // 0 detects black, 1 detects white 
  onRight = lineSensor.readSensor2() == lineColor;
}


int LineSensor::getError(bool leftSensor, bool rightSensor) {
  if (leftSensor && rightSensor) return 0;  // Both sensors on the line
  else if (leftSensor && !rightSensor) return 1; // Left sensor on the line, right sensor off the line | 1 because we want left subtracted when rotating left
  else if (!leftSensor && rightSensor) return -1;  // Right sensor on the line, left sensor off the line | -1 because we want left added when rotating right
  else return 0;  // Both sensors off the line, handle separately if needed
}


float LineSensor::getCorrection() {
  // Update Sensors
  checkLine();

  // Determine Error
  int error = getError(onLeft, onRight);

  // Update Integral and Derivative
  integral += error;
  int derivative = error - previousError;

  // Calculate correction using PID
  return kP * error + kI * integral + kD * derivative;
}