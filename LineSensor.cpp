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
  onLeft = lineSensor.readSensor1() == 1; // 0 detects black, 1 detects white 
  onRight = lineSensor.readSensor2() == 1;
}