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
    onLeft = lineSensor.readSensor1() == 0;
    onRight = lineSensor.readSensor2() == 0;
}