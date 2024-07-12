#include "UltrasonicSensor.h"

// Check Ports
#if (ULTRASONIC_PORT == NC)
#error "Invalid Ultrasonic Sensor Port"
#endif


UltrasonicSensor::UltrasonicSensor() :
  sensor(ULTRASONIC_PORT) {
    // Constructor
  }


double UltrasonicSensor::getDistanceCM() {
    return sensor.distanceCm();
}

void UltrasonicSensor::checkProximity() {
    double distance = getDistanceCM();
    atClosest = distance <= closestProximityCM;
    atPickUp = distance <= pickUpProximityCM;
    startSlow = distance <= startSlowProximityCM;
}

