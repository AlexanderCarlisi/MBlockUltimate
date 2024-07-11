#include "MeMegapi.h"

#ifndef ULTRASONICSENSOR_H
#define ULTRASONICSENSOR_H

#define ULTRASONIC_PORT PORT_5 // ?

class UltrasonicSensor {
  private:
    const double closestProximityCM = 3; // ~?
    const double pickUpProximityCM = 10.5; // ~?
    MeUltrasonicSensor sensor;

  public:
    bool atClosest = false;
    bool atPickUp = false;

    /**
      Blank Constructor
      Port Checks and Initializes Sensor.
     */
    UltrasonicSensor();

    /**
      Returns the distance in CM.
      \return double : Distance in CM.
     */
    double getDistanceCM();

    /**
      Checks if the distance is within closestProximityCM and/or pickUpProximityCM.
      Sets atClosest/atPickUp to true if it is.
     */
    void checkProximity();
};

#endif // ULTRASONICSENSOR_H