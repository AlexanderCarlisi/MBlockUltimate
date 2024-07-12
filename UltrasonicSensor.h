#include "MeMegapi.h"

#ifndef ULTRASONICSENSOR_H
#define ULTRASONICSENSOR_H

#define ULTRASONIC_PORT PORT_6

class UltrasonicSensor {
  private:
    MeUltrasonicSensor sensor;

  public:
    bool atClosest = false;
    bool atPickUp = false;
    bool startSlow = false;
    const double closestProximityCM = 4;
    const double pickUpProximityCM = 10.5;
    const double startSlowProximityCM = 15;

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