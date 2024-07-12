#include "MeMegaPi.h"

#ifndef LINESENSOR_H
#define LINESENSOR_H

#define LINESENSOR_PORT PORT_5


class LineSensor {
  private:
    MeLineFollower lineSensor;

  public:
    const int16_t initialLineFollowSpeed = 125;
    bool onLeft = false;
    bool onRight = false;
    int16_t lineFollowSpeed = 125;
    bool flip = false; // Go in Reverse,

    /**
      Blank Constructor
      Port Checks and Initializes Sensor.
     */
    LineSensor();

    /**
      Checks if the Line Sensor is inside the Black Line.
      Sets onLine to true if it is.
     */
    void checkLine();
};

#endif // LINESENSOR_H