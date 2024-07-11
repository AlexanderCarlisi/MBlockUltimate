#include "MeMegaPi.h"

#ifndef LINESENSOR_H
#define LINESENSOR_H

#define LINESENSOR_PORT PORT_8 // ?


class LineSensor {
  private:
    MeLineFollower lineSensor;

  public:
    bool onLeft = false;
    bool onRight = false;

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