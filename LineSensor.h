#include "MeMegaPi.h"

#ifndef LINESENSOR_H
#define LINESENSOR_H

#define LINESENSOR_PORT PORT_5

#define STRAIGHT_TURN 0
#define LEFT_TURN 1
#define RIGHT_TURN 2

#define LINE_COLOR_BLACK 0
#define LINE_COLOR_WHITE 1


class LineSensor {
  private:
    const MeLineFollower lineSensor;
    const uint8_t lineColor = LINE_COLOR_WHITE;

    // PID Constants
    double kP = 1.0;
    double kI = 0.1;
    double kD = 0.1;

    long integral = 0;
    int previousError = 0;

  public:
    const int16_t initialLineFollowSpeed = 200;
    int16_t lineFollowSpeed = initialLineFollowSpeed;
    double turnModifier = 1.5;

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


    /**
     * Returns the Error Value for the PID Controller.
     * Due to the Sensors being Boolean this leads to some complications.
     * Values range from [-1, 1].
     * 0 is straight or reverse, no error.
     * -1 is Left on, Right off, turn left.
     * 1 is Right on, Left off, turn right.
     * 
     * \return int error value {-1, 0, 1}
     */
    int getError(bool leftSensor, bool rightSensor);


    /**
     * Returns the Correction Value for the PID Controller.
     * This function utilizes the PID Controller to determine the correction value.
     * 
     * - update sensors
     * - calculate error
     * - update integral
     * - calculate derivative
     * - return correction : kP * error + kI * integral + kD * derivative
     * 
     * Since getError returns -1 or 1 based on turn, the correction will either be Positive or Negative.
     * This means that the Left Motor will be subtracted by the correction value, and the Right Motor will be added by the correction value.
     * Double negative magic will make the robot turn left or right based on the error.
     * 
     * \return float correction kP * error + kI * integral + kD * derivative
     */
    float getCorrection();
};

#endif // LINESENSOR_H