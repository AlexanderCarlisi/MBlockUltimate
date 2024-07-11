#include "MeMegaPi.h"

#ifndef DRIVESUBSYSTEM_H
#define DRIVESUBSYSTEM_H

#define DRIVE_MOTOR_LEFT_PORT PORT_2
#define DRIVE_MOTOR_RIGHT_PORT PORT_1

class DriveSubsystem {
  private:
    MeMegaPiDCMotor motorLeft;
    MeMegaPiDCMotor motorRight;

  public:
    int16_t leftSpeed = 0;
    int16_t rightSpeed = 0;

    /**
      Blank Constructor
      Port Checks and Initializes Motors.
     */
    DriveSubsystem();

    /**
      Does motor.run(respectiveSpeed) for each Drive Motor.
      Set and Forget.
     */
    void drive();

    /**
      Sets leftSpeed and rightSpeed to parameters, then calls drive().
      Set and Forget.
      \param left : Signed 16 Bit Integer for speed. [-255, 255]
      \param right : Signed 16 Bit Integer for speed. [-255, 255]
     */
    void drive(int16_t left, int16_t right);

    /**
      Does motor.stop() for each Drive Motor.
      Set and Forget.
     */
    void stop();

    /**
      Sets the leftSpeed and rightSpeed of the Drive Motors.
      \param left : Signed 16 Bit Integer for speed. [-255, 255]
      \param right : Signed 16 Bit Integer for speed. [-255, 255]
     */
    void setSpeeds(int16_t left, int16_t right);
};

#endif // DRIVESUBSYSTEM_H