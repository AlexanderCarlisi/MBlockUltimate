#include "DriveSubsystem.h"


// Check Ports
#if (DRIVE_MOTOR_LEFT_PORT == NC || DRIVE_MOTOR_RIGHT_PORT == NC)
#error "Invalid Drive Motor Ports"
#endif


DriveSubsystem::DriveSubsystem() :
  motorLeft(DRIVE_MOTOR_LEFT_PORT),
  motorRight(DRIVE_MOTOR_RIGHT_PORT) {
    // Constructor
  }


void DriveSubsystem::drive() {
  motorLeft.run(leftSpeed); // invert left motor
  motorRight.run(-rightSpeed);
}

void DriveSubsystem::drive(int16_t left, int16_t right) {
  setSpeeds(left, right);
  drive();
}

void DriveSubsystem::stop() {
  motorLeft.stop();
  motorRight.stop();
}

void DriveSubsystem::setSpeeds(int16_t left, int16_t right) {
  leftSpeed = left;
  rightSpeed = right;
}