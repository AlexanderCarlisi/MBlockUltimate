// #include "ArmSubsystem.h"

// // Check Ports
// #if (ARM_MOTOR_PORT == NC || CLAW_MOTOR_PORT == NC)
// #error "Invalid Arm Motor Ports"
// #endif


// ArmSubsystem::ArmSubsystem() :
//   motorArm(ARM_MOTOR_PORT),
//   motorClaw(CLAW_MOTOR_PORT) {
//     // Constructor
//   }


// void ArmSubsystem::moveArm() {
//     motorArm.run(armSpeed);
// }

// void ArmSubsystem::moveArm(int16_t arm) {
//     setArmSpeed(arm);
//     moveArm();
// }

// void ArmSubsystem::stopArm() {
//     motorArm.stop();
// }

// void ArmSubsystem::setArmSpeed(int16_t arm) {
//     armSpeed = arm;
// }

// void ArmSubsystem::moveClaw() {
//     motorClaw.run(clawSpeed);
// }

// void ArmSubsystem::moveClaw(int16_t claw) {
//     clawSpeed = claw;
//     moveClaw();
// }

// void ArmSubsystem::stopClaw() {
//     motorClaw.stop();
// }

// void ArmSubsystem::setClawSpeed(int16_t claw) {
//     clawSpeed = claw;
// }


#include "ArmSubsystem.h"

ArmSubsystem::ArmSubsystem() :
  armMotor(ENCODER_PORT, ARM_SLOT),
  clawMotor(ENCODER_PORT, CLAW_SLOT) {
    armMotor.begin();
    clawMotor.begin();
  }


void ArmSubsystem::moveArmTo(float position) {
    if (position > ARM_TOP_POSITION || position < ARM_BASE_POSITION) return;
    armMotor.moveTo(position, armSpeed);
}

void ArmSubsystem::calibrateArm() {
    armMotor.runSpeedAndTime(-armSpeed, 3); // time in seconds?
    delay(3500);
    armMotor.reset();
}

float ArmSubsystem::getArmPosition() {
    return armMotor.getCurrentPosition();
}

void ArmSubsystem::moveClawTo(float position) {
    if (position > CLAW_OPEN_POSITION || position < CLAW_CLOSED_POSITION) return;
    clawMotor.moveTo(position, clawSpeed);
}

void ArmSubsystem::calibrateClaw() {
    clawMotor.runSpeedAndTime(-clawSpeed, 3); // time in seconds?
    delay(3500);
    clawMotor.reset();
}

float ArmSubsystem::getClawPosition() {
    return clawMotor.getCurrentPosition();
}