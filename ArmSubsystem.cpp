#include "ArmSubsystem.h"

// Check Ports
#if (ARM_MOTOR_PORT == NC || CLAW_MOTOR_PORT == NC)
#error "Invalid Arm Motor Ports"
#endif


ArmSubsystem::ArmSubsystem() :
  motorArm(ARM_MOTOR_PORT),
  motorClaw(CLAW_MOTOR_PORT) {
    // Constructor
  }


void ArmSubsystem::moveArm() {
    motorArm.run(armSpeed);
}

void ArmSubsystem::moveArm(int16_t arm) {
    setArmSpeed(arm);
    moveArm();
}

void ArmSubsystem::stopArm() {
    motorArm.stop();
}

void ArmSubsystem::setArmSpeed(int16_t arm) {
    armSpeed = arm;
}

void ArmSubsystem::moveClaw() {
    motorClaw.run(clawSpeed);
}

void ArmSubsystem::moveClaw(int16_t claw) {
    clawSpeed = claw;
    moveClaw();
}

void ArmSubsystem::stopClaw() {
    motorClaw.stop();
}

void ArmSubsystem::setClawSpeed(int16_t claw) {
    clawSpeed = claw;
}