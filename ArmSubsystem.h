#include "MeMegaPi.h"

#ifndef ARMSUBSYSTEM_H
#define ARMSUBSYSTEM_H

#define ARM_MOTOR_PORT PORT_3
#define CLAW_MOTOR_PORT PORT_12

class ArmSubsystem {
  private:
  	MeMegaPiDCMotor motorArm;
    MeMegaPiDCMotor motorClaw;

  public:
		int16_t armSpeed = 0;
		int16_t clawSpeed = 0;

		/**
			Blank Constructor
			Port Checks and Initializes Motors.
		 */
		ArmSubsystem();

		/**
			Does motor.run(speed) for the Arm Motor.
			Set and Forget.
		 */
		void moveArm();

		/**
			Sets armSpeed then calls moveArm().
			Set and Forget.
			\param arm : Signed 16 Bit Integer for speed. [-255, 255]
		 */
		void moveArm(int16_t arm);

		/**
			Does motor.stop() for the Arm Motor.
			Set and Forget.
		 */
		void stopArm();

		/**
			Sets the armSpeed of the Arm Motor.
			\param arm : Signed 16 Bit Integer for speed. [-255, 255]
		 */
		void setArmSpeed(int16_t arm);

		/**
			Does motor.run(speed) for the Claw Motor.
			Set and Forget.
		 */
		void moveClaw();

		/**
			Sets clawSpeed then calls moveClaw().
			Set and Forget.
			\param claw : Signed 16 Bit Integer for speed. [-255, 255]
		 */
		void moveClaw(int16_t claw);

		/**
			Does motor.stop() for the Claw Motor.
			Set and Forget.
		 */
		void stopClaw();

		/**
			Sets the clawSpeed of the Claw Motor.
			\param claw : Signed 16 Bit Integer for speed. [-255, 255]
		 */
		void setClawSpeed(int16_t claw);
};

#endif // ARMSUBSYSTEM_H