// #include "MeMegaPi.h"

// #ifndef ARMSUBSYSTEM_H
// #define ARMSUBSYSTEM_H

// #define ARM_MOTOR_PORT PORT_11
// #define CLAW_MOTOR_PORT PORT_12

// class ArmSubsystem {
//   private:
//   	MeMegaPiDCMotor motorArm;
//     MeMegaPiDCMotor motorClaw;

//   public:
// 		int16_t armSpeed = 0;
// 		int16_t clawSpeed = 0;

// 		/**
// 			Blank Constructor
// 			Port Checks and Initializes Motors.
// 		 */
// 		ArmSubsystem();

// 		/**
// 			Does motor.run(speed) for the Arm Motor.
// 			Set and Forget.
// 		 */
// 		void moveArm();

// 		/**
// 			Sets armSpeed then calls moveArm().
// 			Set and Forget.
// 			\param arm : Signed 16 Bit Integer for speed. [-255, 255]
// 		 */
// 		void moveArm(int16_t arm);

// 		/**
// 			Does motor.stop() for the Arm Motor.
// 			Set and Forget.
// 		 */
// 		void stopArm();

// 		/**
// 			Sets the armSpeed of the Arm Motor.
// 			\param arm : Signed 16 Bit Integer for speed. [-255, 255]
// 		 */
// 		void setArmSpeed(int16_t arm);

// 		/**
// 			Does motor.run(speed) for the Claw Motor.
// 			Set and Forget.
// 		 */
// 		void moveClaw();

// 		/**
// 			Sets clawSpeed then calls moveClaw().
// 			Set and Forget.
// 			\param claw : Signed 16 Bit Integer for speed. [-255, 255]
// 		 */
// 		void moveClaw(int16_t claw);

// 		/**
// 			Does motor.stop() for the Claw Motor.
// 			Set and Forget.
// 		 */
// 		void stopClaw();

// 		/**
// 			Sets the clawSpeed of the Claw Motor.
// 			\param claw : Signed 16 Bit Integer for speed. [-255, 255]
// 		 */
// 		void setClawSpeed(int16_t claw);
// };

// #endif // ARMSUBSYSTEM_H

#include "MeMegaPi.h"
#include <Wire.h>
#include <SoftwareSerial.h>

#ifndef ARMSUBSYSTEM_H
#define ARMSUBSYSTEM_H

#define ARM_TOP_POSITION 180 // ?
#define ARM_BASE_POSITION 0

#define CLAW_OPEN_POSITION 180 // ?
#define CLAW_CLOSED_POSITION 0

#define ENCODER_PORT 0x09
#define ARM_SLOT SLOT1
#define CLAW_SLOT SLOT2

class ArmSubsystem {

	private:
		// 0x09 seems constant, SLOT1-4 are the slots on the MegaPi
		MeEncoderMotor armMotor;   //  Motor at slot1
		MeEncoderMotor clawMotor;   //  motor at slot2

	public:

		float armSpeed = 200;
		float clawSpeed = 255;

		/**
		 * Blank Constructor
		 * Initializes Encoder Motors.
		 * calls Encoder.begin() for both motors.
		 */
		ArmSubsystem();


		/**
		 * Moves the Arm Motor to the base, then resets the encoder.
		 * Ensuring the Encoder at base is always 0.
		 * 
		 * Might not be necessary depending on testing.
		 */
		void calibrateArm();

		
		/**
		 * Moves the Claw Motor to the position.
		 * armSpeed is the speed used.
		 * \param position : float value for the position of the claw.
		 */
		void moveArmTo(float position);


		/**
		 * \return float : the current position of the arm.
		 */
		float getArmPosition();


		/**
		 * Moves the Claw Motor to fully closed position.
		 * clawSpeed is the speed used.
		 * Resets the Encoder after closing.
		 */
		void calibrateClaw();


		/**
		 * Moves the Claw Motor to the position.
		 * clawSpeed is the speed used.
		 * \param position : float value for the position of the claw.
		 */
		void moveClawTo(float position);

		/**
		 * \return float : the current position of the claw.
		 */
		float getClawPosition();
};

#endif // ARMSUBSYSTEM_H