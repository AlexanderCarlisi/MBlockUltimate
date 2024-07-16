///
///
///

#include "DriveSubsystem.h"
#include "ArmSubsystem.h"
#include "LineSensor.h"
#include "UltrasonicSensor.h"
#include <arduino-timer.h>


///
/// Define Systems
///

DriveSubsystem driveSubsystem;
ArmSubsystem armSubsystem;
LineSensor lineSensor;
UltrasonicSensor ultrasonicSensor;
Timer<> timer;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  // testDriveSubsystem();
  // testDriveWithTimer();
  
  // Test Subsystems for Tuesday
  // testArmWithTimer();
  testClawWithTimer();

  // testUltrasonicSensorWithTimer();
  // testLineSensorWithTimer();
  // timer.every(100, followLine);
  // challenge1();
}

void loop() {

  // testDriveSubsystem();

  // put your main code here, to run repeatedly:
  timer.tick(); // lastly, run the timer
}


///
/// System Tests
///


/**
  Tests the Drive motors, port checking.
 */
void testDriveSubsystem() {
  int16_t speed = 175;
	// driveSubsystem.drive(speed, speed);
	// delay(1000);
	// driveSubsystem.stop();
	// delay(1000);
	// // driveSubsystem.drive(-speed, -speed);
  // driveSubsystem.drive(-speed, speed);
	// delay(1000);
	// driveSubsystem.stop();
  driveSubsystem.drive(speed * -0.75, speed * 0.75);
}
/**
  Tests the Drive Motors utilizing the Timer class.
  Should loop infinitely when
 */
void testDriveWithTimer() {
	timer.every(3000, testDriveSubsystem);
}

/**
  Tests the Arm Motors, port checking.
 */
void testArm() {
	// armSubsystem.moveArm(255);
	// delay(1500);
	// armSubsystem.stopArm();
	// delay(1500);
	// armSubsystem.moveArm(-255);
	// delay(1500);
	// armSubsystem.stopArm();

	armSubsystem.calibrateArm();
	delay(5000);
	Serial.println("CALIBRATED ARM : " + String(armSubsystem.getArmPosition()));
	armSubsystem.moveArmTo(ARM_TOP_POSITION);
	delay(5000);
	Serial.println("ARM POSITION : " + String(armSubsystem.getArmPosition()));
	delay(5000);
	armSubsystem.moveArmTo(ARM_BASE_POSITION);
	delay(5000);
	Serial.println("ARM POSITION : " + String(armSubsystem.getArmPosition()));
}
/**
  Tests the Arm motors utilizing the Timer class.
  Should loop forever when called once in Setup.
 */
void testArmWithTimer() {
	timer.every(20000, testArm);
}

/**
  Tests the claw Motors, port checking.
 */
void testClaw() {
	// armSubsystem.moveClaw(255);
	// delay(1500);
	// armSubsystem.stopClaw();
	// delay(1500);
	// armSubsystem.moveClaw(-255);
	// delay(1500);
	// armSubsystem.stopClaw();

	armSubsystem.calibrateClaw();
	delay(3000);
	Serial.println("CALIBRATED CLAW : " + String(armSubsystem.getClawPosition()));
	armSubsystem.moveClawTo(CLAW_OPEN_POSITION);
	delay(3000);
	Serial.println("CLAW POSITION : " + String(armSubsystem.getClawPosition()));
	delay(3000);
	armSubsystem.moveClawTo(CLAW_CLOSED_POSITION);
	delay(3000);
	Serial.println("CLAW POSITION : " + String(armSubsystem.getClawPosition()));
}
/**
  Tests the claw motors using the Timer Class
  Should loop infinitely when called once in Setup.
 */
void testClawWithTimer() {
	timer.every(12000, testClaw);
}

/**
  Prints out the State of the Left Sensor.
 */
void testLineSensor() {
	lineSensor.checkLine(); // Update onLeft and onRight checks
	// if (lineSensor.onLeft) {
  //   Serial.println("Left on Line");
  // }
  // if (lineSensor.onRight) {
  //   Serial.println("Right on Line");
  // }
  Serial.println(lineSensor.onLeft);
}
/**
  Loops the LineSensor test, should only be called once in Setup for looping.
 */
void testLineSensorWithTimer() {
	timer.every(1000, testLineSensor);
}

/**
  Tests the Ultrasonic Sensor, prints the distance read by the Sensor.
 */
void testUltrasonicSensor() {
	Serial.println(ultrasonicSensor.getDistanceCM());
}
/**
  Using the Timer class every 1 second the sensor will be read and print out distance.
 */
void testUltrasonicSensorWithTimer() {
	timer.every(1000, testUltrasonicSensor);
}


///
/// Timer Tasks
///

/**
  FollowLine Task, Tasks are used by the Timer.
  \return True: Repeat the task next time frame || False: Cancel the task in the Timer.
 */
bool followLine() {
//   lineSensor.checkLine();
//   int16_t leftSpeed = lineSensor.lineFollowSpeed;
//   int16_t rightSpeed = lineSensor.lineFollowSpeed;

//   bool bothOnLine = (lineSensor.onLeft && lineSensor.onRight);
//   bool leftOnLine = (lineSensor.onLeft && !lineSensor.onRight);
//   bool rightOnLine = (!lineSensor.onLeft && lineSensor.onRight);

//   if (bothOnLine) {
// 	// Drive Straight
// 	driveSubsystem.drive(leftSpeed, rightSpeed);

//   } else if (leftOnLine) {
// 	// Turn Right
// 	driveSubsystem.drive(leftSpeed * lineSensor.turnModifier, rightSpeed / lineSensor.turnModifier);

//   } else if (rightOnLine) {
// 	// Turn Left
// 	driveSubsystem.drive(leftSpeed / lineSensor.turnModifier, rightSpeed * lineSensor.turnModifier);

//   } else {
// 	// OFF LINE LOGIC, GO IN REVERSE TO CORRECT COURSE
// 	// driveSubsystem.drive(-leftSpeed, -rightSpeed);
//   }

  // Initial Speeds
  int16_t leftSpeed = lineSensor.lineFollowSpeed;
  int16_t rightSpeed = lineSensor.lineFollowSpeed;
  
  // get PID Correction
  float correction = lineSensor.getCorrection();

  leftSpeed -= correction; // -1 turn = -correction = addition
  rightSpeed += correction; // ^^^ same side should turn properly based on Error with PID.
  
  // Drive the Robot with PID Speeds
  driveSubsystem.drive(leftSpeed, rightSpeed);

  // Debug
  Serial.println("Left Speed: " + String(leftSpeed) + " | Right Speed: " + String(rightSpeed) + " | Correction: " + String(correction));

  return true; // Repeat? Yes 
}


/**
 * Stop at Pick Up Proximity Task, Tasks are used by the Timer.
 * \return True: Repeat the task next time frame || False: Cancel the task in the Timer.
 */
bool stopAtClosestProx() {
	ultrasonicSensor.checkProximity();

  // Slow Down lineFollowSpeed while approaching block.
  double distance = ultrasonicSensor.getDistanceCM();
  
  // if within slow zone, slow down the Line Following speed.
  if (distance <= ultrasonicSensor.startSlowProximityCM) {
    lineSensor.lineFollowSpeed = lineSensor.initialLineFollowSpeed * (distance / ultrasonicSensor.startSlowProximityCM);
  } else {
    // If outside of slow zone, return to normal speed.
    lineSensor.lineFollowSpeed = lineSensor.initialLineFollowSpeed;
  }
  return true; // Repeat? Yes
}


///
/// Autos
///

/**
 * Challenge 1: Follow Line, then Stop at Closest Proximity.
 * MONDAY DONE
 */
void challenge1() {
  auto followLineTask = timer.every(20, followLine);
	auto proxTask = timer.every(20, stopAtClosestProx);
}


/**
 * Challenge 2: Follow Line, then Stop at Pick Up Proximity and Pick up and Set Down.
 * TUESDAY
 */
void challenge2() {
	// Schedule Functions to run Periodically
	auto followLineTask = timer.every(20, followLine);
	auto proxTask = timer.every(20, stopAtClosestProx);

	// Check if the Functions should end, if it's time to pick up the block
	timer.every(50, []() -> void {
		if (ultrasonicSensor.atClosest) {
			driveSubsystem.stop();
			timer.cancel(); // need to test to see if this breaks everything or not

			delay(10000); // Give 10 seconds for block to be setup

			// Pick Up Block
			armSubsystem.moveClawTo(CLAW_OPEN_POSITION); // open claw
			armSubsystem.moveArmTo(ARM_BASE_POSITION); // bring arm down
			delay(3000);
			armSubsystem.moveClawTo(CLAW_CLOSED_POSITION); // close claw
			delay(3000);
			armSubsystem.moveArmTo(ARM_TOP_POSITION); // bring arm up, LIFT
			delay(5000);
			armSubsystem.moveArmTo(ARM_BASE_POSITION); // bring arm down, SET DOWN
		}
	}); 
}


/**
 * Challenge 3: 
 * 1. Find Midpoint of Arm *also maybe use encoders, but also maybe just a timer?
 * 2. Drive through a Line, then reverse to the midpoint. *use encoders?
 * 3. Open and Close Claw 5 times throughout this process. *easy
 * WENSDAY
 */
void challenge3() {}