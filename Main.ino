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
}

void loop() {
  // put your main code here, to run repeatedly:
  timer.tick(); // lastly, run the timer
}


///
/// System Tests
///

void testDriveSubsystem() {
	driveSubsystem.drive(255, 255);
	delay(1500);
	driveSubsystem.stop();
	delay(1500);
	driveSubsystem.drive(-255, -255);
	delay(1500);
	driveSubsystem.stop();
}
void testDriveWithTimer() {
	timer.every(6000, testDriveSubsystem);
}

void testArm() {
	armSubsystem.moveArm(255);
	delay(1500);
	armSubsystem.stopArm();
	delay(1500);
	armSubsystem.moveArm(-255);
	delay(1500);
	armSubsystem.stopArm();
}
void testArmWithTimer() {
	timer.every(6000, testArm);
}

void testClaw() {
	armSubsystem.moveClaw(255);
	delay(1500);
	armSubsystem.stopClaw();
	delay(1500);
	armSubsystem.moveClaw(-255);
	delay(1500);
	armSubsystem.stopClaw();
}
void testClawWithTimer() {
	timer.every(6000, testClaw);
}

void testLineSensor() {
	lineSensor.checkLine();
	Serial.println(lineSensor.onLeft ? "On Left" : "Not On Left");
}
void testLineSensorWithTimer() {
	timer.every(1000, testLineSensor);
}

void testUltrasonicSensor() {
	Serial.println(ultrasonicSensor.getDistanceCM());
}
void testUltrasonicSensorWithTimer() {
	timer.every(1000, testUltrasonicSensor);
}


///
/// Timer Tasks
///
bool followLine() {
	lineSensor.checkLine();

	// Both Within Line
	if (lineSensor.onLeft && lineSensor.onRight) {
		driveSubsystem.drive(255, 255);

	// Left in, Right out : Turn Left
	} else if (lineSensor.onLeft && !lineSensor.onRight) {
		driveSubsystem.drive(128, 255);

	// Left out, Right in : Turn Right
	} else if (!lineSensor.onLeft && lineSensor.onRight) {
		driveSubsystem.drive(255, 128);
	
	// Both Out of Line : Stop and End Task
	} else {
		driveSubsystem.setSpeeds(0, 0);
		driveSubsystem.stop();
		return false; // Repeat? No
	}

	return true; // Repeat? Yes
}

bool stopAtPickupProx() {
	ultrasonicSensor.checkProximity();

	if (ultrasonicSensor.atPickUp) {
		driveSubsystem.stop();
		return false; // Repeat? No
	}

	return true; // Repeat? Yes
}

bool stopAtClosestProx() {
	ultrasonicSensor.checkProximity();

	if (ultrasonicSensor.atClosest) {
		driveSubsystem.stop();
		return false; // Repeat? No
	}

	return true; // Repeat? Yes
}


///
/// Autos
///

/**
 * Challenge 1: Follow Line, then Stop at Closest Proximity.
 * FRIDAY
 */
void challenge1() {
	auto followLineTask = timer.every(100, followLine);
	auto proxTask = timer.every(100, stopAtClosestProx);
	timer.every(50, []() -> void {
		if (ultrasonicSensor.atClosest) {
			driveSubsystem.stop();
			timer.cancel();
		}
	});
}


/**
 * Challenge 2: Follow Line, then Stop at Pick Up Proximity and Pick up and Set Down.
 * MONDAY
 */
void challenge2() {
	auto followLineTask = timer.every(100, followLine);
	auto proxTask = timer.every(100, stopAtPickupProx);
	timer.every(50, []() -> void {
		if (ultrasonicSensor.atPickUp) {
			driveSubsystem.stop();
			timer.cancel();

			// Configure + should be moved to a function
			armSubsystem.moveArm(255);
			delay(1500);
			armSubsystem.stopArm();
			delay(1500);
			armSubsystem.moveArm(-255);
			delay(1500);
			armSubsystem.stopArm();
		}
	});
}


/**
 * Challenge 3: 
 * 1. Find Midpoint of Arm *also maybe use encoders, but also maybe just a timer?
 * 2. Drive through a Line, then reverse to the midpoint. *use encoders?
 * 3. Open and Close Claw 5 times throughout this process. *easy
 * TUESDAY
 */
void challenge3() {}