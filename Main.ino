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
  // testArmWithTimer();
  // testClawWithTimer();
  // testUltrasonicSensorWithTimer();
  // testLineSensorWithTimer();
  // timer.every(100, followLine);
  challenge1();
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
  int16_t speed = 125;
	driveSubsystem.drive(speed, speed);
	delay(1000);
	driveSubsystem.stop();
	delay(1000);
	driveSubsystem.drive(-speed, -speed);
	delay(1000);
	driveSubsystem.stop();
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
	armSubsystem.moveArm(255);
	delay(1500);
	armSubsystem.stopArm();
	delay(1500);
	armSubsystem.moveArm(-255);
	delay(1500);
	armSubsystem.stopArm();
}
/**
  Tests the Arm motors utilizing the Timer class.
  Should loop forever when called once in Setup.
 */
void testArmWithTimer() {
	timer.every(6000, testArm);
}

/**
  Tests the claw Motors, port checking.
 */
void testClaw() {
	armSubsystem.moveClaw(255);
	delay(1500);
	armSubsystem.stopClaw();
	delay(1500);
	armSubsystem.moveClaw(-255);
	delay(1500);
	armSubsystem.stopClaw();
}
/**
  Tests the claw motors using the Timer Class
  Should loop infinitely when called once in Setup.
 */
void testClawWithTimer() {
	timer.every(6000, testClaw);
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

  update line sensors
  get current speeds to drive at, these speeds are manipulated by the ultrasonic sensor Tasks
  if off the line, go in reverse, but don't invert the Rotation motors to rotate in the proper direction.

  \return True: Repeat the task next time frame || False: Cancel the task in the Timer.
 */
bool followLine() {
	lineSensor.checkLine();
  int16_t leftSpeed = lineSensor.lineFollowSpeed;
  int16_t rightSpeed = lineSensor.lineFollowSpeed;

	// Go Forward, or Reverse if within or outside the line.
  lineSensor.flip = (!lineSensor.onLeft && !lineSensor.onRight);

	// Left in, Right out : Turn Left
	if (lineSensor.onLeft && !lineSensor.onRight) {
		rightSpeed *= -0.5;
    leftSpeed *= (lineSensor.flip) ? -1 : 1; // Flip Forward Direction

	// Left out, Right in : Turn Right
	} else if (!lineSensor.onLeft && lineSensor.onRight) {
		leftSpeed *= -0.5;
    rightSpeed *= (lineSensor.flip) ? -1 : 1; // Flip Forward Direction

  } else {
    if (lineSensor.flip) {
      leftSpeed *= -0.5; // Reverse, and go slower.
      rightSpeed *= -0.5;
    }
  }
  
  driveSubsystem.drive(leftSpeed, rightSpeed);
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

  // Slow Down lineFollowSpeed while approaching block.
  double distance = ultrasonicSensor.getDistanceCM();
  
  // if within slow zone, slow down the Line Following speed.
  if (distance > ultrasonicSensor.startSlowProximityCM) {
    lineSensor.lineFollowSpeed = lineSensor.initialLineFollowSpeed * (distance / ultrasonicSensor.startSlowProximityCM);
  }

  // at or under Desired Proximity, stop driving and end the task loop.
	if (distance <= ultrasonicSensor.closestProximityCM) {
		driveSubsystem.stop();
		return false; // Repeat? No
	}

  Serial.println(lineSensor.lineFollowSpeed);
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

	timer.every(50, []() -> bool { // Every 50 miliseconds Check if it's time to end the Challenge.
		if (ultrasonicSensor.atClosest) {
      timer.cancel(); // Cancels all Tasks assigned to timer.
			driveSubsystem.stop();
      return false; // Stop looping lambda function
		}
    return true; // Continue looping lambda function
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
      timer.cancel();
			driveSubsystem.stop();

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