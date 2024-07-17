///
///
///



// Include Libraries
#include "MeMegaPi.h" // MBlock Library for our Arduino Board
#include <Wire.h> // For Encoders
#include <arduino-timer.h>


///
/// DRIVE
///

// Encoder Constants
#define DRIVE_ENCODER_LEFT SLOT2
// #define DRIVE_ENCODER_RIGHT SLOT1
#define DRIVE_MOTOR_RIGHT PORT_9

// DCMotor Constants
#define DRIVE_MOTOR_LEFT 10
#define DRIVE_MOTOR_RIGHT 9

// Drive Declarations
MeEncoderOnBoard driveEncoderLeft(DRIVE_ENCODER_LEFT);
MeMegaPiDCMotor driveMotorRight(DRIVE_MOTOR_RIGHT);


///
/// ARM
///

// Encoder Constants
#define ARM_ENCODER_SLOT SLOT3
#define ARM_TOP_POS -1300
#define ARM_BASE_POS 0

// DCMotor Constants
#define CLAW_MOTOR_PORT PORT_12

// Arm Declarations
MeMegaPiDCMotor clawMotor(CLAW_MOTOR_PORT);
MeEncoderOnBoard armEncoder(ARM_ENCODER_SLOT);



///
/// SENSORS
///

// Sensor Constants
#define ULTRASONINC_PORT PORT_6
#define LINESENSOR_PORT PORT_5
#define LINE_COLOR 1 // 0 = black | 1 = white
#define STOP_PROXIMITY 5
#define SLOW_PROXIMITY 20

// Sensor Declarations
MeUltrasonicSensor ultrasonicSensor(ULTRASONINC_PORT);
MeLineFollower lineSensor(LINESENSOR_PORT);

const int16_t lineFollowInitialSpeed = 125;
int16_t lineFollowSpeed = lineFollowInitialSpeed;
bool doLineFollow = false;
bool doUSSProximity = false;


///
/// CHALLENGE DECLARATIONS
///

// claw
uint8_t c3_clawOpenCount = 0;
uint8_t c3_clawCloseCount = 0;

// arm
float c3_previousArmPosition = 0;
bool c3_foundArmMidpoint = false;
float c3_armMidpoint = 0;
float c3_armMidpointTolerance = 1;

float c3_armROCTolerance = 20;

// drive
bool c3_foundDriveMidpoint = false;
float c3_driveMidpoint = 0;
float c3_driveMidpointTolerance = 10;



///
/// SYSTEM TEST DECLARATIONS
///
int16_t driveTestSpeed = 150;


///
/// ARDUINO INO STUFF
///
Timer<> scheduler;

void setup() {
  // Initialize Serial
  Serial.begin(9600);
  Serial.println("Initialized Systems...");

  // Encoder Interrupts
  attachInterrupt(driveEncoderLeft.getIntNum(), isrProcessDriveLeft, RISING);
  // attachInterrupt(driveEncoderRight.getIntNum(), isrProcessDriveRight, RISING);
  attachInterrupt(armEncoder.getIntNum(), isrProcessArm, RISING);
  Serial.println("Attached Interrupts for Encoders...");

  // Set PWM 8KHz for Timer1 (pins 11 and 12)
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  // Set PWM 8KHz for Timer4 (pins 7 and 8)
  TCCR4A = _BV(WGM40);
  TCCR4B = _BV(CS41) | _BV(WGM42);

  // Set PWM 8KHz for Timer5 (pins 44, 45, 46) - assuming use for ARM if needed
  TCCR5A = _BV(WGM50);
  TCCR5B = _BV(CS51) | _BV(WGM52);

  // ARM -> 42, 43, 47, 48, 49, 396
  // LEFT MOTOR -> 36, 37, 40, 41, 38, 19, 8, 7
  // RIGHT MOTOR -> 35, 34, 33, 32, 31, 18, 12, 11

  Serial.println("PWM Set to 8KHz for Encoder Motors...");
  Serial.println("... Setup Complete ...");

  // Tests
  // testDrive();
  // bringArmToBase();
  // testArm();
  // closeClaw();
  // testClaw();

  // driveMotorRight.run(255);

  // lowerArm(100);

  // bringArmToBase();
  // bringArmToTop();

  // testClaw();
  // openClaw();

  // raiseArm(100);
  // scheduler.every(1000, []() -> bool {
  //   armEncoder.updateCurPos();
  //   Serial.println(armEncoder.getCurPos());
  //   return true;
  // });

  Challenge3();

  // doLineFollow = true;
  // scheduler.every(50, followLine);
  // scheduler.every(1000, []() -> bool {
  //   Serial.println("LEFT : " + String(lineOnLeft()));
  //   Serial.println("RIGHT : " + String(lineOnRight()));
  //   return true;
  // });
}


void loop() {
  // put your main code here, to run repeatedly:
  scheduler.tick();
}



///
/// ENCODER INTERRUPT Functions
///

void isrProcessDriveLeft() {
    if (digitalRead(driveEncoderLeft.getPortB()) == 0) {
        driveEncoderLeft.pulsePosMinus();
    } else {
        driveEncoderLeft.pulsePosPlus();
    }
}

// void isrProcessDriveRight() {
//     if (digitalRead(driveEncoderRight.getPortB()) == 0) {
//         driveEncoderRight.pulsePosMinus();
//     } else {
//         driveEncoderRight.pulsePosPlus();
//     }
// }

void isrProcessArm() {
    if (digitalRead(armEncoder.getPortB()) == 0) {
        armEncoder.pulsePosMinus();
    } else {
        armEncoder.pulsePosPlus();
    }
}



///
/// SYSTEM FUNCTIONS
///

// DRIVE
void drive(int16_t leftSpeed, int16_t rightSpeed) {
  constrain(leftSpeed, -255, 255);
  constrain(rightSpeed, -255, 255);

  driveEncoderLeft.setMotorPwm(-leftSpeed);
  driveEncoderLeft.updateSpeed();

  driveMotorRight.run(-rightSpeed);
}

// ARM
void bringArmToTop() {
  scheduler.every(1000, []() -> bool {
    int16_t speed = 150; // n / 255
    raiseArm(speed);

    bool isFinished = armAtTop();
    if (isFinished) raiseArm(0); // Stop
    return isFinished;
  });
}

void bringArmToBase() {
  scheduler.every(1000, []() -> bool {
    int16_t speed = 150; // n / 255
    lowerArm(speed);

    bool isFinished = armAtBase();
    if (isFinished) lowerArm(0); // Stop
    return isFinished;
  });
}

/**
  IGNORES LIMITS, WILL DRIVE MOTOR OUT OF BOUNDS
 */
void lowerArm(int16_t speed) {
  // Positive = Down
  if (speed < 0) speed *= -1;
  constrain(speed, -255, 255);

  armEncoder.setMotorPwm(speed);
  armEncoder.updateSpeed();
}

/**
  IGNORES LIMITS, WILL DRIVE MOTOR OUT OF BOUNDS
 */
void raiseArm(float speed) {
  if (speed > 0) speed *= -1;
  constrain(speed, -255, 255);

  armEncoder.setMotorPwm(speed);
  armEncoder.updateSpeed();
}

bool armAtTop() {
  armEncoder.updateCurPos();
  return armEncoder.getCurPos() <= ARM_TOP_POS;
}

bool armAtBase() {
  armEncoder.updateCurPos();
  return armEncoder.getCurPos() >= ARM_BASE_POS;
}

/**
  Raises the Arm until the Positional Rate of Change is 0.
 */
void raiseArmTillROCStops() {
  c3_foundArmMidpoint = false;
  raiseArm(255);

  scheduler.in(500, []() -> void { // Give time for ROC to start
    scheduler.every(10, []() -> bool {
      // Get Current Pos
      armEncoder.updateCurPos();
      float currentPos = armEncoder.getCurPos();

      Serial.println("RAISE ARM : " + String(currentPos - c3_previousArmPosition));

      // When ROC is 0 end Loop and set MidPoint Position
      if (currentPos - c3_previousArmPosition == 0) {
        c3_foundArmMidpoint = true;
        c3_armMidpoint = currentPos / 2;
        raiseArm(0);
        Serial.println("RAISE ARM STOP...");
      }

      // Update Previous Position
      c3_previousArmPosition = currentPos;
      return !c3_foundArmMidpoint; // loop?
    });
  });
}

/**
  Raises the Arm until the Positional Rate of Change is 0.
 */
void lowerArmTillROCStops() {
  lowerArm(255);
  scheduler.in(500, []() -> void { // Give time for ROC to start
    scheduler.every(10, []() -> bool {
      // Get Current Pos
      armEncoder.updateCurPos();
      float currentPos = armEncoder.getCurPos();

      Serial.println("LOWER ARM : " + String(currentPos - c3_previousArmPosition));

      // When ROC is 0 end Loop and set MidPoint Position
      if (currentPos - c3_previousArmPosition == 0) {
        lowerArm(0);
        Serial.println("LOWER ARM STOP...");
        return false; // stop looping
      }

      c3_previousArmPosition = currentPos;

      return true; // continue looping
    });
  });
}


// CLAW
void openClaw() {
  clawMotor.run(-255);
  scheduler.in(2000, []() -> void {
    clawMotor.stop();
  });
}

void closeClaw() {
  clawMotor.run(255);
  scheduler.in(2000, []() -> void {
    clawMotor.stop();
  });
}


// LINE SENSOR
bool lineOnLeft() {
  return lineSensor.readSensor1() == LINE_COLOR;
}

bool lineOnRight() {
  return lineSensor.readSensor2() == LINE_COLOR;
}

bool followLine() {
  int16_t leftSpeed = lineFollowSpeed;
  int16_t rightSpeed = lineFollowSpeed;

  bool onLeft = lineOnLeft();
  bool onRight = lineOnRight();

	// Both Within Line
	if (onLeft && onRight) {
		leftSpeed *= 1;
    rightSpeed *= 1;

	} else if (onLeft && !onRight) {
		rightSpeed *= 1;
    leftSpeed *= -0.75; // Flip Forward Direction

	// Left out, Right in : Turn Right
	} else if (!onLeft && onRight) {
		leftSpeed *= 1;
    rightSpeed *= -0.75; // Flip Forward Direction

	// Both Out of Line : Stop and End Task
	} else {
		leftSpeed *= -0.5;
    rightSpeed *= -0.5;
	}

  drive(leftSpeed, rightSpeed);

  return doLineFollow;
}

bool followLineINVERTED() {
  int16_t leftSpeed = -lineFollowSpeed;
  int16_t rightSpeed = -lineFollowSpeed;

  bool onLeft = lineOnLeft();
  bool onRight = lineOnRight();

	// Both Within Line
	if (onLeft && onRight) {
		leftSpeed *= 1;
    rightSpeed *= 1;

	} else if (onLeft && !onRight) {
		rightSpeed *= 1;
    leftSpeed *= -0.75; // Flip Forward Direction

	// Left out, Right in : Turn Right
	} else if (!onLeft && onRight) {
		leftSpeed *= 1;
    rightSpeed *= -0.75; // Flip Forward Direction

	// Both Out of Line : Stop and End Task
	} else {
		leftSpeed *= -0.5;
    rightSpeed *= -0.5;
	}

  drive(leftSpeed, rightSpeed);

  return doLineFollow;
}

bool stopAtClosestProximity() {
  // Slow Down lineFollowSpeed while approaching block.
  double distance = ultrasonicSensor.distanceCm();
  
  // if within slow zone, slow down the Line Following speed.
  if (distance <= SLOW_PROXIMITY) {
    lineFollowSpeed = lineFollowInitialSpeed * (distance / SLOW_PROXIMITY);
  } else {
    lineFollowSpeed = lineFollowInitialSpeed; // if not in close range, then
  }

  if (distance <= STOP_PROXIMITY) {
    doUSSProximity = false;
    drive(0, 0);
  }

  return doUSSProximity;
} 



///
/// SYSTEM TESTS
/// Call these Functions once at Setup.
/// Ensure these functions are the only Code being run, no interference.
/// Take these tests one thing at a time.
///

void testDrive() {
  scheduler.every(5000, []() -> bool {
    Serial.println("Drive Forward...");
    drive(driveTestSpeed, driveTestSpeed); // Forward

    scheduler.in(1000, []() -> void {
      Serial.println("Drive Backwards...");
      drive(-driveTestSpeed, -driveTestSpeed); // Reverse
    });

    scheduler.in(2000, []() -> void {
      Serial.println("Rotate Left");
      drive(-driveTestSpeed / 2, driveTestSpeed); // Rotate Left
    });

    scheduler.in(3000, []() -> void {
      Serial.println("Rotate Right");
      drive(driveTestSpeed, -driveTestSpeed / 2); // Rotate Right
    });

    scheduler.in(4000, []() -> void {
      Serial.println("Stop Driving");
      drive(0, 0); // Stop
    });

    return true; // Continue Looping
  });
}

void testArm() {
  // bringArmToTop();
  // scheduler.every(1000, []() -> bool {
  //   if (!armAtTop()) return true; // keep looping
  //   Serial.println("bringArmToTop() Finished...");

  //   scheduler.in(1000, []() -> void {
  //     bringArmToBase();

  //     scheduler.every(1000, []() -> bool {
  //       if (!armAtBase()) return true;

  //       Serial.println("bringArmToBase() Finished...");
  //       raiseArmTillROCStops();

  //       scheduler.every(1000, []() -> bool {
  //         if (!c3_foundArmMidpoint) return true;

  //         Serial.println("raiseArmTillROCStops Finished...");
  //         Serial.println("... testArm Finished ...");
  //         return false;
  //       });

  //       return false;
  //     });
  //   });

  //   return false;
  // });

  // // Loop Test
  // scheduler.every(10000, []() -> bool {
  //   Serial.println("STARTING RAISE ARM ROC...");
  //   raiseArmTillROCStops();

  //   scheduler.in(5000, []() -> void {
  //     Serial.println("STARTING LOWER ARM ROC...");
  //     lowerArmTillROCStops();
  //   });
  // });

  scheduler.every(10000, []() -> bool {
    bringArmToTop();
    scheduler.in(5000, bringArmToBase);
    return true;
  });
}

void testClaw() {
  scheduler.every(5000, []() -> bool {
    Serial.println("CLOSING CLAW...");
    closeClaw();

    scheduler.in(2500, []() -> bool {
      Serial.println("OPENING CLAW...");
      openClaw();
    });

    return true;
  });
}



///
/// CHALLENGES
///

/**
  Follow Line, stop at Closest Proximity.
 */
void challenge1() {

  doLineFollow = true;
  doUSSProximity = true;

  // CONDITIONAL CHECKER, if the challenge is finished.
  scheduler.every(300, []() -> bool {
    doLineFollow = doUSSProximity; // End lineFollow once STOP_PROXIMITY has been tripped.
    return doUSSProximity; // stop checking once USSProx has ended.
  });

  scheduler.every(50, followLine);
  scheduler.every(50, stopAtClosestProximity);
}


/**
  FollowLine, Stop at Closest Proximity, then PickUp and SetDown.
  Assumes Claw is Open, and Arm starts at Base.
 */
void challenge2() {
  challenge1();
  bringArmToTop(); // so it doesn't hit the block

  // Check for Challenge1 to Finish, then PICKUP AND PLACE DOWN OBJECT
  scheduler.every(300, []() -> bool {
    if (doUSSProximity == true) return true; // Challenge1 not done

    Serial.println("STARTING PICKUP ROUTINE...");

    // Delay for the block to be moved
    scheduler.in(5000, []() -> bool {

      // bring arm down
      bringArmToBase();

      // Check if Arm has been brought down.
      scheduler.every(1000, []() -> bool {
        if (!armAtBase()) return true; // Loop

        Serial.println("ARM AT BASE...");

        // Inch forward slowly
        int inchForwardTime = 650;
        drive(80, 80);
        // Stop Drive after time
        scheduler.in(inchForwardTime, []() -> bool {
          drive(0, 0);
          closeClaw();

          Serial.println("STOP INCHING FORWARD, AND CLOSE CLAW...");
          return false; // Don't loop
        });

        scheduler.in(inchForwardTime + 3000 /* time to close claw */, []() -> bool {
          Serial.println("CLAW AND INCH FINISHED, BRING ARM TO TOP...");

          // Lift Up
          bringArmToTop();

          // Check if Arm is Lifted.
          scheduler.every(1000, []() -> bool {
            if (!armAtTop()) return true; // keep checking

            Serial.println("ARM AT TOP, BRING ARM BACK TO BASE...");

            // Bring arm back down
            bringArmToBase();
            scheduler.every(1000, []() -> bool {
              if (!armAtBase()) return true; // keep checking

              Serial.println("ARM AT BASE, OPEN CLAW AND FINISH! ...");
              openClaw(); // DONE! :) This is encapsulation hell I know

              Serial.println("... CHALLENGE2 FINISHED ...");

              return false; // Don't loop
            });

            return false; // Don't loop
          }); 

          return false; // Don't loop
        });

        return false; // Don't loop
      });

      return false; // Don't loop
    });
  });
}


/**
  ALL AT ONCE : 
    - Find Midpoint of the Arm, and stop there.
    - Find Midpoint of the line, and stop there.
    - Open and Close the Claw 5 times??? Assumes claw starts open.
 */
bool finishedClaw = false;
bool finishedArm = false;
void Challenge3() {

  scheduler.in(3000, []() -> void {
    // Open and Close Claw 5 times
    scheduler.every(3000, []() -> bool {
      // Close Claw
      if (c3_clawCloseCount < c3_clawOpenCount) {
        closeClaw();
        c3_clawCloseCount++;

        // Open Claw
      } else if (c3_clawOpenCount < c3_clawCloseCount) {
        openClaw();
        c3_clawOpenCount++;

        // if they're equal, CLOSE
      } else if (c3_clawOpenCount != 5 && c3_clawCloseCount != 5) {
        closeClaw();
        c3_clawCloseCount++;
      }

      finishedClaw = c3_clawOpenCount == 5 && c3_clawCloseCount == 5;
      return !(c3_clawOpenCount == 5 && c3_clawCloseCount == 5);
    });

    scheduler.every(1000, []() -> bool {
      if (!finishedClaw) return true;
      // Find Midpoint of the Arm, then stop there.
      bringArmToTop();
      scheduler.every(1000, []() -> bool {
        if (!armAtTop()) return true; // loop till found

        Serial.println("FOUND ARM MIDPOINT...");
        armEncoder.updateCurPos();
        c3_armMidpoint = 650;

        scheduler.every(10, []() -> bool {
          // Maths
          armEncoder.updateCurPos();
          float currentPos = abs(armEncoder.getCurPos());
          float midpoint = abs(c3_armMidpoint);
          float min = midpoint - c3_armMidpointTolerance;
          float max = midpoint + c3_armMidpointTolerance;

          // Check if Arm is within Midpoint Tolerance
          if (currentPos >= min && currentPos <= max) {
            Serial.println("... AT ARM MIDPOINT...");
            Serial.println("MIDPOINT = " + String(currentPos));
            lowerArm(0); // Stop
            finishedArm = true;
            return false;
          
          // Too low, go up (inverted)
          } else if (armEncoder.getCurPos() > c3_armMidpoint) {
            raiseArm(100);
          }

          // Too high, go low (inverted)
          else if (armEncoder.getCurPos() < c3_armMidpoint) {
            lowerArm(100);
          }

          return true;
        });

        return false; // Stop Loop
      });

      return false;
    });

    scheduler.every(1000, []() -> bool {
      if (!finishedArm) return true;

      // Find midpoint of the line, then stop there.
      doLineFollow = true;
      scheduler.every(50, followLine);

      Serial.println("STARTING LINE DRIVE...");

      // Check
      scheduler.every(300, []() -> bool {
        // doLineFollow, until completely off the line.
        doLineFollow = lineOnLeft() || lineOnRight();

        if (!doLineFollow) {
          driveEncoderLeft.updateCurPos();
          c3_driveMidpoint = driveEncoderLeft.getCurPos() / 2;

          Serial.println("INVERTING DRIVE...");

          doLineFollow = true;
          scheduler.every(50, followLineINVERTED);
          // Drive Reverse Followline until at Midpoint
          scheduler.every(300, []() -> bool {
            float currentPos = abs(armEncoder.getCurPos());
            float midpoint = abs(c3_driveMidpoint);
            float min = midpoint - c3_driveMidpointTolerance;
            float max = midpoint + c3_driveMidpointTolerance;

            // Check if Arm is within Midpoint Tolerance
            if (currentPos >= min && currentPos <= max) {
              doLineFollow = false;
              drive(0, 0);
              Serial.println("... AT DRIVE MIDPOINT ...");
              return false; // Stop and DONE!
            }

            return true; // Keep looping
          });

          return false; // Stop loop
        }

        return true; // keep checking
      });

      return false;
    });

  });
}

