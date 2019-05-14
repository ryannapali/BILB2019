#include "Adafruit_VL6180X.h"
#include "Motors.h"
#include "LIDARS.h"

#define MAX_SPEED 180.0
#define INTERRUPT_PIN 39
#define SOLENOID_PIN 27
#define BUTTON_PIN 12
#define FIELD_WIDTH 78
#define FIELD_LENGTH 105
#define RED_PIN 21
#define GREEN_PIN 22

Adafruit_VL6180X vl = Adafruit_VL6180X();
Motors motor = Motors();
LIDARS lidars = LIDARS();

float ballAngle;
float goalAngle;
int ballRanges [5] = {100, 100, 100, 100, 100};

enum State { has_ball, sees_ball, invisible_ball };
State state = invisible_ball;

float xPos = 1;
float yPos = 1;
float oldXPos = 0.0;
float oldYPos = 0.0;
int failedBallReadingCount = 0;
float tPos = 1;
float oPos = 1;
float oldTPos = 0.0;
float oldOPos = 0.0;
int failedGoalReadingCount = 0;

float frontSensor;
float backSensor;
float leftSensor;
float rightSensor;
float oldFrontSensor;
float oldBackSensor;
float oldLeftSensor;
float oldRightSensor;
int failedFrontReadCount = 0;
int failedBackReadCount = 0;
int failedLeftReadCount = 0;
int failedRightReadCount = 0;

bool interrupted = false;
bool turnFixed = false;

void interrupt() {
  interrupted = true;
}

void setup() {
  Serial5.begin(19200);
  Serial.begin(115200);
  pinMode(INTERRUPT_PIN, INPUT);
  pinMode(SOLENOID_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), interrupt, RISING); //Interrupts when digitalpin rises from LOW to HIGH

  // red led
  pinMode(RED_PIN, OUTPUT);
  // green led
  pinMode(GREEN_PIN, OUTPUT);
  analogWrite(RED_PIN, 255);
  analogWrite(GREEN_PIN, 255);

  motor.imuInit();
  if (! vl.begin()) {
    Serial.println("Failed to find TOF sensor");
    while (1);
  }
}

float lastReadTime = 0.0;
int lostBallDueToPosition = 0;
void loop() {
//  if (interrupted) {
//    fixOutOfBounds();
//    return;
//  }

  getCameraReadings();
  calculateAngles();
  checkFieldReorient();
  
// Get ball TOF sensor readings:
  ballRanges[4] = ballRanges[3];
  ballRanges[3] = ballRanges[2];
  ballRanges[2] = ballRanges[1];
  ballRanges[1] = ballRanges[0];
  ballRanges[0] = vl.readRange();

  uint8_t status = vl.readRangeStatus();
  if (status != VL6180X_ERROR_NONE) {
    Serial.println(status);
    return;
  }

  // Monitoring hiccups in ball possession
  bool inRange = ballRanges[0] < 57 and ballRanges[1] < 57 and ballRanges[2] < 57 and ballRanges[3] < 57 and ballRanges[4] < 57 and ballRanges[0] > 20 and ballRanges[1] > 20 and ballRanges[2] > 20 and ballRanges[3] > 20 and ballRanges[4] > 20;
  if (xPos > 120 or xPos < 0 and inRange) {
    Serial.println("POSITION OF BALL PREVENTING POSESSION");
    lostBallDueToPosition += 1;
  } else if (xPos < 120 and xPos > 0 and inRange) {
    lostBallDueToPosition = 0;
  }
  
  if ((xPos < 120 and xPos > 0 or lostBallDueToPosition < 4) and inRange) {
    state = has_ball;
  } else if (ballAngle != 2000 and (yPos != 0.0 and xPos != 0.0)) {
    state = sees_ball;
  } else {
    state = invisible_ball;
  }

  switch (state) {
    case invisible_ball: 
      // Do something smarter here later
      analogWrite(GREEN_PIN, 255);
      motor.stopMotors();
      motor.dribble(200);
      break;
    case sees_ball:
      analogWrite(GREEN_PIN, 255);
      quadraticBall();
      break;
    case has_ball:
      analogWrite(GREEN_PIN, 0);
      goBackwardsToShoot();
      break;
  }
}

// Need to tweak for new LIDARS
void fixOutOfBounds() {
  if (abs(motor.getRelativeAngle(0.0)) < 5 or turnFixed) {
    turnFixed = true;
    
    frontSensor = lidars.readSensor1();
    backSensor = lidars.readSensor3();
    leftSensor = lidars.readSensor2();
    rightSensor = lidars.readSensor4();

    float minReading = min(min(min(frontSensor, backSensor), leftSensor), rightSensor);

    if (leftSensor != -1 and rightSensor != -1 and backSensor != -1 and frontSensor != -1) {
      if (frontSensor <= minReading and frontSensor < 21) {
        motor.driveToHeadingCorrected(-180, 0, MAX_SPEED);
      } else if (backSensor <= minReading and backSensor < 21) {
        motor.driveToHeadingCorrected(0, 0, MAX_SPEED);
      } else if (rightSensor <= minReading and rightSensor < 21) {
        motor.driveToHeadingCorrected(270, 0, MAX_SPEED);
      } else if (leftSensor <= minReading and leftSensor < 21) {
        motor.driveToHeadingCorrected(90, 0, MAX_SPEED);
      } else {
        motor.stopMotors();
        interrupted = false;
        turnFixed = false;
      }
      Serial.print("front: ");
      Serial.println(frontSensor);
      Serial.print("right: ");
      Serial.println(rightSensor);
      Serial.print("back: ");
      Serial.println(backSensor);
      Serial.print("left: ");
      Serial.println(leftSensor);
    }
  } else {
    motor.turnToAbsoluteHeading(0.0, MAX_SPEED);
  }
}
