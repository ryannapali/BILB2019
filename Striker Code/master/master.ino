#include "Adafruit_VL6180X.h"
#include "Motors.h"
#include "LIDARS.h"

#define MAX_SPEED 230.0

#define INTERRUPT_PIN 39
#define SOLENOID_PIN 27
#define BUTTON_PIN 12
#define FRONT_IR_PIN 20
#define RED_PIN 21
#define GREEN_PIN 22
#define BLUE_PIN 23

#define FIELD_WIDTH 185
#define FIELD_LENGTH 244

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

  delay(600);

  pinMode(INTERRUPT_PIN, INPUT);
  pinMode(SOLENOID_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), interrupt, RISING); //Interrupts when digitalpin rises from LOW to HIGH

  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  motor.imuInit();
  
  if (! vl.begin()) {
    Serial.println("Failed to find TOF sensor");
    while (1);
  }

  delay(100);
}

float lastBallReadTime = 0.0;
int lostBallDueToPosition = 0;

void loop() {
  if (interrupted) {
    fixOutOfBounds();
    return;
  }

  checkForIMUZero();
  
  getCameraReadings();
  calculateAngles();
  updateTOFReadings();

  // Monitoring hiccups in ball possession
  bool inRange = ballRanges[0] < 59 and ballRanges[1] < 59 and ballRanges[2] < 59 and ballRanges[3] < 59 and ballRanges[4] < 59 and ballRanges[0] > 20 and ballRanges[1] > 20 and ballRanges[2] > 20 and ballRanges[3] > 20 and ballRanges[4] > 20;
  if (xPos > 130 or xPos < 40 and inRange) {
    lostBallDueToPosition += 1;
  } else if (xPos < 130 and xPos > 40 and inRange) {
    lostBallDueToPosition = 0;
  }
  
  if (((xPos < 130 and xPos > 40) or lostBallDueToPosition < 4) and inRange) {
    state = has_ball;
  } else if (ballAngle != 2000 and (yPos != 0.0 and xPos != 0.0)) {
    state = sees_ball;
  } else {
    state = invisible_ball;
  }

  switch (state) {
    case invisible_ball: 
      // Do something smarter here later
      ledRed();
      if (lastBallReadTime - millis() > 250) motor.stopMotors();
      if (lastBallReadTime - millis() > 500) motor.turnToAbsoluteHeading(0.0, MAX_SPEED);
      motor.dribble(200);
      break;
    case sees_ball:
      if ((xPos >= 130 or xPos <= 40) and lostBallDueToPosition >= 4) ledCyan();
      else ledBlue();
      lastBallReadTime = millis();
      diagonalBall();
      break;
    case has_ball:
      ledGreen();
      goBackwardsToShoot();
      break;
  }
}

void fixOutOfBounds() {
  logLIDARS();
    
  if (abs(motor.getRelativeAngle(0.0)) < 5 or turnFixed) {
    turnFixed = true;
    
    frontSensor = lidars.readSensor1();
    backSensor = lidars.readSensor3();
    leftSensor = lidars.readSensor2();
    rightSensor = lidars.readSensor4();

    float minReading = min(min(min(frontSensor, backSensor), leftSensor), rightSensor);

    if (frontSensor <= minReading and frontSensor < 36) {
      motor.driveToHeadingCorrected(-180, 0, MAX_SPEED);
      return;
    } else if (backSensor <= minReading and backSensor < 36) {
      motor.driveToHeadingCorrected(0, 0, MAX_SPEED);
      return;
    } else if (rightSensor <= minReading and rightSensor < 36) {
      if (leftSensor < 36) {
        if (frontSensor < backSensor) {
          motor.driveToHeadingCorrected(-180, 0, MAX_SPEED);
        } else {
          motor.driveToHeadingCorrected(0, 0, MAX_SPEED);
        }
      } else {
        motor.driveToHeadingCorrected(270, 0, MAX_SPEED);
      }
      return;
    } else if (leftSensor <= minReading and leftSensor < 36) {
      if (rightSensor < 36) {
        if (frontSensor < backSensor) {
          motor.driveToHeadingCorrected(-180, 0, MAX_SPEED);
        } else {
          motor.driveToHeadingCorrected(0, 0, MAX_SPEED);
        }
      } else {
        motor.driveToHeadingCorrected(90, 0, MAX_SPEED);
      }
      return;
    } else {
      motor.stopMotors();
      interrupted = false;
      turnFixed = false;
    }
  } else {
    motor.turnToAbsoluteHeading(0.0, MAX_SPEED);
  }
}
