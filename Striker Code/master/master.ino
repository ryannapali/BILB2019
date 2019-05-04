#include "Adafruit_VL6180X.h"
#include "Motors.h"
#include "LIDARS.h"

Adafruit_VL6180X vl = Adafruit_VL6180X();
Motors motor = Motors();
LIDARS lidars = LIDARS();

#define MAX_SPEED 140
#define INTERRUPT_PIN 39

#define SOLENOID_PIN 27

float ballAngle;
float goalAngle;
int ballRanges [3] = {100, 100, 100};

enum State { has_ball, sees_ball, invisible_ball };
State state = invisible_ball;

float xPos = 1;
float yPos = 1;
float tPos = 1;
float oPos = 1;
float oldXPos = 1;
float oldYPos = 1;

float frontSensor;
float backSensor;
float leftSensor;
float rightSensor;

bool interrupted = false;
bool turnFixed = false;

void setup() {
  // put your setup code here, to run once:
  Serial5.begin(19200);
  Serial.begin(115200);
  pinMode(INTERRUPT_PIN, INPUT);
  pinMode(SOLENOID_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), interrupt, RISING); //Interrupts when digitalpin rises from LOW to HIGH

  motor.imuInit();
  if (! vl.begin()) {
    Serial.println("Failed to find sensor");
    while (1);
  }
}

void loop() {
  if (interrupted) {
    fixOutOfBounds();
    return;
  }
  
  getCameraReadings();
  calculateAngle();
  
  ballRanges[2] = ballRanges[1];
  ballRanges[1] = ballRanges[0];
  ballRanges[0] = vl.readRange();

  uint8_t status = vl.readRangeStatus();
  if (status != VL6180X_ERROR_NONE) {
    return;
  }
  
  if (ballRanges[0] < 50 and ballRanges[1] < 50 and ballRanges[2] < 50) {
    state = has_ball;
  } else if (xPos < 1280 and yPos < 960 and yPos != 0 and yPos != 0) {
    state = sees_ball;
  } else {
    state = invisible_ball;
  }
  
  switch (state) {
    case invisible_ball: 
      motor.stopMotors();
      motor.dribble(255);
      break;
    case sees_ball:
      quadraticBall();
      break;
    case has_ball:
      turnShoot();
      break;
  }
}

void fixOutOfBounds() {
  if (abs(motor.getAdjustedAngle(0.0)) < 5 or turnFixed) {
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
    motor.turnToHeadingGyro(0.0, MAX_SPEED);
  }
}

void interrupt() {
  interrupted = true;
}
