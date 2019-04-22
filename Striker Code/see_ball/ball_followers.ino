#include "Motors.h"
#include "LIDARS.h"

Motors motor = Motors();
LIDARS lidars = LIDARS();

#define MAX_SPEED 180
#define INTERRUPT_PIN 39

float speedScalar = MAX_SPEED/150.0;

float ballAngle;

float k = 20.0;

void setup() {
  // put your setup code here, to run once:
  Serial5.begin(19200);
  Serial.begin(115200);
  pinMode(INTERRUPT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), interrupt, RISING); //Interrupts when digitalpin rises from LOW to HIGH
}

bool interrupted = false;
void interrupt() {
  interrupted = true;
}

float xTargetDiff = 1000;

float xPos = 1;
float yPos = 1;
float tPos = 1;
float oPos = 1;
float oldXPos = 1;
float oldYPos = 1;

bool turnFixed = false;
void loop() {  
  if (interrupted) {
    fixOutOfBounds();
    return;
  }
  motor.dribble();
  getCameraReadings();
  calculateAngle();
  if (xPos == 0 or yPos == 0) {
    motor.stopMotors();
    return;
  }

//  quadraticBall();
//  diagonalBall();
//  zigBall();
}

void zigBall(){
  xTargetDiff = 100 - xPos;

  if (abs(xTargetDiff) > 50) { 
    if (xTargetDiff > 0) {
      motor.driveToHeadingCorrected(180, 0.0, min(MAX_SPEED, abs(xTargetDiff)*speedScalar));
    } else {
      motor.driveToHeadingCorrected(0, 0.0, min(MAX_SPEED, abs(xTargetDiff)*speedScalar));
    }
  } else {
    if (yPos > 0) {
      motor.driveToHeadingCorrected(270, 0.0, min(MAX_SPEED, abs(yPos)*speedScalar*2));
    } else {
      motor.driveToHeadingCorrected(90, 0.0, min(MAX_SPEED, abs(yPos)*speedScalar*2));
    }
  }
}

void diagonalBall(){
  calculateAngle();
  float distanceFromBall = sqrt(xPos*xPos + yPos*yPos);
  motor.driveToHeadingCorrected(ballAngle, ballAngle + motor.getAdjustedAngle(0.0), min(MAX_SPEED, max(distanceFromBall-20, 0))*speedScalar);
}

void quadraticBall(){
  float velocityVectorAngle = atan(1.0/getBQuadraticTerm());
  velocityVectorAngle *= 57.2957795129;
  if (xPos-30 < 0) {
    velocityVectorAngle += 180;
  } else if (yPos < 0) {
    velocityVectorAngle += 360;
  }
  if (velocityVectorAngle < 0) {
    velocityVectorAngle += 180;
  }
  float distanceFromBall = sqrt(xPos*xPos + yPos*yPos);
  motor.driveToHeadingCorrected(velocityVectorAngle, 0.0, min(MAX_SPEED, max(distanceFromBall-10, 0))*speedScalar);
}

void fixOutOfBounds() {
  if (abs(motor.getAdjustedAngle(0.0)) < 5 or turnFixed) {
    turnFixed = true;
    
    float frontSensor = lidars.readSensor1();
    float backSensor = lidars.readSensor3();
    float leftSensor = lidars.readSensor2() + 6.0;
    float rightSensor = lidars.readSensor4() + 6.0;

    float minReading = min(min(min(frontSensor, backSensor), leftSensor), rightSensor);

    if (leftSensor != 5 and rightSensor != 5 and backSensor != -1 and frontSensor != -1) {
      if (frontSensor <= minReading and frontSensor < 25) {
        motor.driveToHeadingCorrected(-180, 0, MAX_SPEED);
      } else if (backSensor <= minReading and backSensor < 25) {
        motor.driveToHeadingCorrected(0, 0, MAX_SPEED);
      } else if (rightSensor <= minReading and rightSensor < 25) {
        motor.driveToHeadingCorrected(270, 0, MAX_SPEED);
      } else if (leftSensor <= minReading and leftSensor < 25) {
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
