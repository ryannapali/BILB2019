#include "Motors.h"
#include "LIDARS.h"

Motors motor = Motors();

#define MAX_SPEED 180
float speedScalar = MAX_SPEED/200.0;

float ballAngle;

void setup() {
  // put your setup code here, to run once:
  Serial5.begin(19200);
}

float xTargetDiff = 1000;

float xPos = 1;
float yPos = 1;
float tPos = 1;
float oPos = 1;
float oldXPos = 1;
float oldYPos = 1;

void loop() {
  getCameraReadings();

  diagonalBall();

  motor.dribble();
}

void zigBall(){
    xTargetDiff = 100 - xPos;

  if (abs(xTargetDiff) > 50) { 
    if (xTargetDiff > 0) {
      motor.driveToHeadingCorrected(180, min(MAX_SPEED, abs(xTargetDiff)*speedScalar));
    } else {
      motor.driveToHeadingCorrected(0, min(MAX_SPEED, abs(xTargetDiff)*speedScalar));
    }
  } else {
    if (yPos > 0) {
      motor.driveToHeadingCorrected(270, min(MAX_SPEED, abs(yPos)*speedScalar*2));
    } else {
      motor.driveToHeadingCorrected(90, min(MAX_SPEED, abs(yPos)*speedScalar*2));
    }
  }
}

void diagonalBall(){
  calculateAngle();
  if (ballAngle < 180) ballAngle += 20;
  else ballAngle -= 20;
  if (ballAngle != 10000) {
      float distanceFromBall = sqrt(xPos*xPos + yPos*yPos);
      motor.driveToHeadingCorrected(ballAngle, min(MAX_SPEED, max(distanceFromBall-20, 0))*speedScalar);
  }
}

void quadraticBall(){
    if (xPos < 0) {
    float velocityVectorAngle = atan(1.0/getSlopeTangentToSpline());
    velocityVectorAngle *= 57.2957795129;
    if (xPos < 0) {
      velocityVectorAngle += 180;
    } else if (yPos < 0) {
      velocityVectorAngle += 360;
    }
    float distanceFromBall = sqrt(xPos*xPos + yPos*yPos);
    motor.driveToHeadingCorrected(velocityVectorAngle, min(MAX_SPEED, max(distanceFromBall-20, 0))*speedScalar);
  }
}

