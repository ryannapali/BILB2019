#include "Motors.h"
#include "LIDARS.h"

Motors motor = Motors();

#define MAX_SPEED 180;
float speedScalar = MAX_SPEED/200;

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

// Quadratic spliner
  if (xPos < 0) {
    velocityVectorAngle = atan(1.0/getSlopeTangentToSpline()(double));
    velocityVectorAngle *= 57.2957795129;
    if (xPos < 0) {
      velocityVectorAngle += 180;
    } else if (yPos < 0) {
      velocityVectorAngle += 360;
    }
    float distanceFromBall = sqrt(xPos*xPos + yPos*yPos);
    motor.driveToHeadingCorrected(velocityVectorAngle, min(MAX_SPEED, max(distanceFromBall-20, 0))*speedScalar);
  }
// Diagonal ball follower:
//  calculateAngle();
//  if (ballAngle < 180) ballAngle += 20;
//  else ballAngle -= 20;
//  if (ballAngle != 10000) {
//      float distanceFromBall = sqrt(xPos*xPos + yPos*yPos);
//      motor.driveToHeadingCorrected(ballAngle, min(MAX_SPEED, max(distanceFromBall-20, 0))*speedScalar);
//  }

// Zig-zag ball follower:
//  xTargetDiff = 100 - xPos;
//
//  if (abs(xTargetDiff) > 50) { 
//    if (xTargetDiff > 0) {
//      motor.driveToHeadingCorrected(180, min(MAX_SPEED, abs(xTargetDiff)*speedScalar));
//    } else {
//      motor.driveToHeadingCorrected(0, min(MAX_SPEED, abs(xTargetDiff)*speedScalar));
//    }
//  } else {
//    if (yPos > 0) {
//      motor.driveToHeadingCorrected(270, min(MAX_SPEED, abs(yPos)*speedScalar*2));
//    } else {
//      motor.driveToHeadingCorrected(90, min(MAX_SPEED, abs(yPos)*speedScalar*2));
//    }
//  }

  motor.dribble();
}

float getBTerm() {
  // Coordinates flipped because stupid
  backXPos = xPos - 30;
  if (yPos > 0) {
    return backXPos/(yPos - (yPos*yPos)/(2*(yPos - 10)));
  } else {
    return backXPos/(yPos - (yPos*yPos)/(2*(yPos + 10)));
  }
}

float getSlopeTangentToSpline() {
  // Evaluating the derivative at 0 leaves just the b term
  return getBTerm();
}

void clearCameraBuffer() {
  Serial5.clear();
}

void getCameraReadings() {
  char lc = Serial5.read();
  long bTimer = millis();
  while (word(0, lc) != 254) {
    lc = Serial5.read();
    if (bTimer + 400 < millis()) {
      clearCameraBuffer();
      bTimer = millis();
    }
  }
  while (Serial5.available() < 2) {
//    if (currentState == ON_LINE) break;
  }
  char highChar1 = Serial5.read();
  char lowChar1 = Serial5.read();
  while (Serial5.available() < 2) {
//    if (currentState == ON_LINE) break;
  }
  char highChar2 = Serial5.read();
  char lowChar2 = Serial5.read();
  while (Serial5.available() < 2) {
//    if (currentState == ON_LINE) break;
  }
  char highChar3 = Serial5.read();
  char lowChar3 = Serial5.read();
  while (Serial5.available() < 2) {
//    if (currentState == ON_LINE) break;
  }
  char highChar4 = Serial5.read();
  char lowChar4 = Serial5.read();
  
  xPos = word(highChar1, lowChar1);
  yPos = word(highChar2, lowChar2);
  tPos = word(highChar3, lowChar3);
  oPos = word(highChar4, lowChar4);
  if (xPos != 0) {
    xPos -= 640;
    xPos *= -1;
    xPos += 35;
  } else {
    xPos = oldXPos;
  }
  oldXPos = xPos;

  if (yPos != 0) {
    yPos -= 480;
  } else {
    yPos = oldYPos;
  }
  oldYPos = yPos;
}


void calculateAngle() {
  // Only run this if you are in fact recieving x and y data. Otherwise, ballAngle does not change
  if (xPos > 1280 || yPos > 960) { //filter out and bad readings. 2000 is sign of bad readings
    ballAngle = 2000;
  } else {
    double m = (float)(yPos) / (float)(xPos);
    ballAngle = atan((double)m);
    ballAngle *= 57.2957795129;
    if (xPos < 0) {
      ballAngle += 180;
    } else if (yPos < 0) {
      ballAngle += 360;
    }

    if (m == .75) {
      ballAngle = 10000; //ballAngle = 10000 when robot doesn't see ball
    }
  }
}
