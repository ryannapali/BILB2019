#include "Adafruit_VL6180X.h"
#include "Motors.h"
#include "LIDARS.h"
#include <algorithm>
#include "robot_defines.h"

Adafruit_VL6180X vl = Adafruit_VL6180X();
Motors motor = Motors();
LIDARS lidars = LIDARS();

float ballAngle;
float goalAngle;

enum State { has_ball, sees_ball, invisible_ball, out_of_bounds };
State state = invisible_ball;

float xPos = 1;
float yPos = 1;
int failedBallReadingCount = 0;
float tPos = 1;
float oPos = 1;
int failedGoalReadingCount = 0;

float goalDist;
float ballDist;

float frontSensor;
float backSensor;
float leftSensor;
float rightSensor;

bool interrupted = false;
bool calibrating = true;

int i = 0;

void setup() {
  Serial.begin(115200);
  Serial5.begin(19200);

  setupUtilities();
  //setupTOF();
  setupIMU();
}

void loop() {
  checkForIMUZero();
  getCameraReadings();
  calculateAngles();
  checkFieldReorient();

  //set state
  if (interrupted) state = out_of_bounds;
  else if (ballAngle != 2000 and (yPos != 0.0 and xPos != 0.0)) {
    if (ballAngle < 5 && ballAngle > 355 & ballDist < 50) state = has_ball; //CHECK THESE NUMBERS
    else state = sees_ball;
  }

  else state = invisible_ball;

  switch (state) {
    case invisible_ball:
      ledRed();
      motor.stopMotors();
      break;
    case sees_ball:
      i++;
      if (i == 10) {
        prepInvBall();
        i = 0;
      }
      ledGreen();
      getToBall();
      //optimalPosition();
      break;
    case has_ball:
      break;
    case out_of_bounds:
      getInBounds();
      break;
  }
}
