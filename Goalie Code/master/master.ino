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

float frontSensor;
float backSensor;
float leftSensor;
float rightSensor;

bool interrupted = false;
bool calibrating = true;

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
  else if (ballAngle != 2000 and (yPos != 0.0 and xPos != 0.0)) state = sees_ball;
  else state = invisible_ball;

  switch (state) {
    case invisible_ball:
      ledRed();
      motor.stopMotors();
      break;
    case sees_ball:
      ledGreen();
      getToBall();
      break;
    case has_ball:
      break;
    case out_of_bounds:
      getInBounds();
      break;
  }
}
