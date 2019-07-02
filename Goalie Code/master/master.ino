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
float newBS;
float newLS;
enum State { sees_ball, invisible_ball, out_of_bounds, charge};
State state = invisible_ball;

float xPos = 1;
float yPos = 1;
int failedBallReadingCount = 0;
float tPos = 1;
float oPos = 1;
int failedGoalReadingCount = 0;

float lastXPos;
float lastYPos;
float lastAngle = 0;


float goalDist;
float ballDist;

float frontSensor;
float backSensor;
float leftSensor;
float rightSensor;

int frontDistance;

int startTimerNow;
int commitTimer;

bool notMoved = false;
bool firstInvBall = true;

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

  frontDistance = analogRead(20);
  checkForIMUZero();
  getCameraReadings();
  calculateAngles();
  checkFieldReorient();
  ballDist = sqrt(sq(xPos) + sq(yPos));

  //set state
  int lastDist = sqrt(sq(lastXPos) + sq(lastYPos));


  if (abs(ballDist - lastDist < 7)  || state == invisible_ball || notMoved) {
    startTimerNow = millis();
    notMoved = false;
  }
  else if (millis() - startTimerNow > 3250 && state != charge) {
    notMoved = true;
  }
  else  notMoved = false;



  if (interrupted) state = out_of_bounds;
  else if (state == charge);
  else if (ballAngle != 2000 and (yPos != 0.0 and xPos != 0.0)) {
    if (digitalRead(36) == LOW && notMoved) { // notMoved || (digitalRead(36) == HIGH && (ballAngle < 5 || ballAngle > 355 && ballDist < 75))
      state = charge;
    }
    else {
      state = sees_ball;
    }
  }
  else state = invisible_ball;


  switch (state) {
    case invisible_ball:
      ledRed();
      commitTimer = millis();
      invBall();
      motor.dribble(0);
      break;
    case sees_ball:
      commitTimer = millis();
      firstInvBall = true;
      ledGreen();
      getToBall();
      motor.dribble(0);
      break;
    case out_of_bounds:
      commitTimer = millis();
      getInBounds();
      break;
    case charge:
      ballCharge();
      break;
  }
}
