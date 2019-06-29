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
float goodDriveAngle;
enum State { sees_ball, invisible_ball, out_of_bounds, charge};
State state = invisible_ball;

float xPos = 1;
float yPos = 1;
int failedBallReadingCount = 0;
float tPos = 1;
float oPos = 1;
boolean firstInvBall = true;
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

int startTimerNow;

bool notMoved = false;

bool interrupted = false;
bool calibrating = true;
int frontDistance;
int i = 0;

int commitTimer;

void setup() {
  Serial.begin(115200);
  Serial5.begin(19200);

  //Serial.println(digitalRead(36));


  setupUtilities();
  //setupTOF();
  setupIMU();
}

void loop() {

  backSensor = lidars.readSensor3();
  leftSensor = lidars.readSensor2();
  rightSensor = lidars.readSensor4();

  frontDistance = analogRead(20);
  checkForIMUZero();
  getCameraReadings();
  calculateAngles();
  checkFieldReorient();
  ballDist = sqrt(sq(xPos) + sq(yPos));

  //set state

  if (abs(xPos - lastXPos != 0)  || state == invisible_ball || notMoved) {
    startTimerNow = millis();
    notMoved = false;
  }
  else if (millis() - startTimerNow > 3250 && state != charge) {
    notMoved = true;
  }
  else  notMoved = false;
  Serial.println(whichSide());



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
      //Serial.println("inv");
      commitTimer = millis();
      ledRed();
      invBall();
      motor.dribble(0);
      break;
    case sees_ball:
      commitTimer = millis();
      Serial.println(determineSide());
      firstInvBall = true;
      ledGreen();
      //getToBall();
      motor.dribble(0);
      break;
    case out_of_bounds:
      commitTimer = millis();
      //Serial.println("out");
      getInBounds();
      break;
    case charge:
      //Serial.println("charge");
      //state = sees_ball;
      ballCharge();
      break;
  }
}
