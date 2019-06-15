#include "Adafruit_VL6180X.h"
#include "Motors.h"
#include "LIDARS.h"
#include <algorithm>

#define MAX_SPEED 255.0
#define INTERRUPT_PIN 39
#define SOLENOID_PIN 27
#define BUTTON_PIN 12
#define FIELD_WIDTH 78
#define FIELD_LENGTH 105
#define LIDAR_GOAL
#define BLUE_PIN 21
#define GREEN_PIN 22
#define RED_PIN 23
#define WHITEA_PIN 17
#define WHITEB_PIN 28


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
float lastXPos = 0.0;
float lastYPos = 0.0;
int failedBallReadingCount = 0;
float tPos = 1;
float oPos = 1;
float oldTPos = 0.0;
float oldOPos = 0.0;
int failedGoalReadingCount = 0;

float timeSinceBallMoved = 0.0;
boolean attackMode = false;
float attackModeStart = 0.0;

float frontSensor;
float backSensor;
float leftSensor;
float rightSensor;

float sideSum = 0.0;
boolean sideSumConfident = false;

bool interrupted = false;

bool calibrating = true;

void interrupt() {
  if (!calibrating) {
    interrupted = true;
    //Serial.println("abasdf");
  }
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

  flash();

  motor.imuInit();
  //  if (! vl.begin()) {
  //    Serial.println("Failed to find TOF sensor");
  //          analogWrite(28,100);
  //
  //    while (1);
  //  }
  //Serial.println("start imu");
  while (motor.isCalibrated() == false) {
    analogWrite(WHITEB_PIN, 0);
  }
  Serial.println("done imu");
  analogWrite(WHITEB_PIN, 255);
}

void loop() {
  if (interrupted) {
    Serial.println("LINE");
    int side = 0;
    int currentAngle = motor.getRelativeAngle(0.0);
    //fixOutOfBounds();
    getInBounds();
    return;
  }

  lastXPos = xPos;
  lastYPos = yPos;
  checkForIMUZero();
  getCameraReadings();
  calculateAngles2();
  checkFieldReorient();
  updateBallMotion();
  //
  //  if (false) { //reimplement
  //    state = has_ball;
  //  }
  if (ballAngle != 2000 and (yPos != 0.0 and xPos != 0.0)) {
    state = sees_ball;
  } else {
    state = invisible_ball;
  }

  switch (state) {
    case invisible_ball:
      ledBlue();
      //centerToGoal();
      motor.stopMotors();
      break;
    case sees_ball:
      getToBall();
      //testfunc();
      ledGreen();
      //quadraticBall();

      //Serial.println(ballAngle);
      //      if(attackMode){
      //        if(millis()-attackModeStart > 2000) attackMode = false;
      //        diagonalBall();
      //      }
      //      else{
      //        attackMode = false;
      //        blockBall();
      //      }
      break;
    case has_ball:
      break;
  }
}


void testfunc() {
        leftSensor = lidars.readSensor2();
        rightSensor = lidars.readSensor4();
        Serial.println(rightSensor);
        if(rightSensor + leftSensor > 170){
            motor.driveToHeadingCorrected(90, 0, min(min(abs(yPos), rightSensor-30), MAX_SPEED));
            if(rightSensor < 60){
              motor.driveToHeadingCorrected(270, 0, 180);
            }
        }

}

