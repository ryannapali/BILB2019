#include "Adafruit_VL6180X.h"
#include "Motors.h"
#include "LIDARS.h"

#define MAX_SPEED 180.0
#define INTERRUPT_PIN 39
#define SOLENOID_PIN 27
#define BUTTON_PIN 12
#define FIELD_WIDTH 78
#define FIELD_LENGTH 105
#define LIDAR_GOAL 
#define BLUE_PIN 21
#define GREEN_PIN 22
#define RED_PIN 23


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

float sideSum = 0.0;
boolean sideSumConfident = false;

bool interrupted = false;
bool turnFixed = false;

void setup() {
  float proportionals[] = {sin(-3.14 + 3.92699082), sin(-3.14 + 5.28834763), sin(-3.14 + 0.994837674), sin(-3.14 + 2.35619449)};
  Serial5.begin(19200);
  Serial.begin(115200);
  delay(600);
  pinMode(INTERRUPT_PIN, INPUT);
  pinMode(SOLENOID_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), interrupt, RISING); //Interrupts when digitalpin rises from LOW to HIGH
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN,OUTPUT);
  
  motor.imuInit();
//  if (! vl.begin()) {
//    Serial.println("Failed to find TOF sensor");
//          analogWrite(28,100);
//
//    while (1);
//  }
    Serial.println("start imu");
  while(motor.isCalibrated()==false){
      analogWrite(28,0);
  }
  Serial.println("done imu");
  analogWrite(28,255);
  delay(300);
  analogWrite(28,0);

}

void loop() {
  Serial.println("doing");
  checkForIMUZero();
  Serial.println("doing 1");
  getCameraReadings();
  calculateAngles();
  checkFieldReorient();

  if (false) { //reimplement
    state = has_ball;
  } else if (ballAngle != 2000 and (yPos != 0.0 and xPos != 0.0)) {
    state = sees_ball;
  } else {
    state = invisible_ball;
  }

  switch (state) {
    case invisible_ball: 
      ledRed();
      centerToGoal();
      break;
    case sees_ball:
      ledGreen();
      blockBall();
      break;
    case has_ball:
//      passBall();
      break;
  }
    Serial.println("doing 4");

}

void interrupt() {
  interrupted = true;
}
