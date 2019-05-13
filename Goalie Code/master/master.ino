#include "Adafruit_VL6180X.h"
#include "Motors.h"
#include "LIDARS.h"

Adafruit_VL6180X vl = Adafruit_VL6180X();
Motors motor = Motors();
LIDARS lidars = LIDARS();

#define MAX_SPEED 220
#define INTERRUPT_PIN 39

#define SOLENOID_PIN 27

#define BUTTON_PIN 12

float ballAngle;
float goalAngle;
int ballRanges [3] = {100, 100, 100};

enum State { has_ball, sees_ball, invisible_ball };
State state = invisible_ball;

float xPos = 1;
float yPos = 1;
float tPos = 1; 
float oPos = 1;

float frontSensor;
float backSensor;
float leftSensor;
float rightSensor;

bool interrupted = false;
bool turnFixed = false;

void setup() {
  Serial5.begin(19200);
  Serial.begin(115200);
  pinMode(INTERRUPT_PIN, INPUT);
  pinMode(SOLENOID_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), interrupt, RISING); //Interrupts when digitalpin rises from LOW to HIGH

  // red led
  pinMode(21, OUTPUT);
  analogWrite(21, 255);

  motor.imuInit();
  if (! vl.begin()) {
    Serial.println("Failed to find TOF sensor");
    while (1);
  }
}

void loop() {
  getCameraReadings();
  calculateAngles();

  if (false) { //reimplement
    state = has_ball;
  } else if (ballAngle != 2000 and (yPos != 0.0 and xPos != 0.0)) {
    state = sees_ball;
  } else {
    state = invisible_ball;
  }
 
  switch (state) {
    case invisible_ball: 
      centerToGoal();
      break;
    case sees_ball:
      blockBall();
      break;
    case has_ball:
      passBall();
      break;
  }
}

void interrupt() {
  interrupted = true;
}
