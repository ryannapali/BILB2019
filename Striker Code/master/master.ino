#include "Adafruit_VL6180X.h"
#include "Motors.h"
#include "LIDARS.h"

#include "robotDefines.h"

Adafruit_VL6180X vl = Adafruit_VL6180X();
Motors motor = Motors();
LIDARS lidars = LIDARS();

float lastCalledTurnToShoot = 0.0;

void interrupt() {
  interrupted = true;
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
  
  Serial.println("set pins");
  digitalWrite(RED_PIN,255);
  digitalWrite(GREEN_PIN,255);
  digitalWrite(BLUE_PIN,255);

  Serial.println("init imu");
  motor.imuInit();

  Serial.println("tof");
  if (! vl.begin()) {
    Serial.println("Failed to find TOF sensor");
    while (1);
  }
  Serial.println("start imu");
  while(motor.isCalibrated()==false){
      analogWrite(WHITEB_PIN,0);
  }
  Serial.println("done imu");
  analogWrite(WHITEB_PIN,255);
}

bool shouldKissForwards = false;

void loop() {
  if (interrupted and millis()-lastCalledTurnToShoot > 100) {
    int side = 0;
    int currentAngle = motor.getRelativeAngle(0.0);
    if(currentAngle > 45) side  = 90;
    if(currentAngle > 135) side  = 180;
    if(currentAngle < -45) side  = 270;
    if(currentAngle < -135) side  = 180;
    fixOutOfBounds(side);
    return;
  }

  checkForIMUZero();
  getCameraReadings();
  calculateAngles();
  updateTOFReadings();

  // Monitoring hiccups in ball possession
  bool inRange = ballRanges[0] < 59 and ballRanges[1] < 59 and ballRanges[2] < 59 and ballRanges[3] < 59 and ballRanges[4] < 59 and ballRanges[0] > 20 and ballRanges[1] > 20 and ballRanges[2] > 20 and ballRanges[3] > 20 and ballRanges[4] > 20;
  if (xPos > 110 or xPos < 40 and inRange) {
    lostBallDueToPosition += 1;
  } else if (xPos < 110 and xPos > 40 and inRange) {
    lostBallDueToPosition = 0;
  }
  
  if (((xPos < 110 and xPos > 40) or lostBallDueToPosition < 4) and inRange) {
    state = has_ball;
  } else if (ballAngle != 2000 and (yPos != 0.0 and xPos != 0.0)) {
    state = sees_ball;
  } else {
    state = invisible_ball;
  }

  switch (state) {
    case invisible_ball: 
      // Do something smarter here later
      motor.stopMotors(); 
      ledRed();
      if (lastBallReadTime - millis() > 250) motor.stopMotors();
      //if (lastBallReadTime - millis() > 500) motor.turnToAbsoluteHeading(0.0, MAX_SPEED);
      break;
    case sees_ball:
      if ((xPos >= 110 or xPos <= 40) and lostBallDueToPosition >= 4) ledWhite();
      else ledBlue();
      lastBallReadTime = millis();
//      turnShoot();
      diagonalBall();
      break;
    case has_ball:
      ledGreen();
      if (shouldKissForwards) {
        KISS();
      } else {
        KISSBackwards();
      }
      break;
  }
}
