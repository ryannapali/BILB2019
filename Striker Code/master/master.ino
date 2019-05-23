#include "Adafruit_VL6180X.h"
#include "Motors.h"
#include "LIDARS.h"

#include "robotDefines.h"

Adafruit_VL6180X vl = Adafruit_VL6180X();
Motors motor = Motors();
LIDARS lidars = LIDARS();

void interrupt() {
  interrupted = true;
}

void setup() {  
  Serial5.begin(19200);
  Serial.begin(115200);

  // For imu problems
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
  bool ballInCameraRange = xPos < MAXIMUM_HAS_BALL_X and xPos > MINIMUM_HAS_BALL_X and yPos < MAXIMUM_HAS_BALL_Y and yPos > MINIMUM_HAS_BALL_Y; 
  if (not ballInCameraRange and inRange) {
    lostBallDueToPosition += 1;
  } else if ((ballInCameraRange and inRange) or (not inRange)) {
    lostBallDueToPosition = 0;
  }

  if (inRange) {
    lostBallDueToTOF = 0;
  } else {
    lostBallDueToTOF += 1;
  }
  
  if ((ballInCameraRange or lostBallDueToPosition < 10) and (inRange or lostBallDueToTOF < 10)) {
    state = has_ball; 
  } else if (ballAngle != 2000 and (yPos != 0.0 and xPos != 0.0)) {
    state = sees_ball;
  } else {
    state = invisible_ball;
  }

  if (state != has_ball) {
    isOrbiting = false;
  }
  if (state != has_ball and millis() - lastHadBall > 1000) {
    if (abs(motor.getRelativeAngle(0.0)) < 90) {
      shouldKissForwards = true;
    } else {
      shouldKissForwards = false;
    }
  }

  switch (state) {
    case invisible_ball: 
      // Do something smarter here later
      ledRed();
      if (millis() - lastBallReadTime > 1000) centerRobot();
      else if (millis() - lastBallReadTime > 0) motor.stopMotors();  
      break;
    case sees_ball:  
      if (not ballInCameraRange and lostBallDueToPosition >= 4) ledYellow();
      else ledBlue();
      lastBallReadTime = millis();
      diagonalBall();
      break;
    case has_ball:
      ledGreen(); 
      
      lastHadBall = millis();
      if (shouldKissForwards) {
        KISS();
      } else {
        KISSBackwards();
      }
      break;
  }
}

float lastLeft = 0.0;
float lastRight = 0.0;

void centerRobot() {
  backSensor = lidars.readSensor3();
  leftSensor = lidars.readSensor2();
  rightSensor = lidars.readSensor4();
  float sideSum = leftSensor + rightSensor;

  lastLeft = leftSensor;
  lastRight = rightSensor;
  
  float distanceFromGoal = 90.0;
  float centerDistance = 93.0;
  float distanceOff = backSensor-distanceFromGoal;

  bool sideSumConfident = (abs(sideSum-185) < 10);
  if (not sideSumConfident) {
    if (abs(lastLeft - leftSensor) < 4) {
      rightSensor = lastRight + lastLeft - leftSensor;
    } else if (abs(lastRight - rightSensor) < 4) {
      leftSensor = lastLeft + lastRight - rightSensor;
    } else {
      motor.stopMotors();
      return;
    }
  }
  
  if (abs(motor.getRelativeAngle(0.0)) > 5) motor.turnToAbsoluteHeading(0.0, MAX_SPEED);
  else if (abs(leftSensor - centerDistance) > 3) motor.driveToHeadingCorrectedHoldDistance(270, 0, 4.0*(leftSensor-centerDistance), distanceOff);
  else if (abs(rightSensor - centerDistance) > 3) motor.driveToHeadingCorrectedHoldDistance(90, 0, 4.0*(rightSensor-centerDistance), distanceOff);
  else motor.stopMotors();
}
