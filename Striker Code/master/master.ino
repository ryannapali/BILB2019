#include "Adafruit_VL6180X.h"
#include "Motors.h"
#include "LIDARS.h"

#include "robotDefines.h"

Adafruit_VL6180X vl = Adafruit_VL6180X();
Motors motor = Motors();
LIDARS lidars = LIDARS();

// Ignore interrupt when this is true
bool doingCornerShot = false;
bool isShooting = false;

int backwardsStrategy = 0;
bool shouldDodge = false;

void interrupt() {
  if (gyroHathBeenSet and (doingCornerShot == false or shouldDodge)) {
    interrupted = true;
  }
}

bool shouldBackUp = false;

float lastDidNotHaveBall = 0.0;
float lastLostBall = 0.0;

void setup() {
  Serial5.begin(19200);
  Serial6.begin(38400);
  Serial.begin(115200);

  // For imu problems
  delay(600);

  pinMode(INTERRUPT_PIN, INPUT);
  pinMode(SOLENOID_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // DIP switches
  pinMode(S_ONE_PIN, INPUT_PULLUP);
  pinMode(S_TWO_PIN, INPUT_PULLUP);
  pinMode(S_THREE_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), interrupt, RISING); //Interrupts when digitalpin rises from LOW to HIGH

  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  Serial6.println("set pins");
  analogWrite(RED_PIN,255);
  analogWrite(GREEN_PIN,255);
  analogWrite(BLUE_PIN,255);
  
  flash();

  Serial6.println("init imu");
  motor.imuInit();

  while (! vl.begin()) {
    Serial6.println("Failed to find TOF sensor");
    (200);
  }
  
  Serial6.println("Found TOF sensor");
  
  Serial6.println("start imu");
  while (motor.isCalibrated() == false) {
      analogWrite(RED_PIN,0);
  }
  Serial6.println("done imu");
  analogWrite(RED_PIN,255);
}

void loop() {
  if (shouldWriteLow and millis() - wroteHigh > 100) {
    digitalWrite(SOLENOID_PIN, LOW);
    shouldWriteLow = false;
  }

  if (interrupted and millis()-lastCalledTurnToShoot > 100) {
    int side = 0;
    int currentAngle = motor.getRelativeAngle(0.0);
    if(currentAngle > 45) side = 90;
    if(currentAngle > 135) side = 180;
    if(currentAngle < -45) side = 270;
    if(currentAngle < -135) side = 180;
    fixOutOfBounds(side);
    
    return;
  }

  checkForIMUZero();
  getCameraReadings();
  calculateAngles();
  updateTOFReadings();

//  logLIDARS();

//  printBallPosition();
//  return;

  if (millis() - lastShootTime < 1000) {
    turningToShoot = false;
    isShooting = false;
  }
  
  // Monitoring hiccups in ball possession
  bool inRange = ballRanges[0] < 59 and ballRanges[1] < 59 and ballRanges[2] < 59 and ballRanges[3] < 59 and ballRanges[4] < 59 and ballRanges[0] > 0 and ballRanges[1] > 0 and ballRanges[2] > 0 and ballRanges[3] > 0 and ballRanges[4] > 0;
  bool ballInCameraRange = xPos < MAXIMUM_HAS_BALL_X and xPos > MINIMUM_HAS_BALL_X and yPos < MAXIMUM_HAS_BALL_Y and yPos > MINIMUM_HAS_BALL_Y; 

  if (ballInCameraRange) {
    lostBallDueToPosition = 0;
  } else {
    lostBallDueToPosition += 1;
  }

  if (inRange) {
    lostBallDueToTOF = 0;
  } else {
    lostBallDueToTOF += 1;
  }

  if ((ballInCameraRange or lostBallDueToPosition < 10) and (inRange or lostBallDueToTOF < 10)) {
    readLIDARS(0.0);
    state = has_ball;
  } else if (ballAngle != 2000 and yPos != 0.0 and xPos != 0.0) {
    readLIDARS(500.0);
    if (state == has_ball) lastLostBall = millis();
    state = sees_ball;
  } else {
    readLIDARS(0.0);
    if (state == has_ball) lastLostBall = millis();
    state = invisible_ball;
  }

  if (state != has_ball and millis() - lastHadBall > 1000) {
    if (abs(motor.getRelativeAngle(0.0)) < 90) {
      shouldKissForwards = true;
    } else {
      shouldKissForwards = false;
    }
  }

  if (state != has_ball and millis() - lastHadBall > 500) {
    shouldDodge = false;
  }

  switch (state) {
    case invisible_ball: 
      // Do something smarter here later
      lastDidNotHaveBall = millis();

      backwardsStrategy = 0;
      if (doingCornerShot and millis() - lastHadBall > 100) {
        doingCornerShot = false;
        useSideLIDAR = false;
        useFrontLIDAR = false;
      }

      if (shouldWriteLow) return;

      if (millis() - lastBallReadTime > 1000) {
        if (abs(motor.getRelativeAngle(0.0)) > 8.0) {
          motor.turnToAbsoluteHeading(0.0, 180);
        } else {
          goToPoint();          
        }
      } else {
        motor.stopMotors();
      }
  
      break;
    case sees_ball:  
      lastDidNotHaveBall = millis();
      backwardsStrategy = 0;
      if (doingCornerShot and millis() - lastHadBall > 100) {
        doingCornerShot = false;
        useSideLIDAR = false;
        useFrontLIDAR = false;
      }

      ledRed();
      lastBallReadTime = millis();
      
      if (shouldWriteLow) return;
      
      diagonalBall();
      break;
    case has_ball:
      ledBlue();
      if (millis() - lastDidNotHaveBall < 200.0) {
        BACK_SPEED = 60.0;
      } else {
        BACK_SPEED = 60.0 + min((millis() - lastDidNotHaveBall - 200.0)/800.0, 1.0)*(MAX_BACK_SPEED-60.0);
      }
      
      lastHadBall = millis();

      if (shouldWriteLow) return;
      if (shouldKissForwards) {
        if (digitalRead(S_ONE_PIN) == HIGH) KISS();
        else KISSBackwards();
      } else {
        if (digitalRead(S_TWO_PIN) == HIGH) KISSBackwards();
        else KISS();
      }
      break;
  }
}

void goToPoint() {
  float slope = (backDistance-90.0)/(rightDistance - FIELD_WIDTH/2.0);
  float distance = sqrt(sq(backDistance-75.0) + sq(rightDistance - FIELD_WIDTH/2.0));
  float angle = atan(slope);
  angle *= 57.2957795129;
  if (rightDistance - FIELD_WIDTH/2.0 < 0) {
    angle = -90.0 + angle;
  } else {
    angle = 90.0 + angle;
  }

  motor.driveToHeadingCorrected(angle, 0.0, min(max(4.0*distance, 60), MAX_SPEED));
}
