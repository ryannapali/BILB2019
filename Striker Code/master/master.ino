#include "Adafruit_VL6180X.h"
#include "Motors.h"
#include "LIDARS.h"

#include "robotDefines.h"

Adafruit_VL6180X vl = Adafruit_VL6180X();
Motors motor = Motors();
LIDARS lidars = LIDARS();

// Ignore interrupt when this is true
bool doingCornerShot = false;

int backwardsStrategy = 0;
bool shouldDodge = false;

void interrupt() {
  if (gyroHathBeenSet and doingCornerShot == false) {
    interrupted = true;
  }
}

float loopTime;
bool shouldBackUp = false;

float lastDidNotHaveBall = 0.0;
float lastLostBall = 0.0;
void setup() {  
  Serial5.begin(19200);
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
  
  flash();
  
  Serial.println("set pins");
  digitalWrite(RED_PIN,255);
  digitalWrite(GREEN_PIN,255);
  digitalWrite(BLUE_PIN,255);

  Serial.println("init imu");
  motor.imuInit();

  while (! vl.begin()) {
    Serial.println("Failed to find TOF sensor");
    delay(200);
  }
  
  Serial.println("Found TOF sensor");
  
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
    if(currentAngle > 45) side = 90;
    if(currentAngle > 135) side = 180;
    if(currentAngle < -45) side = 270;
    if(currentAngle < -135) side = 180;
    fixOutOfBounds(side);
    
    return;
  }

  logLIDARS();

  checkForIMUZero();
  getCameraReadings();
  calculateAngles();
  updateTOFReadings();
  
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
    if (not turningToShoot) readLIDARS(0.0);
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
//      if (digitalRead(S_TWO_PIN) == HIGH) shouldKissForwards = false;
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
      ledRed();
      motor.stopMotors();
      backwardsStrategy = 0;
      doingCornerShot = false;

      break;
    case sees_ball:  
      lastDidNotHaveBall = millis();
      backwardsStrategy = 0;
      doingCornerShot = false;
            
      if (not ballInCameraRange and lostBallDueToPosition >= 10 and inRange) ledWhite();
      else ledBlue();
      lastBallReadTime = millis();
      diagonalBall();
      break;
    case has_ball:
      if (millis() - lastDidNotHaveBall < 200.0) {
        BACK_SPEED = 60.0;
      } else {
        BACK_SPEED = 60.0 + min((millis() - lastDidNotHaveBall - 200.0)/800.0, 1.0)*(MAX_BACK_SPEED-60.0);
      }
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
