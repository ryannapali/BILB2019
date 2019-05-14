#define INTERRUPT_PIN 39
#define IR_FRONT 20
#define DODGE_RADIUS 8.0
// Inches per second
#define SPEED_ESTIMATE 30.0
#define SOLENOID_PIN 27

#define TOTAL_DODGE_TIME 1000.0*DODGE_RADIUS*PI/SPEED_ESTIMATE

bool hasClearShot = false; 
float strafeStatus = 0;
bool shouldStrafe = false;

float frontDistIR = 0.0;
bool isExecutingForwardSpinMove = false;
bool isExecutingDodgeOnGoalie = false;

float dodgeStartTime = 0.0;

bool turningToShoot = false;

void turnShoot() {
  if (abs(goalAngle) > 10.0) {
    if (goalAngle == 0.0) {
      motor.stopMotors();
    } else {
      motor.driveToRelativeHeadingCorrected(45.0, 180, 250);
    }
    motor.dribble(255);
  } else {
    motor.stopMotors();
    shoot();
    turningToShoot = false;
  }
}

float lastShootTime = 0.0;
bool shoot() {
  if (lastShootTime == 0) {
    lastShootTime = millis();
    digitalWrite(SOLENOID_PIN, HIGH);
    delay(100);
    motor.dribble(0);
    digitalWrite(SOLENOID_PIN, LOW);
    return true;
  } else if (millis() - lastShootTime > 1000) {
    lastShootTime = millis();
    motor.dribble(0);
    digitalWrite(SOLENOID_PIN, HIGH);
    delay(100);
    digitalWrite(SOLENOID_PIN, LOW);
    return true;
  } else {
    return false;
  }
}

void goBackwardsToShoot() {
  Serial.println(tPos);
//  bool isDodging = dodgeIfNeeded();
//  if (isDodging) return;
  if (abs(motor.getRelativeAngle(180.0)) > 20.0 and turningToShoot == false) {
//    motor.turnToAbsoluteHeading(180.0, 50);
    motor.driveToRelativeHeadingCorrected(45.0, 10.0, 250);
  } else {
    if (tPos != 0 or turningToShoot) {
      if (tPos < -400 and turningToShoot == false) {
        motor.driveToHeadingCorrected(goalAngle, 180.0, 120);
      } else if (tPos != 0) {
        turnShoot();
        turningToShoot = true;
      } else {
        motor.stopMotors();
      }
    } else {
      motor.stopMotors();
    }
  }
}

bool dodgeIfNeeded() {
  frontDistIR = analogRead(IR_FRONT);
  if (frontDistIR >= 200 and isExecutingForwardSpinMove == false) {
    isExecutingForwardSpinMove = true;
  }
  if (isExecutingForwardSpinMove) executeForwardSpinMove();
  return isExecutingForwardSpinMove;
}

bool hasRotated = false;
void executeDodgeOnGoalie(){
     // Dodging left? Probably need to do some turning w/out moving first
    Serial.println("dodging on goalie");
    if (abs(motor.getRelativeAngle(270.0)) > 3.0 and hasRotated == false) {
      motor.turnToAbsoluteHeading(270.0, 180);
      return;
    } else {
      hasRotated = true;
    }
    float fractionComplete = (millis()-dodgeStartTime)/TOTAL_DODGE_TIME;
    motor.driveToHeadingCorrected(180.0, 270.0-(1.0-fractionComplete)*180.0, MAX_SPEED);
    if (fractionComplete >= 1) {
      while (abs(motor.getRelativeAngle(0.0)) > 3.0) {
        motor.turnToAbsoluteHeading(0.0, 180);
      }
      bool shotSuccessful = shoot();
      if (shotSuccessful) {
        isExecutingDodgeOnGoalie = false;
      }
    }
}

void executeForwardSpinMove(){
  // Dodging left? Probably need to do some turning w/out moving first
  Serial.println("spinning forward");
  if (abs(motor.getRelativeAngle(180.0)) > 20.0) {
    motor.driveToRelativeHeadingCorrected(90.0, 1.0, 100);
  } else {
    isExecutingForwardSpinMove = false;
  }
}

bool lateralReadingsValid = false;
bool verticalReadingsValid = false;

float persistentGoalAngleForShow = 0.0;
float verticalDistance = 20.0;

void showAndShoot() {
  motor.dribble(0);

  if (isExecutingDodgeOnGoalie) {
//    executeDodgeOnGoalie();
    return;
  }
  
  frontSensor = lidars.readSensor1();
  if (frontSensor == -1) {
    failedFrontReadCount += 1;
    if (failedFrontReadCount < 5) frontSensor = oldFrontSensor;
  } else {
    failedFrontReadCount = 0;
    oldFrontSensor = frontSensor;
  }
  
  backSensor = lidars.readSensor3();
  if (backSensor == -1) {
    failedBackReadCount += 1;
    if (failedBackReadCount < 5) backSensor = oldBackSensor;
  } else {
    failedBackReadCount = 0;
    oldBackSensor = backSensor;
  }
  
  leftSensor = lidars.readSensor2();
  if (leftSensor == -1) {
    failedLeftReadCount += 1;
    if (failedLeftReadCount < 5) leftSensor = oldLeftSensor;
  } else {
    failedLeftReadCount = 0;
    oldLeftSensor = leftSensor;
  }
  
  rightSensor = lidars.readSensor4();
  if (rightSensor == -1) {
    failedRightReadCount += 1;
    if (failedRightReadCount < 5) rightSensor = oldRightSensor;
  } else {
    failedRightReadCount = 0;
    oldRightSensor = rightSensor;
  }

  lateralReadingsValid = abs((leftSensor + rightSensor) - FIELD_WIDTH) < 8;
  verticalReadingsValid = abs((frontSensor + backSensor) - FIELD_LENGTH) < 10;
  
  Serial.println("");
  Serial.println(frontSensor);
  Serial.println(backSensor);
  Serial.println(verticalDistance);
  Serial.println(tPos);

  if (verticalReadingsValid) {
    verticalDistance = frontSensor - 31.0;
  } else {
    if (tPos != 0.0 and tPos > 200) {
      if (verticalDistance == 20.0) {
        verticalDistance = 25.0;
      }
    } else if (tPos != 0.0 and tPos < 200) {
      verticalDistance = 0.0;
    }
  }
    
  if (lateralReadingsValid) {
    if (verticalDistance < 0 or rightSensor < 25 or leftSensor < 25) {
      motor.stopMotors();
      delay(200);
      isExecutingDodgeOnGoalie = true;
    }
    if (leftSensor > rightSensor) {
      persistentGoalAngleForShow = atan(verticalDistance/(rightSensor-25.0));
      persistentGoalAngleForShow *= 57.2957795129;
      persistentGoalAngleForShow = 90 - persistentGoalAngleForShow;
    } else {
      persistentGoalAngleForShow = atan(verticalDistance/-(leftSensor-25.0));
      persistentGoalAngleForShow *= 57.2957795129;  
      persistentGoalAngleForShow = 270 - persistentGoalAngleForShow;
    }
  } else if (persistentGoalAngleForShow != 0.0) {
    motor.driveToHeadingCorrected(persistentGoalAngleForShow, 0.0, MAX_SPEED);
  } else {
    motor.driveToHeadingCorrected(0.0, 0.0, MAX_SPEED);
  }
}
