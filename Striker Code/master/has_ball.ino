#define INTERRUPT_PIN 39
#define IR_FRONT 20
#define DODGE_RADIUS 8.0
// Inches per second
#define SPEED_ESTIMATE 30.0
#define SOLENOID_PIN 27
#define MAXIMUM_SHOT_DISTANCE 400
#define CORNER_WALL_DISTANCE 63.0

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
  if (goalAngle == 0.0) {
    motor.stopMotors();
    return;
  }
  
  if (abs(goalAngle) > 5.0) {
    if (goalAngle < 0) {
      motor.driveToRelativeHeadingCorrected(80.0, 10.0, 200);
    } else {
      motor.driveToRelativeHeadingCorrected(-80.0, -10.0, 200);
    }
//    motor.turnToRelativeHeading(goalAngle, 100);
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

bool needsToTurn = false;

void goBackwardsToShoot() {
  Serial.print("goal position: ");
  Serial.println(tPos);

  float error = motor.getRelativeAngle(180.0);
  
  if (abs(error) > 90 and turningToShoot == false) {
    needsToTurn = true;
  }
  
  if (error > 15.0 and needsToTurn and turningToShoot == false) {
    Serial.println(motor.getRelativeAngle(0.0));
    Serial.println(motor.getRelativeAngle(180.0));
    motor.driveToRelativeHeadingCorrected(90.0, 10.0, min(error*error/2, 150));
    return;
  } else if (error < -15.0 and needsToTurn and turningToShoot == false) {
    Serial.println(motor.getRelativeAngle(0.0));
    Serial.println(motor.getRelativeAngle(180.0));
    motor.driveToRelativeHeadingCorrected(-90.0, -10.0, min(error*error/2, 150));
    return;
  }

  if (abs(error) < 15 and turningToShoot == false) {
    needsToTurn = false;
  }
  
  if (tPos != 0 or turningToShoot) {
    if (tPos < -MAXIMUM_SHOT_DISTANCE and turningToShoot == false) {
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
  motor.dribble(255);

  if (isExecutingDodgeOnGoalie) {
    Serial.println("made it");

    if (goalAngle > 0) {
      motor.driveToRelativeHeadingCorrected(-90.0, -10.0, min(goalAngle*goalAngle/2, 150));
    } else if (goalAngle < 0) {
      motor.driveToRelativeHeadingCorrected(90.0, 10.0, min(goalAngle*goalAngle/2, 150));
    } else {
      motor.stopMotors();
    }

    return;
  }

  float error = motor.getRelativeAngle(180.0);

  if (abs(error) > 90) {
    needsToTurn = true;
  }
  
  if (error > 15.0 and needsToTurn) {
    Serial.println(motor.getRelativeAngle(0.0));
    Serial.println(motor.getRelativeAngle(180.0));
    motor.driveToRelativeHeadingCorrected(70.0, 10.0, min(error*error/2, 150));
    return;
  } else if (error < -15.0 and needsToTurn) {
    Serial.println(motor.getRelativeAngle(0.0));
    Serial.println(motor.getRelativeAngle(180.0));
    motor.driveToRelativeHeadingCorrected(-70.0, -10.0, min(error*error/2, 150));
    return;
  }

  if (abs(error) < 15) {
    needsToTurn = false;
  }
  
  frontSensor = lidars.readSensor1();  
  backSensor = lidars.readSensor3();  
  leftSensor = lidars.readSensor2();
  rightSensor = lidars.readSensor4();
  
  lateralReadingsValid = abs((leftSensor + rightSensor) - FIELD_WIDTH) < 8;
  verticalReadingsValid = abs((frontSensor + backSensor) - FIELD_LENGTH) < 10;
  
  Serial.println("");
  Serial.print("readings valid: ");
  Serial.println(lateralReadingsValid and verticalReadingsValid);
  Serial.print("left error: ");
  Serial.println(leftSensor-CORNER_WALL_DISTANCE);
  Serial.print("right error: ");
  Serial.println(rightSensor-CORNER_WALL_DISTANCE);
  Serial.print("vertical error: ");
  Serial.println(verticalDistance);
  Serial.print("drive angle: ");
  Serial.println(persistentGoalAngleForShow);

  if (verticalReadingsValid) {
    verticalDistance = backSensor - CORNER_WALL_DISTANCE;
  } else {
    if (abs(backSensor - oldBackSensor) < 5) {
       verticalDistance = backSensor - CORNER_WALL_DISTANCE;
       frontSensor = oldBackSensor - backSensor + oldFrontSensor;
    } else if (abs(frontSensor - oldFrontSensor) < 5) {
       backSensor = oldFrontSensor - frontSensor + oldBackSensor;
       verticalDistance = backSensor - CORNER_WALL_DISTANCE;
    } else if (tPos != 0.0 and tPos > 200) {
      if (verticalDistance == 20.0) {
        verticalDistance = 100.0;
      }
    } else if (tPos != 0.0 and tPos > -200 and tPos < 0) {
      verticalDistance = -0.5;
    }
  }

  if (lateralReadingsValid == false) {
    if (abs(leftSensor - oldLeftSensor) < 5) {
       rightSensor = oldLeftSensor - leftSensor + oldRightSensor;
    } else if (abs(rightSensor - oldRightSensor) < 5) {
       leftSensor = oldRightSensor - rightSensor + oldLeftSensor;
    }
  }
  
  if (verticalDistance < 0 and (rightSensor < CORNER_WALL_DISTANCE + 1 or leftSensor < CORNER_WALL_DISTANCE + 1)) {
      motor.stopMotors();
      delay(200);
      isExecutingDodgeOnGoalie = true;
  }
    
  if (lateralReadingsValid) {
    if (leftSensor < rightSensor) {
      persistentGoalAngleForShow = atan(-verticalDistance/-(leftSensor-CORNER_WALL_DISTANCE));
      persistentGoalAngleForShow *= 57.2957795129;
      if (-(leftSensor-CORNER_WALL_DISTANCE) < 0) {
        persistentGoalAngleForShow = -90.0 - persistentGoalAngleForShow;
      } else {
        persistentGoalAngleForShow = 90 - persistentGoalAngleForShow;
      }
    } else {
      persistentGoalAngleForShow = atan(-verticalDistance/(rightSensor-CORNER_WALL_DISTANCE));
      persistentGoalAngleForShow *= 57.2957795129;  
      if (rightSensor-CORNER_WALL_DISTANCE < 0) {
        persistentGoalAngleForShow = -90.0 - persistentGoalAngleForShow;
      } else {
        persistentGoalAngleForShow = 90 - persistentGoalAngleForShow;
      }
    }
  } 
  
  if (persistentGoalAngleForShow != 0.0) {
    motor.driveToHeadingCorrected(persistentGoalAngleForShow, 180.0, 100);
  } else {
    motor.stopMotors();
//    motor.driveToHeadingCorrected(0.0, 0.0, 100);
  }

  oldFrontSensor = frontSensor;
  oldBackSensor = backSensor;
  oldLeftSensor = leftSensor;  
  oldRightSensor = rightSensor;
}
