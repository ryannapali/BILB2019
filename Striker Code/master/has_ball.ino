float lastShootTime = 0.0;

float directionToTurn = 1.0;
bool directionToTurnSet = false;

void turnShoot() {
  lastCalledTurnToShoot = millis();
  
  if (goalAngle == 0.0) {
    motor.stopMotors();
    return;
  }
  
  float angle = motor.getRelativeAngle(0.0);
  if (abs(angle) > 135) {
    if (directionToTurnSet == false) {
      leftSensor = lidars.readSensor2();
      rightSensor = lidars.readSensor4();
      if (leftSensor < rightSensor) {
        directionToTurn = 1.0;
      } else {
        directionToTurn = -1.0;
      }
      directionToTurnSet = true;
    }
  } else {
    directionToTurn = 0.0;
  }
  
  if (abs(goalAngle) > 7 and (millis() - lastShootTime > 200)) {
    if (goalAngle < 0) {
      if (directionToTurn != 0.0) {
        motor.driveToRelativeHeadingCorrected(90.0*directionToTurn, 40.0*directionToTurn, min(abs(goalAngle)*PIVOT_K, 180));
      } else {
        motor.driveToRelativeHeadingCorrected(90.0, 40.0, min(abs(goalAngle)*PIVOT_K, 180));
      }
    } else {
      if (directionToTurn != 0.0) {
        motor.driveToRelativeHeadingCorrected(90.0*directionToTurn, 40.0*directionToTurn, min(abs(goalAngle)*PIVOT_K, 180));
      } else {
        motor.driveToRelativeHeadingCorrected(-90.0, -40.0, min(abs(goalAngle)*PIVOT_K, 180));
      }
   }
//    motor.turnToRelativeHeading(goalAngle, 100);
    motor.dribble(255);
  } else {
    motor.stopMotors();
    shoot();
    directionToTurn = 1.0;
    directionToTurnSet = false;
    turningToShoot = false;
  }
}

bool shoot() {
  Serial.println(state);
  if ((lastShootTime == 0 or millis() - lastShootTime > 2000) and state == 0) {
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
    motor.driveToRelativeHeadingCorrected(90.0, 10.0, min(error*error/2, 150));
    return;
  } else if (error < -15.0 and needsToTurn and turningToShoot == false) {
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
  frontDistIR = analogRead(FRONT_IR_PIN);
  if (frontDistIR >= 200 and isExecutingForwardSpinMove == false) {
    isExecutingForwardSpinMove = true;
  }
  if (isExecutingForwardSpinMove) executeForwardSpinMove();
  return isExecutingForwardSpinMove;
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

bool shouldFixHeading = false;


bool directionToTurnKISSSet = false;
float directionToTurnKISS = 1.0;

// Keep It Simple Stupid
void KISS() {
  motor.dribble(255);

  float goalDistance = sqrt(tPos*tPos + oPos*oPos);
  if (goalAngle == 0 or goalDistance > 220) {
    float angleError = motor.getRelativeAngle(0.0);
    if (abs(angleError) > 30) {
      shouldFixHeading = true;
    }

    Serial.println(angleError);
    Serial.println(directionToTurnKISS);
    Serial.println(directionToTurnKISSSet);
    Serial.println(shouldFixHeading);
    Serial.println("");
    
    if (abs(angleError) > 135) {
      if (directionToTurnKISSSet == false) {
        leftSensor = lidars.readSensor2();
        rightSensor = lidars.readSensor4();
        if (leftSensor < rightSensor) {
          directionToTurnKISS = 1.0;
        } else {
          directionToTurnKISS = -1.0;
        }
        directionToTurnKISSSet = true;
      }
    } else {
      directionToTurnKISS = 0.0;
    }
    
    if (shouldFixHeading and abs(angleError) > 10) {
      if (directionToTurnKISS != 0.0) {
        motor.driveToRelativeHeadingCorrected(120.0*directionToTurnKISS, 40.0*directionToTurnKISS, 180);
        return;
      }
      if (angleError > 0) {
        motor.driveToRelativeHeadingCorrected(120.0, 40.0, 180);
      } else {
        motor.driveToRelativeHeadingCorrected(-120.0, -40.0, 180);
      }
      return;
    } else if (shouldFixHeading and abs(angleError) <= 10) {
      directionToTurnKISSSet = false;
      directionToTurnKISS = 0.0;
      shouldFixHeading = false;
    }
    motor.driveToHeadingCorrected(0.0, 0.0, MAX_SPEED);
  } else {
    turnShoot();
  }
}

void KISSBackwards() {
  motor.dribble(255);

  if (goalAngle == 0) {
    frontDistIR = analogRead(FRONT_IR_PIN);
    if (frontDistIR >= 200) {
      shouldKissForwards = true;
      return;
    }
    
    float angleError = motor.getRelativeAngle(180);
    if (abs(angleError) > 30) {
      shouldFixHeading = true;
    }
    if (shouldFixHeading and abs(angleError) > 10) {
      if (angleError > 0) {
        motor.driveToRelativeHeadingCorrected(90.0, 40.0, min(abs(angleError)*PIVOT_K, 180));
      } else {
        motor.driveToRelativeHeadingCorrected(-90.0, -40.0, min(abs(angleError)*PIVOT_K, 180));
      }
      return;
    } else if (shouldFixHeading and abs(angleError) <= 10) {
      shouldFixHeading = false;
    }
    motor.driveToHeadingCorrected(motor.getRelativeAngle(0.0), 180.0, MAX_BACK_SPEED);
  } else {
    turnShoot();
  }
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
