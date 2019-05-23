float directionToTurn = 1.0;
bool directionToTurnSet = false;

void turnShoot() {
  lastCalledTurnToShoot = millis();
  
  if (goalAngle == 0.0) {
    motor.stopMotors();
    return;
  }

  // Decide which direction to turn and shoot
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

  // Aim at goal
  if (abs(goalAngle) > 7 and (millis() - lastShootTime > 200)) {
    if (goalAngle < 0) {
      if (directionToTurn != 0.0) {
        motor.driveToRelativeHeadingCorrected(80.0*directionToTurn, 40.0*directionToTurn, min(abs(goalAngle)*PIVOT_K, 200));
      } else {
        motor.driveToRelativeHeadingCorrected(80.0, 40.0, min(abs(goalAngle)*PIVOT_K, 200));
      }
    } else {
      if (directionToTurn != 0.0) {
        motor.driveToRelativeHeadingCorrected(80.0*directionToTurn, 40.0*directionToTurn, min(abs(goalAngle)*PIVOT_K, 200));
      } else {
        motor.driveToRelativeHeadingCorrected(-80.0, -40.0, min(abs(goalAngle)*PIVOT_K, 200));
      }
   }
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

bool shouldFixHeading = false;

float directionToTurnKISS = 1.0;
bool directionToTurnKISSSet = false;

// Keep It Simple Stupid (forwards)
void KISS() {
  motor.dribble(255);

  float goalDistance = sqrt(tPos*tPos + oPos*oPos);
    
  if (goalAngle == 0 or (goalDistance > MAXIMUM_SHOT_DISTANCE and turningToShoot == false)) {
    turningToShoot = false;
    float angleError = motor.getRelativeAngle(0.0);
    
    if (abs(angleError) > 30) {
      shouldFixHeading = true;
    }

    // Handle robot in front of us as we're moving forward
    if (abs(angleError) < 45) {
      frontDistIR = analogRead(FRONT_IR_PIN);
      if (frontDistIR >= 200) {
        shouldKissForwards = false;
        lastChangedStrategy = millis();
        return;
      }
    }

    // Decide which direction to about face
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

    // Strong angle correction through ball orbital so we face forwards
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
    
    motor.driveToHeadingCorrected(goalAngle, 0.0, MAX_SPEED);
  } else {
    if (goalDistance <= MAXIMUM_SHOT_DISTANCE and turningToShoot == false and abs(motor.getRelativeAngle(0.0)) < 45) {
      turningToShoot = true;
    } if (turningToShoot) {
      turnShoot();
    }
  }
}

void KISSBackwards() {
  motor.dribble(255);

  float goalDistance = sqrt(tPos*tPos + oPos*oPos);

  if (goalAngle == 0 or (goalDistance > MAXIMUM_SHOT_DISTANCE and turningToShoot == false)) {
    turningToShoot = false;

    float angleError = motor.getRelativeAngle(180);

    // Handle robot chasing our ball/dribbler as we're moving backwards
    frontDistIR = analogRead(FRONT_IR_PIN);
    if (frontDistIR >= 200 and abs(angleError) < 45) {
      shouldKissForwards = true;
      return;
    }
    
    if (abs(angleError) > 35) {
      shouldFixHeading = true;
    }

    // Strong angle correction through ball orbital so we face forwards
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

    motor.driveToHeadingCorrected(180.0, 180.0, MAX_BACK_SPEED);
  } else {
    if (goalDistance <= MAXIMUM_SHOT_DISTANCE and turningToShoot == false and (abs(motor.getRelativeAngle(180.0)) < 30 or (millis() - lastChangedStrategy) > 2000)) {
      turningToShoot = true;
    }
    if (turningToShoot) {
      turnShoot();
    }
  }
}
