float directionToTurn = 1.0;
bool directionToTurnSet = false;

// Last front distance before we began to aim to shoot
float lastFrontDistance = 0.0;

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
      if (leftDistance > rightDistance) {
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
  float power = min(max(0.1*goalAngle*goalAngle, 45), 220);

  if (shouldKissForwards) {
    if (abs(goalAngle) > 7) {
      motor.driveToRelativeHeadingCorrectedProportionalThrust(0.0, -1.4*goalAngle, max((lastFrontDistance - MAXIMUM_SHOT_DISTANCE)/40.0, 0), MAX_SPEED);
      motor.dribble(255);
    } else {
      motor.stopMotors();
      shoot();
      directionToTurnSet = false;
      turningToShoot = false;
    }
  } else {
    if (abs(goalAngle) > 7) {
      if (goalAngle < 0) {
        if (directionToTurn != 0.0) {
          motor.driveToRelativeHeadingCorrected(80.0*directionToTurn, 40.0*directionToTurn, power);
        } else {
          motor.driveToRelativeHeadingCorrected(80.0, 40.0, power);
        }
      } else {
        if (directionToTurn != 0.0) {
          motor.driveToRelativeHeadingCorrected(80.0*directionToTurn, 40.0*directionToTurn, power);
        } else {
          motor.driveToRelativeHeadingCorrected(-80.0, -40.0, power);
        }
     }
      motor.dribble(255);
    } else {
      motor.stopMotors();
      shoot();
      directionToTurnSet = false;
      turningToShoot = false;
    }
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
float lastEncounteredEnemyFrontUs = 0.0;
bool shouldStrafeDodgeRight;

// Keep It Simple Stupid (forwards)
void KISS() {
  motor.dribble(255);

  float goalDistance = sqrt(tPos*tPos + oPos*oPos);

  float angleError = motor.getRelativeAngle(0.0);

  if (not turningToShoot) {
    if (abs(angleError) > 20) {
      shouldFixHeading = true;
    }
    
    // Strong angle correction through ball orbital so we face forwards
    if (shouldFixHeading) {
      if (abs(angleError) > 10) {
        if (angleError > 0) {
          motor.driveToRelativeHeadingCorrected(min(abs(angleError)*2.0, 120), 40.0, 160);
        } else {
          motor.driveToRelativeHeadingCorrected(-1.0*min(abs(angleError)*2.0, 120), -40.0, 160);
        }
        return;
      } else {
        shouldFixHeading = false;
      }
    }

    if (frontDistance < MAXIMUM_SHOT_DISTANCE) {
      motor.driveToHeadingCorrected(180.0, 0.0, 70);
      return;
    }
  }

  
  if (goalAngle == 0 or (frontDistance > MAXIMUM_SHOT_DISTANCE + 40.0 and turningToShoot == false)) {
    turningToShoot = false;

    // Dodge laterally if robot is in front
    if (abs(angleError) < 35) {
      frontDistIR = analogRead(FRONT_IR_PIN);
      if (frontDistIR >= 250 or millis() - lastEncounteredEnemyFrontUs < 300) {
          Serial.println("Dodging forwards");

          // Don't want to constantly be changing direction as we cross the mid line:
          // only the first time we see them/when we get too close to the edge
          if (millis() - lastEncounteredEnemyFrontUs > 300 or leftDistance < 40 or rightDistance < 40) {
            shouldStrafeDodgeRight = leftDistance < rightDistance;
          }
          
          if (shouldStrafeDodgeRight) {
            motor.driveToHeadingCorrected(50.0, 0.0, MAX_SPEED);
          } else {
            motor.driveToHeadingCorrected(-50.0, 0.0, MAX_SPEED);
          }
          
          if (frontDistIR >= 250) lastEncounteredEnemyFrontUs = millis();

          return;
        }
    }
// Head away from the open goal
//    if (goalAngle != 0.0) {
//      if ((rightDistance < 45 and goalAngle < 0.0) or (leftDistance < 45 and goalAngle > 0.0)) {
//        motor.driveToHeadingCorrected(0.0, 0.0, MAX_SPEED);
//      } else {
//        motor.driveToHeadingCorrected(-goalAngle, 0.0, MAX_SPEED);
//      }
//    } else {
      motor.driveToHeadingCorrected(0.0, 0.0, MAX_SPEED);
//    }
  } else {
    if (frontDistance <= MAXIMUM_SHOT_DISTANCE + 40.0 and turningToShoot == false) {
      lastFrontDistance = frontDistance;
      turningToShoot = true;
    } 
    if (turningToShoot) {
      turnShoot();
    }
  }
}

bool shouldNotFixHeading = false;
float lastChangedBackStrategy = 0.0;
float lastEncounteredEnemyBehindUs = 0.0;
float backDistIR;
float startedStrafingAtGoal = 0.0;

void KISSBackwards() {
  ledWhite();
  
  motor.dribble(255);

  float angleError = motor.getRelativeAngle(180.0);

  if (backwardsStrategy == 0) {
    if (abs(angleError) > 10) {
      shouldFixHeading = true;
    }

    if (millis() - lastLostBall < 3000) {
      lastChangedBackStrategy = millis() - 200;
      shouldFixHeading = false;
    }

    if (shouldFixHeading == false) {
      if (millis() - lastChangedBackStrategy < 100) {
        motor.stopMotors();
        return;
      } else {
        if (leftDistance > 65 and rightDistance > 65) {
          backwardsStrategy = 1;
        } else {
          backwardsStrategy = 2;
        }
//        if (digitalRead(S_ONE_PIN) == HIGH) backwardsStrategy = 1;
      }
    }

    // Strong angle correction through ball orbital so we face backwards
    if (shouldFixHeading) {
      if (abs(angleError) > 10) {
        float power = min(max(0.1*angleError*angleError, 60), 200);
        if (angleError > 0) {
          motor.driveToRelativeHeadingCorrected(80.0, 40.0, power);
        } else {
          motor.driveToRelativeHeadingCorrected(-80.0, -40.0, power);
        }
      } else {
        shouldFixHeading = false;
        lastChangedBackStrategy = millis();
      }
    }

    return;
  }

  if (backwardsStrategy == 1) {
    doingCornerShot = false;
    
    float goalDistance = sqrt(tPos*tPos + oPos*oPos);
  
    if (goalAngle == 0 or (frontDistance > MAXIMUM_SHOT_DISTANCE and turningToShoot == false)) {
      startedStrafingAtGoal = 0.0;
      turningToShoot = false;

      // Handle robot in front of us
      backDistIR = analogRead(BACK_IR_PIN);
      if ((backDistIR >= 250 and abs(angleError) < 30) or millis() - lastEncounteredEnemyBehindUs < 300) {
        Serial.println("Dodging backwards");
        
        // Don't want to constantly be changing direction as we cross the mid line:
        // only the first time we see them/when we get too close to the edge
        if (millis() - lastEncounteredEnemyBehindUs > 300 or leftDistance < 40 or rightDistance < 40) {
          shouldStrafeDodgeRight = leftDistance < rightDistance;
        }

        if (shouldStrafeDodgeRight) {
          motor.driveToHeadingCorrected(-120.0, 180.0, 70);
        } else {
          motor.driveToHeadingCorrected(120.0, 180.0, 70);
        }
        
        if (backDistIR >= 250) lastEncounteredEnemyBehindUs = millis();

        return;
      }

      // Drive towards the goal
      if (goalAngle != 0.0) {
        Serial.print("Goal angle: ");
        Serial.println(goalAngle);
        motor.driveToHeadingCorrected(1.5*goalAngle, 180.0, BACK_SPEED);
      } else {
        motor.driveToHeadingCorrected(180.0, 180.0, BACK_SPEED);
      }
    } else {
      // Strafe away from goalie if necessary before shooting
      backDistIR = analogRead(BACK_IR_PIN);
      if ((not turningToShoot) and backDistIR >= 250 and (millis() - startedStrafingAtGoal < 1000 or startedStrafingAtGoal == 0.0)) {
        if (startedStrafingAtGoal == 0.0) startedStrafingAtGoal = millis();
        if (leftDistance < rightDistance) {
          motor.driveToHeadingCorrected(90.0, 180.0, 70);
        } else {
          motor.driveToHeadingCorrected(-90.0, 180.0, 70);
        }
        return;
      }
      
      if (frontDistance <= MAXIMUM_SHOT_DISTANCE and turningToShoot == false) {
        startedStrafingAtGoal = 0.0;
        turningToShoot = true;
      }
      
      if (turningToShoot) {
        turnShoot();
      }
    }
  } else {
    cornerShoot();
  }
}

bool isShooting = false;
float targetAngle = 0.0;
float targetDriveDirection = 0.0;
float dodgeTargetAngle = 0.0;
bool wasFacingOut = false;
bool shouldFixSideHeading = false;

void cornerShoot() {
  float currentHeading = motor.getRelativeAngle(0.0);
  float sideAngle = 90.0;
  float sideDistance = rightDistance;
  
  doingCornerShot = true;
  
  if (rightDistance > leftDistance) {
    sideAngle = 270.0;
    sideDistance = leftDistance;
  }
  
  // Goes sideways to the goal
  if (frontDistance < 50 or (targetAngle == sideAngle + 0.1 and frontDistance < 65)) {
    if (targetAngle != sideAngle + 0.1) {
      motor.stopMotors();
      delay(400);
    }

    if (analogRead(BACK_IR_PIN) >= 350) {
      // MISSION ABORT: DODGE!!!
      if (not shouldDodge) dodgeTargetAngle = sideAngle - 180.0;
      shouldDodge = true;
    }
    targetAngle = sideAngle + 0.1;
    targetDriveDirection = 360.0 - sideAngle - currentHeading;
  }

  bool prettyPerpendicular = abs(currentHeading/90.0 - round(currentHeading/90.0)) < 0.15;

  // Moves facing backwards to the goal
  if ((frontDistance > 90 and not wasFacingOut) or (frontDistance > 110 and prettyPerpendicular) or frontDistance == 0.0) {
    targetAngle = 180.0;
    
    float wallFollowCorrection = 2.0*(sideDistance - 30.0);
    if (sideDistance == 0.0) wallFollowCorrection = 0.0;
    if (sideAngle == 270.0) wallFollowCorrection *= -1.0;

    targetDriveDirection = -currentHeading + wallFollowCorrection;
    Serial.print("Current heading: ");
    Serial.println(currentHeading);
    Serial.print("Wall follow correction: ");
    Serial.println(wallFollowCorrection);
    Serial.print("Target drive direction: ");
    Serial.println(targetDriveDirection);
  } 

  // Move facing outwards to the goal
  if (((frontDistance > 50 and targetAngle != (sideAngle + 0.1)) or frontDistance > 65) and (frontDistance < 90 or (frontDistance < 110 and prettyPerpendicular and wasFacingOut))) {
    targetAngle = sideAngle;
    
    if (abs(currentHeading - (180.0 - targetAngle)) > 25) {
      shouldFixSideHeading = true;
    } else if (abs(currentHeading - (180.0 - targetAngle)) < 10) {
      shouldFixSideHeading = false;
    }
    
    if (shouldFixSideHeading and targetAngle == 90.0) {
      targetDriveDirection = 70.0;
      targetAngle = 30.0;
    } else if (shouldFixSideHeading and targetAngle == 270.0) {
      targetDriveDirection = -70.0;
      targetAngle = -30.0;
    } else {
      float wallFollowCorrection = 2.0*(sideDistance - 30.0);
      if (sideDistance == 0.0) wallFollowCorrection = 0.0;
      if (sideAngle == 270.0) wallFollowCorrection *= -1.0;
      targetDriveDirection = -currentHeading + wallFollowCorrection;
    }
  }

  if (targetAngle == sideAngle or abs(targetAngle) == 30.0) {
    wasFacingOut = true;
  } else {
    wasFacingOut = false;
  }

  if (((sideDistance > 70 and frontDistance < 65) or isShooting) and frontDistance != 0.0 and sideDistance != 0.0) {
    isShooting = true;
    if (abs(currentHeading) > 8) {
      Serial.println("Taking shot on corner...");
      motor.turnToAbsoluteHeading(0.0, 70);
    } else {
      motor.stopMotors();
      shouldDodge = false;
      isShooting = false;
      shoot();
    }
  } else if (shouldDodge) {
    Serial.println("Dodging the goalie...");
    if (abs(currentHeading - dodgeTargetAngle) > 5) {
      if (dodgeTargetAngle == -90.0) {
        motor.driveToRelativeHeadingCorrected(80.0, -13.0, 70);
      } else {
        motor.driveToRelativeHeadingCorrected(-80.0, 13.0, 70);
      }
    } else {
      shouldDodge = false;
    }
  } else if (targetAngle != 0 and targetDriveDirection != 0) {
    Serial.println("Just coasting (corner shoot)...");
    motor.driveToHeadingCorrected(targetDriveDirection, targetAngle, 70);
  } else {
    motor.stopMotors();
  }
}
