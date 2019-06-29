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
//  float power = min(max(0.1*goalAngle*goalAngle, 50), 230);
  float power = min(max(1.8*abs(goalAngle), 50), 230);

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
    } else {
      motor.stopMotors();
      shoot();
      directionToTurnSet = false;
      turningToShoot = false;
    }
  }
}

bool shoot() {
  Serial6.println("Shooting...");
  if ((lastShootTime == 0 or millis() - lastShootTime > 2000) and state == 0 and abs(motor.getRelativeAngle(0.0)) < 90.0) {
    lastShootTime = millis();
    motor.dribble(0);
    digitalWrite(SOLENOID_PIN, HIGH);
    wroteHigh = millis();
    shouldWriteLow = true;
    return true;
  } else {
    return false;
  }
}

bool shouldFixHeading = false;
float lastEncounteredEnemyFrontUs = 0.0;
bool shouldStrafeDodgeRight;
float angleError;

// Keep It Simple Stupid (forwards)
void KISS() {
  motor.dribble(255);

  useSideLIDAR = false;
  useFrontLIDAR = false;

  angleError = motor.getRelativeAngle(0.0);

  if (not turningToShoot) {
    if (abs(angleError) > 15) {
      shouldFixHeading = true;
    }
    
    // Strong angle correction through ball orbital so we face forwards
    if (shouldFixHeading) {
      if (abs(angleError) > 10) {
        if (angleError > 0) {
          motor.driveToRelativeHeadingCorrected(min(abs(angleError)*3.0, 80), 40.0, 160);
        } else {
          motor.driveToRelativeHeadingCorrected(-1.0*min(abs(angleError)*3.0, 80), -40.0, 160);
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
      if (frontDistIR >= 200 or millis() - lastEncounteredEnemyFrontUs < 600) {
//          Serial6.println("Dodging forwards");

          // Don't want to constantly be changing direction as we cross the mid line:
          // only the first time we see them/when we get too close to the edge
          if (millis() - lastEncounteredEnemyFrontUs > 600 or leftDistance < 40 or rightDistance < 40) {
            shouldStrafeDodgeRight = leftDistance < rightDistance;
          }
          
          if (shouldStrafeDodgeRight) {
            motor.driveToHeadingCorrected(50.0, 0.0, MAX_SPEED);
          } else {
            motor.driveToHeadingCorrected(-50.0, 0.0, MAX_SPEED);
          }
          
          if (frontDistIR >= 200) lastEncounteredEnemyFrontUs = millis();

          return;
        }
    }
// Head away from the open goal
    if (goalAngle != 0.0) {
      if ((rightDistance < 45 and goalAngle < 0.0) or (leftDistance < 45 and goalAngle > 0.0)) {
        motor.driveToHeadingCorrected(0.0, 0.0, MAX_SPEED);
      } else {
        motor.driveToHeadingCorrected(-2.0*goalAngle, 0.0, MAX_SPEED);
      }
    } else {
      motor.driveToHeadingCorrected(0.0, 0.0, MAX_SPEED);
    }
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

float lastReorientedBackwards = 0.0;
float lastEncounteredEnemyBehindUs = 0.0;
float backDistIR;
float startedStrafingAtGoal = 0.0;

void KISSBackwards() {      
  motor.dribble(255);

  angleError = motor.getRelativeAngle(180.0);

  if (backwardsStrategy == 0) {
    doingCornerShot = true;
    
    if (abs(angleError) > 10) {
      shouldFixHeading = true;
    }

    if (millis() - lastLostBall < 3000) {
      lastReorientedBackwards = millis() - 300;
      shouldFixHeading = false;
    }

    if (shouldFixHeading == false) {
      if (millis() - lastReorientedBackwards < 100) {
        motor.stopMotors();
        return;
      } else {
        sideAngle = 0.0;
        if (leftDistance > 70 and rightDistance > 70) {
          backwardsStrategy = 1;
        } else {
          backwardsStrategy = 2;
        }
//        motor.driveToHeadingCorrected(180.0, 180.0, 60);
        if (digitalRead(S_THREE_PIN) == LOW) backwardsStrategy = 1;
      }
    }

    // Strong angle correction through ball orbital so we face backwards
    if (shouldFixHeading and millis() - lastDidNotHaveBall > 100) {
      if (abs(angleError) > 8) {
        float power = min(max(5.0*abs(angleError), 50), 200);
        if (angleError > 0) {
          motor.driveToRelativeHeadingCorrected(80.0, 40.0, power);
        } else {
          motor.driveToRelativeHeadingCorrected(-80.0, -40.0, power);
        }
      } else {
        shouldFixHeading = false;
        lastReorientedBackwards = millis();
      }
    }

    return;
  }

  if (backwardsStrategy == 1) {
    backToGoal();
  } else {
    cornerShoot();
  }
}

void backToGoal() {
  doingCornerShot = false;
  useSideLIDAR = false;
  useFrontLIDAR = false;
  
  if (goalAngle == 0 or (frontDistance > MAXIMUM_SHOT_DISTANCE and turningToShoot == false)) {
    startedStrafingAtGoal = 0.0;
    turningToShoot = false;

    // Handle robot in front of us
    backDistIR = analogRead(BACK_IR_PIN);
    if ((backDistIR >= 200 and abs(angleError) < 30) or millis() - lastEncounteredEnemyBehindUs < 800) {
//        Serial6.println("Dodging backwards...");
      
      // Don't want to constantly be changing direction as we cross the mid line:
      // only the first time we see them/when we get too close to the edge
      if (millis() - lastEncounteredEnemyBehindUs > 800 or leftDistance < 45 or rightDistance < 45) {
        shouldStrafeDodgeRight = leftDistance < rightDistance;
      }

      if (shouldStrafeDodgeRight) {
        motor.driveToHeadingCorrected(-75.0, 180.0, 90);
      } else {
        motor.driveToHeadingCorrected(75.0, 180.0, 90);
      }
      
      if (backDistIR >= 200) lastEncounteredEnemyBehindUs = millis();
      lastDidNotHaveBall = millis();
      return;
    }

    // Drive towards the goal
    if (goalAngle != 0.0) {
//        motor.driveToHeadingCorrected(180.0, 180.0, BACK_SPEED);
      if (goalAngle < 0) {
        motor.driveToHeadingCorrected(-180.0 + 2.0*(180.0 - abs(goalAngle)), 180.0, BACK_SPEED);
      } else {
        motor.driveToHeadingCorrected(180.0 - 2.0*(180.0 - abs(goalAngle)), 180.0, BACK_SPEED);
      }
    } else {
      motor.driveToHeadingCorrected(180.0, 180.0, BACK_SPEED);
    }
  } else {
    // Strafe away from goalie if necessary before shooting
    backDistIR = analogRead(BACK_IR_PIN);
    if ((not turningToShoot) and backDistIR >= 200 and (millis() - startedStrafingAtGoal < 1000 or startedStrafingAtGoal == 0.0)) {
      if (startedStrafingAtGoal == 0.0) startedStrafingAtGoal = millis();
      if (leftDistance < rightDistance) {
        motor.driveToHeadingCorrected(80.0, 180.0, 70);
      } else {
        motor.driveToHeadingCorrected(-80.0, 180.0, 70);
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
}

float targetAngle = 0.0;
float targetDriveDirection = 0.0;
float dodgeTargetAngle = 0.0;
//bool wasFacingOut = false;
bool shouldFixSideHeading = false;
float decidedToDodge = 0;

void cornerShoot() {
  float currentHeading = motor.getRelativeAngle(0.0);

//  Serial6.println("corner shooting");
  
  doingCornerShot = true;

  bool prettyPerpendicular = abs(currentHeading/90.0 - round(currentHeading/90.0)) < 0.12;
//  if ((not isShooting and prettyPerpendicular and millis() - lastDidNotHaveBall < 2500) or sideAngle == 0.0) {
  if (sideAngle == 0.0) {
    if (rightDistance > leftDistance) {
      sideAngle = 270.0;
    } else {
      sideAngle = 90.0;
    }
  }

  if (sideAngle == 270.0) {
    sideDistance = leftDistance;
  } else {
    sideDistance = rightDistance;
  }

  if (millis() - decidedToDodge > 10000) shouldDodge = false;
  
  // Goes sideways to the goal
  if ((frontDistance < 55 or (targetAngle == sideAngle + 0.1 and frontDistance < 70)) and not shouldDodge) {
//    useFrontLIDAR = true;
    
    if (targetAngle != sideAngle + 0.1) {
      motor.stopMotors();
      delay(200);
    }

    if (analogRead(BACK_IR_PIN) >= 350 and isShooting == false) {
      // MISSION ABORT: DODGE!!!
      if (not shouldDodge) {
        dodgeTargetAngle = sideAngle - 180.0;
        decidedToDodge = millis();
      }
      shouldDodge = true;
    }
    targetAngle = sideAngle + 0.1;
    targetDriveDirection = 360.0 - sideAngle - currentHeading;
  }

  if (((frontDistance > 55 and targetAngle != (sideAngle + 0.1)) or frontDistance > 70) and not shouldDodge) {
    targetAngle = sideAngle;
    
    if (abs(currentHeading - (180.0 - targetAngle)) > 25) {
      shouldFixSideHeading = true;
    } else if (abs(currentHeading - (180.0 - targetAngle)) < 8) {
      shouldFixSideHeading = false;
      useSideLIDAR = true;
      useFrontLIDAR = true;
    }
    
    if (frontDistance < 60 and isShooting == false) {
      if (analogRead(BACK_IR_PIN) >= 300) {
        // MISSION ABORT: DODGE!!!
        if (not shouldDodge) {
          dodgeTargetAngle = sideAngle - 180.0;
          decidedToDodge = millis();
        }
        shouldDodge = true;
      }
    }
    
    if (shouldFixSideHeading and targetAngle == 90.0) {
      targetDriveDirection = 80.0;
      targetAngle = 40.0;
    } else if (shouldFixSideHeading and targetAngle == 270.0) {
      targetDriveDirection = -80.0;
      targetAngle = -40.0;
    } else {
      float wallFollowCorrection = 2.0*(sideDistance - 30.0);
      wallFollowCorrection = min(wallFollowCorrection, 30.0);
      if (sideDistance == 0.0) wallFollowCorrection = 0.0;
      if (sideAngle == 270.0) wallFollowCorrection *= -1.0;
      targetAngle = sideAngle;
      targetDriveDirection = -currentHeading + wallFollowCorrection;
    }
  }
  Serial6.println(sideDistance);
  if (((sideDistance > 68 and frontDistance < 60) or isShooting) and frontDistance != 0.0 and sideDistance != 0.0) {
    isShooting = true;

    float targetHeading = goalAngle;
    if (goalAngle == 0.0) {
      if (sideAngle == 90.0) {
        targetHeading = currentHeading + 30;
      } else {
        targetHeading = currentHeading - 30;
      }
    }
    if (abs(targetHeading) > 7) {
//      Serial6.println("Taking shot on corner...");
      float power = min(max(0.1*targetHeading*targetHeading, 50), 230);
      if (sideAngle == 90.0) {
        motor.driveToRelativeHeadingCorrected(80.0, 40.0, power);
      } else {
        motor.driveToRelativeHeadingCorrected(-80.0, -40.0, power);
      }
    } else {
      motor.stopMotors();
      shouldDodge = false;
      isShooting = false;
      shoot();
    }
  } else if (shouldDodge) {
    useSideLIDAR = false;
    useFrontLIDAR = false;
    
    if (abs(currentHeading - dodgeTargetAngle) > 3) {
      if (millis() - decidedToDodge < 200.0) {
        motor.stopMotors();
        return;
      }
      
      if (dodgeTargetAngle == -90.0) {
        motor.driveToRelativeHeadingCorrected(80.0, -11.0, (min(millis() - decidedToDodge - 200.0, 1000.0)/1000.0)*55 + 15);
      } else {
        motor.driveToRelativeHeadingCorrected(-80.0, 11.0, (min(millis() - decidedToDodge - 200.0, 1000.0)/1000.0)*55 + 15);
      }
    } else {
      shouldDodge = false;
    }
  } else if (targetAngle != 0 and targetDriveDirection != 0) {
    if (abs(targetAngle) - 40 == 0.0 and abs(targetDriveDirection) - 80.0 == 0.0) {
      motor.driveToRelativeHeadingCorrected(targetDriveDirection, targetAngle, 100);
    } else {
      motor.driveToHeadingCorrected(targetDriveDirection, targetAngle, 80);
    }
  } else {
    motor.stopMotors();
  }
}
