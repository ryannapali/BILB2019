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
      if (leftDistance < rightDistance) {
        if (leftDistance - 30.0*(oPos/75.0) > 89) {
          directionToTurn = -1.0;
        } else {
          directionToTurn = 1.0;
        }
      } else {
        if (rightDistance + 30.0*(oPos/75.0) > 96) {
          directionToTurn = 1.0;
        } else {
          directionToTurn = -1.0;
        }
      }
//      if (leftDistance < 55) {
//        directionToTurn = 1.0;
//      } else if (rightDistance < 55) {
//        directionToTurn = -1.0;
//      } else if (oPos > 0) {
//        directionToTurn = 1.0;
//        analogWrite(WHITEA_PIN,255);
//      } else {
//        directionToTurn = -1.0;
//      }
      directionToTurnSet = true;
    }
  } else {
    directionToTurn = 0.0;
  }

  // Aim at goal
  //and (millis() - lastShootTime > 200)
  float power = min(max(0.1*goalAngle*goalAngle, 45), 210);

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

  if (shouldBackUp) {
    motor.driveToRelativeHeadingCorrected(-180, 0, BACK_SPEED);
    if (frontDistance > 65) {
      shouldBackUp = false;
    }
  }

  float goalDistance = sqrt(tPos*tPos + oPos*oPos);

  if (goalAngle == 0 or (frontDistance > MAXIMUM_SHOT_DISTANCE and turningToShoot == false)) {
    turningToShoot = false;
    
    float angleError = motor.getRelativeAngle(0.0);
    
    if (abs(angleError) > 35) {
      shouldFixHeading = true;
    }

    // Handle robot in front of us as we're moving forward
//    if (abs(angleError) < 45) {
//      frontDistIR = analogRead(FRONT_IR_PIN);
//      if (frontDistIR >= 200) {
//        shouldKissForwards = false;
//        return;
//      }
//    }

    // Decide which direction to about face (if we're dodging)
    if (abs(angleError) > 135) {
      if (directionToTurnKISSSet == false) {
        if (leftDistance < rightSensor) {
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
        motor.driveToRelativeHeadingCorrected(directionToTurnKISS*min(abs(angleError)*2.0, 120), directionToTurnKISS*40.0, 200);
      } else {
        if (angleError > 0) {
          motor.driveToRelativeHeadingCorrected(min(abs(angleError)*2.0, 120), 40.0, 200);
        } else {
          motor.driveToRelativeHeadingCorrected(-1.0*min(abs(angleError)*2.0, 120), -40.0, 200);
        }
      }
      return;
    } else if (shouldFixHeading and abs(angleError) <= 10) {
      directionToTurnKISSSet = false;
      directionToTurnKISS = 0.0;
      shouldFixHeading = false;
    }
    
    motor.driveToHeadingCorrected(goalAngle, 0.0, MAX_SPEED);
  } else {
    // and abs(motor.getRelativeAngle(0.0)) < 45
    if (frontDistance <= MAXIMUM_SHOT_DISTANCE and turningToShoot == false) {
      turningToShoot = true;
    } if (turningToShoot) {
      turnShoot();
    }
  }
}

bool shouldNotShoot = false;
bool shouldNotFixHeading = false;

void KISSBackwards() {
  motor.dribble(255);

  cornerShoot();
  return;

  float goalDistance = sqrt(tPos*tPos + oPos*oPos);

  if (shouldBackUp) {
    motor.driveToRelativeHeadingCorrected(-180, 0, BACK_SPEED);
    if (frontDistance > 65) {
      shouldBackUp = false;
    }
  }
  
  //goalAngle == 0 or 
//  if (goalAngle == 0) {
//    analogWrite(WHITEA_PIN,255);
//  }

  if (goalAngle == 0 or (frontDistance > MAXIMUM_SHOT_DISTANCE and turningToShoot == false)) {
    turningToShoot = false;

    float angleError = motor.getRelativeAngle(180.0);
    

    // Handle robot chasing our ball/dribbler as we're moving backwards
//    frontDistIR = analogRead(FRONT_IR_PIN);
//    if (frontDistIR >= 200 and abs(angleError) < 45) {
//      shouldKissForwards = true;
//      return;
//    }

    if (abs(angleError) > 35) {
      shouldFixHeading = true;
    }

    if (not shouldNotFixHeading) {
      shouldFixHeading = false;
    }

    // Strong angle correction through ball orbital so we face forwards
    // old power: min(abs(angleError)*PIVOT_K, 180) and 90 for first arg
    if (shouldFixHeading and abs(angleError) > 10) {
      float power = min(max(0.1*angleError*angleError, 30), 210);
      if (angleError > 0) {
        motor.driveToRelativeHeadingCorrected(80.0, 40.0, power);
      } else {
        motor.driveToRelativeHeadingCorrected(-80.0, -40.0, power);
      }
      return;
    } else if (shouldFixHeading and abs(angleError) <= 10) {
      shouldFixHeading = false;
    }

    shouldNotShoot = true;
    if (frontDistance > 90) {
      shouldNotFixHeading = false;
      motor.driveToHeadingCorrected(180.0, 180.0, BACK_SPEED);
    } else if (frontDistance > 50) {
      shouldNotFixHeading = true;
      motor.driveToHeadingCorrected(-motor.getRelativeAngle(0.0), 90.0, 90);
    } 
    else if (rightDistance < 70) {
      motor.driveToHeadingCorrected(270, 90.0, 90);
    }
    else if (analogRead(FRONT_IR_PIN) >= 200) {
      // MISSION ABORT

    }
    else {
      shouldNotFixHeading = true;
      motor.stopMotors();
    }
    
//    if (goalAngle != 0.0) {
//      motor.driveToHeadingCorrected(goalAngle, 180.0, BACK_SPEED);
//    } else {
//      motor.driveToHeadingCorrected(180.0, 180.0, BACK_SPEED);
//    }
  } else {
    // and (abs(motor.getRelativeAngle(180.0)) < 30 or (millis() - lastChangedStrategy) > 2000)
    //and shouldNotShoot == false
    if (frontDistance <= MAXIMUM_SHOT_DISTANCE and turningToShoot == false and shouldNotShoot == false) {
      turningToShoot = true;
    }
    if (turningToShoot) {
      turnShoot();
    }
  }
}


bool isShooting = false;
float targetAngle = 0.0;
float targetDriveDirection = 0.0;
bool shouldDodge = false;
float dodgeTargetAngle = 0.0;
bool wasFacingOut = false;


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
  if (frontDistance < 57 or (targetAngle == sideAngle + 0.1 and frontDistance < 62)) {
    if (targetAngle != sideAngle + 0.1) {
      motor.stopMotors();
      delay(300);
    }

    if (analogRead(BACK_IR_PIN) >= 400) {
      // MISSION ABORT: DODGE!!!
      if (not shouldDodge) dodgeTargetAngle = sideAngle - 180.0;
      shouldDodge = true;
    }
    targetAngle = sideAngle + 0.1;
    targetDriveDirection = 360.0 - sideAngle - currentHeading;
  }

  // Moves facing backwards to the goal
  if ((frontDistance > 90 and not wasFacingOut) or frontDistance > 120 or frontDistance == 0.0) {
    targetAngle = 180.0;
    
    float wallFollowCorrection = 2.0*(sideDistance - 40.0);
    if (sideDistance == 0.0) wallFollowCorrection = 0.0;
    if (sideDistance == leftDistance) wallFollowCorrection*=-1.0;
    
    targetDriveDirection = -currentHeading + wallFollowCorrection;
  } 

  // Move facing outwards to the goal
  if (((frontDistance > 57 and targetAngle != (sideAngle + 0.1)) or frontDistance > 60) and (frontDistance < 90 or (frontDistance < 120 and wasFacingOut))) {
    targetAngle = sideAngle;
    
    float headingOffsetForPossession = min(2.0*abs(currentHeading - targetAngle)/90.0, 1.0)*180.0;
    if (sideAngle == 270.0) {
      headingOffsetForPossession = min(2.0*abs(-90.0 - currentHeading)/90.0, 1.0)*180.0;
    }
    if (headingOffsetForPossession < 30.0) {
      headingOffsetForPossession = 0.0;
    } else {
      headingOffsetForPossession = 180.0;
    }
    
    float wallFollowCorrection = 2.0*(sideDistance - 40.0);
    if (sideDistance == 0.0) wallFollowCorrection = 0.0;
    if (sideDistance == leftDistance) wallFollowCorrection*=-1.0;
    
    targetDriveDirection = -currentHeading + wallFollowCorrection + headingOffsetForPossession;
  }

  if (targetAngle == sideAngle) {
    wasFacingOut = true;
  } else {
    wasFacingOut = false;
  }

  if (((sideDistance > 70 and frontDistance < 62) or isShooting) and frontDistance != 0.0 and sideDistance != 0.0) {
    isShooting = true;
    if (abs(currentHeading) > 8) {
      motor.turnToAbsoluteHeading(0.0, 100);
//      float power = min(max(0.1*currentHeading*currentHeading, 45), 210);
//      if (currentHeading < 0) {
//        motor.driveToRelativeHeadingCorrected(-80.0, -40.0, power);
//      } else {
//        motor.driveToRelativeHeadingCorrected(80.0, 40.0, power);
//      }
    } else {
      motor.stopMotors();
      shouldDodge = false;
      isShooting = false;
      shoot();
    }
  } else if (shouldDodge) {
    if (abs(currentHeading - dodgeTargetAngle) > 5) {
      if (dodgeTargetAngle == -90.0) {
        motor.driveToRelativeHeadingCorrected(75.0, -15.0, 85);
      } else {
        motor.driveToRelativeHeadingCorrected(-75.0, 15.0, 85);
      }
    } else {
      shouldDodge = false;
    }
  } else if (targetAngle != 0 and targetDriveDirection != 0) {
    motor.driveToHeadingCorrected(targetDriveDirection, targetAngle, 100);
  } else {
    motor.stopMotors();
  }
}
