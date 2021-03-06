float getPathSlope() {
  if (xPos < 0) {
    return PATH_CURVINESS*(xPos-TARGET_DIST_BEHIND_BALL)/yPos;
  } else {
    return (1.0/PATH_CURVINESS)*(xPos-TARGET_DIST_BEHIND_BALL)/yPos;
  }
}

// MAKE SURE SLOPE WAS CALCULATED USING EXACTLY YPOS
float angleFromSlope(float slope) {
  float angle = atan(slope);
  angle *= 57.2957795129;
  if (yPos < 0) {
    angle = -90.0 - angle;
  } else {
    angle = 90.0 - angle;
  }

  return angle;
}

void clearCameraBuffer() {
  Serial5.clear();
}

void getCameraReadings() {
  char lc = Serial5.read();
  long bTimer = millis();
  while (word(0, lc) != 254) {
    lc = Serial5.read();
    if (bTimer + 400 < millis()) {
      clearCameraBuffer();
      bTimer = millis();
    }
  }
  while (Serial5.available() < 2) {
    if (interrupted) break;
  }
  char highChar1 = Serial5.read();
  char lowChar1 = Serial5.read();
  while (Serial5.available() < 2) {
    if (interrupted) break;
  }
  char highChar2 = Serial5.read();
  char lowChar2 = Serial5.read();
  while (Serial5.available() < 2) {
    if (interrupted) break;
  }
  char highChar3 = Serial5.read();
  char lowChar3 = Serial5.read();
  while (Serial5.available() < 2) {
    if (interrupted) break;
  }
  char highChar4 = Serial5.read();
  char lowChar4 = Serial5.read();
  
  xPos = word(highChar1, lowChar1);
  yPos = word(highChar2, lowChar2);
  tPos = word(highChar3, lowChar3);
  oPos = word(highChar4, lowChar4);

  if (xPos < VIDEO_WIDTH and xPos > 0) {
    xPos -= VIDEO_WIDTH/2.0;
    xPos *= -1.0;
    xPos += X_ORIGIN_CALIBRATION;
    oldXPos = xPos;
    failedBallReadingCount = 0;
  } else {
    failedBallReadingCount += 1;
    if (failedBallReadingCount < MAX_FAILED_BALL_READS) {
      xPos = oldXPos;
    } else {
      xPos = 0;
    }
  }

//  Serial6.println(xPos);

  if (yPos < VIDEO_HEIGHT and yPos > 0) {
    yPos -= VIDEO_HEIGHT/2.0;
    yPos += Y_ORIGIN_CALIBRATION;
    oldYPos = yPos;
  } else if (failedBallReadingCount < MAX_FAILED_BALL_READS) {
    yPos = oldYPos;
  } else {
    yPos = 0;
  }

  if (tPos < VIDEO_WIDTH and tPos > 0) {
    tPos -= VIDEO_WIDTH/2.0;
    tPos *= -1;
    oldTPos = tPos;
    failedGoalReadingCount = 0;
  } else {
    failedGoalReadingCount += 1;
    if (failedGoalReadingCount < MAX_FAILED_GOAL_READS) {
      tPos = oldTPos;
    } else {
      tPos = 0;
    }
  }  
  
  if (oPos < VIDEO_HEIGHT and oPos > 0) {
    oPos -= VIDEO_HEIGHT/2.0;
    oldOPos = oPos;
  } else if (failedGoalReadingCount < MAX_FAILED_GOAL_READS) {
    oPos = oldOPos;
  } else {
    oPos = 0;
  }
}


void calculateAngles() {
  // Only run this if you are in fact recieving x and y data. Otherwise, ballAngle does not change
  if (xPos > VIDEO_WIDTH || yPos > VIDEO_HEIGHT) { 
    ballAngle = 0;
  } else {
    double m = (float)(yPos) / (float)(xPos);
    ballAngle = atan((double)m);
    ballAngle *= 57.2957795129;
    if (xPos < 0) {
      ballAngle += 180.0;
    } else if (yPos < 0) {
      ballAngle += 360.0;
    }

    if (m == .75) {
      ballAngle = 0;
    }
  }

  if (tPos > VIDEO_WIDTH || oPos > VIDEO_HEIGHT) {
    goalAngle = 0;
  } else {
    double m = (float)(oPos) / (float)(tPos);
    goalAngle = atan((double)m);
    goalAngle *= 57.2957795129;
    if (tPos < 0) {
      goalAngle += 180.0;
    } else if (oPos < 0) {
      goalAngle += 360.0;
    }
    if (m == .75) {
      goalAngle = 0;
    }
  }

  if (goalAngle >= 180) {
    goalAngle -= 360.0;
  }

  if (oPos == 0 or tPos == 0) {
    goalAngle = 0;
  }
}

void updateTOFReadings() {
  ballRanges[4] = ballRanges[3];
  ballRanges[3] = ballRanges[2];
  ballRanges[2] = ballRanges[1];
  ballRanges[1] = ballRanges[0];

  float range = vl.readRange();

  uint8_t rangeStatus = vl.readRangeStatus();
  if (rangeStatus != VL6180X_ERROR_NONE) {
    Serial6.print("TOF read error: ");
    Serial6.println(rangeStatus);
    return;
  } else {
    ballRanges[0] = range;
  }
}

void fixOutOfBounds(int side) {
//  Serial6.print("Out of bounds... current angle = ");
//  Serial6.println(motor.getRelativeAngle(side));
  logLIDARS();
  
  int slowerSpeed = 100; 

  if (abs(motor.getRelativeAngle(side)) < 5 or turnFixed) {
    turnFixed = true;
    
    frontSensor = lidars.readSensor1();
    backSensor = lidars.readSensor3();
    leftSensor = lidars.readSensor2();
    rightSensor = lidars.readSensor4();

    float minReading = min(min(min(frontSensor, backSensor), leftSensor), rightSensor);

    if (frontSensor <= minReading and frontSensor < 49) {
      if (backSensor < 45) {
        if (leftSensor < rightSensor) {
          motor.driveToRelativeHeadingCorrected(90, 0, slowerSpeed);
        } else {
          motor.driveToRelativeHeadingCorrected(270, 0, slowerSpeed);
        }
      } else {
        motor.driveToRelativeHeadingCorrected(-180, 0, slowerSpeed);
      }
      return;
    } else if (backSensor <= minReading and backSensor < 49) {
      if (frontSensor < 45) {
        if (leftSensor < rightSensor) {
          motor.driveToRelativeHeadingCorrected(90, 0, slowerSpeed);
        } else {
          motor.driveToRelativeHeadingCorrected(270, 0, slowerSpeed);
        }
      } else {
        motor.driveToRelativeHeadingCorrected(0, 0, slowerSpeed);
      }
      return;
    } else if (rightSensor <= minReading and rightSensor < 49) {
      if (leftSensor < 45) {
        if (frontSensor < backSensor) {
          motor.driveToRelativeHeadingCorrected(-180, 0, slowerSpeed);
        } else {
          motor.driveToRelativeHeadingCorrected(0, 0, slowerSpeed);
        }
      } else {
        motor.driveToRelativeHeadingCorrected(270, 0, slowerSpeed);
      }
      return;
    } else if (leftSensor <= minReading and leftSensor < 49) {
      if (rightSensor < 45) {
        if (frontSensor < backSensor) {
          motor.driveToRelativeHeadingCorrected(-180, 0, slowerSpeed);
        } else {
          motor.driveToRelativeHeadingCorrected(0, 0, slowerSpeed);
        }
      } else {
        motor.driveToRelativeHeadingCorrected(90, 0, slowerSpeed);
      }
      return;
    } else {
      motor.stopMotors();
      interrupted = false;
      turnFixed = false;
    }
  } else {
    motor.turnToAbsoluteHeading(side, MAX_SPEED);
  }
}

void checkForIMUZero() {
  int val = 0;
  val = digitalRead(BUTTON_PIN);
  if (val == LOW) {
    Serial6.println("RESETTING GYRO");
    shouldDodge = false;
    useSideLIDAR = false;
    turningToShoot = false;
    isShooting = false;
    motor.resetGyro();
    gyroHathBeenSet = true;
    analogWrite(RED_PIN,0);
  }
  else analogWrite(RED_PIN,255);
}

void ledWhite(){
  analogWrite(RED_PIN,0);
  analogWrite(BLUE_PIN,0);
  analogWrite(GREEN_PIN,0);
}
void ledYellow(){
  analogWrite(RED_PIN,255);
  analogWrite(BLUE_PIN,0);
  analogWrite(GREEN_PIN,0);
}
void ledCyan(){
  analogWrite(RED_PIN,0);
  analogWrite(BLUE_PIN,255);
  analogWrite(GREEN_PIN,0);
}
void ledMagenta(){
  analogWrite(RED_PIN,0);
  analogWrite(BLUE_PIN,0);
  analogWrite(GREEN_PIN,255);
}
void ledGreen(){
  analogWrite(RED_PIN,255);
  analogWrite(BLUE_PIN,255);
  analogWrite(GREEN_PIN,0);
}
void ledBlue(){
  analogWrite(RED_PIN,0);
  analogWrite(BLUE_PIN,255);
  analogWrite(GREEN_PIN,255);
}
void ledRed(){
  analogWrite(RED_PIN,255); //255
  analogWrite(BLUE_PIN,0); //0
  analogWrite(GREEN_PIN,255); //0
}
void ledPurple(){
  analogWrite(RED_PIN,44);
  analogWrite(GREEN_PIN,0);
  analogWrite(BLUE_PIN,63);
}

void logLIDARS() {
  Serial6.print("LIDARS: ");
  Serial6.print(frontDistance);
  Serial6.print(" --- ");
  Serial6.print(rightDistance);
  Serial6.print(" --- ");
  Serial6.print(backDistance);
  Serial6.print(" --- ");
  Serial6.print(leftDistance);
  Serial6.print(" --- ");
  Serial6.print(abs(frontSensor + backSensor - FIELD_LENGTH));
  Serial6.print(" --- ");
  Serial6.println(abs(leftSensor + rightSensor - FIELD_WIDTH));
}

void flash(){
  analogWrite(RED_PIN,255);
//  analogWrite(WHITEA_PIN,255);
  delay(100);
  analogWrite(RED_PIN,0);
//  analogWrite(WHITEA_PIN,0);
  delay(100);
  analogWrite(RED_PIN,255);
//  analogWrite(WHITEA_PIN,255);
  delay(100);
  analogWrite(RED_PIN,0);
//  analogWrite(WHITEA_PIN,0);
  delay(100);
  analogWrite(RED_PIN,255);
//  analogWrite(WHITEA_PIN,255);
  delay(100);
  analogWrite(RED_PIN,0);
//  analogWrite(WHITEA_PIN,0);
  delay(100);
  analogWrite(RED_PIN,255);
//  analogWrite(WHITEA_PIN,255);
  delay(100);
  analogWrite(RED_PIN,0);
//  analogWrite(WHITEA_PIN,0);
  delay(100);
  analogWrite(RED_PIN,255);
//  analogWrite(WHITEA_PIN,255);
  delay(100);
  analogWrite(RED_PIN,0);
//  analogWrite(WHITEA_PIN,0);
  delay(100);
  analogWrite(RED_PIN,255);
//  analogWrite(WHITEA_PIN,255);
  delay(100);
  analogWrite(RED_PIN,0);
//  analogWrite(WHITEA_PIN,0);
  delay(100);
  analogWrite(RED_PIN,255);
//  analogWrite(WHITEA_PIN,255);
  delay(100);
  analogWrite(RED_PIN,0);
//  analogWrite(WHITEA_PIN,0);
  delay(100);
}

float lastLIDARRead = 0;
float lastFrontSensor = 100000;
float lastBackSensor = 100000;
float lastLeftSensor = 100000;
float lastRightSensor = 100000;
void readLIDARS(float readInterval) {
  if (millis() - lastLIDARRead >= readInterval) {
    lastLIDARRead = millis();
    float heading = motor.getRelativeAngle(0.0);
    if (abs(heading) < 15) {
      float lengthScalar = cos(PI*abs(heading)/180.0);
      frontSensor = (lidars.readSensor1() - 5.0)*lengthScalar;
      backSensor = (lidars.readSensor3() + 5.0)*lengthScalar;
      leftSensor = (lidars.readSensor2() - 7.0)*lengthScalar;
      rightSensor = (lidars.readSensor4() + 7.0)*lengthScalar;
    } else if (abs(heading) > 165) {
      float lengthScalar = abs(cos(PI*abs(heading)/180.0));
      frontSensor = (lidars.readSensor3() + 5.0)*lengthScalar;
      backSensor = (lidars.readSensor1() - 5.0)*lengthScalar;
      leftSensor = (lidars.readSensor4() + 7.0)*lengthScalar;
      rightSensor = (lidars.readSensor2() - 7.0)*lengthScalar;
    } else if (heading > 75 and heading < 105) {
      float lengthScalar = abs(cos(PI*abs(heading - 90.0)/180.0));
      frontSensor = (lidars.readSensor2() - 7.0)*lengthScalar;
      backSensor = (lidars.readSensor4() + 7.0)*lengthScalar;
      leftSensor = (lidars.readSensor3() + 5.0)*lengthScalar;
      rightSensor = (lidars.readSensor1() - 5.0)*lengthScalar;
    } else if (heading < -75 and heading > -105) {
      float lengthScalar = abs(cos(PI*abs(heading + 90.0)/180.0));
      frontSensor = (lidars.readSensor4() + 7.0)*lengthScalar;
      backSensor = (lidars.readSensor2() - 7.0)*lengthScalar;
      leftSensor = (lidars.readSensor1() - 5.0)*lengthScalar;
      rightSensor = (lidars.readSensor3() + 5.0)*lengthScalar;
    } else { return; }

    if (abs(frontSensor + backSensor - FIELD_LENGTH) < 15 or useFrontLIDAR) {
      if (useFrontLIDAR and frontSensor < FIELD_LENGTH) {
//        Serial6.println("Using front LIDAR...");
        frontDistance = frontSensor;
        lastFrontSensor = frontSensor;
        backDistance = FIELD_LENGTH - frontSensor;
      } else if (not useFrontLIDAR) {
        frontDistance = frontSensor;
        backDistance = backSensor;
        lastFrontSensor = frontSensor;
        lastBackSensor = backSensor;
      }
    } else if (abs(frontSensor - lastFrontSensor) < 10) {
      frontDistance = frontSensor;
      backDistance = backDistance - (frontSensor - lastFrontSensor);
      lastFrontSensor = frontSensor;
    } else if (abs(backSensor - lastBackSensor) < 10) { 
      backDistance = backSensor;
      frontDistance = frontDistance - (backSensor - lastBackSensor);
      lastBackSensor = backSensor;
    }

    if (abs(leftSensor + rightSensor - FIELD_WIDTH) < 10 or useSideLIDAR) {
      if (useSideLIDAR) {
//        Serial6.println("Using side LIDAR...");
        if (sideAngle == 90.0 and rightSensor < FIELD_WIDTH) {
          rightDistance = rightSensor;
          lastRightSensor = rightSensor;
          leftDistance = FIELD_WIDTH - rightSensor;
        } else if (leftSensor < FIELD_WIDTH) {
          leftDistance = leftSensor;
          lastLeftSensor = leftSensor;
          rightDistance = FIELD_WIDTH - leftSensor;
        }
      } else {
        leftDistance = leftSensor;
        rightDistance = rightSensor;
        lastLeftSensor = leftSensor;
        lastRightSensor = rightSensor;
      }
    } else if (abs(leftSensor - lastLeftSensor) < 10) {
      leftDistance = leftSensor;
      rightDistance = rightDistance - (leftSensor - lastLeftSensor);
      lastLeftSensor = leftSensor;
    } else if (abs(rightSensor - lastRightSensor) < 10) { 
      rightDistance = rightSensor;
      leftDistance = leftDistance - (rightSensor - lastRightSensor);
      lastRightSensor = rightSensor;
    }
  }
}

void printBallPosition() {
  Serial6.print("Ball position: ");
  Serial6.print(xPos);
  Serial6.print(", ");
  Serial6.println(yPos);
}

void printGoalPosition() {
  Serial6.println("Goal position: ");
  Serial6.print(tPos);
  Serial6.print(", ");
  Serial6.print(oPos);
  Serial6.print(", ");
  Serial6.println(goalAngle);
}
