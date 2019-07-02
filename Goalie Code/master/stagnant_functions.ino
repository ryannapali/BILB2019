float getPathSlope() {
  if (xPos < 0) {
    return PATH_CURVINESS * (xPos - TARGET_DIST_BEHIND_BALL) / yPos;
  } else {
    return (1.0 / PATH_CURVINESS) * (xPos - TARGET_DIST_BEHIND_BALL) / yPos;
  }
}

// MAKE SURE SLOPE WAS CALCULATED USING EXACTLY YPOS
float angleFromSlope(float slope) {
  float angle = atan(slope);
  angle *= 57.2957795129;
  if (yPos < 0) angle = -90.0 - angle;
  else angle = 90.0 - angle;
  return angle;
}

void checkFieldReorient() {
  if (digitalRead(BUTTON_PIN) == LOW)  motor.resetGyro();
}

void clearCameraBuffer() {
  Serial5.clear();
}

float oldXPos = 0.0;
float oldYPos = 0.0;
float oldTPos = 0.0;
float oldOPos = 0.0;

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
    //    if (currentState == ON_LINE) break;
  }
  char highChar1 = Serial5.read();
  char lowChar1 = Serial5.read();
  while (Serial5.available() < 2) {
    //    if (currentState == ON_LINE) break;
  }
  char highChar2 = Serial5.read();
  char lowChar2 = Serial5.read();
  while (Serial5.available() < 2) {
    //    if (currentState == ON_LINE) break;
  }
  char highChar3 = Serial5.read();
  char lowChar3 = Serial5.read();
  while (Serial5.available() < 2) {
    //    if (currentState == ON_LINE) break;
  }
  char highChar4 = Serial5.read();
  char lowChar4 = Serial5.read();

  xPos = word(highChar1, lowChar1);
  yPos = word(highChar2, lowChar2);
  tPos = word(highChar3, lowChar3);
  oPos = word(highChar4, lowChar4);



  // and (abs(xPos - (X_ORIGIN_CALIBRATION - oldXPos + xDimension)) < 30 or oldXPos == 0.0)
  if (xPos > 0 and xPos < 1000) {
    xPos -= xDimension;
    xPos *= -1.0;
    xPos += X_ORIGIN_CALIBRATION;
    oldXPos = xPos;
    failedBallReadingCount = 0;
  } else {
    failedBallReadingCount += 1;
    if (failedBallReadingCount < 10) {
      xPos = oldXPos;
    }
  }

  // and (abs(yPos - (oldYPos - Y_ORIGIN_CALIBRATION + yDimension)) < 30 or oldYPos == 0.0)
  if (yPos > 0 and yPos < 1000) {
    yPos -= yDimension;
    yPos += Y_ORIGIN_CALIBRATION;
    oldYPos = yPos;
  } else if (failedBallReadingCount < 10) {
    yPos = oldYPos;
  }


  if (tPos > 0 and tPos < 1000) {
    tPos -= xDimension;
    tPos *= -1;
    failedGoalReadingCount = 0;
  } else {
    failedGoalReadingCount += 1;
    if (failedGoalReadingCount < 4) {
      tPos = oldTPos;
    }
  }

  tPos = getMedianTPos();
  oldTPos = tPos;

  if (oPos < 1000 and oPos > 0) {
    oPos -= yDimension;
  } else if (failedGoalReadingCount < 4) {
    oPos = oldOPos;
  }
  oPos = getMedianOPos();
  oldOPos = oPos;
}

void checkForIMUZero() {
  bool gyroSet = false;
  int val = 0;
  val = digitalRead(BUTTON_PIN);
  if (val == LOW) {
    Serial.println("RESETTING");
    motor.resetGyro();
    gyroSet = true;
  }
  else analogWrite(WHITEA_PIN, 0);

  if (gyroSet) {
    analogWrite(WHITEA_PIN, 255);
    gyroSet = false;
  } else {
    analogWrite(WHITEA_PIN, 0);
  }
  calibrating = false;
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


  if (xPos != 0 && yPos != 0) {
    lastXPos = xPos;
    lastYPos = yPos;
    lastAngle = ballAngle;
  }
  goalDist = sqrt(sq(tPos) + sq(oPos));
  ballDist = sqrt(sq(xPos) + sq(yPos));
}




int getMedianOPos() {
  for (int i = 8; i > 0; i--) {
    oPosArr[i] = oPosArr[i - 1];
  }
  oPosArr[0] = oPos;

  int n = sizeof(oPosArr) / sizeof(oPosArr[0]);
  int stupid [9] = {oPosArr[0], oPosArr[1], oPosArr[2], oPosArr[3], oPosArr[4], oPosArr[5], oPosArr[6], oPosArr[7], oPosArr[8]};
  std::sort(stupid, stupid + n);
  return stupid[4];
}

int getMedianTPos() {
  for (int i = 8; i > 0; i--) {
    tPosArr[i] = tPosArr[i - 1];
  }
  tPosArr[0] = tPos;

  int n = sizeof(tPosArr) / sizeof(tPosArr[0]);
  int stupid [9] = {tPosArr[0], tPosArr[1], tPosArr[2], tPosArr[3], tPosArr[4], tPosArr[5], tPosArr[6], tPosArr[7], tPosArr[8]};
  std::sort(stupid, stupid + n);
  return stupid[4];
}


int getMedianXPos() {
  for (int i = 4; i > 0; i--) {
    xPosArr[i] = xPosArr[i - 1];
  }
  xPosArr[0] = xPos;

  int n = sizeof(xPosArr) / sizeof(xPosArr[0]);
  int stupid [5] = {xPosArr[0], xPosArr[1], xPosArr[2], xPosArr[3], xPosArr[4]};
  std::sort(stupid, stupid + n);
  return stupid[2];
}


int getMedianYPos() {
  for (int i = 4; i > 0; i--) {
    yPosArr[i] = yPosArr[i - 1];
  }
  yPosArr[0] = yPos;

  int n = sizeof(yPosArr) / sizeof(yPosArr[0]);
  int stupid [5] = {yPosArr[0], yPosArr[1], yPosArr[2], yPosArr[3], yPosArr[4]};
  std::sort(stupid, stupid + n);
  return stupid[2];
}



boolean fastmovingBall() {
  int ballSpeeds[10];
  int ballSpeed;
  for (int i = 0; i < 10; i++) {
    getCameraReadings();
    ballSpeeds[i] = xPos;
    if (i > 0) {
      ballSpeed += abs(ballSpeeds[i] - ballSpeeds[i - 1]);
    }
  }
  if (ballSpeed > 26) return true;
  else return false;
}
int theirGoal;
int theirGoalAngle() {
  if (abs(motor.getRelativeAngle(0.0)) < 10) {
    theirGoal = 180 - goalAngle;
    if (theirGoal < 0) theirGoal = 360 + theirGoal;
  }
  return theirGoal;
}


int whichSide() {
  if (rightSensor + leftSensor > 160) { //perspective of robot, left = 0, right = 2, center = 1;
    if (abs(rightSensor - leftSensor) < 30) return 1;
    else if (rightSensor > leftSensor) return 0;
    else return 2;
  }
  else {
    return -1;
  }

}
