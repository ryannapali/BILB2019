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

void getCameraReadings() {
  float oldXPos = 0.0;
  float oldYPos = 0.0;
  float oldTPos = 0.0;
  float oldOPos = 0.0;
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

  if (xPos != 0) {
    xPos -= xDimension;
    xPos *= -1.0;
    xPos += X_ORIGIN_CALIBRATION;
    oldXPos = xPos;
    failedBallReadingCount = 0;
  } else {
    failedBallReadingCount += 1;
    if (failedBallReadingCount < 4) {
      xPos = oldXPos;
    }
  }

  if (yPos != 0) {
    yPos -= yDimension;
    yPos += Y_ORIGIN_CALIBRATION;
    oldYPos = yPos;
  } else if (failedBallReadingCount < 4) {
    yPos = oldYPos;
  }

  if (tPos != 0) {
    tPos -= xDimension;
    tPos *= -1;
    oldTPos = tPos;
    failedGoalReadingCount = 0;
  } else {
    failedGoalReadingCount += 1;
    if (failedGoalReadingCount < 4) {
      tPos = oldTPos;
    }
  }

  if (oPos != 0) {
    oPos -= yDimension;
  } else if (failedGoalReadingCount < 4) {
    oPos = oldOPos;
  }
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
  if (xPos > 1280 || yPos > 960) { //filter out and bad readings. 2000 is sign of bad readings
    ballAngle = 2000;
  } else {
    double m = (float)(yPos) / (float)(xPos);
    ballAngle = atan((double)m);
    ballAngle *= 57.2957795129;
    if (xPos < 0) {
      ballAngle += 180;
    } else if (yPos < 0) {
      ballAngle += 360;
    }
    if (m == .75) {
      ballAngle = 2000; //ballAngle = 2000 when robot doesn't see ball
    }
  }
  if (tPos > 1280 || oPos > 960) { //filter out and bad readings. 2000 is sign of bad readings
    goalAngle = 2000;
  } else {
    double m = (float)(oPos) / (float)(tPos);
    goalAngle = atan((double)m);
    goalAngle *= 57.2957795129;
    if (tPos < 0) {
      goalAngle += 180;
    } else if (oPos < 0) {
      goalAngle += 360;
    }
    if (m == .75) { //480/640 = .75 so it is reads the max.
      goalAngle = 2000; //goalAngle = 2000 when robot doesn't see goal
    }
  }
}

//not used
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
