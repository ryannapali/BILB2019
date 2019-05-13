#define X_ORIGIN_CALIBRATION 30.0 
#define Y_ORIGIN_CALIBRATION -10.0 
#define PATH_CURVINESS 3.0
#define TARGET_DIST_BEHIND_BALL 110.0

float dumpLidarData(){
  //frontSensor = lidars.readSensor1();
  //delay(5);
  backSensor = lidars.readSensor1();
  delay(5);
  //delay(5);
  //leftSensor = lidars.readSensor2();
  //delay(5);
  //rightSensor = lidars.readSensor4();
  //Serial.println(frontSensor);
  if(backSensor != -2){
    Serial.println(backSensor);
    return backSensor;
  }
  else return 0;
  //Serial.println(leftSensor);
  //Serial.println(rightSensor);
}

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

void checkFieldReorient() {
  if(digitalRead(BUTTON_PIN) == LOW){
    analogWrite(22, 0);
    motor.resetGyro();
  } else {
    analogWrite(22, 255);
  }
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
    xPos -= 640.0;
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
    yPos -= 480.0;
    yPos += Y_ORIGIN_CALIBRATION;
    oldYPos = yPos;
  } else if (failedBallReadingCount < 4) {
    yPos = oldYPos;
  }

  if (tPos != 0) {
    tPos -= 640;
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
    oPos -= 480;
  } else if (failedGoalReadingCount < 4) {
    oPos = oldOPos;
  }
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
    if (m == .75) {
      goalAngle = 2000; //goalAngle = 2000 when robot doesn't see goal
    }
  }

  if (goalAngle >= 180) {
    goalAngle -= 360;
  }
}
