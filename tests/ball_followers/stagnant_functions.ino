
float getBQuadraticTerm() {
  // Coordinates flipped because stupid
  // such goofs avert your eyes
  float backXPos = xPos - 30.0;
//  if (backXPos < k and abs(yPos) > 10) {
//    return (2.0*(backXPos-k)*(yPos + (abs(yPos)/yPos)*sqrt((k*yPos*yPos)/(k-backXPos))))/(yPos*yPos);
//  } else {
//    // Do sideways quadratic
//    return 2.0*yPos/backXPos;
//  }
  if (backXPos < 0) {
    return 2*backXPos/yPos;
  } else {
    return 0.5*backXPos/yPos;
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
    xPos -= 640;
    xPos *= -1;
    xPos += 35;
  } else {
    xPos = oldXPos;
  }
  oldXPos = xPos;

  if (yPos != 0) {
    yPos -= 480;
  } else {
    yPos = oldYPos;
  }
  oldYPos = yPos;
}


void calculateAngle() {
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
      ballAngle = 10000; //ballAngle = 10000 when robot doesn't see ball
    }
  }
}
