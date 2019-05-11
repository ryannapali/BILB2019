float xTargetDiff = 1000;

float coneSize = 2.0;
float coneSizeIncreaseTime = 0;

void quadraticBall(){
  float theta = -motor.getRelativeAngle(0.0)/180.0*PI;
  float rotatedYPos = cos(theta)*yPos - sin(theta)*xPos;
  float rotatedXPos = sin(theta)*yPos + cos(theta)*xPos;

  if (millis() - coneSizeIncreaseTime > 1500) {
    coneSize = 2.0;
    coneSizeIncreaseTime = 0;
  }
  
  if (abs(rotatedXPos) > abs(coneSize*rotatedYPos)) {
    // Turn on red LED
    analogWrite(21, 0);
    motor.dribble(255);
    
    if (xPos < 0 and coneSizeIncreaseTime == 0) {
      coneSize = 0.5;
      coneSizeIncreaseTime = millis();
    }
    
    diagonalBall();
  } else {
    // Turn off red LED
    analogWrite(21, 255);
    motor.dribble(0);
  
    float velocityDirection = angleFromSlope(getPathSlope());
    float distanceFromBall = sqrt(xPos*xPos + yPos*yPos);
    motor.driveToHeadingCorrected(velocityDirection, 0.0, min(MAX_SPEED, 0.6*distanceFromBall));
  }
}

void diagonalBall() {
  float distanceFromBall = sqrt(xPos*xPos + yPos*yPos);
  float directToBallHeading = angleFromSlope(xPos/yPos);
  motor.driveToRelativeHeadingCorrected(directToBallHeading, -directToBallHeading/2, min(MAX_SPEED, distanceFromBall));
}

void zigBall(){
  xTargetDiff = 100 - xPos;
  float distanceFromBall = sqrt(xPos*xPos + yPos*yPos);

  if (abs(xTargetDiff) > 50) { 
    if (xTargetDiff > 0) {
      motor.driveToHeadingCorrected(180, 0.0, min(MAX_SPEED, 0.6*distanceFromBall));
    } else {
      motor.driveToHeadingCorrected(0, 0.0, min(MAX_SPEED, 0.6*distanceFromBall));
    }
  } else {
    if (yPos > 0) {
      motor.driveToHeadingCorrected(270, 0.0, min(MAX_SPEED, 0.6*distanceFromBall));
    } else {
      motor.driveToHeadingCorrected(90, 0.0, min(MAX_SPEED, 0.6*distanceFromBall));
    }
  }
}
