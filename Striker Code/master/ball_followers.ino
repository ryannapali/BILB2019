void quadraticBall() {
  float theta = -motor.getRelativeAngle(0.0)/180.0*PI;
  float rotatedYPos = cos(theta)*yPos - sin(theta)*xPos;
  float rotatedXPos = sin(theta)*yPos + cos(theta)*xPos;
  
  if (millis() - coneSizeIncreaseTime > 2000) {
    coneSize = 2.5;
    coneSizeIncreaseTime = 0;
  }
  
  if (abs(rotatedXPos) > abs(coneSize*rotatedYPos)) {
    // Turn on red LED
    motor.dribble(255);
    
    if (xPos < 0 and coneSizeIncreaseTime == 0) {
      coneSize = 0.5;
      coneSizeIncreaseTime = millis();
    }
    
    diagonalBall();
  } else {
    // Turn off red LED
    motor.dribble(255);
  
    float velocityDirection = angleFromSlope(getPathSlope());
    float distanceFromBall = sqrt(xPos*xPos + yPos*yPos);
    motor.driveToHeadingCorrected(velocityDirection, 0.0, min(MAX_SPEED, distanceFromBall));
  }
}

void diagonalBall() {
  float distanceFromBall = sqrt(xPos*xPos + yPos*yPos);
  float directToBallHeading = angleFromSlope(xPos/yPos);
  if (distanceFromBall < 100) {
    motor.dribble(255);
  } else {
    motor.dribble(0);
  }

  float directToBallTurnHeading = min(abs(directToBallHeading), 180.0);
  if (directToBallHeading < 0) directToBallTurnHeading *= -1.0;

//  motor.driveToRelativeHeadingCorrected(directToBallHeading, -directToBallTurnHeading*0.8, min(max(distanceFromBall, 75), MAX_SPEED));
//  Serial.println(min(max(distanceFromBall, 75.0), MAX_SPEED)/MAX_SPEED);
//  min(max(distanceFromBall, 75.0), MAX_SPEED)/MAX_SPEED
  float fracAngleOff = 1.0 - abs(directToBallHeading)/180.0;
  float distanceFactor = max(abs(distanceFromBall - 50.0)/150.0, 1.0);
  distanceFactor = 1.0;
  //fracAngleOff * distanceFactor * MAX_SPEED
  //min(max(distanceFromBall, 130.0), MAX_SPEED)/MAX_SPEED
  motor.driveToRelativeHeadingCorrectedProportionalThrust(directToBallHeading, -directToBallTurnHeading, min(max(distanceFromBall, 130.0), MAX_SPEED)/MAX_SPEED, MAX_SPEED);
}
