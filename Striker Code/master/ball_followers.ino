float xTargetDiff = 1000;

float coneSize = 2.0;
float coneSizeIncreaseTime = 0;

void quadraticBall() {
  float theta = -motor.getRelativeAngle(0.0)/180.0*PI;
  float rotatedYPos = cos(theta)*yPos - sin(theta)*xPos;
  float rotatedXPos = sin(theta)*yPos + cos(theta)*xPos;

  Serial.print("Ball xPos: ");
  Serial.println(xPos);
  Serial.print("Ball yPos: ");
  Serial.println(yPos);
  Serial.println("");
  
  if (millis() - coneSizeIncreaseTime > 2000) {
    coneSize = 2.5;
    coneSizeIncreaseTime = 0;
  }
  
  if (abs(rotatedXPos) > abs(coneSize*rotatedYPos)) {
    // Turn on red LED
    analogWrite(RED_PIN, 0);
    motor.dribble(255);
    
    if (xPos < 0 and coneSizeIncreaseTime == 0) {
      coneSize = 0.5;
      coneSizeIncreaseTime = millis();
    }
    
    diagonalBall();
  } else {
    // Turn off red LED
    analogWrite(RED_PIN, 255);
    motor.dribble(255);
  
    float velocityDirection = angleFromSlope(getPathSlope());
    float distanceFromBall = sqrt(xPos*xPos + yPos*yPos);
    motor.driveToHeadingCorrected(velocityDirection, 0.0, min(MAX_SPEED, distanceFromBall));
  }
}

void diagonalBall() {
  float distanceFromBall = sqrt(xPos*xPos + yPos*yPos);
  float directToBallHeading = angleFromSlope(xPos/yPos);
  motor.driveToRelativeHeadingCorrected(directToBallHeading, -directToBallHeading, min(MAX_SPEED, distanceFromBall));
}
