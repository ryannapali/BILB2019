float speedScalar = MAX_SPEED/150.0;

float k = 20.0;

float xTargetDiff = 1000;

float lastYPos = 0.0;
float lastTime = 0.0;

float lastAngle = 0.0;
void quadraticBall(){
  float velocityVectorAngle = angleFromSlope(getBQuadraticTerm());
  
  float distanceFromBall = sqrt(xPos*xPos + yPos*yPos);
  if (abs(xPos) < 200 and abs(yPos) > abs(3.0*xPos)) {
    // Turn on red LED
    analogWrite(21, 0);
    motor.dribble(255);

    float directToBallHeading = angleFromSlope(xPos/yPos);
    motor.driveToHeadingCorrected(directToBallHeading, motor.getRelativeAngle(0.0) + directToBallHeading, MAX_SPEED);
    return;
  } else {
    // Turn off red LED
    analogWrite(21, 255);
    motor.dribble(0);
  }

//  float averageYSpeed = 1000.0*(lastYPos-yPos)/(millis()-lastTime);
//  if (abs(yPos) < 200 and yPos != 0.0 and abs(averageYSpeed) > 300) {
//    if (yPos < 0) {
//      motor.driveToHeadingCorrected(270.0, 0.0, min(MAX_SPEED, distanceFromBall));
//    } else {
//      motor.driveToHeadingCorrected(90.0, 0.0, min(MAX_SPEED, distanceFromBall));
//    }
//  } else {
//    motor.driveToHeadingCorrected(velocityVectorAngle, 0.0, min(MAX_SPEED, 0.5*distanceFromBall));
//  }

  motor.driveToHeadingCorrected(velocityVectorAngle, 0.0, min(MAX_SPEED, 0.6*distanceFromBall));

  lastAngle = velocityVectorAngle;
  lastYPos = yPos;
  lastTime = millis();
}

void zigBall(){
  xTargetDiff = 100 - xPos;

  if (abs(xTargetDiff) > 50) { 
    if (xTargetDiff > 0) {
      motor.driveToHeadingCorrected(180, 0.0, min(MAX_SPEED, abs(xTargetDiff)*speedScalar));
    } else {
      motor.driveToHeadingCorrected(0, 0.0, min(MAX_SPEED, abs(xTargetDiff)*speedScalar));
    }
  } else {
    if (yPos > 0) {
      motor.driveToHeadingCorrected(270, 0.0, min(MAX_SPEED, abs(yPos)*speedScalar*2));
    } else {
      motor.driveToHeadingCorrected(90, 0.0, min(MAX_SPEED, abs(yPos)*speedScalar*2));
    }
  }
}

void diagonalBall(){
  calculateAngles();
  float distanceFromBall = sqrt(xPos*xPos + yPos*yPos);
  motor.driveToHeadingCorrected(ballAngle, ballAngle, min(MAX_SPEED, max(distanceFromBall-10, 80))*speedScalar);
}
