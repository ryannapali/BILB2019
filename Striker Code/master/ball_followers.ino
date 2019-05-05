float speedScalar = MAX_SPEED/150.0;

float k = 20.0;

float xTargetDiff = 1000;

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
  motor.driveToHeadingCorrected(ballAngle, ballAngle + motor.getRelativeAngle(0.0), min(MAX_SPEED, max(distanceFromBall-20, 0))*speedScalar);
}

void quadraticBall(){
  float velocityVectorAngle = atan(1.0/getBQuadraticTerm());
  velocityVectorAngle *= 57.2957795129;
  if (xPos < 0) {
    velocityVectorAngle += 180;
  } else if (yPos < 0) {
    velocityVectorAngle += 360;
  }
  if (velocityVectorAngle < 0) {
    velocityVectorAngle += 180;
  }
  
  float distanceFromBall = sqrt(xPos*xPos + yPos*yPos);
  if (distanceFromBall < 30) {
    motor.dribble(255);
  } else {
    motor.dribble(0);
  }
  motor.driveToHeadingCorrected(velocityVectorAngle, 0.0, min(MAX_SPEED, max(distanceFromBall-10, 0))*speedScalar);
}
