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
  motor.driveToHeadingCorrected(ballAngle, ballAngle, min(MAX_SPEED, max(distanceFromBall-10, 80))*speedScalar);
}

float lastAngle = 0.0;
void quadraticBall(){
  float velocityVectorAngle = atan(getBQuadraticTerm());
  velocityVectorAngle *= 57.2957795129;
  if (yPos < 0) {
    velocityVectorAngle = -90.0 - velocityVectorAngle;
  } else {
    velocityVectorAngle = 90.0 - velocityVectorAngle;
  }
  
  float distanceFromBall = sqrt(xPos*xPos + yPos*yPos);
  if (xPos < 20 and xPos > -150 and abs(yPos) < 25) {
    // Turn on red LED
    analogWrite(21, 0);
    Serial.println("charging");
    motor.dribble(255);
    motor.driveToHeadingCorrected(0.0, 0.0, MAX_SPEED/2);
    return;
  } else {    
    // Turn off red LED
    analogWrite(21, 255);
    motor.dribble(0);
  }

//  Serial.println(velocityVectorAngle);
//  Serial.println(getBQuadraticTerm());
//  Serial.print(xPos);
//  Serial.print(" ");
//  Serial.println(yPos);
  motor.driveToHeadingCorrected(velocityVectorAngle, 0.0, min(MAX_SPEED, max(distanceFromBall-10, 80))*speedScalar);
  lastAngle = velocityVectorAngle;
}
