float lastLeft = 0.0;
float lastRight = 0.0;

void centerToGoal(){
  backSensor = lidars.readSensor3();
  leftSensor = lidars.readSensor2();
  rightSensor = lidars.readSensor4();
  sideSum = leftSensor + rightSensor;

  lastLeft = leftSensor;
  lastRight = rightSensor;

  Serial.println(leftSensor);
  
  float distanceFromGoal = 37.0;
  float centerDistance = 93.0;
  float distanceOff = backSensor-distanceFromGoal;

  sideSumConfident = (abs(sideSum-185) < 10);
  if (sideSumConfident) {

  } else if (abs(lastLeft - leftSensor) < 4) {
    rightSensor = lastRight + lastLeft - leftSensor;
  } else if (abs(lastRight - rightSensor) < 4) {
    leftSensor = lastLeft + lastRight - rightSensor;
  } else {
    motor.stopMotors();
    return;
  }

  if(abs(motor.getRelativeAngle(0.0)) > 5) motor.turnToAbsoluteHeading(0.0,200);
  else if(leftSensor - centerDistance > 2) motor.driveToHeadingCorrectedHoldDistance(270,0,5*(leftSensor-centerDistance),distanceOff);
  else if (rightSensor - centerDistance > 2) motor.driveToHeadingCorrectedHoldDistance(90,0,3*(rightSensor-centerDistance),distanceOff);
  else motor.stopMotors();
}

void blockBall(){
  backSensor = lidars.readSensor3();
  leftSensor = lidars.readSensor2();
  rightSensor = lidars.readSensor4();
  sideSum = leftSensor + rightSensor;
//  if(sideSum > 170 && sideSum < 20) sideSumConfident = true;
//  else sideSumConfident = false;
  float distanceFromGoal = 37;

  sideSumConfident = (abs(sideSum-185) < 10);
  if (sideSumConfident) {

  } else if (abs(lastLeft - leftSensor) < 4) {
    rightSensor = lastRight + lastLeft - leftSensor;
  } else if (abs(lastRight - rightSensor) < 4) {
    leftSensor = lastLeft + lastRight - rightSensor;
  } else {
    motor.stopMotors();
    return;
  }
  lastLeft = leftSensor;
  lastRight = rightSensor;
  
  float distanceOff = backSensor-distanceFromGoal;
  float thing = min(abs(yPos) * 3.65, 200);
  if (yPos < 0){
    if (leftSensor > 72) motor.driveToHeadingCorrectedHoldDistance(270,0,thing,distanceOff);
    else motor.stopMotors();
  }
  else if (yPos > 0){
    if (rightSensor > 72) motor.driveToHeadingCorrectedHoldDistance(90,0,thing,distanceOff);
    else motor.stopMotors();
  }
  else motor.stopMotors();
}

void passBall(){
  return;
}

void diagonalBall() {
  float distanceFromBall = sqrt(xPos*xPos + yPos*yPos);
  float directToBallHeading = angleFromSlope(xPos/yPos);
  if (distanceFromBall < 100) {
    motor.dribble(255);
  } else {
    motor.dribble(0);
  }
  motor.driveToRelativeHeadingCorrected(directToBallHeading, -directToBallHeading, min(MAX_SPEED, distanceFromBall*1.2));
}

