void centerToGoal(){
  motor.stopMotors();
  Serial.println("searching for ball");
  return;
}
float lastLeft = 0.0;
float lastRight = 0.0;

void blockBall(){
  backSensor = lidars.readSensor3();
  leftSensor = lidars.readSensor2();
  rightSensor = lidars.readSensor4();
  sideSum = leftSensor + rightSensor;
  Serial.println(sideSum);
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
  Serial.println(leftSensor);
  Serial.print("     ");
  Serial.println(rightSensor);
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

