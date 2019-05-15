void centerToGoal(){
  Serial.println("searching for ball");
  return;
}

void blockBall(){
  backSensor = lidars.readSensor3();
  leftSensor = lidars.readSensor2();
  rightSensor = lidars.readSensor4();
  sideSum = leftSensor + rightSensor;
  if(sideSum > 170 && sideSum < 20) sideSumConfidant = true;
  else sideSumConfidant = false;
  float distanceFromGoal = 37 ;
  float distanceOff = backSensor-distanceFromGoal;
  Serial.println(leftSensor);
  Serial.print("     ");
  Serial.println(rightSensor);
  float thing = abs(yPos) * abs(yPos) / 100;
  if(yPos < 0){
    if(leftSensor > 72) motor.driveToHeadingCorrectedHoldDistance(270,0,thing,distanceOff);
    else motor.stopMotors();
  }
  else if(yPos>0){
    if(rightSensor > 72) motor.driveToHeadingCorrectedHoldDistance(90,0,thing,distanceOff);
    else motor.stopMotors();
  }
  else motor.stopMotors();
  return;
}

void passBall(){
  return;
}

