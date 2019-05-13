void centerToGoal(){
  Serial.println("searching for ball");
  return;
}

void blockBall(){
  float backDistance = lidars.readSensor1();
  Serial.println(lidars.readSensor1());
  if(yPos > 0) motor.driveToHeadingCorrected(90,0,yPos);
  else motor.driveToHeadingCorrected(270,0,-yPos);
  return;
}

void passBall(){
  return;
}

