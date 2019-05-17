void getToGoal(){
  boolean turning = true;
  bool far = farFromGoal();
  if(abs(motor.getRelativeAngle(180) > 15)){
    motor.turnToAbsoluteHeading(180, 80);
    motor.dribble(255);
  }
  else if (far){
  //drive towards goal
    motor.dribble(255);
    rightSensor = lidars.readSensor4();
    motor.driveToHeadingCorrected(180+(100-rightSensor),180,DRIB_SPEED);
  }
  else if(far == false) readyToShoot = true;
}

void shootToOpenGoal(){
  while(abs(goalAngle>5)){ 
    if(goalAngle<0) motor.spin(40);
    else motor.spin(-40);
    motor.dribble(255);
  }
  if(abs(goalAngle < 5)){
    motor.dribble(0);
    shoot();
  return;
  }
}

boolean farFromGoal(){ 
  if(backSensor > 80) return true;
  else{
    return false;
  }
}

