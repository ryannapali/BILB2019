void getToGoal(){
  boolean turning = true;
  backSensor = lidars.readSensor3();
  bool far = farFromGoal();
  //turn backwards
  //center on field
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
  while(abs(goalAngle<5)){ 
  if(goalAngle<0) motor.spin(40);
  else motor.spin(-40);
  //if(goalAngle < 0) motor.spin(max(-goalAngle,20));
  //else motor.spin(min(-goalAngle,-20));
  if(abs(goalAngle < 5)){
    motor.dribble(0);
    shoot();
  }
  else motor.dribble(255);
  return;
  }
}

boolean farFromGoal(){ 
  if(backSensor > 80) return true;
  else{
    return false;
  }
}

