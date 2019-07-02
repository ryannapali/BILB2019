boolean farAway;
int startTime;

void invBall() {
  backSensor = lidars.readSensor3();
  leftSensor = lidars.readSensor2();
  rightSensor = lidars.readSensor4();
  if (firstInvBall) {
    startTime = millis();
    firstInvBall = false;
  }
  else if (millis() - startTime < 1000 && backSensor < 100) {
    //Serial.println("wating");
    motor.stopMotors();
  }
  else if (digitalRead(37) == HIGH) {
    //Serial.println("going to center");
    goToLocation(90, 45, max(leftSensor, rightSensor));
  }
  else {
    if (lastXPos > 200) farAway = true;
    else farAway = false;

    if (farAway) goToLocation(90, 45, max(leftSensor, rightSensor)); //ball is far and invisible
    else if (lastAngle < 20 && lastAngle > 360 - 20) { //ball is close and invisible and infront
      motor.stopMotors();
    }
    else {

      goToLocation(determineSide(), 45, max(leftSensor, rightSensor));
    }
  }
}

int determineSide() {
  int xLocation;
  if (whichSide() == 1) { //center
    Serial.println("center");
    if (ballAngle < 60 || ballAngle > 300) xLocation = 90;
    else if (ballAngle > 60 && ballAngle < 180) xLocation = 112;
    else xLocation = 60;
  }

  else if (whichSide() == 0) { //left
    Serial.println("left");
    if (ballAngle < 40 || ballAngle > 180) xLocation = 60;
    else if (ballAngle > 90 && ballAngle < 180) xLocation = 112;
    else xLocation = 90;
  }
  else if (whichSide() == 2) { //right
    Serial.println("right");
    if (ballAngle > 320 || ballAngle < 180) xLocation = 112;
    else if (ballAngle < 270 && ballAngle > 180) xLocation = 60;
    else xLocation = 90;
  }
  return xLocation;
}
