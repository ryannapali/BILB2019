boolean shot = false;

void ballCharge() {
  backSensor = lidars.readSensor3();
  

  if (millis() - commitTimer > 5000 || shot || (yPos == 0.0 and xPos == 0.0)) {
    notMoved = false;
    state = sees_ball;
    shot = false;
    //Serial.println("done");
  }
  else {
    //Serial.println("doing");
    if (ballDist < 42 && ballAngle < 3 && ballAngle > 357) { //has ball
      hasBall();
    }
    else {
      chargeBall();
    }
  }
}

void hasBall() {
  //Serial.println("driving to shoot");
  motor.dribble(255);
  int driveAngle = 0;
  leftSensor = lidars.readSensor2();
  rightSensor = lidars.readSensor4();
  if (leftSensor > rightSensor) driveAngle += (leftSensor - rightSensor);
  else driveAngle = 360 - (leftSensor - rightSensor);

  if ( backSensor < 100) motor.driveToHeadingCorrected(driveAngle, 0, min(2 * (125 - backSensor), MAX_SPEED));
  else getToShoot();
}

void getToShoot() {
  motor.stopMotors();
  shoot();
}

float lastShootTime = 0.0;
void shoot() {
  if ((lastShootTime == 0 or millis() - lastShootTime > 2000)) {
    lastShootTime = millis();
    motor.dribble(0);
    digitalWrite(SOLENOID_PIN, HIGH);
    delay(100);
    digitalWrite(SOLENOID_PIN, LOW);
    shot = true;
  }
}

void chargeBall() {
  if (ballDist > 50) {
    motor.dribble(140);
    motor.driveToHeadingCorrected(ballAngle, 0, min(ballDist * 2, MAX_SPEED));
  }
  else {
    hasBall();
  }
}
