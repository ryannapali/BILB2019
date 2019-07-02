boolean goBack = false;
int driveSpeed;


void getToBall() {
  if (backSensor > 120 || goBack) { //
    goBack = true;
    goToLocation(90, 45, leftSensor);
    if (backSensor < 70) goBack = false;
  }
  else if (digitalRead(35) == LOW) {
    option1();
  }

  else {

    option2();

  }
}


void option1() {
  int driveAngle;
  int sideBlockRange = 90 / 2;
  backSensor = lidars.readSensor3();
  leftSensor = lidars.readSensor2();
  rightSensor = lidars.readSensor4();

  if ((backSensor < 90 || ballDist < 50) && (ballAngle > sideBlockRange) && (ballAngle < 360 - sideBlockRange)) {
    driveAngle = sideBlock();
  }
  else {
    driveAngle = optimalPosition();
  }

  if ((rightSensor + leftSensor > 150) and ((rightSensor < 65 and ballAngle > 10 and ballAngle < 170) || (leftSensor < 65 and ballAngle <350 and ballAngle > 190)) and (backSensor > 50)) {
    int num = 45;
    if ((ballAngle < 350 && ballAngle > 270 + num) || (ballAngle < 90 - num && ballAngle > num)) {
      if (leftSensor < 55) driveAngle = 90;
      else if (rightSensor < 55) driveAngle = 180;
      else driveAngle = 360;
    }
    else if ((ballAngle < 270 - num && ballAngle > 180) || (ballAngle < 180 && ballAngle > 90 + num)) {
      if (leftSensor < 55) driveAngle = 90;
      else if (rightSensor < 55) driveAngle = 180;
      else driveAngle = 180;
    }
    else {
      motor.stopMotors();
    }
  }
  else if (backSensor < 50) {
    motor.driveToHeadingCorrected(0, 0, 7 * (55 - backSensor));
  }
  else {
    motor.driveToHeadingCorrected(driveAngle, 0, driveSpeed);
  }
}

int oldDriveSpeed;

void option2() {
  backSensor = lidars.readSensor3();
  leftSensor = lidars.readSensor2();
  rightSensor = lidars.readSensor4();
  int angle = 270 - (backSensor - 50);
  int stayBuffer = 5;
  int k = .04;
  int bA = ballAngle;
  if (bA > 180) bA = 360 - bA;
  if (ballAngle > stayBuffer and ballAngle < 180) {
    angle = 90 + (backSensor - 50);
  }
  Serial.print(bA);
  Serial.print("    ");
  int driveSpeed = min(0.2 * bA * bA, MAX_SPEED);
  //if (abs(oldDriveSpeed - driveSpeed) > 50) driveSpeed = oldDriveSpeed;
  Serial.println(driveSpeed);
  oldDriveSpeed = driveSpeed;
  if (bA < 15) driveSpeed = 0;
  int blockSize = 70;
  if (min(leftSensor, rightSensor) < blockSize) {
    if (leftSensor < blockSize && ballAngle < 360 && ballAngle > 180) {
      motor.stopMotors();

    }
    else if (rightSensor < blockSize && ballAngle > 0 && ballAngle < 180) {
      motor.stopMotors();
    }
    else motor.driveToHeadingCorrected(angle, 0, driveSpeed);
  }
  else motor.driveToHeadingCorrected(angle, 0, driveSpeed);

}

/*
   Assumptions: We assume that the optimal position for the robot to be is the midpoint
                of the line containing the center point of the ball and the goal.
   Returns: Whether or not the robot is in the optimal position.
   Do: Calculates the angle the robot has to move at to get in between ball and goal.
       Moves to that angle
*/


int optimalPosition() { //returns ball angle
  ballDist += 75;
  int newBA = ballAngle;
  int newGA = goalAngle;
  newGA += 17;
  //gets rid of all numbers more than 180
  if (newBA >= 180) newBA -= 360;
  if (newGA >= 180) newGA -= 360;

  //calculates distances
  float h;

  //calculates angle (h) that is necessary for the calculations to find the final angle
  if (newGA / abs(newGA) == newBA / abs(newBA)) h = abs(abs(newGA) - abs(newBA));
  else h = abs(newGA) + abs(newBA);
  if (h > 180) h = 360 - h;

  float cosH = cos(h * .01745);

  //calculates driveAngle by some wonko math.
  float g = goalDist * cosH + ballDist;
  float f = sqrt(sq(ballDist) + sq(goalDist) + (2 * ballDist * goalDist * cosH));
  float driveAngle = acos(g / f) * 57.2957795131;
  driveAngle += abs(newBA);
  int linearAng = newBA - newGA;

  //if it should drive left, than it converts driveAngle into a number more than 180.
  if ((linearAng > -180 && linearAng < 0) || linearAng > 180) {
  }
  else  { //((abs(newBA) + abs(newGA) > 180) ||  (newBA < 0 && newGA < 0))
    driveAngle = 360 - driveAngle;
  }

  //the proportional speed for it moving to the right point.
  int bufferAng = 20;
  int bufferDist = 30;
  boolean linear = false;


  linearAng = abs(linearAng);
  if ((linearAng > 180 - (bufferAng / 2)) && (linearAng < 180 + (bufferAng / 2))) linear = true;
  else linear = false;
  int blockSpeed = min(MAX_SPEED, max(4.6 * abs((180 - linearAng)), .5 * abs(goalDist - ballDist)));

  if (abs(goalDist - ballDist) < bufferDist && linear) {
    driveSpeed = 0;
  }
  else {
    //motor.driveToHeadingCorrected(driveAngle, 0, blockSpeed);
    driveSpeed = blockSpeed;
  }
  return driveAngle;

}

int sideBlock() {
  int driveDirection = 0;
  int k = 2;
  int backMove = abs(backSensor - 50);
  if (ballAngle > 180) driveDirection = 270 - backMove;
  else driveDirection = 90 + backMove;
  driveSpeed =  min(k * abs(yPos), MAX_SPEED);
  return driveDirection;
}


void goToLocation(int x, int y, int sensorData) { //assumes the bottom left corner is (0,0)
  int k = .7;
  if (sensorData == leftSensor) {
    float newBS = backSensor - y;
    float newLS = leftSensor - x;
    float m = (newBS) / (newLS);
    float driveAngle = angleWithSlope(m, newLS);
    //Serial.println(driveAngle);

    if (newBS < 15 && newLS > -10 && newLS < 10) {
      motor.stopMotors();
    }
    else motor.driveToHeadingCorrected(driveAngle, 0, min(MAX_SPEED, max(newBS, newLS)*max(newBS, newLS)*max(newBS, newLS)));
  }
  else {
    float newBS = backSensor - y;
    float newRS = FIELD_WIDTH - rightSensor - x;
    double m = (float)(newBS) / (float)(newRS);
    double driveAngle = atan((double)m);
    driveAngle *= 57.2957795129;
    driveAngle = angleWithSlope(driveAngle, newRS);

    if (newBS < 25 && newRS > -15 && newRS < 15) {
      motor.stopMotors();
    }
    else motor.driveToHeadingCorrected(driveAngle, 0, min(MAX_SPEED, max(newBS, newRS)*max(newBS, newRS)*max(newBS, newRS)));
  }
}


float angleWithSlope(float slope, int yDiff) {
  float angle = atan(slope);
  angle *= 57.2957795129;
  if (yDiff < 0) {
    angle = 180 - 90.0 - angle;
  }
  else angle = 369 - 90 - angle;
  return angle;
}
