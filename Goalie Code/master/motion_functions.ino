void getToBall() {
  //dimensions for the protected zone
  int maxDistance = 100;
  int minDistance = 45;

  //frontSensor = lidars.readSensor1();
  backSensor = lidars.readSensor3();
  leftSensor = lidars.readSensor2();
  rightSensor = lidars.readSensor4();

  if (backSensor < minDistance) {
    motor.driveToHeadingCorrected(0, 0, MAX_SPEED - backSensor);
  }
  else if (backSensor > maxDistance) {
    motor.driveToHeadingCorrected(180, 0, backSensor);
  }
  else {
    //if the robot is on the front of the protected zone then just go right and left to defend the robot.
    //this should probably be the optimal position, but limited to the protected zone.
    if (backSensor > maxDistance - 20 && xPos > 20) {
      int k = 3;
      int left = 0;
      if (ballAngle < 180 && ballAngle > 5) left = 90;
      else if (ballAngle > 180 && ballAngle < 355) left = 270; //if ball is on the left then move 270 degrees instead of 90.
      motor.driveToHeadingCorrected(left, 0, min(abs(yPos)*k, 255));
    }
    else if (backSensor < maxDistance - 10) { //goes forward if the ball is infront of it and it is within the protected zone.
      motor.driveToHeadingCorrected(0, 0, MAX_SPEED - backSensor);
    }
    else { //if the ball is in the protected zone we have to get to the ball.
      quadraticBall();
    }
  }
}

void quadraticBall() {
  int k = 10;
  float theta = -motor.getRelativeAngle(0.0) / 180.0 * PI;
  float rotatedYPos = cos(theta) * yPos - sin(theta) * xPos;
  float rotatedXPos = sin(theta) * yPos + cos(theta) * xPos;
  float velocityDirection = 0;
  float distanceFromBall = sqrt(xPos * xPos + yPos * yPos);
  int robotSpeed = MAX_SPEED;

  if (ballAngle > 80 && ballAngle < 160) {
    velocityDirection = 160;
  }
  else if (ballAngle > 200 && ballAngle < 280) {
    velocityDirection = 200;
  }
  else if (ballAngle > k && ballAngle < 360 - k) {
    robotSpeed = min(MAX_SPEED, distanceFromBall);
    velocityDirection = angleFromSlope(getPathSlope());
  }
  else {
    robotSpeed = min(MAX_SPEED, distanceFromBall);
    velocityDirection = 0;
  }
  motor.driveToHeadingCorrected(velocityDirection, 0.0, robotSpeed);
}



/*
   Assumptions: We assume that the optimal position for the robot to be is the midpoint
                of the line containing the center point of the ball and the goal.
   Returns: Whether or not the robot is in the optimal position.
   Do: Calculates the angle the robot has to move at to get in between ball and goal.
       Moves to that angle
*/

boolean optimalPosition() {
  int newBA = ballAngle;
  int newGA = goalAngle;
  //gets rid of all numbers more than 180
  if (newBA >= 180) newBA -= 360;
  if (newGA >= 180) newGA -= 360;

  //calculates distances
  float goalDist = sqrt(tPos * tPos + oPos * oPos);
  float ballDist = sqrt(xPos * xPos + yPos * yPos);
  float h;

  //calculates angle (h) that is necessary for the calculations to find the final angle
  if (newGA / abs(newGA) == newBA / abs(newBA)) h = abs(abs(newGA) - abs(newBA));
  else h = abs(newGA) + abs(newBA);


  //calculates driveAngle by some wonko math.
  float g = goalDist * cos(h) + ballDist;
  float f = sqrt(sq(ballDist) + sq(goalDist) + (2 * ballDist * goalDist * cos(h)));
  float driveAngle = acos(g / f);

  //if it should drive left, than it converts driveAngle into a number more than 180.
  if ((180 - abs(newBA) > abs(newGA)) || (newBA < 0 && newGA < 0)) driveAngle = 360 - driveAngle;

  //the proportional speed for it moving to the right point.
  int blockSpeed = min(MAX_SPEED, max(abs(abs(newBA) - abs(newGA)), abs(abs(goalDist) - abs(ballDist))));

  Serial.printf("Drive Angle: ", driveAngle, "     Block speed ", blockSpeed);

  //return statements below. If it is in the right place than good, otherwise move.
  if (abs(goalDist - ballDist) < 10 && (abs(newBA) + abs(newGA) > 170) && (abs(newBA) + abs(newGA) < 190)) {
    motor.stopMotors();
    return true;
  }
  else {
    motor.driveToHeadingCorrected(driveAngle, 0, blockSpeed);
    return false;
  }
}
