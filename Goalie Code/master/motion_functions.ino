float lastLeft = 0.0;
float lastRight = 0.0;

void centerToGoal() {
  backSensor = lidars.readSensor3();
  leftSensor = lidars.readSensor2();
  rightSensor = lidars.readSensor4();
  sideSum = leftSensor + rightSensor;

  lastLeft = leftSensor;
  lastRight = rightSensor;

  //Serial.println(leftSensor);

  float distanceFromGoal = 37.0;
  float centerDistance = 93.0;
  float distanceOff = backSensor - distanceFromGoal;

  sideSumConfident = (abs(sideSum - 185) < 10);
  if (sideSumConfident) {

  } else if (abs(lastLeft - leftSensor) < 4) {
    rightSensor = lastRight + lastLeft - leftSensor;
  } else if (abs(lastRight - rightSensor) < 4) {
    leftSensor = lastLeft + lastRight - rightSensor;
  } else {
    motor.stopMotors();
    return;
  }

  if (abs(motor.getRelativeAngle(0.0)) > 5) motor.turnToAbsoluteHeading(0.0, 200);
  else if (leftSensor - centerDistance > 2) motor.driveToHeadingCorrectedHoldDistance(270, 0, 5 * (leftSensor - centerDistance), distanceOff);
  else if (rightSensor - centerDistance > 2) motor.driveToHeadingCorrectedHoldDistance(90, 0, 3 * (rightSensor - centerDistance), distanceOff);
  else motor.stopMotors();
}


void getToBall() {
  int maxDistance = 83;
  int minDistance = 35;
  //frontSensor = lidars.readSensor1();
  backSensor = lidars.readSensor3();
  leftSensor = lidars.readSensor2();
  rightSensor = lidars.readSensor4();

  float distanceFromBall = sqrt(xPos * xPos + yPos * yPos);
  //Serial.println(distanceFromBall);
  int bufferBackDist = 100;//max(75, xPos/k);
  if (backSensor < 45) { //not 0 but 35
    motor.driveToHeadingCorrected(0, 0, 100 - backSensor);
  }
  else if (backSensor > bufferBackDist) { //not 900 but 75
    motor.driveToHeadingCorrected(180, 0, backSensor); //it goes back and a bit left but this should be proportional
  }
  //  else if (rightSensor + leftSensor > 170){
  //    if (rightSensor < 45) motor.driveToHeadingCorrected(270, 0, leftSensor);
  //    else if (leftSensor < 45) motor.driveToHeadingCorrected(90, 0, rightSensor);
  //  }
  else {
    int bufferZoneSize = 40; //degree in the back in which the robot does something else because going directly to the ball would result in an own goal
    if (backSensor > bufferBackDist - 20 && xPos > 20) { //(distanceFromBall < 200) || (ballAngle > 45 && ballAngle < 315)
      int k = 3;
      if (ballAngle < 180 && ballAngle > 5) {
        motor.driveToHeadingCorrected(90, 0, min(abs(yPos)*k, 255));
      }
      else if (ballAngle > 180 && ballAngle < 355) {
        motor.driveToHeadingCorrected(270, 0, min(abs(yPos)*k, 255));
      }
    }
    //    else if (backSensor < bufferZoneSize - 10) {
    //      Serial.println("goinfoward");
    //      motor.driveToHeadingCorrected(0, 0, MAX_SPEED - backSensor);
    //    }

    else {
      Serial.println("quadraticball");
      quadraticBall();
    }
  }
  //motor.turnToAbsoluteHeading(0, 180);}
}

void passBall() {
  return;
}

void quadraticBall() {
  int k = 10;
  float theta = -motor.getRelativeAngle(0.0) / 180.0 * PI;
  float rotatedYPos = cos(theta) * yPos - sin(theta) * xPos;
  float rotatedXPos = sin(theta) * yPos + cos(theta) * xPos;
  float velocityDirection = angleFromSlope(getPathSlope());
  float distanceFromBall = sqrt(xPos * xPos + yPos * yPos);

  if (ballAngle > 80 && ballAngle < 160) motor.driveToHeadingCorrected(160, 0.0, MAX_SPEED);

  else if (ballAngle > 200 && ballAngle < 280) motor.driveToHeadingCorrected(200, 0.0, MAX_SPEED);

  else if (ballAngle > k && ballAngle < 360 - k) {
    motor.driveToHeadingCorrected(velocityDirection, 0.0, min(MAX_SPEED, distanceFromBall));
  }
  else {
    motor.driveToHeadingCorrected(0, 0, min(MAX_SPEED, distanceFromBall));
  }
}



/*
 * Assumptions: We assume that the optimal position for the robot to be is the midpoint
 *              of the line containing the center point of the ball and the goal.
 * Returns: Whether or not the robot is in the optimal position.
 * Do: Calculates the angle the robot has to move at to get in between ball and goal.
 *     Moves to that angle
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
