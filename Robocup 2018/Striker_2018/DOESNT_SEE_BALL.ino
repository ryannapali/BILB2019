void doesnt_see_ball() {
  calculateAngle();
  if (ballAngle != 10000) { 
    currentState = SEES_BALL; //if sees ball at any point, switches state
  }
  else if (millis() - lastTimeSawBall <= 400 && spinningToBall == false) { //if the ball is in the front half of the robot when it loses it, immediately stops. If the ball is in the back half of the robot (meaning it is spinning in place) keeps spinning for 250ms to see if it sees it
    stopMotors();
    if (checkPossession() == true) {
      currentState = HAS_BALL;
      stopMotors();
    }
  }
  else if (millis() - lastTimeSawBall >= 400 && millis() - lastTimeSawBall <= 2000) { //stops motors no matter what after 250ms. Stays stopped looking for ball for 1.85 seconds
    stopMotors();
    checkPossession();
  }
  else if (millis() - lastTimeSawBall >= 2000 && millis() - lastTimeSawBall <= 3000) { //spins to direction while looking for ball for one second
    IMU_spinToDirection(g_goal);
  }
  else if (millis() - lastTimeSawBall >= 3000 && millis() - lastTimeSawBall <= 7000) { //tries to get back to center for 4 seconds
    updateDistances();
    int x = (rightDist >= leftDist) ? 1 : -1;
    int y = (frontDist >= backDist) ? 1 : -1;

    int lrdiff = abs(rightDist - leftDist);
    int fbdiff = abs(frontDist - backDist);

    boolean lrcentered = (lrdiff < 30 && rightDist + leftDist > 120); //if the difference between right and left is less than 30 and greater than 120 (not blocked by robot) means that it is centered
    boolean fbcentered = (fbdiff < 30 && frontDist + backDist > 120);

    if (lrcentered && fbcentered) {
      stopMotors();
    } else {
      driveToHeadingIMU(g_goal, xyToAngle(x, y), 200); //drive to center
    }

  }
  else if (millis() - lastTimeSawBall >= 7000 && millis()) {
    stopMotors(); //stop motors after trying to get to center for 4 seconds
  }
}

