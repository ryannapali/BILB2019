void scoreGoal() {
  updateDistances();
  printDistances();
  boolean forwards = abs(IMU_calcError(g_goal)) < 90;
  if (checkPossession() == false) {
    setRGB(100, 100, 0);
    facingGoal = false;
    currentState = DOESNT_SEE_BALL;
    dribblerOff();
    clearCameraBuffer();
  }
  else if (forwards == true) {
    Serial6.print("FORWARD: ");
    setRGB(0, 255, 0);
    if (facingGoal == false) {
      Serial6.print(" SPINNING");
      dribblerIn();
      spinSlowCheckPossesion(g_goal);
      facingGoal = true;
      updateDistances();
    }
    if (frontDist + backDist > 200 && frontDist >= 40) { //clear in front and back direction
      straightForwardsShoot();
    }
    else { //not clear view in front or back
      if (frontDist < 40) { //blocked in front
        Serial6.println(" BLOCKED IN FRONT");
        dribblerIn();
        spinSlowCheckPossesion(g_goal + 180);
      }
      else { //not sure where you are just go forward
        Serial6.println(" NOT SURE WHERE YOU ARE");
        setDribbler(-20);
        driveToHeadingIMU(g_goal, 0, forwardSpeedWithBall);
      }
    }
  } //end of if facing forwards
  else { //facing backwards
    Serial6.print("BACKWARD: ");
    if (facingGoal == false) {
      Serial6.println(" SPINNING");
      setRGB(0, 255, 0);
      dribblerIn();
      spinSlowCheckPossesion(g_goal + 180);
      facingGoal = true;
      updateDistances();
    }
    if (frontDist + backDist > 200 && backDist >= 40) { //clear in front and back direction
      setRGB(0, 255, 0);
      straightBackwardsShoot();
    }
    else { //not clear in front or back direction
      setRGB(0, 255, 255);
      if (backDist < 40) {
        Serial6.println(" BLOCKED IN BACK");
        dribblerIn();
        setRGB(255, 0, 0);
        moveToOpenSide();
      }
      else { //not blocked in direction you are going, but blocked somewhere
        if (IRDistance() < 15) {
          Serial6.println(" BEING TRAILED");
          dribblerIn();
          spinSlowCheckPossesion(g_goal);
        }
        else { //not blocked in back so probably read the goalie in "not clear" reading
          Serial6.println(" NOT SURE WHERE YOU ARE");
          dribblerIn();
          setRGB(255, 0, 0);
          moveToOpenSide();
        }
      }
    }
  }
}

void moveToOpenSide() {
  checkWhichHemisphere();
  if (hemisphere == 'r') {
    driveToHeadingIMU(g_goal + 180, 90, sideSpeedWithBall);
  }
  else {
    driveToHeadingIMU(g_goal + 180, 270, sideSpeedWithBall);
  }
}

boolean checkPossession() {
  uint8_t range = vl.readRange();
  if (range < 50) {
    return true;
  }
  else return false;
}

void straightForwardsShoot() {
  if (frontDist > 90) {
    Serial6.println(" DRIVING STRAIGHT");
    setDribbler(-20);
    checkWhichHemisphere();
    driveToHeadingIMU(g_goal, 0, forwardSpeedWithBall);
  }
  else {
    dribblerIn();
    if (hemisphere == 'r') {
      Serial6.println(" SPINNING IN RIGHT HEMISPHERE");
      spinSlowCheckPossesion(g_goal - 45);

    }
    else if (hemisphere == 'l') {
      Serial6.println(" SPINNING IN LEFT HEMISPHERE");
      spinSlowCheckPossesion(g_goal + 45);
    }
    else {
      Serial6.println(" SPINNING IN CENTER HEMISPHERE");
      stopMotors();
    }
    Serial6.println(" SHOOOTTT");
    kick();
  }
}

void straightBackwardsShoot() {
  if (backDist > 90) {
    Serial6.println(" DRIVING STRAIGHT");
    dribblerIn();
    checkWhichHemisphere();
    driveToHeadingIMU(g_goal + 180, 180, backwardSpeedWithBall);
  }
  else {
    dribblerIn();
    if (hemisphere == 'r') { //actually closer to left side
      Serial6.println(" SPINNING IN LEFT HEMISPHERE");
      spinSlowCheckPossesion(g_goal - 90);
      spinSlowCheckPossesion(g_goal + 45);
    }
    else if (hemisphere == 'l') { //actually closer to right side
      Serial6.println(" SPINNING IN RIGHT HEMISPHERE");

      spinSlowCheckPossesion(g_goal + 90);
      spinSlowCheckPossesion(g_goal - 45);
    }
    else {
      Serial6.println(" SPINNING IN CENTER HEMISPHERE");
      if (rightDist > leftDist) {
        spinSlowCheckPossesion(g_goal + 90);
        spinSlowCheckPossesion(g_goal);
      }
      else {
        spinSlowCheckPossesion(g_goal - 90);
        spinSlowCheckPossesion(g_goal);
      }
    }
    Serial6.println(" SHOOOTTT");
    kick();
  }
}


void checkWhichHemisphere() {
  if ((rightDist + leftDist) > 150) { //if the side sensors add up to around the width of the field or greater
    if (rightDist > 80 && rightDist < 110) {
      hemisphere = 'c';
    }
    else if (rightDist < leftDist) { //robot is on the right side
      hemisphere = 'r';
    }
    else {
      hemisphere = 'l';
    }
  }
  //  else{
  //    hemisphere = 'n';
  //  }
}




