
void IMU_calibrate()
{
  /* Display calibration status for each sensor. */
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);

  if (gyro == 3 && mag == 3) setRGB(0, 255, 0);
  else if (gyro < 2 && mag < 2) setRGB(255, 0, 0);
  else if (gyro < 2 && mag == 3) setRGB(255, 0, 255);
  else if (mag < 2 && gyro == 3) setRGB(255, 255, 0);
  else setRGB(0, 0, 255);
  delay(100);
}

void IMU_GetReadings() {
  sensors_event_t event; //Get a new sensor event
  bno.getEvent(&event);
  g_xPos = event.orientation.x; //set X Position equal to xPos
}


/*
   Purpose:
      to calculate how far off the target angle the robot is pointing
   Parameters:
      float targetAngle    the angle you want the robot to point to
   Returns:
      float: how far off the robot is from the target angle
*/
float IMU_calcError(float targetAngle) {
  IMU_GetReadings();
  float diff = g_xPos - targetAngle; //how far off the target you are
  float e; //rounds/translates the differnce into a useable number
  if (diff > 180) e = -360 + diff; //calculates the error (degrees off goal)
  else if (diff < -180) e = 360 + diff; //different cases for whether error is on the left or right side
  else e = diff; //edge case when error is equal to difference
  g_error = e;
  return e;
}

/*
   Returns if facing goal
*/
boolean IMU_atGoal() {
  IMU_GetReadings();
  if (abs(IMU_calcError(g_goal)) < 30) {
    return true;
  }
  return false;
}

/*
   Chekcs if the momentary switch is hit to reset goal direction
*/
void IMU_checkIfResetGoal() {
  if (digitalRead(12) == LOW) {
    IMU_GetReadings();
    g_goal = g_xPos;
    delay(500);
  }
}

void IMU_spinToDirection(float targetDirection) {
  int k = 2;
  while (abs(IMU_calcError(targetDirection)) > 10) {
    spin(IMU_calcError(targetDirection) * k);
  }
  stopMotors();
}

/*
   Drives to heading with an IMU P(ID) But for this to work, robot needs
   to be oriented to a specific heading (eg: magnetic north, goal, etc.)
   This is the reason for third parameter "facing", so the robot can go
   to that heading facing a different direction.
*/

void driveToHeadingIMU(float facing, float angle, float speed) {
  int k = 1.5;
  IMU_GetReadings();
  float facingError = IMU_calcError(facing);
  float rad = getRad(angle);
  float proportionals[] = {sin(-rad + 3.92699082), sin(-rad + 5.28834763), sin(-rad + 0.994837674), sin(-rad + 2.35619449)};

  setM1Speed(speed * proportionals[0] - (facingError)*k);
  setM2Speed(speed * proportionals[1] - (facingError)*k);
  setM3Speed(speed * proportionals[2] - (facingError)*k);
  setM4Speed(speed * proportionals[3] - (facingError)*k);
}

void spinSlowCheckPossesion(float targetDirection) {
    int k = 2;
    while (abs(IMU_calcError(targetDirection)) > 10 && checkPossession() == true) {
      if (IMU_calcError(targetDirection)*k > 70) spin(70); //sets a max and min speed it can turn at
      else if (IMU_calcError(targetDirection)*k < -70) spin(-70);
      else spin(IMU_calcError(targetDirection)*k);
    }
    stopMotors();
    //clearCameraBuffer(); COMMENT THIS BACK IN
  }

