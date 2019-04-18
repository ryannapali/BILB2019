void MotorsInit() {
  // Define pinMode for the pins and set the frequency for timers 1 and 2.
  pinMode(_M1PWM, OUTPUT);
  pinMode(_M1CS, INPUT);
  pinMode(_M1DIR, OUTPUT);
  pinMode(_M2PWM, OUTPUT);
  pinMode(_M2CS, INPUT);
  pinMode(_M2DIR, OUTPUT);
  pinMode(_M3PWM, OUTPUT);
  pinMode(_M3CS, INPUT);
  pinMode(_M3DIR, OUTPUT);
  pinMode(_M4CS, INPUT);
  pinMode(_M4DIR, OUTPUT);

  analogWriteFrequency(3, 58593);
  analogWriteFrequency(4, 58593);
  analogWriteFrequency(29, 58593);
  analogWriteFrequency(30, 58593);
}

void setGoalAndRunProgram() {
  while (digitalRead(12) == HIGH) IMU_calibrate();
  IMU_GetReadings(); //gets x position
  g_goal = g_xPos; //sets goal to x pos

}

void kickerInit() {
  pinMode(31, OUTPUT); //kciker init
}

void checkToSetGoal() {
  if (digitalRead(12) == LOW) {
    IMU_GetReadings(); //gets x position
    g_goal = g_xPos; //sets goal to x pos
  }
}

void buttonInit() {
  pinMode(26, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
}

void qtrInit() {
  pinMode(INTERRUPT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), interrupt, RISING); //Interrupts when digitalpin rises from LOW to HIGH
}

void IMUInit() {
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
}


void RGBLEDInit() {
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  digitalWrite(13, HIGH);
}

//LIDAR SETUP
void LIDARinit() {
 }

void initializeLidarDigPins(int numLidars, int lidarPinArray[]) {
  for (int i = 0; i < numLidars; i++) {
    pinMode(lidarPinArray[i], OUTPUT);
    digitalWrite(lidarPinArray[i], LOW);
  }
}

/*
   Purpose: All the LIDARs start off with the same I2C address, so we assign each LIDAR
   different addresses so they don't conflict.
   Parameters:
    int numLidars          number of LIDARs used
    int lidarPinArray[]    pins of LIDARs used
    char lidarI2cAdress[]  array of addresses to assign the LIDARs
   Return value:
    void
*/

/*
   Purpose: Helper function to assign individual LIDARs new addresses.
   Parameters:
    char newI2cAddress            address to be assigned
    char currentLidarLiteAddress  starting address of LIDARs
   Return value:
    void
*/

/*
   converts the x and y vector components to a usable angle
   you should be able to use simple trig to do this but for some reason i couldn't figure out to do it (thanks summer brain)

*/
int xyToAngle(int x, int y) {
  if (x == 1 && y == 0) {
    return 90;
  } else if (x == -1 && y == 0) {
    return 270;
  } else if (x == 0 && y == 1) {
    return 0;
  } else if (x == 1 && y == 1) {
    return 45;
  } else if (x == -1 && y == 1) {
    return 315;
  } else if (x == 0 && y == -1) {
    return 180;
  } else if (x == 1 && y == -1) {
    return 135;
  } else {
    return 225;
  }
}



void dribblerInit() {
  pinMode(11, OUTPUT);
  pinMode(32, OUTPUT);
  pinMode(8, OUTPUT);
}

void TOFInit() {
  if (! vl.begin()) {
    Serial.println("Failed to find sensor");
    while (1);
  }
}

boolean checkMotorSwitchOn() {
  if (digitalRead(26) == HIGH) {
    return true;
  }
  else {
    return false;
  }
}
