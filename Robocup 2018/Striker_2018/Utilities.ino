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
  initializeLidarDigPins(numLidars, lidarPinArray);
  lidarChangeMultAddress(numLidars, lidarPinArray, lidarI2cAdress);
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
void lidarChangeMultAddress(int numLidars, int lidarPinArray[], char lidarI2cAdress[]) {
  for (int i = 0; i < numLidars; i++) {
    if (i != 4) {
      digitalWrite(lidarPinArray[i], HIGH);
      delay(200);
      myLidarLite.begin(0, true, 0x62); // Set configuration to default and I2C to 400 kHz
      lidarChangeAddress(lidarI2cAdress[i], 0x62);
      myLidarLite.configure(0, lidarI2cAdress[i]); // Change this number to try out alternate configurations
    }
  }
  for (int i = 0; i < numLidars; i++) {
    digitalWrite(lidarPinArray[i], HIGH);
  }
}

/*
   Purpose: Helper function to assign individual LIDARs new addresses.
   Parameters:
    char newI2cAddress            address to be assigned
    char currentLidarLiteAddress  starting address of LIDARs
   Return value:
    void
*/

void lidarChangeAddress(char newI2cAddress, char currentLidarLiteAddress) {
  if (newI2cAddress != currentLidarLiteAddress) {
    unsigned char serialNumber[2];
    //  Read two bytes from 0x96 to get the serial number
    myLidarLite.read(0x96, 2, serialNumber, false, currentLidarLiteAddress);
    //  Write the low byte of the serial number to 0x18
    myLidarLite.write(0x18, serialNumber[0], currentLidarLiteAddress);
    //  Write the high byte of the serial number of 0x19
    myLidarLite.write(0x19, serialNumber[1], currentLidarLiteAddress);
    //  Write the new address to 0x1a
    myLidarLite.write(0x1a, newI2cAddress, currentLidarLiteAddress);
    myLidarLite.write(0x1e, 0x08, currentLidarLiteAddress);
  }
}


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
