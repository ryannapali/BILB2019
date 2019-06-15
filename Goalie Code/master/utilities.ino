

void ledWhite() {
  analogWrite(RED_PIN, 0);
  analogWrite(BLUE_PIN, 0);
  analogWrite(GREEN_PIN, 0);
}
void ledYellow() {
  analogWrite(RED_PIN, 255);
  analogWrite(BLUE_PIN, 0);
  analogWrite(GREEN_PIN, 0);
}
void ledCyan() {
  analogWrite(RED_PIN, 0);
  analogWrite(BLUE_PIN, 255);
  analogWrite(GREEN_PIN, 0);
}
void ledMagenta() {
  analogWrite(RED_PIN, 0);
  analogWrite(BLUE_PIN, 0);
  analogWrite(GREEN_PIN, 255);
}
void ledGreen() {
  analogWrite(RED_PIN, 255);
  analogWrite(BLUE_PIN, 255);
  analogWrite(GREEN_PIN, 0);
}
void ledBlue() {
  analogWrite(RED_PIN, 0);
  analogWrite(BLUE_PIN, 255);
  analogWrite(GREEN_PIN, 255);
}
void ledRed() {
  analogWrite(RED_PIN, 255);
  analogWrite(BLUE_PIN, 0);
  analogWrite(GREEN_PIN, 255);
}

void setupUtilities() {
  delay(600);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), interrupt, RISING); //Interrupts when digitalpin rises from LOW to HIGH

  pinMode(INTERRUPT_PIN, INPUT);
  pinMode(SOLENOID_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  for (int i = 0; i < 4; i++) {
    analogWrite(WHITEB_PIN, 255);
    analogWrite(WHITEA_PIN, 255);
    delay(100);
    analogWrite(WHITEB_PIN, 0);
    analogWrite(WHITEA_PIN, 0);
    delay(100);
  }
}

void setupTOF() {
  if (! vl.begin()) {
    Serial.println("Failed to find TOF sensor");
    analogWrite(28, 100);

    while (1);
  }
}

void setupIMU() {
  motor.imuInit();
  Serial.println("start imu");
  while (motor.isCalibrated() == false) analogWrite(WHITEB_PIN, 0);
  Serial.println("done imu");
  analogWrite(WHITEB_PIN, 255);
}

