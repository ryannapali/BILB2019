Adafruit_BNO055 bno = Adafruit_BNO055();
uint8_t sys, gyro, accel, mag;
void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

void IMUInit() {
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
}

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
