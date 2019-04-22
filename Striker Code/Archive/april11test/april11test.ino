#include "Motors.h"

#include <Wire.h>
#include<QTRSensors.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS 100
#define MAX_TURNING_POWER 100
#define DEAD_ZONE 10
#define INTERRUPT_PIN 39
unsigned char QTR_PINS[] = {A0, A1, A2, A3};

Adafruit_BNO055 bno = Adafruit_BNO055();

Motors motor = Motors();
float ballAngle;
float xTargetAngle = 180;
QTRSensorsAnalog qtrs = QTRSensorsAnalog(QTR_PINS, 4, 4, INTERRUPT_PIN);

void setup() {
//  Serial.begin(9600);
  Serial5.begin(19200);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialize the sensor */
  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);
  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
}

float xPos = 1;
float yPos = 1;
float tPos = 1;
float oPos = 1;

void clearCameraBuffer() {
  Serial5.clear();
}

void getCameraReadings() {

  char lc = Serial5.read();
  long bTimer = millis();
//  Serial.println(millis());
  while (word(0, lc) != 254) {
    lc = Serial5.read();
    if (bTimer + 400 < millis()) {
      clearCameraBuffer();
      bTimer = millis();
    }
  }
  while (Serial5.available() < 2) {
//    if (currentState == ON_LINE) break;
  }
  char highChar1 = Serial5.read();
  char lowChar1 = Serial5.read();
  while (Serial5.available() < 2) {
//    if (currentState == ON_LINE) break;
  }
  char highChar2 = Serial5.read();
  char lowChar2 = Serial5.read();
  while (Serial5.available() < 2) {
//    if (currentState == ON_LINE) break;
  }
  char highChar3 = Serial5.read();
  char lowChar3 = Serial5.read();
  while (Serial5.available() < 2) {
//    if (currentState == ON_LINE) break;
  }
  char highChar4 = Serial5.read();
  char lowChar4 = Serial5.read();
  
  // say what you got:
  xPos = word(highChar1, lowChar1);
  yPos = word(highChar2, lowChar2);
  tPos = word(highChar3, lowChar3);
  oPos = word(highChar4, lowChar4);
  Serial.print(xPos);
  Serial.print(" ");
  Serial.println(yPos);
}

float duration;
int cycles = 0;
int counter = 0;
void loop() {
  getCameraReadings();

  duration = millis()/1000.0 - cycles*2.0;
  motor.dribble();
  calculateAngle();
  spinToBall();
  unsigned int qtr_values[4];
  qtrs.read(qtr_values);

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  
  /* Make angle rolloverish adjustments by applying a rotational 
     transformation equal to the target angle to recenter the coordinate system */
  float xAngle = euler.x();
  float adjustedAngle = xAngle - xTargetAngle;
  if (adjustedAngle < 0) {
    adjustedAngle += 360;
  }
  if (adjustedAngle >= 180) {
    adjustedAngle -= 360;
  }

  /* Calculate required turning power */
  
}

void spinToBall() {
  float k = 1;
  motor.spin(ballAngle * k);
}

void calculateAngle() {
  // Only run this if you are in fact recieving x and y data. Otherwise, ballAngle does not change
  if (xPos > 1280 || yPos > 960) { //filter out and bad readings. 2000 is sign of bad readings
    ballAngle = 2000;
  } else {
    xPos = xPos - 640; //makes the center of the screen (640*480) 0 instead of having it be top left corner
    yPos = yPos - 480;

    xPos = xPos * -1;
    yPos = yPos * -1;
    double m = (float)(yPos) / (float)(xPos);
    ballAngle = atan((double)m);
    ballAngle = ballAngle * 57296 / 1000;
    if (xPos < 0 && yPos < 0) ballAngle = ballAngle + 180;
    else if (xPos > 0 && yPos < 0) ballAngle = ballAngle + 360;
    else if (xPos < 0 && yPos > 0) ballAngle = ballAngle + 180;

    //comment two lines out if orientation is flipped 180 degrees
    ballAngle = ballAngle + 180;
    if (ballAngle > 360) ballAngle = ballAngle - 360;

    ballAngle = ballAngle - 180;

    if (m == .75) { //needs to be at end so overrides any other calculations
      ballAngle = 10000; //ballAngle = 10000 when robot doesn't see ball
    }
  }
}
