#include "Motors.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS 100
#define MAX_TURNING_POWER 100
#define DEAD_ZONE 10
#define INTERRUPT_PIN 39

Adafruit_BNO055 bno = Adafruit_BNO055();

Motors motor = Motors();
float ballAngle;
float xTargetAngle = 180;

void setup() {
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
  }
  char highChar1 = Serial5.read();
  char lowChar1 = Serial5.read();
  while (Serial5.available() < 2) {
  }
  char highChar2 = Serial5.read();
  char lowChar2 = Serial5.read();
  while (Serial5.available() < 2) {
  }
  char highChar3 = Serial5.read();
  char lowChar3 = Serial5.read();
  while (Serial5.available() < 2) {
  }
  char highChar4 = Serial5.read();
  char lowChar4 = Serial5.read();
  
  // say what you got:
  xPos = word(highChar1, lowChar1) - 640;
  yPos = word(highChar2, lowChar2)-480;
  tPos = word(highChar3, lowChar3); //goalie
  oPos = word(highChar4, lowChar4);
//  Serial.print(xPos);
//  Serial.print(" ");
//  Serial.print(yPos);
}

float duration;
int cycles = 0;
int counter = 0;
void loop() {
  getCameraReadings();
  duration = millis()/1000.0 - cycles*2.0;
  calculateAngle();
  spinToBall();

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
  calculateDist();
  
}

void spinToBall() {
  float k = .85;
  if(ballAngle<190){
      motor.spin(ballAngle * k);
  }
}

void calculateAngle() {
  // Only run this if you are in fact recieving x and y data. Otherwise, ballAngle does not change
  if (xPos > 1280 || yPos > 960) { //filter out and bad readings. 2000 is sign of bad readings
    ballAngle = 2000;
  } else {
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

void calculateDist(){
  float distx;
  float disty;
  Serial.print(xPos);
  Serial.print(yPos);
  
  //distx = (pow(abs(xPos), 2.23)+xPos)/14500 + 7; //https://www.desmos.com/calculator/fdhuaqxbo8
  //disty = (pow(abs(yPos), 3.3))/1000000+1.2; //https://www.desmos.com/calculator/b4pcna6bd1
  //Serial.println(pow(pow(distx, 2) + pow(disty,2),.5));

  
}

