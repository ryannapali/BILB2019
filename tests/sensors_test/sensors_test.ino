#include "Motors.h"
#include "LIDARS.h"

#include <Wire.h>
#include <QTRSensors.h>
//#include "Adafruit_VL6180X.h"

#define INTERRUPT_PIN 39

#define IR_FRONT 20
#define IR_BACK 16

Adafruit_VL6180X vl = Adafruit_VL6180X();

Motors motor = Motors();
LIDARS lidars = LIDARS();

float ballAngle;

void setup() {
  Serial5.begin(19200);
  
  pinMode(INTERRUPT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), interrupt, RISING); //Interrupts when digitalpin rises from LOW to HIGH
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
  
  xPos = word(highChar1, lowChar1);
  yPos = word(highChar2, lowChar2);
  tPos = word(highChar3, lowChar3);
  oPos = word(highChar4, lowChar4);
}

void loop() {
  getCameraReadings();

//  Serial.print("Adjusted angle: ");
//  Serial.println(motor.getAdjustedAngle(0));
//  calculateAngle();
//  Serial.println(ballAngle);
//  Serial.print("LIDAR 1 distance: ");
//  Serial.println(lidars.readSensor1());
//  Serial.print("LIDAR 2 distance: ");
//  Serial.println(lidars.readSensor2());
//  Serial.print("IR distances: ");
//  Serial.println(analogRead(IR_FRONT));
//  Serial.print("   ");
//  Serial.println(analogRead(IR_BACK));
//  Serial.print("Dribbler TOF: ");
//  Serial.println(vl.readRange());
//  Serial.print("Ball position: ");
//  Serial.print(xPos);
//  Serial.print("   ");
//  Serial.println(yPos);
}

void interrupt() {
  // Slave is delayed due to other errands
  Serial.println("                      On the line :)");
}

void calculateAngle() {
  // Only run this if you are in fact recieving x and y data. Otherwise, ballAngle does not change
  if (xPos > 1280 || yPos > 960) { //filter out and bad readings. 2000 is sign of bad readings
    ballAngle = 2000;
  } else {
    double m = (float)(yPos) / (float)(xPos);
    ballAngle = atan((double)m);
    ballAngle *= 57.2957795129;
    if (xPos < 0) {
      ballAngle += 180;
    } else if (yPos < 0) {
      ballAngle += 360;
    }

    if (m == .75) {
      ballAngle = 10000; //ballAngle = 10000 when robot doesn't see ball
    }
  }
}
