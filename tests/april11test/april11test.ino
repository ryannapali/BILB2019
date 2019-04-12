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

float xTargetAngle = 180;
QTRSensorsAnalog qtrs = QTRSensorsAnalog(QTR_PINS, 4, 4, INTERRUPT_PIN);

void setup() {
  Serial.begin(9600);
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

float xPos;
float yPos;
void getCameraReadings() {
  if (Serial2.available() > 0) {
    char highChar1 = Serial2.read();
    char lowChar1 = Serial2.read();
    char highChar2 = Serial2.read();
    char lowChar2 = Serial2.read();
    // say what you got:
    xPos = word(highChar1, lowChar1);
    yPos = word(highChar2, lowChar2);
  }
}

float duration;
int cycles = 0;
void loop() {
  duration = millis()/1000.0 - cycles*2.0;
  motor.dribble();

  unsigned int qtr_values[4];
  qtrs.read(qtr_values);
  
//  Serial.println(qtr_values[0]);
  Serial.print(xPos);
  Serial.print(" ");
  Serial.println(yPos);
  //Serial.println(qtr_values[1]);
  //Serial.println(qtr_values[2]);
  //Serial.println(qtr_values[3]);

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
  int power = 1000.0*adjustedAngle/360.0;
  
  if (power < DEAD_ZONE and power > -DEAD_ZONE) {
    power = 0;
  }
  
  power = min(power, MAX_TURNING_POWER);
  power = max(power, -MAX_TURNING_POWER);

  /* Drive, turn, repeat */
  if (duration < 3.0 and duration > 2.0) {
    motor.driveToHeading(-adjustedAngle, 170);
  } else if (duration < 4.0) {
    motor.spin(power);
  } else {
    cycles+=1;
    xTargetAngle -= 180;
    if (xTargetAngle < 0) {
      xTargetAngle = 180;
    }
  }
//  delay(BNO055_SAMPLERATE_DELAY_MS);
}
