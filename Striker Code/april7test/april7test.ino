#include "Motors.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

#define MAX_TURNING_POWER 100;
#define DEAD_ZONE 10;

Adafruit_BNO055 bno = Adafruit_BNO055();

Motors motor = Motors();

int DIR = 11;
int SLP = 24;
int PWM = 8;
int CS = 15;

float xTargetAngle = 180;
    
void setup() {
  /* Setup dribbler */
  pinMode(DIR, OUTPUT);  //DIR HIGH = A->B
  pinMode(SLP, OUTPUT);  //SLP Default = LOW. Must turn HIGH to run
  pinMode(PWM, OUTPUT);   //Analog PWM  0-255
  pinMode(CS, INPUT);   //Analog CS   0-1023

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

float duration;
int cycles = 0;
void loop() {
  duration = millis()/1000.0 - cycles*2.0;

  /* Dribble */
  digitalWrite(DIR, HIGH);
  digitalWrite(SLP, HIGH);
  analogWrite(PWM, 255);
  
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
  
  power = std::min(power, MAX_TURNING_POWER);
  power = std::max(power, -MAX_TURNING_POWER);

  /* Drive, turn, repeat */
  if (duration < 3.0 and duration > 2.0) {
    motor.driveToHeading(0, 170);
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
