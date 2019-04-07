#include "Motors.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();

Motors motor = Motors();

int DIR = 11;
int SLP = 24;
int PWM = 8;
int CS = 15;
float xTargetAngle = 180;
    
void setup() {
  pinMode(DIR, OUTPUT);  //DIR HIGH = A->B
  pinMode(SLP, OUTPUT);  //SLP Default = LOW. Must turn HIGH to run
  pinMode(PWM, OUTPUT);   //Analog PWM  0-255
  pinMode(CS, INPUT);   //Analog CS   0-1023
//  Serial.begin(115200);

  Serial.begin(9600);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
//  xOriginalAngle = euler.x();
}

float duration;
int cycles = 0;
void loop() {
  duration = millis()/1000.0 - cycles*2.0;

  Serial.println(duration);
  Serial.println(xTargetAngle);
  digitalWrite(DIR, HIGH);
  digitalWrite(SLP, HIGH);
  analogWrite(PWM, 255);

  
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  /* Display the floating point data */
  float xAngle = euler.x();
  if (xAngle > 180) {
    xAngle-=360;
  }
  float angleDiff = xAngle-xTargetAngle;
  int power = 1050*angleDiff/360;
  if (power < 10 and power > -10) {
    power = 0;
  } else if (power > 100) {
    power = 100;
  } else if (power < -100) {
    power = -100;
  }
//  motor.spin(int(560*angleDiff/360));

  if (duration < 5) {
    
  } else if (duration < 6.0) {
    motor.driveToHeading(0, 170);
  } else if (duration < 7.0) {
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
