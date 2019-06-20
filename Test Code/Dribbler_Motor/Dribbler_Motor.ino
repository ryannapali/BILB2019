#include "Motors.h"
Motors motor = Motors();

int DIR = 11;
int SLP = 24;
int PWM = 8;
int CS = 15;

void setup() {
  pinMode(DIR, OUTPUT);  //DIR HIGH = A->B
  pinMode(SLP, OUTPUT);  //SLP Default = LOW. Must turn HIGH to run
  pinMode(PWM, OUTPUT);   //Analog PWM  0-255
  pinMode(CS, INPUT);   //Analog CS   0-1023
  Serial.begin(115200);
}

void loop() {
 digitalWrite(DIR, HIGH);
 digitalWrite(SLP, HIGH);
 analogWrite(PWM, 255);
 Serial.println(analogRead(CS));
 motor.driveToHeading(90, 100);
// motor.driveToHeadingCorrected(180.0, 180.0, MAX_BACK_SPEED);

}
