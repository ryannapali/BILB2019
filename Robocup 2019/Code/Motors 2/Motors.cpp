#include "Arduino.h"
#include "Motors.h"

Motors::Motors(){
	MotorsInit();
	buttonInit();
}

Motors::~Motors(){

}

void Motors::MotorsInit(){
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
  pinMode(31, OUTPUT); //dribbler init


  analogWriteFrequency(3, 58593);
  analogWriteFrequency(4, 58593);
  analogWriteFrequency(29, 58593);
  analogWriteFrequency(30, 58593);
}

void Motors::buttonInit() {
  pinMode(26, INPUT_PULLUP);
  pinMode(25, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
}

bool Motors::checkMotorSwitchOn() {
  if (digitalRead(26) == HIGH) return true;
  else {
    return false;
  }
}

void Motors::setM1Speed(int speed){
	speed = -speed;
  	if (checkMotorSwitchOn() == true) {
    	if (speed < 0) {
      		digitalWrite(_M1DIR, HIGH);
      		speed = -speed;
    	}
    	else digitalWrite(_M1DIR, LOW);
   		analogWrite(_M1PWM, speed);
  	}
  	else {
    	analogWrite(_M1PWM, 0);
  	}
}

void Motors::setM2Speed(int speed){
	speed = -speed;
 	if (checkMotorSwitchOn() == true) {
    	if (speed < 0) {
      		digitalWrite(_M2DIR, HIGH);
      		speed = -speed;
    	}
    	else digitalWrite(_M2DIR, LOW);
    	analogWrite(_M2PWM, speed);
  	}
  	else {
    	analogWrite(_M2PWM, 0);
  	}
}

void Motors::setM3Speed(int speed){
	speed = -speed;
  	if (checkMotorSwitchOn() == true) {
    	if (speed < 0) {
      		digitalWrite(_M3DIR, HIGH);
      		speed = -speed;
    	}
    else digitalWrite(_M3DIR, LOW);
    analogWrite(_M3PWM, speed);
  	}
  	else {
    	analogWrite(_M3PWM, 0);
  	}
}
void Motors::setM4Speed(int speed){
	speed = -speed;
  	if (checkMotorSwitchOn() == true) {
    	if (speed < 0) // if reverse is
    	{
      		digitalWrite(_M4DIR, HIGH);
      		speed = -speed;
    	}
    	else digitalWrite(_M4DIR, LOW);
    	analogWrite(_M4PWM, speed);
  	} 
  	else analogWrite(_M4PWM, 0);
}

void Motors::setSpeeds(int m1Speed, int m2Speed){
	setM1Speed(m1Speed);
	setM2Speed(m2Speed);
}

unsigned int Motors::getM1CurrentMilliamps(){
	  // 18v18 results in 244 mA per count.
  unsigned int mAPerCount = 244;
  int reading = (getM1CurrentReading() - _offsetM1) ;
  if (reading > 0)
  {
    return reading * mAPerCount;
  }
  return 0;
}

unsigned int Motors::getM2CurrentMilliamps(){
	  // 18v18 results in 244 mA per count.
  unsigned int mAPerCount = 244;
  int reading = (getM2CurrentReading() - _offsetM2) ;
  if (reading > 0)
  {
    return reading * mAPerCount;
  }
  return 0;
}

unsigned int Motors::getM3CurrentMilliamps(){
	  // 18v18 results in 244 mA per count.
  unsigned int mAPerCount = 244;
  int reading = (getM3CurrentReading() - _offsetM3) ;
  if (reading > 0)
  {
    return reading * mAPerCount;
  }
  return 0;
}

unsigned int Motors::getM4CurrentMilliamps(){
	  // 18v18 results in 244 mA per count.
  unsigned int mAPerCount = 244;
  int reading = (getM4CurrentReading() - _offsetM4) ;
  if (reading > 0)
  {
    return reading * mAPerCount;
  }
  return 0;
}

unsigned int Motors::getM1CurrentReading() {
  return analogRead(_M1CS);
}
unsigned int Motors::getM2CurrentReading() {
  return analogRead(_M2CS);
}
unsigned int Motors::getM3CurrentReading() {
  return analogRead(_M3CS);
}
unsigned int Motors::getM4CurrentReading() {
  return analogRead(_M4CS);
}

void Motors::spin(int mSpeed){
	setM1Speed(-mSpeed);
  	setM2Speed(-mSpeed);
  	setM3Speed(-mSpeed);
 	setM4Speed(-mSpeed);
}

float Motors::getRad(float angle) {
  return angle * 0.01745329251;
}

void Motors::stopMotors(){
	setM1Speed(0);
    setM2Speed(0);
    setM3Speed(0);
    setM4Speed(0);
}

void Motors::driveToHeading(float angle, float speed){
	float rad = getRad(angle);
  	float proportionals[] = {sin(-rad + 3.92699082), sin(-rad + 5.28834763), sin(-rad + 0.994837674), sin(-rad + 2.35619449)};

 	setM1Speed(-speed * proportionals[0]);
 	setM2Speed(-speed * proportionals[1]);
  	setM3Speed(-speed * proportionals[2]);
  	setM4Speed(-speed * proportionals[3]);
}

void Motors::driveToHeadingGyro(float angle, float speed){
	float rad = getRad(angle);
  	float proportionals[] = {sin(-rad + 3.92699082), sin(-rad + 5.28834763), sin(-rad + 0.994837674), sin(-rad + 2.35619449)};

 	setM1Speed(-speed * proportionals[0]);
 	setM2Speed(-speed * proportionals[1]);
  	setM3Speed(-speed * proportionals[2]);
  	setM4Speed(-speed * proportionals[3]);
}