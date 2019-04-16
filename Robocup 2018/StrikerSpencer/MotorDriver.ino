/*
  Maxon motors require a PWM frequency > 50kHz.  This needs to be changed on the Te1nsy
  using pins 3, 4, 29, 30
*/

// Set speed for motor 1, speed is a number between -1023 and 1023
// Set speed for motor 1, speed is a number between -1023 and 1023
void setM1Speed(int speed)
{
  speed = -speed;
  if (checkMotorSwitchOn() == true) {
    if (speed < 0) // if reverse is
    {
      digitalWrite(_M1DIR, HIGH);
      speed = -speed;
    }
    else
    {
      digitalWrite(_M1DIR, LOW);
    }
    analogWrite(_M1PWM, speed);
  }
  else {
    analogWrite(_M1PWM, 0);
  }
}

void setM2Speed(int speed)
{
  speed = -speed;
  if (checkMotorSwitchOn() == true) {
    if (speed < 0) // if reverse is
    {
      digitalWrite(_M2DIR, HIGH);
      speed = -speed;
    }
    else
    {
      digitalWrite(_M2DIR, LOW);
    }
    analogWrite(_M2PWM, speed);
  }
  else {
    analogWrite(_M2PWM, 0);
  }
}

void setM3Speed(int speed)
{
  speed = -speed;
  if (checkMotorSwitchOn() == true) {
    if (speed < 0) // if reverse is
    {
      digitalWrite(_M3DIR, HIGH);
      speed = -speed;
    }
    else
    {
      digitalWrite(_M3DIR, LOW);
    }
    analogWrite(_M3PWM, speed);
  }
  else {
    analogWrite(_M3PWM, 0);
  }
}
void setM4Speed(int speed)
{
  speed = -speed;
  if (checkMotorSwitchOn() == true) {
    if (speed < 0) // if reverse is
    {
      digitalWrite(_M4DIR, HIGH);
      speed = -speed;
    }
    else
    {
      digitalWrite(_M4DIR, LOW);
    }
    analogWrite(_M4PWM, speed);
  } else {
    analogWrite(_M4PWM, 0);
  }
}

void setSpeeds(int m1Speed, int m2Speed)
{
  setM1Speed(m1Speed);
  setM2Speed(m2Speed);
}


// Return M1 current value in milliamps
unsigned int getM1CurrentMilliamps()
{
  // 18v18 results in 244 mA per count.
  unsigned int mAPerCount = 244;
  int reading = (getM1CurrentReading() - _offsetM1) ;
  if (reading > 0)
  {
    return reading * mAPerCount;
  }
  return 0;
}

// Return M2 current value in milliamps
unsigned int getM2CurrentMilliamps()
{
  // 18v18 results in 244 mA per count.
  unsigned int mAPerCount = 244;
  int reading = (getM2CurrentReading() - _offsetM2) ;
  if (reading > 0)
  {
    return reading * mAPerCount;
  }
  return 0;
}

// Return M3 current value in milliamps
unsigned int getM3CurrentMilliamps()
{
  // 18v18 results in 244 mA per count.
  unsigned int mAPerCount = 244;
  int reading = (getM3CurrentReading() - _offsetM3) ;
  if (reading > 0)
  {
    return reading * mAPerCount;
  }
  return 0;
}

// Return M4 current value in milliamps
unsigned int getM4CurrentMilliamps()
{
  // 18v18 results in 244 mA per count.
  unsigned int mAPerCount = 244;
  int reading = (getM4CurrentReading() - _offsetM4) ;
  if (reading > 0)
  {
    return reading * mAPerCount;
  }
  return 0;
}

unsigned int getM1CurrentReading()
{
  return analogRead(_M1CS);
}

unsigned int getM2CurrentReading()
{
  return analogRead(_M2CS);
}

unsigned int getM3CurrentReading()
{
  return analogRead(_M3CS);
}

unsigned int getM4CurrentReading()
{
  return analogRead(_M4CS);
}

void spin(int mSpeed)
{
  setM1Speed(-mSpeed);
  setM2Speed(-mSpeed);
  setM3Speed(-mSpeed);
  setM4Speed(-mSpeed);

}



//Converts degrees to radians
float getRad(float angle) {
  return angle * 0.01745329251;
}

void stopMotors()
{
  setM1Speed(0);
  setM2Speed(0);
  setM3Speed(0);
  setM4Speed(0);

}

void dribblerIn() {
  digitalWrite(11, LOW);
  digitalWrite(32, HIGH);
  analogWrite(8, 255);
}
void dribblerOut() {
  digitalWrite(11, HIGH);
  digitalWrite(32, LOW);
  analogWrite(8, 255);
}

void dribblerOff() {
  digitalWrite(11, LOW);
  digitalWrite(32, LOW);
  analogWrite(8, 0);
}

void setDribbler(int speed) {
  if (speed > 0) {
    digitalWrite(11, LOW);
    digitalWrite(32, HIGH);
    analogWrite(8, speed);
  }
  else {
    digitalWrite(11, HIGH);
    digitalWrite(32, LOW);
    analogWrite(8, abs(speed));
  }
}

void driveToHeading(float angle, float speed) {
  float rad = getRad(angle);
  float proportionals[] = {sin(-rad + 3.92699082), sin(-rad + 5.28834763), sin(-rad + 0.994837674), sin(-rad + 2.35619449)};

  setM1Speed(-speed * proportionals[0]);
  setM2Speed(-speed * proportionals[1]);
  setM3Speed(-speed * proportionals[2]);
  setM4Speed(-speed * proportionals[3]);
}

void kick() {
  digitalWrite(31, HIGH);
  delay(120);
  digitalWrite(31, LOW);
  delay(500);
}

