#ifndef MOTORS_H
#define MOTORS_H

#include "Arduino.h"
#include <MiniPID.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

class Motors {
public:
    Motors();
    ~Motors();
    
    void imuInit();
    
    void setM1Speed(int speed);
    void setM2Speed(int speed);
    void setM3Speed(int speed);
    void setM4Speed(int speed);
    
    void setSpeeds(int m1Speed, int m2Speed);
    
    unsigned int getM1CurrentMilliamps();
    unsigned int getM2CurrentMilliamps();
    unsigned int getM3CurrentMilliamps();
    unsigned int getM4CurrentMilliamps();
    
    unsigned int getM1CurrentReading();
    unsigned int getM2CurrentReading();
    unsigned int getM3CurrentReading();
    unsigned int getM4CurrentReading();
    
    void spin(int mSpeed);
    
    float getRad(float angle);
    
    void stopMotors();
    
    void dribble(float power);
    
    void driveToHeading(float angle, float speed);
    
    void driveToHeadingCorrected(float angle, float targetOrientation, float speed);
    
    void turnToAbsoluteHeading(float targetAngle, float maxSpeed);
    
    void turnToRelativeHeading(float targetAngle, float maxSpeed);
    
    float getRelativeAngle(float targetAngle);
private:
    void MotorsInit();
    void buttonInit();
    
    bool checkMotorSwitchOn();
    
    Adafruit_BNO055 bno;
    
    unsigned int _offsetM1;
    unsigned int _offsetM2;
    unsigned int _offsetM3;
    unsigned int _offsetM4;
    
    static const unsigned char _M1PWM = 3;
    static const unsigned char _M2PWM = 4;
    static const unsigned char _M3PWM = 29;
    static const unsigned char _M4PWM = 30;
    
    static const unsigned char DEAD_POWER_ZONE = 10;
    static const unsigned char DEAD_ANGLE_ZONE = 3;
    
    unsigned char _M1DIR = 2;
    unsigned char _M2DIR = 5;
    unsigned char _M3DIR = 6;
    unsigned char _M4DIR = 7;
    
    unsigned int _MTRSLP = 24;
    
    unsigned int _DBDIR = 11;
    unsigned int _DBPWM = 8;
    unsigned int _DBCS = 15;
    
    unsigned char _M1CS = A15;
    unsigned char _M2CS = A17;
    unsigned char _M3CS = A18;
    unsigned char _M4CS = A19;
    
    int max_speed = 255;
    int pause = 10;
    
};

#endif
