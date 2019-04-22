#include "Motors.h"
#include "LIDARS.h"

Motors motor = Motors();
LIDARS lidars = LIDARS();

#define MAX_SPEED 100
#define INTERRUPT_PIN 39
#define IR_FRONT 20
#define DODGE_RADIUS 8
// Inches per second
#define SPEED_ESTIMATE 30

#define TOTAL_DODGE_TIME DODGE_RADIUS*PI/SPEED_ESTIMATE

float speedScalar = MAX_SPEED/150.0;

float ballAngle;

float k = 20.0;

void setup() {
  // put your setup code here, to run once:
  Serial5.begin(19200);
  Serial.begin(115200);
  pinMode(INTERRUPT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), interrupt, RISING); //Interrupts when digitalpin rises from LOW to HIGH
}

bool interrupted = false;
void interrupt() {
  interrupted = true;
}

bool turnFixed = false;
float frontDistIR = 0.0;
bool isExecutingDodge = false;
float dodgeStartTime = 0.0;
void loop() {  
//  if (interrupted) {
//    fixOutOfBounds();
//    return;
//  }
  frontDistIR = analogRead(IR_FRONT);
  //Serial.println(frontDistIR);
  if (frontDistIR >= 500 and isExecutingDodge == false) {
    isExecutingDodge = true;
    dodgeStartTime = millis();
  }

  if (isExecutingDodge) {
   executeDodge();
  } else {
    if (abs(motor.getAdjustedAngle(0.0)) > 5) {
      motor.turnToHeadingGyro(0.0, MAX_SPEED);
    } else {
      Serial.println("driving");
      motor.driveToHeadingCorrected(0.0, 0.0, MAX_SPEED);
    }
  }
  motor.dribble();
}

void fixOutOfBounds() {
  if (abs(motor.getAdjustedAngle(0.0)) < 5 or turnFixed) {
    turnFixed = true;
    
    float frontSensor = lidars.readSensor1();
    float backSensor = lidars.readSensor3();
    float leftSensor = lidars.readSensor2() + 6.0;
    float rightSensor = lidars.readSensor4() + 6.0;

    float minReading = min(min(min(frontSensor, backSensor), leftSensor), rightSensor);

    if (leftSensor != 5 and rightSensor != 5 and backSensor != -1 and frontSensor != -1) {
      if (frontSensor <= minReading and frontSensor < 25) {
        motor.driveToHeadingCorrected(-180, 0, MAX_SPEED);
      } else if (backSensor <= minReading and backSensor < 25) {
        motor.driveToHeadingCorrected(0, 0, MAX_SPEED);
      } else if (rightSensor <= minReading and rightSensor < 25) {
        motor.driveToHeadingCorrected(270, 0, MAX_SPEED);
      } else if (leftSensor <= minReading and leftSensor < 25) {
        motor.driveToHeadingCorrected(90, 0, MAX_SPEED);
      } else {
        motor.stopMotors();
        interrupted = false;
        turnFixed = false;
      }
      Serial.print("front: ");
      Serial.println(frontSensor);
      Serial.print("right: ");
      Serial.println(rightSensor);
      Serial.print("back: ");
      Serial.println(backSensor);
      Serial.print("left: ");
      Serial.println(leftSensor);
    }
  } else {
    motor.turnToHeadingGyro(0.0, MAX_SPEED);
  }
}

void executeDodge(){
     // Dodging left? Probably need to do some turning w/out moving first
    Serial.println("dodging");
    float fractionComplete = (millis()-dodgeStartTime)/TOTAL_DODGE_TIME;
    motor.driveToHeadingCorrected(90.0, (1-fractionComplete)*180.0/180.0, MAX_SPEED);
    if (fractionComplete >= 1) {
      isExecutingDodge = false;
    }
}

