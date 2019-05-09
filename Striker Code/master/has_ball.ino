#define INTERRUPT_PIN 39
#define IR_FRONT 20
#define DODGE_RADIUS 8
// Inches per second
#define SPEED_ESTIMATE 30
#define SOLENOID_PIN 27

#define TOTAL_DODGE_TIME DODGE_RADIUS*PI/SPEED_ESTIMATE

bool hasClearShot = false; 
float strafeStatus = 0;
bool shouldStrafe = false;

float frontDistIR = 0.0;
bool isExecutingDodge = false;
float dodgeStartTime = 0.0;

bool turningToShoot = false;

void turnShoot() {
  if (abs(goalAngle) > 3.0) {
    motor.turnToRelativeHeading(goalAngle, 100);
  } else {
    motor.stopMotors();
    delay(300);
    digitalWrite(SOLENOID_PIN, HIGH);
    delay(100);
    digitalWrite(SOLENOID_PIN, LOW);
    delay(1500);
    turningToShoot = false;
  }
}

void goBackwardsToShoot() {
  if (abs(motor.getRelativeAngle(180.0)) > 5.0 and turningToShoot == false) {
    motor.turnToAbsoluteHeading(180.0, 100);
  } else {
    if (tPos != 0) {
      if (tPos < -350) {
        motor.driveToHeadingCorrected(goalAngle, 180.0, 180);
      } else {
        turnShoot();
        turningToShoot = true;
      }
    } else {
      backSensor = lidars.readSensor3();
      if (backSensor > 20) {
        motor.driveToHeadingCorrected(180.0, 180.0, 180);
      }
    }
  }
}

//  frontDistIR = analogRead(IR_FRONT);
//  //Serial.println(frontDistIR);
//  if (frontDistIR >= 500 and isExecutingDodge == false) {
////    isExecutingDodge = true;
//    dodgeStartTime = millis();
//  }
//
//  frontSensor = lidars.readSensor1();
//  backSensor = lidars.readSensor3();
//  
//  if (frontSensor < 20 and frontSensor != -1 and backSensor > 65 and backSensor != 25801.18) {
//    shouldStrafe = true;
//  }
//
//  if (isExecutingDodge) {
////    executeDodge();
//  } else if (hasClearShot) {
//    motor.turnToHeadingGyro(goalAngle, MAX_SPEED);
//    digitalWrite(SOLENOID_PIN, HIGH);
//    delay(100);
//  } else if (shouldStrafe and strafeStatus == 0) {
//    leftSensor = lidars.readSensor2();
//    rightSensor = lidars.readSensor4();
//    if (rightSensor > leftSensor) {
//      strafeStatus = 1;
//    } else {
//      strafeStatus = -1;
//    }
//  } else if (shouldStrafe and strafeStatus == 1) {
//    motor.driveToHeadingCorrected(90.0, 0.0, MAX_SPEED);
//    
//    leftSensor = lidars.readSensor2();
//    rightSensor = lidars.readSensor4();
//    if (leftSensor < 15) {
//      strafeStatus = 1;
//    } else if (rightSensor < 15) {
//      strafeStatus = -1;
//    }
//  } else if (shouldStrafe and strafeStatus == -1) {
//    motor.driveToHeadingCorrected(270.0, 0.0, MAX_SPEED);
//    
//    leftSensor = lidars.readSensor2();
//    rightSensor = lidars.readSensor4();
//    if (leftSensor < 15 and leftSensor != -1) {
//      strafeStatus = 1;
//    } else if (rightSensor < 15 and rightSensor != -1) {
//      strafeStatus = -1;
//    }
//  }

void executeDodge(){
     // Dodging left? Probably need to do some turning w/out moving first
    Serial.println("dodging");
    float fractionComplete = (millis()-dodgeStartTime)/TOTAL_DODGE_TIME;
    motor.driveToHeadingCorrected(90.0, (1-fractionComplete)*180.0/180.0, MAX_SPEED);
    if (fractionComplete >= 1) {
      isExecutingDodge = false;
    }
}
