#define INTERRUPT_PIN 39
#define IR_FRONT 20
#define DODGE_RADIUS 8.0
// Inches per second
#define SPEED_ESTIMATE 30.0
#define SOLENOID_PIN 27

#define TOTAL_DODGE_TIME 1000.0*DODGE_RADIUS*PI/SPEED_ESTIMATE

bool hasClearShot = false; 
float strafeStatus = 0;
bool shouldStrafe = false;

float frontDistIR = 0.0;
bool isExecutingDodge = false;
bool isExecutingDodgeOnGoalie = false;

float dodgeStartTime = 0.0;

bool turningToShoot = false;

void turnShoot() {
  if (abs(goalAngle) > 5.0) {
    if (goalAngle == 0.0) {
      motor.stopMotors();
    } else {
      motor.turnToRelativeHeading(goalAngle, 150);
    }
    motor.dribble(255);
  } else {
    motor.stopMotors();
    shoot();
    turningToShoot = false;
  }
}

float lastShootTime = 0.0;
bool shoot() {
  if (lastShootTime == 0) {
    lastShootTime = millis();
    motor.dribble(0);
    digitalWrite(SOLENOID_PIN, HIGH);
    delay(100);
    digitalWrite(SOLENOID_PIN, LOW);
    return true;
  } else if (millis() - lastShootTime > 1000) {
    lastShootTime = millis();
    motor.dribble(0);
    digitalWrite(SOLENOID_PIN, HIGH);
    delay(100);
    digitalWrite(SOLENOID_PIN, LOW);
    return true;
  } else {
    return false;
  }
}

bool lateralReadingsValid = false;
bool verticalReadingsValid = false;

float persistentGoalAngleForShow = 0.0;
float verticalDistance = 20.0;
void showAndShoot() {
  motor.dribble(0);

  if (isExecutingDodgeOnGoalie) {
//    executeDodgeOnGoalie();
    return;
  }
  
  frontSensor = lidars.readSensor1();
  backSensor = lidars.readSensor3();
  leftSensor = lidars.readSensor2();
  rightSensor = lidars.readSensor4();

  lateralReadingsValid = abs((leftSensor + rightSensor) - FIELD_WIDTH) < 8;
  verticalReadingsValid = abs((frontSensor + backSensor) - FIELD_LENGTH) < 10;
  
  Serial.println("");
  Serial.println(frontSensor);
  Serial.println(backSensor);
  Serial.println(verticalDistance);
  Serial.println(tPos);

  if (verticalReadingsValid) {
    verticalDistance = frontSensor - 31.0;
  } else {
    if (tPos != 0.0 and tPos > 200) {
      if (verticalDistance == 20.0) {
        verticalDistance = 25.0;
      }
    } else if (tPos != 0.0 and tPos < 200) {
      verticalDistance = 0.0;
    }
  }
    
  if (lateralReadingsValid) {
    if (verticalDistance < 0 or rightSensor < 25 or leftSensor < 25) {
      motor.stopMotors();
      delay(200);
      isExecutingDodgeOnGoalie = true;
    }
    if (leftSensor > rightSensor) {
      persistentGoalAngleForShow = atan(verticalDistance/(rightSensor-25.0));
      persistentGoalAngleForShow *= 57.2957795129;
      persistentGoalAngleForShow = 90 - persistentGoalAngleForShow;
    } else {
      persistentGoalAngleForShow = atan(verticalDistance/-(leftSensor-25.0));
      persistentGoalAngleForShow *= 57.2957795129;  
      persistentGoalAngleForShow = 270 - persistentGoalAngleForShow;
    }
  } else if (persistentGoalAngleForShow != 0.0) {
    motor.driveToHeadingCorrected(persistentGoalAngleForShow, 0.0, MAX_SPEED);
  } else {
    motor.driveToHeadingCorrected(0.0, 0.0, MAX_SPEED);
  }
}

void goBackwardsToShoot() {
  if (abs(motor.getRelativeAngle(180.0)) > 20.0 and turningToShoot == false) {
    motor.turnToAbsoluteHeading(180.0, 100);
  } else {
    if (tPos != 0 or turningToShoot) {
      if (tPos < -350 and turningToShoot == false) {
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

void dodgeIfNeeded() {
  frontDistIR = analogRead(IR_FRONT);
  if (frontDistIR >= 500 and isExecutingDodge == false) {
    isExecutingDodge = true;
    dodgeStartTime = millis();
  }
  if (isExecutingDodge) executeDodge();
}

bool hasRotated = false;
void executeDodgeOnGoalie(){
     // Dodging left? Probably need to do some turning w/out moving first
    Serial.println("dodging on goalie");
    if (abs(motor.getRelativeAngle(270.0)) > 3.0 and hasRotated == false) {
      motor.turnToAbsoluteHeading(270.0, 180);
      return;
    } else {
      hasRotated = true;
    }
    float fractionComplete = (millis()-dodgeStartTime)/TOTAL_DODGE_TIME;
    motor.driveToHeadingCorrected(180.0, 270.0-(1.0-fractionComplete)*180.0, MAX_SPEED);
    if (fractionComplete >= 1) {
      while (abs(motor.getRelativeAngle(0.0)) > 3.0) {
        motor.turnToAbsoluteHeading(0.0, 180);
      }
      bool shotSuccessful = shoot();
      if (shotSuccessful) {
        isExecutingDodgeOnGoalie = false;
      }
    }
}

void executeDodge(){
     // Dodging left? Probably need to do some turning w/out moving first
    Serial.println("dodging");
    float fractionComplete = (millis()-dodgeStartTime)/TOTAL_DODGE_TIME;
    motor.driveToHeadingCorrected(90.0, (1.0-fractionComplete)*180.0, MAX_SPEED);
    if (fractionComplete >= 1) {
      isExecutingDodge = false;
    }
}
