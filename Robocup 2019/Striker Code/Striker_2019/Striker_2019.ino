#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <algorithm>
#include <math.h>
#include "Adafruit_VL6180X.h"
#include <SharpIR.h>

Adafruit_VL6180X vl = Adafruit_VL6180X();

#include "robotDefines.h"


void setup() {
  Serial.begin(115200);
  Serial2.begin(4800);
  Serial6.begin(9600); //xbee
  qtrInit();
  dribblerInit();
  MotorsInit();
  buttonInit();
  IMUInit();
  RGBLEDInit();
  LIDARinit();
  kickerInit();
  TOFInit();
  delay(1000);
  setGoalAndRunProgram();
  delay(1000);
  currentState = DOESNT_SEE_BALL; //initialize at this state because interrupt can be triggered while calibrating
}

void loop() {
  checkToSetGoal();
  switch (currentState) {
    case ON_LINE: //out of bounds
      setRGB(255, 0, 0);
      getInBounds();
      break;
    case SEES_BALL:
      goToBall(130);
      break;
    case DOESNT_SEE_BALL:
      setRGB(255, 0, 255);
      doesnt_see_ball();
      break;
    case HAS_BALL:
      scoreGoal();
      setRGB(255, 0, 255);
      break;
 }
}

void interrupt() {
  currentState = ON_LINE;
}
