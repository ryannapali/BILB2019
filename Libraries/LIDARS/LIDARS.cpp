//
//  LIDARS.cpp
//
//
//  Created by Tibor Rothschild on 4/12/19.
//

#include "LIDARS.h"
#include "Arduino.h"
#include <Wire.h>
#include <LIDARLite.h>

LIDARS::LIDARS() {
    Wire.begin();
    myLidarLite.begin(0, true);
    myLidarLite.configure(0);
}

void tcaselect(uint8_t i) {
    if (i > 7) return;
    
    Wire.beginTransmission(0x70); // TCA Address
    Wire.write(1 << i);
    Wire.endTransmission();
}

float LIDARS::readSensor1() {
    tcaselect(0);
    return myLidarLite.distance();
}

float LIDARS::readSensor2() {
    tcaselect(1);
    return myLidarLite.distance();
}

float LIDARS::readSensor3() {
    tcaselect(2);
    return myLidarLite.distance();
}

float LIDARS::readSensor4() {
    tcaselect(3);
    return myLidarLite.distance();
}
