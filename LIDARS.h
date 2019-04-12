//
//  LIDARS.h
//  
//
//  Created by Tibor Rothschild on 4/12/19.
//

#ifndef LIDARS_H
#define LIDARS_H

#include <stdio.h>
#include "Arduino.h"
#include <Wire.h>

class LIDARS {
public:
    LIDARS();
    float readSensor1();
    float readSensor2();
    float readSensor3();
    float readSensor4();
    
private:
    const byte DEFAULT_SENSOR = 0x10; //TFMini I2C Address
    
    uint16_t distance = 0; //distance
    uint16_t strength = 0; // signal strength
    uint8_t rangeType = 0; //range scale
    
    const byte SENSOR_1 = 0x12;
    const byte SENSOR_2 = 0x14;
    const byte SENSOR_3 = 0x16;
    const byte SENSOR_4 = 0x18;
};

#endif
