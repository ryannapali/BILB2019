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
#include <LIDARLite.h>

class LIDARS {
public:
    LIDARS();
    
    float readSensor1();
    float readSensor2();
    float readSensor3();
    float readSensor4();
    
private:
    void tcaselect(uint8_t i);
    
    LIDARLite myLidarLite;
};

#endif
