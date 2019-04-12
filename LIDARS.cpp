//
//  LIDARS.cpp
//  
//
//  Created by Tibor Rothschild on 4/12/19.
//

#include "LIDARS.h"
#include "Arduino.h"
#include <Wire.h>

LIDARS::LIDARS() {
    Wire.begin();
}

float readSensor(uint8_t deviceAddress) {
    Wire.beginTransmission(deviceAddress);
    Wire.write(0x01);
    Wire.write(0x02);
    Wire.write(7); //Data length: 7 bytes for distance data
    if (Wire.endTransmission(false) != 0) {
        return -1; //Sensor did not ACK
    }
    Wire.requestFrom(deviceAddress, (uint8_t)7); //Ask for 7 bytes
    
    if (Wire.available()) {
        for (uint8_t x = 0 ; x < 7 ; x++) {
            uint8_t incoming = Wire.read();
            
            if (x == 0) {
                //Trigger done
                if (incoming == 0x00) {
                    return -1;
                }
                else if (incoming == 0x01) {
                    // Data is valid
                }
            }
            else if (x == 2)
                distance = incoming; //LSB of the distance value "Dist_L"
            else if (x == 3)
                distance |= incoming << 8; //MSB of the distance value "Dist_H"
            else if (x == 4)
                strength = incoming; //LSB of signal strength value
            else if (x == 5)
                strength |= incoming << 8; //MSB of signal strength value
            else if (x == 6)
                rangeType = incoming; //range scale
        }
    }
    else {
        // No wire data available
        return -1;
    }
    
    return distance/2.54;
}

float readSensor1() {
    return readSensor(SENSOR_1);
}

float readSensor2() {
    return readSensor(SENSOR_2);
}

float readSensor3() {
    return readSensor(SENSOR_3);
}

float readSensor4() {
    return readSensor(SENSOR_4);
}
