#include <Wire.h>

uint16_t distance = 0; //distance
uint16_t strength = 0; // signal strength
uint8_t rangeType = 0; //range scale
/*Value range:
 00 (short distance)
 03 (intermediate distance)
 07 (long distance) */

boolean valid_data = false; //ignore invalid ranging data

const byte defaultsensor = 0x10; //TFMini I2C Address
const byte sensor1 = 0x12;
const byte sensor2 = 0x14;
const byte sensor3 = 0x16;
const byte sensor4 = 0x18;

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  delay(150);
  Serial.println("TFMini I2C Test");

//  Wire.beginTransmission(defaultsensor);
//  Wire.write(0x00);
//  Wire.write(0x26);
//  Wire.write(1); //Data length: 1
//  Wire.endTransmission(false);
  
//  Wire.beginTransmission(defaultsensor);
//  Wire.write(sensor4);
//  Wire.endTransmission(true);
  
//  Wire.beginTransmission(defaultsensor); 
//  Wire.write(0x06);
//  Wire.endTransmission(true);
//  delay(100);
}

//Write two bytes to a spot
boolean readDistance(uint8_t deviceAddress)
{
  Wire.beginTransmission(deviceAddress);
  Wire.write(0x01);
  Wire.write(0x02);
  Wire.write(7); //Data length: 7 bytes for distance data
  if (Wire.endTransmission(false) != 0) {
    return (false); //Sensor did not ACK
  }
  Wire.requestFrom(deviceAddress, (uint8_t)7); //Ask for 7 bytes

  if (Wire.available())
  {
    for (uint8_t x = 0 ; x < 7 ; x++)
    {
      uint8_t incoming = Wire.read();

      if (x == 0)
      {
        //Trigger done
        if (incoming == 0x00)
        {
          //Serial.print("Data not valid: ");//for debugging
          valid_data = false;
          //return(false);
        }
        else if (incoming == 0x01)
        {
          Serial.print("Data valid:     ");
          valid_data = true;
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
  else
  {
    Serial.println("No wire data avail");
    return (false);
  }

  return (true);
}

void loop()
{
  if (readDistance(sensor1) == true)
  {
    if (valid_data == true) {
      Serial.print("\tDist[");
      Serial.print(distance);
      Serial.print("]\tstrength[");
      Serial.print(strength);
      Serial.print("]\tmode[");
      Serial.print(rangeType);
      Serial.print("]");
      Serial.println("^^ Sensor 1");
    }
    else {
      //don't print invalid data
    }
  }
  else {
    Serial.println("Read fail");
  }
  delay(50); //Delay small amount between readings

if (readDistance(sensor2) == true)
  {
    if (valid_data == true) {
      Serial.print("\tDist[");
      Serial.print(distance);
      Serial.print("]\tstrength[");
      Serial.print(strength);
      Serial.print("]\tmode[");
      Serial.print(rangeType);
      Serial.print("]");
      Serial.println("^^ Sensor 2");
    }
    else {
      //don't print invalid data
    }
  }
  else {
    Serial.println("Read fail");
  }
  delay(50); //Delay small amount between readings
  if (readDistance(sensor3) == true)
  {
    if (valid_data == true) {
      Serial.print("\tDist[");
      Serial.print(distance);
      Serial.print("]\tstrength[");
      Serial.print(strength);
      Serial.print("]\tmode[");
      Serial.print(rangeType);
      Serial.print("]");
      Serial.println("^^ Sensor 3");
    }
    else {
      //don't print invalid data
    }
  }
  else {
    Serial.println("Read fail");
  }
  delay(50); //Delay small amount between readings

if (readDistance(sensor4) == true)
  {
    if (valid_data == true) {
      Serial.print("\tDist[");
      Serial.print(distance);
      Serial.print("]\tstrength[");
      Serial.print(strength);
      Serial.print("]\tmode[");
      Serial.print(rangeType);
      Serial.print("]");
      Serial.println("^^ Sensor 4");
    }
    else {
      //don't print invalid data
    }
  }
  else {
    Serial.println("Read fail");
  }
  delay(50); //Delay small amount between readings
}
