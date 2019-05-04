#include <Wire.h>

#include <Adafruit_BNO055.h>

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


int r1 = 21;
int g1 = 22;
int b1 = 23;
int r2 = 9;
int g2 = 10;
int b2 = 14;
int wr = 17;
int wl = 28;
Adafruit_BNO055 bno;

void setup()
{
  pinMode(r1, OUTPUT);
  pinMode(g1, OUTPUT);
  pinMode(b1, OUTPUT);  
  pinMode(r2, OUTPUT);
  pinMode(g2, OUTPUT);
  pinMode(b2, OUTPUT);  
  pinMode(wr, OUTPUT);
  pinMode(wl, OUTPUT);
  
  Wire.begin();
  Serial.begin(9600);
  delay(150);
  Serial.println("TFMini I2C Test");

    bno = Adafruit_BNO055();
        if(!bno.begin()) {
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }
    
    delay(100);
    bno.setExtCrystalUse(true);
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
//          Serial.print("Data valid:     ");
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
//      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
//    Serial.println(euler.x());
  if (readDistance(sensor1) == true)
  {
    if (valid_data == true) {
      Serial.print("sensor 1 ");
      Serial.println(distance);
      if(distance==65533) analogWrite(wr,255);
      else analogWrite(wr,0);
    }
    else {
      //don't print invalid data
    }
  }
  else {
    Serial.println("Read fail");
  }
  delay(5); //Delay small amount between readings

if (readDistance(sensor2) == true)
  {
    if (valid_data == true) {
      Serial.print("sensor 2 ");
      Serial.println(distance);
      if(distance==65533) analogWrite(wl,255);
      else analogWrite(wl,0);
    }
    else {
      //don't print invalid data
    }
  }
  else {
    Serial.println("Read fail");
  }
  delay(5); //Delay small amount between readings
  if (readDistance(sensor3) == true)
  {
    if (valid_data == true) {
      Serial.print("sensor 3 ");
      Serial.println(distance);
      if(distance==65533) analogWrite(r1,255);
      else analogWrite(r1,0);
    }
    else {
      //don't print invalid data
    }
  }
  else {
    Serial.println("Read fail");
  }
  delay(5); //Delay small amount between readings

if (readDistance(sensor4) == true)
  {
    if (valid_data == true) {
      Serial.print("sensor 4 ");
      Serial.println(distance / 2.54);
      if(distance==65533) analogWrite(r2,255);
      else analogWrite(r2,0);
    }
    else {
      //don't print invalid data
    }
  }
  else {
    Serial.println("Read fail");
  }
  delay(5); //Delay small amount between readings
}
