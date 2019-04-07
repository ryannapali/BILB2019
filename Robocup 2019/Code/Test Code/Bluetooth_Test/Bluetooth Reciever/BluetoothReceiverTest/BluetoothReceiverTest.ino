#include <SoftwareSerial.h>
int rx = 9;
int tx = 10;

SoftwareSerial BTserial(rx,tx);  //RX,TX
 
void setup()
{ 
  delay(1000);
  BTserial.begin(9600);
  Serial.begin(9600);
  Serial.println("HC-05 reading stuff");
}
 
void loop()
{
  // Read from HC-05 and send new data to Serial Monitor
  if (BTserial.available())
  {
    Serial.println(BTserial.read());
  }
}
