#include <SoftwareSerial.h>
#include <sstream>
int rx = 31;
int tx = 32;

SoftwareSerial BTserial(rx,tx);

int i;

void setup()  {
  delay(1000);
  BTserial.begin(9600); 
  Serial.begin(9600);
  Serial.println("HC-05 transmitting stuff");
}
 
void loop() {
  String str = "sending: ";
  Serial.println(str + i);
  BTserial.write(i);
  i++;
  if (i>255) {
    i=0;
  }
  delay(500);
}
