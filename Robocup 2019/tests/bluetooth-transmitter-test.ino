#include <SoftwareSerial.h>
#include <sstream>
int rx = 31;
int tx = 32;
  int data =0;

SoftwareSerial BTserial(rx,tx);

int i;

void setup()  {
  delay(1000);
  BTserial.begin(9600); 
  Serial.begin(9600);
  Serial.println("HC-05 transmitting stuff");
  pinMode(12, INPUT_PULLUP);    // declare pushbutton as input

}
 
void loop() {
  int inPin = 12;   // input pin (for a pushbutton)
  int val=digitalRead(inPin);  // read input value
  if (val == HIGH) {         // check if the input is HIGH (button released)
    data = 0;
  } else {
    data =1;
  }
   BTserial.write(data);
   delay(10);
}
