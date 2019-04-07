#include <SoftwareSerial.h>

//Change these to whatever pins you're using
int rx = 31;
int tx = 32;
SoftwareSerial BTserial(rx, tx); // RX, TX

char reading = ' ';
boolean BTconnected = false;

void setup(){
  pinMode(rx, OUTPUT);
  digitalWrite(rx, HIGH);

    //Serial turns on in 1 second. Be patient and wait.
  delay(1000);

  // wait until the HC-05 has made a connection
  while (!BTconnected) {
    if (digitalRead(rx) == HIGH) { BTconnected = true; };
  }

  Serial.begin(9600);
  Serial.println("HC-05 is now connected");
  Serial.println("");

  // Start serial communication with the bluetooth module
  // HC-05 default serial speed: 9600 or 38400
  Serial.println("Enter AT commands:");
  BTserial.begin(38400);  // HC-05 default speed in AT command mode
}

void loop()
{
  // Keep reading from HC-05 and send to Arduino Serial Monitor
  if (BTserial.available())
  {
    reading = BTserial.read();
    Serial.write( reading );
  }

  // Keep reading from Arduino Serial Monitor and send to HC-05
  if (Serial.available())
  {
    reading = Serial.read();
    BTserial.write( reading );
  }
}
