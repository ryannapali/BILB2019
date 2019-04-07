#include <Wire.h>
#include "Adafruit_VL6180X.h"
Adafruit_VL6180X vl = Adafruit_VL6180X();

int DistIRFront = 20;
int DistIRBack = 16;

void setup() {
  Serial.begin(115200);

  while (!Serial) {
    delay(1);
  Serial.println("Adafruit VL6180x test!");
  if (! vl.begin()) {
    Serial.println("Failed to find sensor");
    while (1);
  }
  Serial.println("Sensor found!");
}
}
void loop() {
  DistIRs();
  TOF();
}
