#include "Motors.h"

Motors motor = Motors();

void setup() {

}

void loop() {
  motor.driveToHeading(0, 100);
  
}
