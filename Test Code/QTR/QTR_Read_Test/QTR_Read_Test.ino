#define NUMSENSORS 4
int pins[NUMSENSORS] = {A0, A1, A2, A3};
void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.print(analogRead(A0));//left
  Serial.print("    ");
  Serial.print(analogRead(A1)); //right
  Serial.print("    ");
  Serial.print(analogRead(A2));//front
  Serial.print("    ");
  Serial.print(analogRead(A3)); //back
  Serial.println(" ");
}
