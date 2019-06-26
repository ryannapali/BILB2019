#define NUMSENSORS 4
int pins[NUMSENSORS] = {A0, A1, A2, A3};
void setup() {
  Serial.begin(115200);
  pinMode(4, OUTPUT);     
  pinMode(13, OUTPUT);     
}

void loop() {
//  Serial.print(analogRead(A0));//left
//  Serial.print("    ");
//  Serial.print(analogRead(A1)); //right
//  Serial.print("    ");
//  Serial.print(analogRead(A2));//front
//  Serial.print("    ");
//  Serial.print(analogRead(A3)); //back
//  Serial.println(" ");

  if (min(min(min(analogRead(A0), analogRead(A1)), analogRead(A2)), analogRead(A3)) < 100) {
    digitalWrite(4, HIGH);
    digitalWrite(13, HIGH);
  } else {
    digitalWrite(4, LOW);
    digitalWrite(13, LOW);
  }
}
