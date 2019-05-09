void setup() {
pinMode(27, OUTPUT);
}

void loop() {
  digitalWrite(27, HIGH);
  delay(100);
  digitalWrite(27, LOW);
  delay(1000);
}
