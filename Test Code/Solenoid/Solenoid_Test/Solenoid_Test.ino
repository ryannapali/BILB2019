#define BUTTON_PIN 12

void setup() {
  pinMode(27, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void loop() {
  int val = 0;
  val = digitalRead(BUTTON_PIN);
  if (val == LOW) {
    digitalWrite(27, HIGH);
    delay(100);
    digitalWrite(27, LOW);
    delay(1500);
  }
}
