#define INTERRUPT_PIN   39

void setup() {
  Serial.begin(115200);
  pinMode(INTERRUPT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), interrupt, RISING); //Interrupts when digitalpin rises from LOW to HIGH
}

void loop() {
}

void interrupt() {
  Serial.println("On the line :)");
}
