int ledPin1 = 17;  // pin for the LED
int ledPin2 = 28;
int inPin = 12;   // input pin (for a pushbutton)
int DIP1 = 38;
int DIP2 = 37;
int DIP3 = 36;
int DIP4 = 35;

void setup() {
  pinMode(ledPin1, OUTPUT);  // declare LED as output
  pinMode(ledPin2, OUTPUT);  // declare LED as output
  pinMode(inPin, INPUT_PULLUP);    // declare pushbutton as input
  pinMode(DIP1, INPUT_PULLUP);
}

void loop() {
  DIPSwitch();
  pushButton();
}
void pushButton() {
  int val = 0;
  val = digitalRead(inPin);  // read input value
  if (val == HIGH) {         // check if the input is HIGH (button released)
    digitalWrite(ledPin1, LOW);  // turn LED OFF
  } else {
    digitalWrite(ledPin1, HIGH);  // turn LED ON
  }
}
void DIPSwitch() {
  int val1 = digitalRead(DIP1);
  int val2 = digitalRead(DIP2);
  int val3 = digitalRead(DIP3);
  int val4 = digitalRead(DIP4);
  if (val1 == LOW) {
    digitalWrite(ledPin1, HIGH);
  }
  if (val2 == LOW) {
    digitalWrite(ledPin2, HIGH);

  }
  if (val3 == LOW) {
    digitalWrite(ledPin1, HIGH);
    delay(50);
    digitalWrite(ledPin1, LOW);
    delay(50);
  }
  if (val4 == LOW) {
    digitalWrite(ledPin2, HIGH);
    delay(50);
    digitalWrite(ledPin2, LOW);
    delay(50);
  }
  else {
    digitalWrite(ledPin1, LOW);
    digitalWrite(ledPin2, LOW);
  }

}
