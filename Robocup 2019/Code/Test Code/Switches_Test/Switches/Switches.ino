int inPin = 12;   // input pin (for a pushbutton)
int DIP1 = 35;
int DIP2 = 36;
int DIP3 = 37;
int DIP4 = 38;
int r1 = 21;
int g1 = 22;
int b1 = 23;
int r2 = 9;
int g2 = 10;
int b2 = 14;
int wr = 17;
int wl = 28;


void setup() {
  pinMode(inPin, INPUT_PULLUP);    // declare pushbutton as input
  pinMode(DIP1, INPUT_PULLUP);
  pinMode(r1, OUTPUT);
  pinMode(g1, OUTPUT);
  pinMode(b1, OUTPUT);  
  pinMode(r2, OUTPUT);
  pinMode(g2, OUTPUT);
  pinMode(b2, OUTPUT);  
  pinMode(wr, OUTPUT);
  pinMode(wl, OUTPUT);
}

void loop() {
  DIPSwitch();
  pushButton();
}
void pushButton() {
  int val = 0;
  val = digitalRead(inPin);  // read input value
  if (val == HIGH) {         // check if the input is HIGH (button released)
    analogWrite(17, 255);
  } else {
    analogWrite(17, 0);
  }
}
void DIPSwitch() {
  int val1 = digitalRead(DIP1);
  int val2 = digitalRead(DIP2);
  int val3 = digitalRead(DIP3);
  int val4 = digitalRead(DIP4);
  if (val1 == LOW) { //if switch #1 is ON
    setColor(255,0,0,1);
  }
  else if (val2 == HIGH) { //if switch #2 is ON
    setColor(255,0,0,2);
  }
  else if (val3 == LOW) { //if switch #3 is ON
    setColor(0,255,0,1);
  }
  else if (val4 == HIGH) { //if switch #4 is ON
    setColor(0,255,0,2);
  }
  else {
    setColor(0,0,0,2);
    setColor(0,0,0,1);
  }

}
