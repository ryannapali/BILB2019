/*
Adafruit Arduino - Lesson 3. RGB LED
*/

//Right RGB (R,G,B) (21, 22, 23)
//Left  RGB (R,G,B) ( 9, 10, 14)
//White Light Right 17
//White Light Left  28

int r1 = 21;
int g1 = 22;
int b1 = 23;
int r2 = 9;
int g2 = 10;
int b2 = 14;
int wr = 17;
int wl = 28;

 
//uncomment this line if using a Common Anode LED
//#define COMMON_ANODE
 
void setup()
{
  pinMode(r1, OUTPUT);
  pinMode(g1, OUTPUT);
  pinMode(b1, OUTPUT);  
  pinMode(r2, OUTPUT);
  pinMode(g2, OUTPUT);
  pinMode(b2, OUTPUT);  
  pinMode(wr, OUTPUT);
  pinMode(wl, OUTPUT);

}
 
void loop()
{
  setColor(255, 0, 0);  // red
  delay(1000);
  setColor(0, 255, 0);  // green
  delay(1000);
  setColor(0, 0, 255);  // blue
  delay(1000);
  setColor(255, 255, 0);  // yellow
  delay(1000);  
  setColor(80, 0, 80);  // purple
  delay(1000);
  setColor(0, 255, 255);  // aqua
  delay(1000);
}
 
void setColor(int red, int green, int blue)
{
  #ifdef COMMON_ANODE
    red = 255 - red;
    green = 255 - green;
    blue = 255 - blue;
  #endif
  analogWrite(r1, red);
  analogWrite(g1, green);
  analogWrite(b1, blue);  
  analogWrite(r2, red);
  analogWrite(g2, green);
  analogWrite(b2, blue);
  analogWrite(wr, 255);
  analogWrite(wl, 255);


}
