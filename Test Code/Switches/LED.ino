/*
  Adafruit Arduino - Lesson 3. RGB LED
*/

//Right RGB (R,G,B) (21, 22, 23)
//Left  RGB (R,G,B) ( 9, 10, 14)
//White Light Right 17
//White Light Left  28



//uncomment this line if using a Common Anode LED
//#define COMMON_ANODE


void setColor(int red, int green, int blue, int LED)
{
#ifdef COMMON_ANODE
  red = 255 - red;
  green = 255 - green;
  blue = 255 - blue;
#endif
  if (LED == 1) {
    analogWrite(r1, red);
    analogWrite(g1, green);
    analogWrite(b1, blue);
  }
  else if (LED == 2) {
    analogWrite(r2, red);
    analogWrite(g2, green);
    analogWrite(b2, blue);
  }
  else{
  analogWrite(wr, red);
  analogWrite(wl, red);
  }
}
