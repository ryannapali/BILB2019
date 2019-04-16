void setRGB(int red, int green, int blue)
{
  red = 255 - red;
  blue = 255 - blue;
  green = 255 - green;

  analogWrite(RED_PIN, red);
  analogWrite(GREEN_PIN, blue);
  analogWrite(BLUE_PIN, green);
}
