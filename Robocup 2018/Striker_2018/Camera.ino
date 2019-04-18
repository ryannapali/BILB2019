void calculateAngle() {
  // Only run this if you are in fact recieving x and y data. Otherwise, ballAngle does not change
  if (Serial2.available() > 0) {
    getCameraReadings();     // read the incoming camera x and y pos
    Serial.print(xPos);
    Serial.print("  ");
    Serial.print(yPos);
    if (xPos > 1280 || yPos > 960) { //filter out and bad readings. 2000 is sign of bad readings
      ballAngle = 2000;
      setRGB(255, 0, 255); //white
    } else {
      xPos = xPos - 640; //makes the center of the screen (640*480) 0 instead of having it be top left corner
      yPos = yPos - 480;

      xPos = xPos * -1;
      yPos = yPos * -1;
      double m = (float)(yPos) / (float)(xPos);
      ballAngle = atan((double)m);
      ballAngle = ballAngle * 57296 / 1000;
      if (xPos < 0 && yPos < 0) ballAngle = ballAngle + 180;
      else if (xPos > 0 && yPos < 0) ballAngle = ballAngle + 360;
      else if (xPos < 0 && yPos > 0) ballAngle = ballAngle + 180;

      //comment two lines out if orientation is flipped 180 degrees
      ballAngle = ballAngle + 180;
      if (ballAngle > 360) ballAngle = ballAngle - 360;

      ballAngle = ballAngle - 180;

      if (m == .75) { //needs to be at end so overrides any other calculations
        ballAngle = 10000; //ballAngle = 10000 when robot doesn't see ball
      }
    }
    Serial.print(" ");
    Serial.println(ballAngle);
  }
}

void getCameraReadings() {
  if (Serial2.available() > 0) {
    char highChar1 = Serial2.read();
    char lowChar1 = Serial2.read();
    char highChar2 = Serial2.read();
    char lowChar2 = Serial2.read();
    // say what you got:
    xPos = word(highChar1, lowChar1);
    yPos = word(highChar2, lowChar2);
  }
}
void spinToBall() {
  calculateAngle();
  float k = .8;

  spin(ballAngle * k);


}
void clearCameraBuffer() {
  for (int x = 0; x < 100; x ++) {
    getCameraReadings(); //make sure camera does not lag
  }
}

