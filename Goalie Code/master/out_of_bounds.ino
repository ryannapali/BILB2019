void getInBounds() {
  int slowerSpeed = 100;
  motor.stopMotors();
  for (int i = 0; i < 10000; i++) {
    frontSensor = lidars.readSensor1();
    backSensor = lidars.readSensor3();
    leftSensor = lidars.readSensor2();
    rightSensor = lidars.readSensor4();
    
    int x = 0;
    int y = 0;
    boolean isClear = true;
    boolean r = rightClear();
    boolean l = leftClear();
    boolean f = frontClear();
    boolean b = backClear();
    if (!r) x--, isClear = false;
    if (!l) x++, isClear = false;
    if (!f) y--, isClear = false;
    if (!b) y++, isClear = false;
    if (x == 0 && y == 0) {
      Serial.println("spin to goal");
      if (backSensor > 120 ){
          motor.driveToHeadingCorrected(180, 0, slowerSpeed);
      }
       else if (frontSensor > 120 ){
          motor.driveToHeadingCorrected(0, 0, slowerSpeed);
      }
    } else {
      
      Serial.println("moving away");
      int ang = xyToAngle(x, y);
      Serial.print("Angle : ");
      Serial.println(ang);
      motor.driveToHeadingCorrected(ang, 0, slowerSpeed);
    }

    if (isClear){
      motor.stopMotors();
      interrupted = false;
      break;
    }
  }
  interrupted = false;
}

boolean rightClear() {
  int sampleSize = 10;
  int distances[sampleSize];
  for (int i = 0; i < sampleSize; i++) {
    distances[i] = rightSensor;
  }
  return clearDistanceLIDAR(50, distances);
}

boolean leftClear() {
  int sampleSize = 10;
  int distances[sampleSize];
  for (int i = 0; i < sampleSize; i++) {
    distances[i] = leftSensor;
  }
  return clearDistanceLIDAR(50, distances);
}

boolean backClear() {
  int sampleSize = 10;
  int distances[sampleSize];
  for (int i = 0; i < sampleSize; i++) {
    distances[i] = backSensor;
  }
  return clearDistanceLIDAR(60, distances);
}

boolean frontClear() {
  int sampleSize = 10;
  int distances[sampleSize];
  for (int i = 0; i < sampleSize; i++) {
    distances[i] = frontSensor;
  }
  return clearDistanceLIDAR(60, distances);
}


int xyToAngle(int x, int y) {
  if (x == 1 && y == 0) {
    return 90;
  } else if (x == -1 && y == 0) {
    return 270;
  } else if (x == 0 && y == 1) {
    return 0;
  } else if (x == 1 && y == 1) {
    return 45;
  } else if (x == -1 && y == 1) {
    return 315;
  } else if (x == 0 && y == -1) {
    return 180;
  } else if (x == 1 && y == -1) {
    return 135;
  } else {
    return 225;
  }
}


boolean clearDistanceLIDAR(int minDistance, int distances[]) {
  int sampleSize = 10;
  std::sort(distances, distances + sampleSize);
  if (distances[1] == 1) {
    return false;
  } else if (distances[sampleSize - 1] - distances[0] > 20) {
    return false;
  } else {
    int mean = 0;
    for (int i = 0; i < sampleSize; i++) {
      mean += distances[i];
    }
    mean = mean / sampleSize;
    return (mean > minDistance);
  }
}
