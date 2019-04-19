int IRDistance() {
  return distIR.distance();
}

/*
   Purpose: Updates the global distance variables
   Parameters: None
   Returns: Void
*/
void updateDistances() {
  rightDist = rightDistance();
  leftDist = leftDistance();
  backDist = backDistance();
  frontDist = frontDistance();
}

void printDistances(){
  Serial6.print("Right: ");
  Serial6.print(rightDist);
  Serial6.print(" Left: "); 
  Serial6.print(leftDist); 
  Serial6.print(" Front: ");
  Serial6.print(frontDist);
  Serial6.print(" Back: ");
  Serial6.println(backDist);
}


/*
   Purpose for rightDistance, leftDistance, backDistance, frontDistance, IRDistance: reads distances for respective sensors
   Parameters: None
   Returns: distance reading of the correct sensor
*/

double frontDistance() { //FRONT
  return myLidarLite.distance(true, 0x68);
}

double rightDistance() { //RIGHT
  return myLidarLite.distance(true, 0x66);
}

double backDistance() { //BACK
  return myLidarLite.distance(true, 0x64);
}

double leftDistance() { //LEFT
  return myLidarLite.distance(true, 0x62);
}

/*
   Purpose:

   Checking if the robot is clear of the walls can be difficult because of the lack of
   reliability LIDARs have at short distances on surfaces of low reflectivity (black walls)
   When simply checking distance while the robot is in bounds you can simply take the distance
   but for checking while moving back in bounds if the robot is far enough in bounds
   (ie. measuring at short distances) you need to get fancy.

   To do this the robot takes 10 distance readings and sorts them based on length

   A characteristic of very short distance (0-6 inches approx.) unreflective surface LIDAR
   readings is a bunch of 1's mixed in with other numbers. If the first number in the sorted
   array of distance readings is a 1 that LIDAR cannot be far enough from the wall to be in bounds

   A characteristic of mid short distance (6-10 inches approx.) unreflective surface LIDAR
   readings is a inconsistant, wide range of returned values. If the diffence between the first
   and last numbers in the sorted array of distance readings is big enough, that LIDAR cannot be
   far enough from the wall to be in bounds

   Otherwise, if the LIDAR is getting good readings, the distance must be greater than a certain
   number for the LIDAR to be "clear" of the wall (or a robot)

   Parameters:

   int minDistance    the minimum distance you need to be in order to be in bounds (different for different sides)
   int distances      array of 10 distance readings

   Returns:
   void

*/

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

/*
   Purpose for rightClear, leftClear, backClear, frontClear: check if each side is far enough from the wall
   Parameters: none
   Returns: void
*/
boolean rightClear() {
  int sampleSize = 10;
  int distances[sampleSize];
  for (int i = 0; i < sampleSize; i++) {
    distances[i] = rightDistance();
  }
  return clearDistanceLIDAR(50, distances);
}

boolean leftClear() {
  int sampleSize = 10;
  int distances[sampleSize];
  for (int i = 0; i < sampleSize; i++) {
    distances[i] = leftDistance();
  }
  return clearDistanceLIDAR(50, distances);
}

boolean backClear() {
  int sampleSize = 10;
  int distances[sampleSize];
  for (int i = 0; i < sampleSize; i++) {
    distances[i] = backDistance();
  }
  return clearDistanceLIDAR(60, distances);
}

boolean frontClear() {
  int sampleSize = 10;
  int distances[sampleSize];
  for (int i = 0; i < sampleSize; i++) {
    distances[i] = frontDistance();
  }
  return clearDistanceLIDAR(60, distances);
}
