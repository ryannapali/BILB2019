void DistIRs (){
  int sensorValueFront = analogRead(DistIRFront);
  int sensorValueBack = analogRead(DistIRBack);
  Serial.print("Front sensor:  ");
  Serial.print(sensorValueFront);
  Serial.print("---------------------Back sensor:  ");
  Serial.print(sensorValueBack);
}
