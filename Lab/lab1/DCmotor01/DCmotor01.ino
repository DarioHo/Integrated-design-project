//Uno board
//pin 2 interrupt

int motorPin1 = 6;
int motorPin2 = 7;

int encoderValue = 0;
long int Time=0;
void setup()
{
  Serial.begin(115200);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), Encoder_subroutine, FALLING);
}

void loop()
{
  
  for (int i=0;i<10;i++){
  motor_clockwise();
  Time=millis();
  Serial.print(" motor_clockwise() ");
  Serial.print(" encoderValue= ");
  Serial.print(encoderValue);
  Serial.print("   ");
  Serial.print("Time:");
  Serial.println(Time);
  delay(50);
  }
  
  Time=millis();
  Serial.print("motor_return");
  Serial.print("   ");
    Serial.print("   encoderValue= ");
  Serial.print(encoderValue);
  Serial.print("  Time:");
  Serial.println(Time);
    Serial.print("motor_return   ");
    Serial.print("   encoderValue= ");
  Serial.print(encoderValue);
  Serial.print("  Time:");
  Serial.println(Time);

// ***** Try the following comments to control motor
  // motor_stop();
  //  motor_counterclockwise();
  // delay(50);
// ***** 
  
  for (int j=0;j<10;j++){
  motor_counterclockwise();
  Time=millis();
  Serial.print(" motor_counterclockwise() ");
  Serial.print(" encoderValue= ");
  Serial.print(encoderValue);
  Serial.print("   ");
  Serial.print("Time:");
  Serial.println(Time);
  delay(50);   
  
  }
  Time=millis();
  Serial.print("motor_stop");
  Serial.print("   ");
  Serial.print("   encoderValue= ");
  Serial.print(encoderValue);
  Serial.print("  Time:");
  Serial.println(Time);
  while(1){
 motor_stop();
  }
}
void Encoder_subroutine() {
  //check the voltage of another encoder pin
  if (digitalRead(3)) {
    encoderValue--;
  }
  else {
    encoderValue++;
  }
}
void motor_clockwise() {
  digitalWrite(motorPin1, 1);
  digitalWrite(motorPin2, 0);
}
void motor_counterclockwise() {
  digitalWrite(motorPin1, 0);
  digitalWrite(motorPin2, 1);
}
void motor_stop() {
  digitalWrite(motorPin1, 0);
  digitalWrite(motorPin2, 0);
}
