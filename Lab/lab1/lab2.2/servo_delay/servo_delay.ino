/*****************************************************
IDP  Feedback and control of servo motor (V20210915)
Potentiometer controlled servo
*****************************************************/

#include <Servo.h>

Servo myServo;  //Create servo object to control a servo

void setup() {
  pinMode(A0, INPUT); //Pin mode definition
  myServo.attach(9);  //Attaches the servo on pin 9 to the servo object
  Serial.begin(115200); //Start serial connection
  myServo.write(0); //Set position
  delay(1000);
}

void loop() {
  int inputSignal = analogRead(A0);
  int pos = map(inputSignal, 0, 1023, 0, 180); //Converting the reading range to 180 degrees
  myServo.write(pos);
  Serial.print("Reads Bit : ");
  Serial.print(inputSignal);
  Serial.print("  ; Writes angle : ");
  Serial.println(pos);
  delay(15);
}
