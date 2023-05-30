/*****************************************************
IDP Feedback and control of servo motor (V20210915)
Potentiometer controlled servo
*****************************************************/

// https://crazymaker.com.tw/arduino-how-to-use-millis-instead-of-delay/

#include <Servo.h>

unsigned long t; //Timer definition
Servo myServo;  //Create servo object to control a servo

void setup() {
  pinMode(A0, INPUT); //Pin mode definition
  myServo.attach(9);  //Attaches the servo on pin 9 to the servo object
  Serial.begin(115200); //Start serial connection
  t = millis(); //Get current processor time
  myServo.write(0); //Set position
  delay(1000);
}

void loop() {
  if(millis()>t+500){ //Timed event without delay, t is time_now, 500 is period
    t = millis(); //Reset timer
    int inputSignal = analogRead(A0);
    int pos = map(inputSignal, 0, 1023, 0, 180); //Converting the reading range to 180 degrees
    myServo.write(pos);
    Serial.print("  Time : ");
    Serial.print(t); 
    Serial.print(" ; Reads Bit : ");
    Serial.print(inputSignal);
    Serial.print(" ; Writes angle : ");
    Serial.println(pos);
  }
}
