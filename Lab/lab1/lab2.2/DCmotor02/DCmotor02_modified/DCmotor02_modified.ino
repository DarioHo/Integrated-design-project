//Uno board
//pin 2 interrupt
//https://blog.jmaker.com.tw/arduino-buttons/

int motorPin1 = 6;
int motorPin2 = 7;
int BUTTON_PIN = 8;  

int encoderValue = 0;
int TargetEC = 200;  // ** Target encoder count, note the value here
int buttonState = 0;

void setup()
{
  Serial.begin(115200);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(2), Encoder_subroutine, FALLING);
  pinMode(BUTTON_PIN, INPUT);
}

void loop()
{
  buttonState = digitalRead(BUTTON_PIN);

  if(buttonState == 1){
    Serial.print(buttonState);

    Serial.print(" button pressed");

    while(encoderValue < TargetEC){  
    motor_clockwise();
    Serial.print(" motor_clockwise() ");
    Serial.print(" encoderValue= ");
    Serial.println(encoderValue);
    //**----------------------------------------------
    //** change the value of delay to learn if problem
    //** try to use other method
    delay(5);
    }

    Serial.println("motor_stop");
    Serial.print("Target encoderValue= ");
    Serial.println(TargetEC);
    Serial.print("FINAL encoderValue= ");
    Serial.println(encoderValue);
    while(1){
    motor_stop();
    delay(5000);

    while(encoderValue > TargetEC){ 
    motor_counterclockwise();
    Serial.print(" motor_counterclockwise() ");
    Serial.print(" encoderValue= ");
    Serial.println(encoderValue);
    }



  }
  
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

// signal the motor to revolve in clockwise direction
void motor_clockwise() {
  digitalWrite(motorPin1, 1);
  digitalWrite(motorPin2, 0);
}

// signal the motor to revolve in counterclockwise direction
void motor_counterclockwise() {
  digitalWrite(motorPin1, 0);
  digitalWrite(motorPin2, 1);
}

// signal the motor to stop
void motor_stop() {
  digitalWrite(motorPin1, 0);
  digitalWrite(motorPin2, 0);
}
