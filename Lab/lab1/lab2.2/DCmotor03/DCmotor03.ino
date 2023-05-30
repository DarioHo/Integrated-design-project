//Uno board
//pin 2 interrupt

int motorPin1 = 6;
int motorPin2 = 7;

int encoderValue = 0;
int encoderValueTarget = -1000;
int cnt=0;
long int Timer=0;
long int InitTimer=0;
long int TimerR=0;
long int TargetTimer=1000;
void setup()
{
  Serial.begin(115200);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), Encoder_subroutine, FALLING);
  attachInterrupt(digitalPinToInterrupt(3), Encoder_subroutine, FALLING);
// ++++  PWM motor speed control  
  pinMode(9,OUTPUT);
}

void loop()
{
  InitTimer=millis();
  while (encoderValueTarget < encoderValue && Timer<TargetTimer) {
    motor_clockwise();
// ++++ PWM motor control    
    analogWrite(9,100);
    TimerR=millis();
    Timer = TimerR-InitTimer;
    Serial.print(" motor_clockwise() ");
    Serial.print(" encoderValue= ");
    Serial.print(encoderValue);
    Serial.print("    ");
    Serial.print("Timer:");
    Serial.println(Timer);
    delay(50);
  }
  Serial.print("   [motor_stop]    ");
  Serial.print(" encoderValue= ");
  Serial.print(encoderValue);
  Timer=millis();
  Serial.print("    ");
  Serial.print("Timer:");
  Serial.println(Timer);
  while (1) {
    motor_stop();
  }
    
}
void Encoder_subroutine() {
  //check the voltage of another encoder pin
  if (digitalRead(2)) {
    encoderValue--;
  }
  else if (digitalRead(3)) {
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
