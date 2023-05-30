//Initializing LED Pin

int led_pin = 8;
int i = 0;
void setup() {
  //Declaring LED pin as output
  pinMode(led_pin, OUTPUT);
  Serial.begin(9600);
}
void loop() {
  //Fading the LED
  for(int i=0; i<255; i++){
    analogWrite(led_pin, i);
    Serial.print("Counter1:");
    Serial.print(i);
    Serial.print(",");
    Serial.print("Counter2:");
    Serial.print(i - 10);
    Serial.println();
    delay(10);
  }

  for(int i=255; i>0; i--){
    analogWrite(led_pin, i);
    Serial.print("Counter1:");
    Serial.print(i);
    Serial.print(",");
    Serial.print("Counter2:");
    Serial.print(i + 5);
    Serial.println();
    delay(10);
  }
}
