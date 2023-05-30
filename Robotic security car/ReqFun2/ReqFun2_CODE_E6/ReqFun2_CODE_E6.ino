#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SoftwareSerial.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     28 //4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
int oldV=1, newV=0;
#include <SoftwareSerial.h>
//UNO: (2, 3)
//SoftwareSerial mySerial(4, 6); // RX, TX
int pan = 90;
int tilt = 120;
int window_size = 0;
int BT_alive_cnt = 0;
int voltCount = 0;
#include <Servo.h>
Servo servo_pan;
Servo servo_tilt;
int servo_min = 20;
int servo_max = 160;
#include <INA226.h>
INA226 ina226;

unsigned long time;

//FaBoPWM faboPWM;
int pos = 0;
int MAX_VALUE = 2000;
int MIN_VALUE = 300;

// Define motor pins
#define PWMA 12    //Motor A PWM
#define DIRA1 34
#define DIRA2 35  //Motor A Direction
#define PWMB 8    //Motor B PWM
#define DIRB1 37
#define DIRB2 36  //Motor B Direction
#define PWMC 9   //Motor C PWM
#define DIRC1 43
#define DIRC2 42  //Motor C Direction
#define PWMD 5    //Motor D PWM
#define DIRD1 A4  //26  
#define DIRD2 A5  //27  //Motor D Direction

#define MOTORA_FORWARD(pwm)    do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,HIGH);analogWrite(PWMA,pwm);}while(0)
#define MOTORA_STOP(x)         do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,LOW); analogWrite(PWMA,0);}while(0)
#define MOTORA_BACKOFF(pwm)    do{digitalWrite(DIRA1,HIGH);digitalWrite(DIRA2,LOW); analogWrite(PWMA,pwm);}while(0)

#define MOTORB_FORWARD(pwm)    do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,HIGH);analogWrite(PWMB,pwm);}while(0)
#define MOTORB_STOP(x)         do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,LOW); analogWrite(PWMB,0);}while(0)
#define MOTORB_BACKOFF(pwm)    do{digitalWrite(DIRB1,HIGH);digitalWrite(DIRB2,LOW); analogWrite(PWMB,pwm);}while(0)

#define MOTORC_FORWARD(pwm)    do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,HIGH);analogWrite(PWMC,pwm);}while(0)
#define MOTORC_STOP(x)         do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,LOW); analogWrite(PWMC,0);}while(0)
#define MOTORC_BACKOFF(pwm)    do{digitalWrite(DIRC1,HIGH);digitalWrite(DIRC2,LOW); analogWrite(PWMC,pwm);}while(0)

#define MOTORD_FORWARD(pwm)    do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,HIGH);analogWrite(PWMD,pwm);}while(0)
#define MOTORD_STOP(x)         do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,LOW); analogWrite(PWMD,0);}while(0)
#define MOTORD_BACKOFF(pwm)    do{digitalWrite(DIRD1,HIGH);digitalWrite(DIRD2,LOW); analogWrite(PWMD,pwm);}while(0)

#define SERIAL  Serial
#define BTSERIAL Serial3

#define LOG_DEBUG

#ifdef LOG_DEBUG
  #define M_LOG SERIAL.print
#else
  #define M_LOG BTSERIAL.println
#endif

//PWM Definition
#define MAX_PWM   2000
#define MIN_PWM   300

int Motor_PWM = 50;

boolean imageCapture = false;

//ultrasonic
#define echoPinLeft 33
#define trigPinLeft 32
#define echoPinRight 22
#define trigPinRight 24

int left = 0;
int LR = 0;
int pen = 0;

//jetson camera
SoftwareSerial SerialJetson(10, 11); // RX, TX



//    ↑A-----B↑
//     |  ↑  |
//     |  |  |
//    ↑C-----D↑
void BACK(uint8_t pwm_A, uint8_t pwm_B, uint8_t pwm_C, uint8_t pwm_D)
{
  MOTORA_BACKOFF(Motor_PWM); 
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); 
  MOTORD_FORWARD(Motor_PWM);
}

//    ↓A-----B↓
//     |  |  |
//     |  ↓  |
//    ↓C-----D↓
void ADVANCE()
{
  MOTORA_FORWARD(Motor_PWM); 
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM); 
  MOTORD_BACKOFF(Motor_PWM);
}
//    =A-----B↑
//     |   ↖ |
//     | ↖   |
//    ↑C-----D=
void LEFT_1()
{
  MOTORA_STOP(Motor_PWM); 
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); 
  MOTORD_STOP(Motor_PWM);
}

//    ↓A-----B↑
//     |  ←  |
//     |  ←  |
//    ↑C-----D↓
void RIGHT_2()
{
  MOTORA_FORWARD(170); 
  MOTORB_FORWARD(140);
  MOTORC_BACKOFF(100); 
  MOTORD_BACKOFF(100);
}
//    ↓A-----B=
//     | ↙   |
//     |   ↙ |
//    =C-----D↓
void LEFT_3()
{
  MOTORA_FORWARD(Motor_PWM); 
  MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM); 
  MOTORD_BACKOFF(Motor_PWM);
}
//    ↑A-----B=
//     | ↗   |
//     |   ↗ |
//    =C-----D↑
void RIGHT_1()
{
  MOTORA_BACKOFF(Motor_PWM); 
  MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM); 
  MOTORD_FORWARD(Motor_PWM);
}
//    ↑A-----B↓
//     |  →  |
//     |  →  |
//    ↓C-----D↑
void LEFT_2()
{
  MOTORA_BACKOFF(150); 
  MOTORB_BACKOFF(150);
  MOTORC_FORWARD(100); 
  MOTORD_FORWARD(100);
}
//    =A-----B↓
//     |   ↘ |
//     | ↘   |
//    ↓C-----D=
void RIGHT_3()
{
  MOTORA_STOP(Motor_PWM); 
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM); 
  MOTORD_STOP(Motor_PWM);
}

//    ↑A-----B↓
//     | ↗ ↘ |
//     | ↖ ↙ |
//    ↑C-----D↓
void rotate_1()  //tate_1(uint8_t pwm_A,uint8_t pwm_B,uint8_t pwm_C,uint8_t pwm_D)
{
  MOTORA_BACKOFF(80); 
  MOTORB_BACKOFF(80);
  MOTORC_BACKOFF(Motor_PWM); 
  MOTORD_BACKOFF(Motor_PWM);
}

//    ↓A-----B↑
//     | ↙ ↖ |
//     | ↘ ↗ |
//    ↓C-----D↑
void rotate_2()  // rotate_2(uint8_t pwm_A,uint8_t pwm_B,uint8_t pwm_C,uint8_t pwm_D)
{
  MOTORA_FORWARD(80);
  MOTORB_FORWARD(80);
  MOTORC_FORWARD(Motor_PWM);
  MOTORD_FORWARD(Motor_PWM);
}
//    =A-----B=
//     |  =  |
//     |  =  |
//    =C-----D=
void STOP()
{
  MOTORA_STOP(Motor_PWM);
  MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);
  MOTORD_STOP(Motor_PWM);
}

bool is_center = false;

void UART_Control()
{
  String myString;

  if (SERIAL.read() > 0 & is_center == false)
  {
    
    char buffer[16];
    int size = Serial.readBytesUntil('\n', buffer, 12);
    if (buffer[0] == 'L') {
      rotate_2();
      delay(100);
      STOP();
      delay(500);
    }
    if (buffer[0] == 'R') {
      rotate_2();
      delay(100);
      STOP();
      delay(500);
    }
    if (buffer[0] == 'S') {
      is_center = true;
    }
  }
}

//void UART_Control()
//{
//  String myString;
//  char BT_Data = 0;
//  // USB data
//  /****
//   * Check if USB Serial data contain brackets
//   */
//
//  if (SERIAL.available())
//  {
//    display.clearDisplay();
//    display.setCursor(0, 0);     // Start at top-left corner
//    display.println("OK");
//    display.display();
//    while (imageCapture = false){
//       if (SERIAL.read() > 0){
//         char inputChar = SERIAL.read();
//         display.println("gg");
//         imageCapture = true;
//      }
//    }
//
////    if (inputChar == '(') { // Start loop when left bracket detected
////      myString = "";
////      inputChar = SERIAL.read();
////      while (inputChar != ')')
////      {
////        myString = myString + inputChar;
////        inputChar = SERIAL.read();
////        if (!SERIAL.available()) {
////          break;
////        }// Break when bracket closed
////      }
////    }
////    int commaIndex = myString.indexOf(','); //Split data in bracket (a, b, c)
////    //Search for the next comma just after the first
////    int secondCommaIndex = myString.indexOf(',', commaIndex + 1);
////    String firstValue = myString.substring(0, commaIndex);
////    String secondValue = myString.substring(commaIndex + 1, secondCommaIndex);
////    String thirdValue = myString.substring(secondCommaIndex + 1); // To the end of the string
////    if ((firstValue.toInt() > servo_min and firstValue.toInt() < servo_max) and  //Convert them to numbers
////        (secondValue.toInt() > servo_min and secondValue.toInt() < servo_max)) {
////      pan = firstValue.toInt();
////      tilt = secondValue.toInt();
////      window_size = thirdValue.toInt();
////    }
////    SERIAL.flush();
////    Serial3.println(myString);
////    Serial3.println("Done");
////    if (myString != "") {
//      display.clearDisplay();
//      display.setCursor(0, 0);     // Start at top-left corner
//      display.println("Serial_Data = ");
//      display.println(myString);
//      display.display();
////    }
//  }





  /*
    Receive data from app and translate it to motor movements
  */
  /*
  // BT Module on Serial 3 (D14 & D15)
  if (Serial3.available())
  {
    BT_Data = Serial3.read();
    SERIAL.print(BT_Data);
    Serial3.flush();
    BT_alive_cnt = 100;
    display.clearDisplay();
    display.setCursor(0, 0);     // Start at top-left corner
    display.println("BT_Data = ");
    display.println(BT_Data);
    display.display();
  }

  BT_alive_cnt = BT_alive_cnt - 1;
  if (BT_alive_cnt <= 0) {
    STOP();
  }
  switch (BT_Data)
  {
    case 'A':  ADVANCE();  M_LOG("Run!\r\n"); break;
    case 'B':  RIGHT_2();  M_LOG("Right up!\r\n");     break;
    case 'C':  rotate_1();                            break;
    case 'D':  RIGHT_3();  M_LOG("Right down!\r\n");   break;
    case 'E':  BACK(500, 500, 500, 500);     M_LOG("Run!\r\n");          break;
    case 'F':  LEFT_3();   M_LOG("Left down!\r\n");    break;
    case 'G':  rotate_2();                              break;
    case 'H':  LEFT_2();   M_LOG("Left up!\r\n");     break;
    case 'Z':  STOP();     M_LOG("Stop!\r\n");        break;
    case 'z':  STOP();     M_LOG("Stop!\r\n");        break;
    case 'd':  LEFT_2();   M_LOG("Left!\r\n");        break;
    case 'b':  RIGHT_2();  M_LOG("Right!\r\n");        break;
    case 'L':  Motor_PWM = 1500;                      break;
    case 'M':  Motor_PWM = 500;                       break;
  }
  */
//}




/*Voltage Readings transmitter
Sends them via Serial3*/
void sendVolt(){
    newV = analogRead(A0);
    if(newV!=oldV) {
      if (!Serial3.available()) {
        Serial3.println(newV);
        Serial.println(newV);
      }
    }
    oldV=newV;
}


//Where the program starts
void setup()
{
  SERIAL.begin(115200); // USB serial setup
  SERIAL.println("Start");
  STOP(); // Stop the robot

  SerialJetson.begin(9600);
  Serial.begin(9600);
  pinMode(trigPinLeft, OUTPUT);
  pinMode(echoPinLeft, INPUT);
  pinMode(trigPinRight, OUTPUT);
  pinMode(echoPinRight, INPUT);
  Wire.begin();
  ina226.begin(0x40);

  /*
  Serial3.begin(9600); // BT serial setup
  */
  //Pan=PL4=>48, Tilt=PL5=>47
   servo_pan.attach(48);
   servo_tilt.attach(47);
  //////////////////////////////////////////////
  //OLED Setup//////////////////////////////////
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
  }
//  display.clearDisplay();
//  display.setTextSize(2);      // Normal 1:1 pixel scale
//  display.setTextColor(SSD1306_WHITE); // Draw white text
//  display.cp437(true);         // Use full 256 char 'Code Page 437' font
//  display.setCursor(0, 0);     // Start at top-left corner
//  display.println("AI Robot");
//  display.display();


  //Setup Voltage detector
  pinMode(A0, INPUT);
}


void loop() {

  //UART_Control();
    display.clearDisplay();
//    display.setTextSize(1);      // Normal 1:1 pixel scale
//    display.setTextColor(SSD1306_WHITE); // Draw white text
//    display.cp437(true);         // Use full 256 char 'Code Page 437' font
//    display.setCursor(0, 0);     // Start at top-left corner
//    // display.println("AI Robot");
//    display.print("Bus Voltage: ");
//    display.print(ina226.readBusVoltage(), 5);
//    display.print(" V, Bus Power: ");
//    display.print(ina226.readBusPower(), 5);
//    display.print(" W, Current: ");
//    display.print(ina226.readShuntCurrent(), 5);
//    display.println(" mA");
//    display.display();
//    delay(100);  
    

  /*
  if (SerialJetson.available()) {
    char signal = SerialJetson.read();
    if (signal == '1') {
        display.println("good");
  display.display();
    } else if (signal == '0') {
        display.println("bad");
        rotate_2();
        delay(1000);
        STOP();
        delay(100);
  display.display();
    }
  }
  */

  while (is_center == false){
    UART_Control();
  }

  while (is_center == true){ // while photo is taken iscenter == 
    display.clearDisplay();
    display.setTextSize(1);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.cp437(true);         // Use full 256 char 'Code Page 437' font
    display.setCursor(0, 0);     // Start at top-left corner
    display.print("Bus Voltage: ");
    display.print(ina226.readBusVoltage(), 5);
    display.print(" V, Bus Power: ");
    display.print(ina226.readBusPower(), 5);
    display.print(" W, Current: ");
    display.print(ina226.readShuntCurrent(), 5);
    display.println(" mA");
    
    display.display();
    delay(100);
    if (ina226.readBusVoltage() > 2){
      display.clearDisplay();
      display.setTextSize(1);      // Normal 1:1 pixel scale
      display.setTextColor(SSD1306_WHITE); // Draw white text
      display.cp437(true);         // Use full 256 char 'Code Page 437' font
      display.setCursor(0, 0);     // Start at top-left corner
      display.print("Bus Voltage: ");
      display.print(ina226.readBusVoltage(), 5);
      display.print(" V, Bus Power: ");
      display.print(ina226.readBusPower(), 5);
      display.print(" W, Current: ");
      display.print(ina226.readShuntCurrent(), 5);
      display.println(" mA");
      display.display();
      delay(1000);
      STOP();
      delay(100000000000000000);
    }
    // Measure distance from the left sensor
    long durationLeft, distanceLeft;
    digitalWrite(trigPinLeft, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPinLeft, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinLeft, LOW);
    durationLeft = pulseIn(echoPinLeft, HIGH);
    distanceLeft = (durationLeft / 2) / 29.1;
  
    // Measure distance from the right sensor
    long durationRight, distanceRight;
    digitalWrite(trigPinRight, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPinRight, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinRight, LOW);
    durationRight = pulseIn(echoPinRight, HIGH);
    distanceRight = (durationRight / 2) / 29.1;
  
    // Output the distance measurements to the serial monitor
    Serial.print("Distance from left sensor: ");
    Serial.print(distanceLeft);
    Serial.print(" cm, Distance from right sensor: ");
    Serial.print(distanceRight);
    Serial.println(" cm");
  
    // Wait for a short period of time before measuring again
    delay(100);
    if (distanceLeft > 20 & distanceRight > 20){
      ADVANCE();
      delay(300);
      STOP();
      delay(10);
    }
    // Rotate until facing the station/wall perpendicularly
    //    else if (distanceLeft > 1000 || distanceRight > 1000){
    //      noForward = 0;
    //      BACK(50,50,50,50);
    //      delay(500);
    //      STOP();
    //      delay(100);
    //    }
    else{
      if(((distanceLeft - distanceRight) > 1)& (pen == 0)){
        rotate_2();
        delay(100);
        STOP();
        delay(100);
      }else if (((distanceRight - distanceLeft) > 1)& (pen == 0)){
        rotate_1();
        delay(100);
        STOP();
        delay(100);
      } 
      else{
        // perpendicular already
        pen = 1;
        STOP();
        delay(100);
        if(distanceLeft > 5 & distanceRight > 5 & LR == 0){ // move forward until it is 5cm in front of the wall 
          ADVANCE();
          delay(100);
          STOP();
          delay(100);
        }
        else{
          LR = 1;
          STOP();
          delay(100);
          if (ina226.readBusVoltage() < 1){ // voltage not enough
            display.clearDisplay();
            display.setTextSize(1);      // Normal 1:1 pixel scale
            display.setTextColor(SSD1306_WHITE); // Draw white text
            display.cp437(true);         // Use full 256 char 'Code Page 437' font
            display.setCursor(0, 0);     // Start at top-left corner
            display.print("Distance from left sensor: ");
            display.print(distanceLeft);
            display.print(" cm, Distance from right sensor: ");
            display.print(distanceRight);
            display.println(" cm");
            display.display();
            delay(500);
            if (left == 0){
              LEFT_2();
              delay(50);
              STOP();
              delay(20);
            }
            if ((distanceLeft > 10 & distanceLeft < 100) || (distanceRight > 10 & distanceRight < 100) || left == 1){
              left = 1;
              RIGHT_2();
              delay(50);
              STOP();
              delay(20);
            }
          } else {
            display.clearDisplay();
            display.setTextSize(1);      // Normal 1:1 pixel scale
            display.setTextColor(SSD1306_WHITE); // Draw white text
            display.cp437(true);         // Use full 256 char 'Code Page 437' font
            display.setCursor(0, 0);     // Start at top-left corner
            display.print("Bus Voltage: ");
            display.print(ina226.readBusVoltage(), 5);
            display.print(" V, Bus Power: ");
            display.print(ina226.readBusPower(), 5);
            display.print(" W, Current: ");
            display.print(ina226.readShuntCurrent(), 5);
            display.println(" mA");
            display.display();
            delay(1000);
            STOP();
            delay(10000000);
          }
         }
        }
      }
    }
  }
