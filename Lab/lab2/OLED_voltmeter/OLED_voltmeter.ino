
#define sclk 13 //--- connect this to the display module CLK pin (Serial Clock)

#define mosi 11 //--- connect this to the display module DIN pin (Serial Data)

#define rst  9 //--- connect this to the display module RES pin (Reset)

#define dc   8 //--- connect this to the display module D/C  pin (Data or Command)

#define cs   10 //--- connect this to the display module CS  pin (Chip Select)



// Color definitions

#define  BLACK           0x0000
#define BLUE            0x0006
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0  
#define WHITE           0xFFFF
#define BACKGROUND      0x0000


#include <Adafruit_SSD1331.h>

Adafruit_SSD1331 display = Adafruit_SSD1331(cs, dc, mosi, sclk, rst);

int analogInput = 0;
float vout = 0.0;
float vin = 0.0;
int value = 0;

void setup()
{
  Serial.begin(9600);

  display.begin();
  display.fillScreen(BLACK);
  tftPrintTest();
  delay(2000);  
  display.fillScreen(BACKGROUND);
  display.setCursor(0,0);
}

void loop() {
  
   // read the value at analog input
   value = analogRead(analogInput);
   vin = (value * 5.0) / 1024.0; // see text
   if (vin<0.01) {
   vin=0.0; }      //statement to remove undesired reading!

  display.fillScreen(BLACK);
  display.setCursor(0, 1);
  display.setTextColor(WHITE);  
  display.setTextSize(2);
  display.println("INPUT V= ");
  display.setCursor(0, 25);
  display.print(vin);
  delay(500);

}


void tftPrintTest() {

  
  display.fillScreen(BLACK);
  display.setCursor(0, 1);
  display.setTextColor(WHITE);  
  display.setTextSize(2);
  display.println("INPUT V= ");
  display.setCursor(0, 25);
  display.print("Ready to measure");
  //display.print(vin);
  delay(500);

}
