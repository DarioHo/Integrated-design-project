#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
 
#define ss 15
#define rst 16
#define dio0 4

String myString;

void setup() {
 //initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial);

  Serial.println("LoRa Receiver");

  //setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);
  Serial.println("----------------------------------------------------------------------------------------------------------");
  Serial.println("Find the hidden message by changing the receiver's frequency");
  Serial.println("The hidden message can be found within the frequency range of 433MHz-443MHz");
  Serial.println("INSTRUCTIONS:");
  Serial.println("1. Input the frequency to initialise the receiver.");
  Serial.println("   The correct format for frequency is: xxxE6 with x as the frequency you want to set the recevier to.");
  Serial.println("   For example, if you want to read from frequency 436MHz, input '436E6' ");
  Serial.println("2. You may change the receiver's frequency by inputting a new value to the serial console.");
  Serial.println("HINT:");
  Serial.println("Start with 433MHz");
  Serial.println("-----------------------------------------------------------------------------------------------------------");
}

void loop() {
  if (Serial.available())  //Check for data
  {
    myString=Serial.readString();
    if (myString.toDouble()<433E6 || myString.toDouble()>443E6)
    {
      Serial.println("Invalid frequency!");
    }
    else 
    {
      Serial.print("Changing frequency to ");
      Serial.println(myString);
      if (!LoRa.begin(myString.toDouble())) 
      {
        Serial.println("Starting LoRa failed!");
        while (1);
      }
    }
  }

  // Receive packets from LoRa
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("\nReceived packet: ");

    // read packet
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }
  }
}