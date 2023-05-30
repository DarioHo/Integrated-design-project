#include <Arduino.h>

#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
 
#define ss 15
#define rst 16
#define dio0 4

void setup() {
  //initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Sender");

  //setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);
  

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  Serial.println("LoRa Initializing OK!");
  
}

void loop() {
  Serial.println("Sending packet: ");
  Serial.println("Channel Frequency for S1: 439MHz, Channel Frequency for S2: 441MHz");
  //Send LoRa packet to receiver
  LoRa.beginPacket();
  LoRa.print("Channel Frequency for S1: 439MHz, Channel Frequency for S2: 441MHz");

  LoRa.endPacket();
}