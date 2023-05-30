#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>               
#include "SSD1306Wire.h"   

// WiFi
const char *ssid = "WIFI_SSID"; // Enter your WiFi name
const char *password = "WIFI_PASSWORD";  // Enter WiFi password

// MQTT Broker
const char *mqtt_broker = "broker's IP";  //Enter your MQTT broker's IP
const char *topic = "GROUP";  //Create a topic
const char *mqtt_username = "client";
const char *mqtt_password = "test";
const int mqtt_port = 1883;

char alertT[20] = "Never"; // Text to display when alerted
// Displays "Never" if not alerted for once

unsigned long aT, t, aSec=0; // Processor time marks
// aT is for time of alerted, t is for current time, aSec is for seconds elapsed after the alert

SSD1306Wire display(0x3c, SDA, SCL);   // Setup OLED display (SDA = D2, SCL = D1)
//SH1106 display(0x3c, SDA, SCL);
boolean alerted=false; //Flag to indicate first alert

WiFiClient espClient;
PubSubClient client(espClient);

void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
      Serial.print((char) payload[i]);
  }
  Serial.println();

//******************************************
//   set aTvalue for use                    
  aT=  millis()/*Write your code here */;
  alerted=true; //Change the flag to signal the main loop

  digitalWrite(LED_BUILTIN, 1); //Turn on the lights
}

void setup() {
  // Set software serial baud to 115200;
  Serial.begin(115200);
  
  //Initialize IO pins, Serial and OLED display
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 0);
  
  display.init();
  display.clear();
  display.display();
  display.flipScreenVertically(); //Flip it display fit the current breadboard setup

  display.setFont(ArialMT_Plain_10); //Set Fonts
  
  // connecting to a WiFi network
  WiFi.begin(ssid, password);

  display.setTextAlignment(TEXT_ALIGN_LEFT); //Show connection status
  display.drawString(0, 0, "Wifi Conn.");
  display.display();
  
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
    
  display.drawString(0, 17, "MQTT Conn.");
  display.display();

//******************************************
//*Enter necessary code here               *
// Enter client.setServer()                *
client.setServer(mqtt_broker, mqtt_port);
// Enter client.setCallback()              *
client.setCallback(callback);

  while (!client.connected()) {
      String client_id = "esp8266-client-";
      client_id += String(WiFi.macAddress());
      Serial.printf("The client %s connects the mqtt broker\n", client_id.c_str());
      if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
          Serial.println("mqtt broker connected");
      } else {
          Serial.print("failed with state ");
          Serial.print(client.state());
          delay(2000);
      }
  }
  client.publish("log", "hello"); //Publish to topic named "log" to indicate a success

//******************************************
// Update the topic if necessary           *
  client.subscribe("alert"); //Subscribe to topic named "alert"

  display.drawString(0, 33, "DONE"); //Indicate all preparations are finished
  display.display();

  delay(1000);
  digitalWrite(LED_BUILTIN, 1);

  //Begin displaying the main display
  display.clear();
  display.drawString(0, 0, "Last Alert");
  display.display();
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 20, String(alertT));
  display.display();

  delay(1000);
  t=millis();
  aT=millis();
  digitalWrite(LED_BUILTIN, 0);
}

void loop() {
  client.loop();
  if( (millis()>(t+1000)) && alerted){ //Timed event setup
    t=millis(); //Mark processor time
    digitalWrite(LED_BUILTIN, 0); //Turn off the lights to get it lit again when MQTT alerts it
    aSec = (millis() - aT)/1000; // Calculate elapsed time in seconds

    sprintf(alertT, "%ld second(s) ago", aSec); //Turn it into a string

    display.clear(); //Print the time out nicely
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "Last Alert");
    display.display();
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 20, String(alertT));
    display.display();
  }
}
