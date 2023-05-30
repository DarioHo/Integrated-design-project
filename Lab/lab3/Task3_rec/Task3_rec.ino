#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// WiFi
const char *ssid = "WIFI_SSID"; // Enter your WiFi name
const char *password = "WIFI_PASSWORD";  // Enter WiFi password

// MQTT Broker
const char *mqtt_broker = "broker IP";  //Enter your MQTT broker's IP
const char *topic = "GROUP";  //Create a topic
const char *mqtt_username = "client";
const char *mqtt_password = "test";
const int mqtt_port = 1883;

unsigned long timer1, timer2, time_diff=0;

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  // Set software serial baud to 115200;
  Serial.begin(115200);
  // connecting to a WiFi network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  //connecting to a mqtt broker
  client.setServer(mqtt_broker, mqtt_port);
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
  client.subscribe(topic);
}

void callback(char *topic, byte *payload, unsigned int length) {
  //find duration between 2 packets
  timer1=millis(); //Mark processor time 
  if (timer2!=0)
  {
    /**********************/
    /*Write your code here*/  
    time_diff = timer1-timer2;//Calculate the time diff. between 2 packets
   
    Serial.print("Duration between 2 packets: ");
    Serial.print(time_diff);
    Serial.println(" ms");  //Define the timer's unit here
  }
  timer2=timer1;
  
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
      Serial.print((char) payload[i]);
  }
  Serial.println();
}

void loop() {
  client.loop();
}
