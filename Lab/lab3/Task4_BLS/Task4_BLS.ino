#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#define sensorPin 4 //Definte QTI sensor pin (D2==GPIO4)

// WiFi
const char *ssid = "WIFI_SSID"; // Enter your WiFi name
const char *password = "WIFI_PASSWORD";  // Enter WiFi password

// MQTT Broker
const char *mqtt_broker = "broker's IP";  //Enter your MQTT broker's IP
const char *topic = "GROUP";  //Create a topic
const char *mqtt_username = "client";
const char *mqtt_password = "test";
const int mqtt_port = 1883;

unsigned long t;  // Processor time 
int delayT = 100; //Timed event delay 

//***********************************//
//   set the initial "sensorValue"   //
int sensorValue = 0;

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT); // Set up pin IO modes
  pinMode(sensorPin, INPUT);
  
  digitalWrite(LED_BUILTIN, 0); // Turn off the lights
  
  // Set software serial baud to 115200;
  Serial.begin(115200);
  
  // connecting to a WiFi network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  
  digitalWrite(LED_BUILTIN, 1); //Turn on the light to indicate Wi-Fi success
  
  //connecting to a mqtt broker
  client.setServer(mqtt_broker, mqtt_port);
  
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
  
  digitalWrite(LED_BUILTIN, 0); //Turn off the light to signal MQTT success
   
   t=millis(); //Mark processor time for comparison

}

void loop() {
  client.loop();
  if(millis() > t+delayT){ //Timed event for every 100ms

    t=millis(); //Renew processor time for next comparison

//*****************************************************//
//   set Serial.print the "sensorvalue" if necessary   //

  sensorValue = digitalRead(sensorPin); //Read the sensor value
  Serial.print("sensorValue: ");
  Serial.println(sensorValue);

    if(!digitalRead(sensorPin)){ //If the sensor is activated

//********************************//
//   Enter the client.publish()   //
      if (client.publish(topic, "Sensor activated")) { //Publish on topic

         digitalWrite(LED_BUILTIN, 1); //Indication of publishing

         Serial.println("Publish ok");


//****************************//
//   set the delayT for use   //
         delayT=1000; //Increase delay to cap the publishing speed to once per second

      }else {
         Serial.println("Publish failed");
      }
    }else {
      
//****************************//
//   set the delayT for use   //
      delayT=100; 

    }//Decrease delay if sensor is not activated

  }else digitalWrite(LED_BUILTIN, 0); //Turn off the lights to ready for next activation
}
