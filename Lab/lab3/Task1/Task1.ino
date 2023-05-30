#include <ESP8266WiFi.h> //Import two new library
#define WIFI_SSID "ELEC3848"
#define WIFI_PW "password"

void setup() {
    pinMode(LED_BUILTIN, OUTPUT); // Set up pin IO modes
    
    digitalWrite(LED_BUILTIN, HIGH); // Turn off the lights
    
    Serial.begin(115200);
    
    WiFi.begin(WIFI_SSID, WIFI_PW); // Start Wi-Fi connection
    while (WiFi.status() != WL_CONNECTED) { // Waiting for connection success
      delay(250);
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      Serial.println("Connecting to WiFi..");
    }
    Serial.println("Connected to the WiFi network");
    delay(100);
    digitalWrite(LED_BUILTIN, LOW); //Turn on the light to indicate Wi-Fi success
    Serial.println("-------------------------");
    Serial.println("Hello from ESP8266");
    Serial.print("Connected to hotspot: ");
    Serial.println(WIFI_SSID);
    Serial.print("IP address is: ");
    Serial.println(WiFi.localIP());
    Serial.println("-------------------------");
}

void loop() {
}
