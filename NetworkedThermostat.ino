// Debugs to run
#define HOMENODE_DEBUG

#include "Thermometer.cpp"

#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// Define I2C pins
#define SDA D2
#define SCL D1

// Instantiate thermometer object
Thermometer therm(SDA, SCL);

//MQTT Prep
WiFiClient espClient;
PubSubClient client(espClient);

char* ssid = "Schniblets";
char* password = "pr3st0n!";
char* mqttServer = "10.0.0.161";
char* clientID = "testNode1";
char* heartbeatTopic = "heartbeat/";

void setup()
{
    Serial.begin(115200);
    therm.setup(client, ssid, password, mqttServer);
}

void loop()
{
    therm.loop(clientID, heartbeatTopic);   // reads temperature from 
    Serial.println("Temperature = " + (String)therm.getTemperatureVal());
    delay(500);
}
