// Debugs to run
#define HOMENODE_DEBUG
#define THERMOMETER_DEBUG

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

char* ssid = "home-automation";
char* password = "AutomateTheHome";
IPAddress mqttServer(192, 168, 2, 201);
char* clientID = "testNode1";
char* heartbeatTopic = "heartbeat/";

HomeNode node;

void setup()
{
    Serial.begin(115200);
    node.setup(client, ssid, password, mqttServer, heartbeatTopic, clientID);
    therm.setup(node);
}

void loop()
{
    therm.loop();   // reads temperature from 
    delay(500);
}
