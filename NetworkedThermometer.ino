// Uncomment/comment to toggle printing debug information to serial port
#define HOMENODE_DEBUG
#define THERMOMETER_DEBUG

#include "Thermometer.h"

#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// Define I2C pins for _sensor
#define SDA D4
#define SCL D2

// Instantiate Si7021 sensor
Si7021 sensor(SDA, SCL);

// Instantiate thermometer object
Thermometer therm;

//MQTT Prep
WiFiClient espClient;
PubSubClient client(espClient);

char* ssid = "";            // add wifi name here
char* password = "";        // add wifi password here
char* mqttServer = "";      // add mqtt dns name or IP addres here
char* clientID = "";        // add clientID here. this identifies the node on mqtt 
char* heartbeatTopic = "heartbeat/";

HomeNode node;

void setup()
{
    Serial.begin(115200);
    sensor.setup();
    node.setup(client, ssid, password, mqttServer, heartbeatTopic, clientID);
    therm.setup(&sensor, &node);
}

void loop()
{
    therm.loop();
    delay(500);
    ESP.deepSleep(600e6);
}
