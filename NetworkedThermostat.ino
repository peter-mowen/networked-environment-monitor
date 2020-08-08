// Debugs to run
#define HOMENODE_DEBUG
#define THERMOMETER_DEBUG

#include "Thermometer.cpp"

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

char* ssid = "Schniblets_2.4_EXT";
char* password = "pr3st0n!";
char* mqttServer = "automationDatabase.local";
char* clientID = "NEM 01";
char* heartbeatTopic = "heartbeat/";

HomeNode node;

void setup()
{
    Serial.begin(115200);
    sensor.setup();
    node.setup(client, ssid, password, mqttServer, heartbeatTopic, clientID);
    therm.setup(sensor, node);

    //wifi_set_sleep_type(LIGHT_SLEEP_T);
}

void loop()
{
    therm.loop();   // reads temperature from 
    delay(500);
    ESP.deepSleep(600e6);
}
