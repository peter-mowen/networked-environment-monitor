#include "HomeNode.h"

HomeNode::HomeNode()
{
    _ssid = "";
    _password = "";
    _heartbeatTopic = "";
    _clientID = "";
}

void HomeNode::setup(PubSubClient client, char* ssid, char* password, char* mqttServer, char* heartbeatTopic, char* clientID)
{
    _client = client;
    _ssid = ssid;
    _password = password;
    _mqttServer = mqttServer;
    _heartbeatTopic = heartbeatTopic;
    _clientID = clientID;

    setupWifi();
    _client.setServer(_mqttServer, 1883);
}

void HomeNode::loop()
{
    if (!_client.connected())
    {
        reconnect();
    }
    _client.loop();
}

void HomeNode::publishMsg(char* topic, char* msg)
{
    _client.publish(topic, msg);
}


void HomeNode::setupWifi()
{
    delay(10);
    // We start by connecting to a WiFi network

    #ifdef HOMENODE_DEBUG
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(_ssid);
    #endif

    WiFi.mode(WIFI_STA);
    WiFi.begin(_ssid, _password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        #ifdef HOMENODE_DEBUG
        Serial.print(".");
        #endif
    }

    randomSeed(micros());
    #ifdef HOMENODE_DEBUG
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    #endif
}

void HomeNode::reconnect()
{
    // Loop until we're reconnected
    while (!_client.connected())
    {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (_client.connect(_clientID))
        {
            Serial.println("connected");
            // Once connected, publish an announcement...
            _client.publish(_heartbeatTopic, _clientID);
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(_client.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}
