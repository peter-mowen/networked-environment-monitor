#ifndef MOWEN_HOMENODE
#define MOWEN_HOMENODE

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

class HomeNode
{

public:
    HomeNode();

    void setup(PubSubClient client, char* ssid, char* password, char* mqttServer, char* heartbeatTopic, char* clientID);

    void loop();

    void publishMsg(char* topic, char* msg);

    char* getClientID() { return _clientID; }

private:
    PubSubClient _client;
    char* _ssid;
    char* _password;
    char* _mqttServer;
    char* _heartbeatTopic;
    char* _clientID;

    void setupWifi();

    void reconnect();
};

#endif  // MOWEN_HOMENODE
