#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

class HomeNode
{
    PubSubClient _client;
    char* _ssid;
    char* _password;
    char* _mqttServer;

public:
    HomeNode()
    {
        _ssid = "";
        _password = "";
        _mqttServer = "";
    }

    void setup(PubSubClient client, char* ssid, char* password, char* mqttServer)
    {
        _client = client;
        _ssid = ssid;
        _password = password;
        _mqttServer = mqttServer;

        setupWifi(ssid, password);
        _client.setServer(mqttServer, 1883);
        //_client.setCallback([this] (char* topic, byte* payload, unsigned int length) { this->callback(topic, payload, length); });
    }

    void loop(char* clientID, char* heartbeatTopic)
    {
        if (!_client.connected()) 
        {
            reconnect(clientID, heartbeatTopic);
        }
        _client.loop();
    }

private:
    void setupWifi(char* ssid, char* password) 
    {
        delay(10);
        // We start by connecting to a WiFi network
        
        #ifdef HOMENODE_DEBUG
        Serial.println();
        Serial.print("Connecting to ");
        Serial.println(ssid);
        #endif
        
        WiFi.begin(ssid, password);
        
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

    //void callback(char* topic, unsigned char* payload, unsigned int length){}
    
    void reconnect(char* clientID, char* heartbeatTopic) 
    {
        // Loop until we're reconnected
        while (!_client.connected()) 
        {
            Serial.print("Attempting MQTT connection...");
            // Attempt to connect
            if (_client.connect(clientID))
            {
                Serial.println("connected");
                // Once connected, publish an announcement...
                _client.publish(heartbeatTopic, "hello world");
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
};
