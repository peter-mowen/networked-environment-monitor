#include <ESP8266WiFi.h>
#include <PubSubClient.h>

class HomeNode
{
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
        _ssid = ssid;
        _password = password;
        _mqttServer = mqttServer;

        setupWifi(ssid, password);
        
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
};
