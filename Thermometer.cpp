#include <Arduino.h>
#include <Wire.h>
#include "HomeNode.cpp"
#include "Si7021.cpp"
class Thermometer
{
    float _temperature;  // current temperature
    Si7021 _sensor;
    HomeNode _node;
    bool networked;
    
public:
    
    Thermometer()
    {
        _temperature = 0;
        networked = false;
    }
    
    void setup(Si7021 sensor)
    {
        _sensor = sensor;
    }

    void setup(Si7021 sensor, HomeNode node)
    {
        _sensor = sensor;
        _node = node;
        networked = true;
    }
    
    void loop()
    {
        // Read temperature
        _temperature = _sensor.readTemperatureC();
        if (networked)
        {
            _node.loop();
            delay(75);
            publishTemperature();
        }
    }

    float getTemperatureVal() { return _temperature; }

private:

    void publishTemperature()
    {
        // Create topic from clientID
        char* clientID = _node.getClientID();
        char* subTopic= "temperature";
        char topic[40];
        sprintf(topic, "%s/%s/", clientID, subTopic);
        
        // Loop the node to make sure it's connected
        _node.loop();
        
        // Package temperature in message
        char temperatureMsg[6];
        sprintf(temperatureMsg, "%0.1f", _temperature);
        
        #ifdef THERMOMETER_DEBUG
        Serial.println("Temperature = " + (String)temperatureMsg);
        #endif
        
        _node.publishMsg(topic, temperatureMsg);
    }
};
