#include "Thermometer.h"

void Thermometer::setup(Si7021 *sensor)
{
    _sensor = sensor;
}

void Thermometer::setup(Si7021 *sensor, HomeNode *node)
{
    _sensor = sensor;
    _node = node;
}

void Thermometer::loop()
{
    if (_sensor == nullptr)
    {
        Serial.println("No Si7021 object!");
        return;
    }
    // Read temperature
    _temperature = _sensor->readTemperatureC();
    if (_node)
    {
        _node->loop();
        delay(75);
        publishTemperature();
    }
}

void Thermometer::publishTemperature()
{
    if (_node == nullptr)
    {
        Serial.println("No HomeNode object!");
        return;
    }
    // Create topic from clientID
    char* clientID = _node->getClientID();
    char* subTopic= "temperature";
    char topic[40];
    sprintf(topic, "%s/%s/", clientID, subTopic);

    // Loop the node to make sure it's connected
    _node->loop();

    // Package temperature in message
    char temperatureMsg[6];
    sprintf(temperatureMsg, "%0.1f", _temperature);

    #ifdef THERMOMETER_DEBUG
    Serial.println("Temperature = " + (String)temperatureMsg);
    #endif

    _node->publishMsg(topic, temperatureMsg);
}
