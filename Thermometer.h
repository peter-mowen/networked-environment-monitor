#ifndef MOWEN_THERMOMETER
#define MOWEN_THERMOMETER

#include <Arduino.h>
#include <Wire.h>

#include "HomeNode.h"
#include "Si7021.h"

class Thermometer
{

public:

    Thermometer():_temperature(0), _node(nullptr), _sensor(nullptr) { }

    void setup(Si7021 *sensor);

    void setup(Si7021 *sensor, HomeNode *node);

    void loop();

    float getTemperatureVal() { return _temperature; }

private:
    float _temperature;  // current temperature
    Si7021 *_sensor;
    HomeNode *_node;

    void publishTemperature();
};

#endif  // MOWEN_THERMOMETER
