#ifndef MOWEN_SI7021
#define MOWEN_SI7021

#include <Arduino.h>
#include <Wire.h>

class Si7021
{

public:

    Si7021() { }

    Si7021(int sda, int scl):_sda(sda), _scl(scl) {  }

    void setup();

    float readTemperatureC();

    float readTemperatureF();

private:
    int _sda;            // SDA Pin
    int _scl;            // SCL Pin
    int addr = 0x40;
    float _temperature;

};

#endif  //MOWEN_SI7021
