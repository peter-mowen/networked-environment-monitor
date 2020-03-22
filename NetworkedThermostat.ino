#include "Thermometer.cpp"

#define SDA D2
#define SCL D1

Thermometer therm(SDA, SCL);
void setup()
{
    Serial.begin(115200);
    therm.setup();
}

void loop()
{
    therm.loop();   // reads temperature from 
    Serial.println("Temperature = " + (String)therm.readAmbientTemperature());
    delay(500);
}
