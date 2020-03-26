#include <Arduino.h>
#include <Wire.h>
class Thermometer
{
    float temperature;  // current temperature
    int sda;            // SDA Pin
    int scl;            // SCL Pin
    int addr = 0x40;   // I2C address of si7021
    
public:
    
    Thermometer(int SDA, int SCL)
    {
        temperature = 0;
        sda = SDA;
        scl = SCL;
    }

    void setup()
    {
        // Connect to si7021
        setupSi7021();
    }

    void loop()
    {
        readAmbientTemperature();
    }
    
    float readAmbientTemperature()
    {    
        unsigned int measurement;
        Wire.beginTransmission(addr);
        // Send temperature measurement code
        Wire.write(0xF3);
        Wire.endTransmission();
        delay(20);
    
        // Request 2 bytes of data
        Wire.requestFrom(addr, 2);
    
        //Read 2 bytes of data for temperature
        if (Wire.available() == 2)
        {
            unsigned int msb = Wire.read();
            unsigned int lsb = Wire.read();
            // Clear the last two bits of LSB to 00.
            // According to datasheet LSB of RH is always xxxxxx10
            lsb &= 0xFC;
            measurement = msb << 8 | lsb;
        }
    
        // Convert the data
        temperature = ((175.72 * (float)measurement) / 65536.0) - 46.85;
    }

    float getTemperatureVal()
    {
        return temperature;
    }

private:
    void setupSi7021()
    {
        Wire.begin(sda, scl);
        Wire.beginTransmission(addr);
        Wire.write(0xFE); // Write reset command
        Wire.endTransmission();
        delay(20);
    }
};
