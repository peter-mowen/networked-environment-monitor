#include <Arduino.h>
#include <Wire.h>

class Si7021
{
    int _sda;            // SDA Pin
    int _scl;            // SCL Pin
    int addr = 0x40;
    float _temperature;
    
public:

    Si7021()
    {
        
    }

    Si7021(int sda, int scl)
    {
        _sda = sda;
        _scl = scl;
    }
    
    void setup()
    {
        Wire.begin(_sda, _scl);
        Wire.beginTransmission(addr);
        Wire.write(0xFE); // Write reset command
        Wire.endTransmission();
        delay(20);
    }
    
    float readTemperatureC()
    {
        unsigned int measurement;
        Wire.beginTransmission(addr);
        
        // Send command to read temperature
        Wire.write(0xF3);
        Wire.endTransmission();
        
        // wait for chip to get temperature
        delay(20);
    
        // Request temperature data - 2 bytes 
        Wire.requestFrom(addr, 2);
    
        //Read 2 bytes of data for temperature
        if (Wire.available() == 2)
        {
            unsigned int msb = Wire.read();
            unsigned int lsb = Wire.read();
            // Clear the last two bits of LSB to 00.
            // According to datasheet LSB of RH is always xxxxxx10
            lsb &= 0xFC;    // 0xFC = 0b11111100
            measurement = msb << 8 | lsb;
        }
    
        // Convert the data
        _temperature = ((175.72 * (float)measurement) / 65536.0) - 46.85;
        return _temperature;
    }

    float readTemperatureF()
    {
        _temperature = 1.8*readTemperatureC() + 32;
        return _temperature;
    }
};
