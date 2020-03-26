#include <Arduino.h>
#include <Wire.h>
#include "HomeNode.cpp"

class Thermometer
{
    float temperature;  // current temperature
    int sda;            // SDA Pin
    int scl;            // SCL Pin
    int addr = 0x40;   // I2C address of si7021
    HomeNode _node;
public:
    
    Thermometer(int SDA, int SCL)
    {
        temperature = 0;
        sda = SDA;
        scl = SCL;
        HomeNode _node();
    }

    void setup()
    {
        // Connect to si7021
        setupSi7021();
    }

    void setup(PubSubClient client, char* ssid, char* password, char* mqttServer)
    {
        // Connect to si7021
        setupSi7021();
        _node.setup(client, ssid, password, mqttServer);
    }
    
    void loop()
    {
        // Read temperature
        readAmbientTemperature();
    }

    void loop(char* clientID, char* heartbeatTopic)
    {
        // Read temperature
        readAmbientTemperature();
        _node.loop(clientID, heartbeatTopic);
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

    float readAmbientTemperature()
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
        temperature = ((175.72 * (float)measurement) / 65536.0) - 46.85;
    }
};
