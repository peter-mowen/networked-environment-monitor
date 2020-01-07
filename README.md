# Networked Thermostat

## Description
The Networked Thermostat is a node in a distributed sensor network. It uses MQTT to communicate back to a central server.

This project is motivated by the fact that my the temperature in my home is controlled by the thermostat in my dining room, but I never spend time in my dining room. By placing sensor nodes throughout the house, I can write a program to control my boiler in winter and air conditioners in summer based on the temperatures of the rooms in which I actually spend time.

This is the code for the node that connects to the boiler. During normal operation, it will read the local temperature. If it is connected to the Wifi network, then it will query the network for the temperatures from the other rooms, calculate the average, and use that to control the boiler. If it is not connected to the Wifi network, then it will use the local temperature to control the boiler.

### Parts
- ESP-12e: microcontroller around which everything is built
- TMP36 Precision Linear Analog Output: thermistor to read temperature
- si7021 Humidity and Temperature sensor
- UCTRONICS 0.96 Inch 128X64 Yellow Blue OLED module: screen

### Build Instructions
- Download this repo to your local machine.
- Put the NetworkedThermostat folder in your Arduino folder (default location is ~Arduino).
- Open the NetworkedThermostat.ino file with the Arduino IDE.
- Add the ESP8266 module to your Arduino IDE
- Connect the ESP-12e module to your computer
- Load the sketch to the ESP-12e using the Arduino IDE

### Dependencies
The following is a list of libraries which this sketch uses as well as a short description:
- PubSubClient - enable MQTT communication
- ArducamSSD1306 - control OLED display
- Adafruit_GFX - help control OLED display
- Wire - needed for I2C communication

## Future Updates
- Push buttons to set desired temperature
- Battery pack and low battery alert
- Humidity sensor (able to measure with current sensor but not implemented)
