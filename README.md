# Networked Environment Monitor

## Description
The Networked Environment Monitor is a node in a distributed sensor network. It uses MQTT to communicate back to a central server.

This project is motivated by the fact that my the temperature in my home is controlled by the thermostat in my dining room, but I never spend time in my dining room. By placing sensor nodes throughout the house, I can write a program to control my boiler in winter and air conditioners in summer based on the temperatures of the rooms in which I actually spend time.

The code to control the boiler is still forthcoming and will be in a different repository under my Home Automation. It will be done using Node-Red flows.

### Parts
- ESP-12e: microcontroller around which everything is built
- TMP36 Precision Linear Analog Output: thermistor to read temperature
- MCP3008: 8-channel analog to digital converter
- UCTRONICS 0.96 Inch 128X64 Yellow Blue OLED module: screen

## Future Updates
- Push button to toggle display
- Push buttons to set desired temperature
- Battery pack and low batter alert
- Various sensors
    - Light dependent resistor to measure lux
    - Humidity sensor
    - Room occupancy detection
