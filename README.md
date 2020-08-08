# Networked Thermometer

## Description
The Networked Thermostater reads the local temperature and reports it via MQTT.

Note that there are a few custom classes in this repository. Eventually, these
will be broken out into their own repos, but for now they are housed here. The
custom classes are:
- Homenode: handles MQTT
- Si7021: handles I2C communication with si7021 chip. Only supports reading temperature at this time
- Thermometer: uses the Si7021 class and Homenode class to read temperature and send it via MQTT

NetworkedThermoter.ino calls the setup functions and loop functions for each class, so the heavy lifting is done in the code for the classes, not the .ino file.

NetworkedThermoter is setup to turn-on, setup everything, take a temperature reading and send it via MQTT, then put itself into a deep sleep for 10 minutes, rinse and repeat.

### Parts
- ESP-12e: microcontroller around which everything is built
- si7021 Humidity and Temperature sensor
- Protoboard
- Wires

### Getting Code onto the ESP8266
- Download this repo to your local machine. It will have the custom classes already included
- Put the NetworkedThermometer folder in your Arduino folder (default location is ~/Arduino)
- Open the NetworkedThermoter.ino file with the Arduino IDE
- Add the ESP8266 module to your Arduino IDE
- Connect the ESP-12e module to your computer
- Load the sketch to the ESP-12e using the Arduino IDE

### Dependencies
The following is a list of libraries which this sketch uses as well as a short description:
- PubSubClient - enable MQTT communication
- Wire - needed for I2C communication
