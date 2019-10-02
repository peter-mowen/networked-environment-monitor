
// Uncomment different ones to show different debug messages
//#define DEBUG_SENSOR            // print sensor's ADC value to serial
//#define DEBUG_TEMPERATURE       // print temperature sensor voltage and calculated temperature to serial

/* Configure pins for MCP3008 ADC  */
#include <MCP3008.h>
#define CS_PIN      D8
#define CLOCK_PIN   D5
#define MOSI_PIN    D7
#define MISO_PIN    D6
// initialize adc object
MCP3008 adc(CLOCK_PIN, MOSI_PIN, MISO_PIN, CS_PIN);

/* define timer values */
#define MQTT_PERIOD 1000            // how often a message is published to MQTT

/* define ADC constants */
#define MAX_ADC_READING 1023        // max num` on analog to digital converter
#define ADC_REF_VOLTAGE 3.3         // max voltage that could appear on ADC in V

/* Configure PubSub */
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// set wifi ssid, password, and the ip address of the mqtt broker
const char* ssid = "tuguestwireless";
const char* password = "";
const char* mqtt_server = "10.33.46.65";

WiFiClient espClient;
PubSubClient client(espClient);

const char* clientID = "RM_MNTR 01";
const char* topic0 = "24/temperature";
const char* topic1 = "24/light level";

/* OLED libraries and constants. Copy/pasted from example code */
#include <ArducamSSD1306.h> // Modification of Adafruit_SSD1306 for ESP8266 compatibility
#include <Adafruit_GFX.h>   // Needs a little change in original Adafruit library (See README.txt file)
#include <Wire.h>           // For I2C comm, but needed for not getting compile error

#define OLED_RESET  16      // Pin 15 -RESET digital signal

#define LOGO16_GLCD_HEIGHT 16
#define LOGO16_GLCD_WIDTH  16

ArducamSSD1306 oled(OLED_RESET); // FOR I2C

// Byte array to print the degree symbol to the OLED
byte degreeSymbol[8] = {
    B01111,
    B01001,
    B01001,
    B01111,
    B00000,
    B00000,
    B00000,
    B00000,
};

/* Global Variables */
unsigned long previousPublishMillis;   // previous millis used to control when MQTT is sent
unsigned long currentMillis;        // current millis

/**
 * setup()
 *  Configure serial, screen, and wifi. Take intial temperature reading.
 */
void setup()
{
    // Open Serial port
    Serial.begin(115200);
    // Send out startup phrase
    Serial.println("Arduino Starting Up...");
    
    // Setup display
    oled.begin();  // Switch OLED
    oled.clearDisplay();
    
    //Set text size, color, and position
    clearAndUpdateTitle("Starting");

    /*
    // Setup wifi
    setup_wifi();
    // Setup MQTT
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
    if (!client.connected()) { reconnect(); }
    */
    // Take initial temperature reading
    float temperature = readAmbientTemperature();
    char temperature_msg[7];
    sprintf(temperature_msg, "%.2f", temperature);

    // Publish initial readings to MQTT
    //publishData(topic0, temperature_msg);

    // Set Initial start time 
    previousPublishMillis = millis();  // initial start time
    /* 
     * TODO: Get the name of the room from the mqtt broker 
     *  based on the clientID to set as the device title
     */
    clearAndUpdateTitle(clientID);
    printDataToOLED(temperature);
}

/**
 * Check if connected to wifi and reconnect if not.
 * Check sensors and send results to MQTT broker and screen.
 */
void loop()
{
    /*
    if (!client.connected()) { reconnect(); }
    client.loop();
    */
    currentMillis = millis();
    
    if ((currentMillis - previousPublishMillis >= MQTT_PERIOD))
    {
        // Get temperature reading
        float temperature = readAmbientTemperature();
        char temperature_msg[7];
        sprintf(temperature_msg, "%.2f", temperature);
        //publishData(topic0, temperature_msg);
        
        previousPublishMillis = currentMillis;
        printDataToOLED(temperature);
    }
}

/**
 * Code from the pubsubclient esp8266 example that connects to the wifi network.
 *  I added code to print output to the screen in addition to the serial port
 *  and added a max attempts before erroring out.
 */
void setup_wifi() 
{
    String connecting = "Connecting to: ";
    delay(10);
    Serial.println();
    Serial.print(connecting);
    Serial.println(ssid);

    // Setup Screen for printing wifi info
    oled.setTextSize(1);
    oled.setCursor(0,16);
    
    // Print message to screen
    oled.println(connecting);
    oled.println(ssid);
    oled.display();

    // We start by connecting to a WiFi network
    WiFi.begin(ssid, password);

    int attempts = 0;       // intialize number of attempts
    int maxAttempts = 20;   // max attempts until erroring out
    while (WiFi.status() != WL_CONNECTED) 
    {
        delay(500);
        Serial.print(".");
        oled.print(".");
        oled.display();
        attempts++;
        if (attempts == maxAttempts)
        {
            String failedConnectMsg = "Failed to connect to WiFi network!";
            haltOnError(failedConnectMsg);  // error out and print error to screen
        }
    }
    
    Serial.println("");
    oled.println("");
    oled.display();
    // Convey success message to serial and screen
    String connectedMsg = "WiFi connected";
    Serial.println(connectedMsg);
    oled.println(connectedMsg);
    // Print IP address to serial and screen
    String ipMsg = "IP address: ";
    Serial.println(ipMsg);
    Serial.println(WiFi.localIP());
    oled.println(ipMsg);
    oled.println((WiFi.localIP()));
    oled.display();
    delay(2000); // wait before moving on so user has time to read the info
}

/**
 * Called when a new message arrives. Copied from pubsubclient for esp8266.
 *  Function parameter descriptions pulled from pubsubclient documentation site:
 *  https://pubsubclient.knolleary.net/api.html#callback
 *  
 *  \param[in]  the topic the message arrived on (const char[])
 *  \param[in]  the message payload (byte array)
 *  \param[in]  the length of the message payload (unsigned int)
 */
void callback(char* topic, byte* payload, unsigned int length)
{
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for (int i = 0; i < length; i++) { Serial.print((char)payload[i]); }
    Serial.println();
}

/**
 * Code from the pubsubclient esp8266 example that reconnects to the wifi network.
 *  I added code to print output to the screen in addition to the serial port
 */
void reconnect() {
    // Reset bottom portion of screen to black to start printing new info
    oled.fillRect(0,16,127,127, BLACK);
    oled.setTextSize(1);
    oled.setCursor(0,16);
    
    int attempt = 0;        // initialize number of attempts
    int maxAttempts = 2;    // set max attempts
    // Loop until connected or max attempts is reached
    while (!client.connected()) {
        attempt++;
        Serial.println("Attempting to connect to MQTT broker at:");
        Serial.println(mqtt_server);
        oled.println("Attempting to connect");
        oled.setCursor(7, 25);
        oled.println("to MQTT broker at:");
        oled.setCursor(14, 34);
        oled.println(mqtt_server);
        oled.display();
        // Attempt to connect
        if (client.connect(clientID))
        {
            Serial.println("connected");
            oled.println("connected");
            oled.display();
            delay(2000);    // wait so user can read screen
            // Once connected, publish an announcement...
            const char* heartbeat = "heartbeat";
            client.publish( heartbeat, clientID);
            // ... and resubscribe
        } else 
        {
            String failedMQTT = "failed, rc=" + String(client.state()) + "\ntry again in 5s";
            if (attempt == maxAttempts)
            {
                haltOnError("Failed to connect to MQTT broker after " 
                + String(maxAttempts) + "  attempts. Reason:\n\n" 
                + "  failed, rc= " + String(client.state())); // error out and print error to screen
            } 
            else 
            {
                Serial.println(failedMQTT);
                oled.println(failedMQTT);
                oled.display();
                // Wait 5 seconds before retrying
                delay(5000);
                clearBody();
            }
        }
    }
}

/**
 * Publish data to MQTT broker. Taken from pubsubclient esp8266 example.
 *  Input list descriptions pulled from pubsubclient documentation site:
 *  https://pubsubclient.knolleary.net/api.html#publish1
 *  
 * \param[in]   topic - the topic to publish to (const char[])
 * \param[in]   msg - the message to publish (const char[])
 */
void publishData(const char* topic, const char* msg)
{
    client.publish(topic, msg);
    #ifdef DEBUG
        Serial.println("topic: " + String(topic) + " , msg: " + String(msg));
    #endif
}

/**
 * Read thermistor and convert voltage to temperature in degrees celsius
 *  \return temperature in degrees celsius
 */
float readAmbientTemperature()
{
    float thermistorVoltage = readSensor(0);
    
    #ifdef DEBUG_TEMPERATURE
    Serial.println("Temperature Voltage = " + String(thermistorVoltage));
    #endif
    
    float temperature = (thermistorVoltage - 0.5)*100;    // [degrees C]
    
    #ifdef DEBUG_TEMPERATURE
    Serial.println("temperature = " + String(temperature));
    #endif
    
    return temperature;
}

/**
 * Reads the sensor value from the MCP3008 and converts it into a voltage
 *  
 *  \param[in]  channelNum - Channel number on MCP3308 to read
 *  
 *  \return voltage
 */
float readSensor(int channelNum)
{   
    int sum = 0;
    int numOfSensorReads = 10;
    for (int i = 0; i< numOfSensorReads ; i++) { 
        sum += adc.readADC(channelNum); 
        delay(100);}
    int sensorVal = sum / numOfSensorReads;
    #ifdef DEBUG_SENSOR
    Serial.println("sensorVal = " + String(sensorVal));
    #endif
    float voltage = ( (float)sensorVal / MAX_ADC_READING ) * ADC_REF_VOLTAGE;   // [V]
    return voltage;
}

/**
 * Print an error message to the blue part of the OLED and go into an infinite loop
 * 
 * \param[in] errMsg - error message to convey what caused the crash
 * 
 */
void haltOnError(String errMsg)
{
    Serial.println("");
    Serial.println("Halted for the following reason:");
    Serial.println("\t" + errMsg);

    clearAndUpdateTitle("ERROR:");
    oled.fillRect(0,16,127,127, BLACK);
    oled.setTextSize(1);
    oled.println(errMsg);
    oled.display();
    
    while (true) { ESP.wdtFeed(); }; // feed the watchdog and hang here forever
}

/**
 * Clears the yellow section of the OLED and prints a new title
 * 
 * \param[in] title - new title to print
 */
 void clearAndUpdateTitle(String title)
 {
    oled.fillRect(0,0,127,16, BLACK);
    oled.setTextSize(2);
    oled.setTextColor(WHITE);
    oled.setCursor(0, 0);
    oled.println(title);
    oled.display();
 }


/**
 * Writes black pixels to the blue part of the display to clear it
 *  and sets the cursor back to the start of that section.
 */
 void clearBody()
 {
    oled.fillRect(0,16,127,127, BLACK);
    oled.setCursor(0,16);
    
 }

 /**
 * Print data to the blue text part of the OLED display.
 * 
 * Right now, this only prints temperature but I might expand 
 *  it to print other data in the future
 *  
 *  \param[in] temperature
 */
void printDataToOLED(float temperature)
{
    int roundedTemp = temperature + 0.5;
    String displayTemp = "Temp: "+ String(roundedTemp);

    clearBody(); // sets cursor to begining of body
    oled.setTextSize(2);
    oled.print(displayTemp);
    oled.drawBitmap(displayTemp.length()*12, 16, degreeSymbol, 8, 8, WHITE);
    oled.println(" C");
    oled.display();
}
