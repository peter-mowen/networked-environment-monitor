// Uncomment different ones to show different debug messages
#define DEBUG
#define DEBUG_SENSOR            // print sensor's ADC value to serial
#define DEBUG_TEMPERATURE       // print temperature sensor voltage and calculated temperature to serial
#define DEBUG_NETWORK

/* Configure pins for MCP3008 ADC  */
#include <MCP3008.h>
#define CS_PIN      D8
#define CLOCK_PIN   D5
#define MOSI_PIN    D7
#define MISO_PIN    D6
// initialize adc object
MCP3008 adc(CLOCK_PIN, MOSI_PIN, MISO_PIN, CS_PIN);

/* define timer values */
#define MQTT_PERIOD 15*1000            // how often a message is published to MQTT

/* define ADC constants */
#define MAX_ADC_READING 1023        // max num` on analog to digital converter
#define ADC_REF_VOLTAGE 3.3         // max voltage that could appear on ADC in V

/* Configure PubSub */
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// set wifi ssid, password, and the ip address of the mqtt broker
const char* ssid = "home-automation";
const char* password = "AutomateTheHome";
const char* mqtt_server = "192.168.2.4";

WiFiClient espClient;
PubSubClient client(espClient);

const char* clientID = "NEM 01";
const char* topic0 = "NEM 01/temperature";
const char* topic1 = "NEM 01/light level";

/* OLED libraries and constants. Copy/pasted from example code */
#include <ArducamSSD1306.h> // Modification of Adafruit_SSD1306 for ESP8266 compatibility
#include <Adafruit_GFX.h>   // Adafruit_SSD1306 OLED header file
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

// Byte array to print the wifi symbols to the OLED
byte wifiSymbolL[16] = {
    0x00, 0x0F, 0x10, 0x20, 0x40, 0x07, 0x08, 0x10, 
    0x00, 0x03, 0x04, 0x00, 0x00, 0x01, 0x01, 0x00
};
byte wifiSymbolR[16] = {
    0x00, 0xF0, 0x08, 0x04, 0x02, 0xE0, 0x10, 0x08, 
    0x00, 0xC0, 0x20, 0x00, 0x00, 0x80, 0x80, 0x00
};

byte noWifiSymbolL[16] = {
    0x00, 0x4F, 0x30, 0x30, 0x48, 0x07, 0x0A, 0x11, 
    0x01, 0x03, 0x04, 0x08, 0x10, 0x21, 0x41, 0x00
};
byte noWifiSymbolR[16] = {
    0x00, 0xF2, 0x0C, 0x0C, 0x12, 0xE0, 0x50, 0x88, 
    0x80, 0xC0, 0x20, 0x10, 0x08, 0x84, 0x82, 0x00
};

uint8_t wifiXpos = 111;

byte mqttSymbolL[16] = {
    0x00, 0x08, 0x1C, 0x2A, 0x08, 0x08, 0x08, 0x08,
    0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00
};

byte mqttSymbolR[16] ={
    0x00, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10,
    0x10, 0x10, 0x10, 0x10, 0x54, 0x38, 0x10, 0x00
};

byte NoMqttSymbolL[16] = {
    0x80, 0x48, 0x3C, 0x3A, 0x08, 0x0C, 0x0A, 0x09,
    0x09, 0x0A, 0x0C, 0x08, 0x18, 0x28, 0x48, 0x00
};

byte NoMqttSymbolR[16] ={
    0x00, 0x12, 0x14, 0x18, 0x10, 0x30, 0x50, 0x90,
    0x90, 0x50, 0x30, 0x10, 0x5C, 0x3C, 0x12, 0x00
};

uint8_t mqttXpos = 95;

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
    oled.setTextSize(2);
    oled.setTextColor(WHITE);
    oled.setCursor(0, 0);

    // Setup wifi
    setup_wifi();
    
    // Setup mqtt if wifi is connected
    if (WiFi.status() == WL_CONNECTED)
    {
        // Setup MQTT
        client.setServer(mqtt_server, 1883);
        client.setCallback(callback);
        if (!client.connected()) { reconnect(); }
    } else
    { // not connected to wifi so we can't connect to MQTT broker
        drawTwoByteSymbol(NoMqttSymbolL, NoMqttSymbolR, mqttXpos, 0);
    }
    
    // Take initial temperature reading
    float temperature = readAmbientTemperature();
    char temperature_msg[7];
    sprintf(temperature_msg, "%.2f", temperature);

    // Publish initial readings to MQTT
    if (client.connected()) { publishData(topic0, temperature_msg); }
    
    // Set Initial start time 
    previousPublishMillis = millis();  // initial start time
    /* 
     * TODO: Get the name of the room from the mqtt broker 
     *  based on the clientID to set as the device title
     */
    //clearAndUpdateTitle(clientID);
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

        // Check MQTT connection
        if (client.connected())
        {   // we're connected. yay.
            drawTwoByteSymbol(mqttSymbolL, mqttSymbolR, mqttXpos, 0);

            // Publish temperature
            publishData(topic0, temperature_msg);

            // Request the boiler status from mqtt broker and toggle based on that
        }
        else
        {   // something is wrong. we're not connected to the mqtt broker      
            
            drawTwoByteSymbol(NoMqttSymbolL, NoMqttSymbolR, mqttXpos, 0);

            // Not connected so control boiler based on local temp this pass.

            // Then try to connect to the network again in prepartion for next time
            
            // Let's check the wifi connection
            if (WiFi.status() == WL_CONNECTED)
            {   // we are connected to wifi but not mqtt broker
                #ifdef DEGBUG_NETWORK
                Serial.println("Connected to network but not mqtt broker");
                #endif
            
                drawTwoByteSymbol(wifiSymbolL, wifiSymbolR, 111, 0);

                // Try reconnecting to the mqtt broker
                reconnect();
            }
            else 
            {   // we're not connected to wifi. can't get to the broker with it!

                #ifdef DEGBUG_NETWORK
                Serial.println("Not connect to wifi and not connected to mqtt broker");
                #endif
                
                drawTwoByteSymbol(noWifiSymbolL, noWifiSymbolR, 111, 0);

                // Let's try connecting to wifi again
                setup_wifi();
            }
        }

        // Network connectivity doesn't matter for stuff beyond this point
        previousPublishMillis = currentMillis;
        printDataToOLED(temperature);
    }
}

/**
 * Code from the pubsubclient esp8266 example that connects to the wifi network.
 *  I added code to print output to the screen in addition to the serial port
 *  and added a max attempts before erroring out.
 */
bool setup_wifi() 
{
    clearBody();
    String connecting = "Connecting to: ";
    delay(10);

    #ifdef DEBUG_NETWORK
    Serial.println();
    Serial.print(connecting);
    Serial.println(ssid);
    #endif
    
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
        
        #ifdef DEBUG_NETWORK
        Serial.print(".");
        #endif
        
        oled.print(".");
        oled.display();
        
        attempts++;
        
        if (attempts == maxAttempts)
        {
            String failedConnectMsg = "Failed to connect to WiFi network!";
            
            #ifdef DEBUG_NETWORK
            Serial.println(failedConnectMsg);
            #endif
            
            oled.println();
            oled.print(failedConnectMsg);
            
            drawTwoByteSymbol(noWifiSymbolL, noWifiSymbolR, 111, 0);
            
            oled.display();
            
            delay(2000);
            
            return false;
        }
    }
    
    // Convey success message
    
    String connectedMsg = "WiFi connected";
    String ipMsg = "IP address: ";

    #ifdef DEBUG_NETWORK
    Serial.println("");
    Serial.println(connectedMsg);
    Serial.println(ipMsg);
    Serial.println(WiFi.localIP());
    #endif
    
    oled.println();    
    oled.println(connectedMsg);
    oled.println(ipMsg);
    oled.println((WiFi.localIP()));
    
    drawTwoByteSymbol(wifiSymbolL, wifiSymbolR, wifiXpos, 0);
    
    oled.display();
    
    delay(2000); // wait before moving on so user has time to read the info
    
    return true;
}

/**
 * Called when a new message arrives.
 *  
 *  \param[in]  the topic the message arrived on (const char[])
 *  \param[in]  the message payload (byte array)
 *  \param[in]  the length of the message payload (unsigned int)
 */
void callback(char* topic, byte* payload, unsigned int length){}

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
        
        #ifdef DEBUG
        Serial.println("Attempting to connect to MQTT broker at:");
        Serial.println(mqtt_server);
        #endif
        
        oled.println("Attempting to connect");
        oled.setCursor(7, 25);
        oled.println("to MQTT broker at:");
        oled.setCursor(14, 34);
        oled.println(mqtt_server);
        oled.display();
        
        // Attempt to connect
        if (client.connect(clientID))
        {
            #ifdef DEBUG_NETWORK
            Serial.println("connected");
            #endif
            
            oled.println("connected");
            
            drawTwoByteSymbol(mqttSymbolL, mqttSymbolR, mqttXpos, 0);
            
            oled.display();
            delay(2000);    // wait so user can read screen
        } 
        else 
        {
            String failedMQTT = "failed, rc=" + String(client.state());
            if (attempt == maxAttempts)
            {
                oled.println("Failed to connect to MQTT broker");
                delay(2000);
                return;
            } 
            else 
            {
                #ifdef DEBUG_NETWORK
                Serial.println(failedMQTT);
                #endif
                
                oled.println(failedMQTT);

                drawTwoByteSymbol(NoMqttSymbolL, NoMqttSymbolR, mqttXpos, 0);
                
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


void drawTwoByteSymbol(byte* one, byte* two, int x, int y)
{
    // clear space where wifi symbol will be drawn
    oled.fillRect(x,y,16,16, BLACK);
    // draw first symbol
    oled.drawBitmap(x,   y, one, 8, 16, WHITE);
    // draw second symbol
    oled.drawBitmap(x+8, y, two, 8, 16, WHITE);
}
