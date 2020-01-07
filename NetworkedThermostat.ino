// Uncomment different ones to show different debug messages
#define DEBUG
#define DEBUG_SENSOR            // print sensor's ADC value to serial
#define DEBUG_TEMPERATURE       // print temperature sensor voltage and calculated temperature to serial
//#define DEBUG_NETWORK

/* define timer values */
#define MQTT_PERIOD 1000            // how often a message is published to MQTT

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

#define si7021Addr 0x40     // SI7021 temperature sensor I2C address is 0x40 (64)

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

byte noMqttSymbolL[16] = {
    0x80, 0x48, 0x3C, 0x3A, 0x08, 0x0C, 0x0A, 0x09,
    0x09, 0x0A, 0x0C, 0x08, 0x18, 0x28, 0x48, 0x00
};

byte noMqttSymbolR[16] = {
    0x00, 0x12, 0x14, 0x18, 0x10, 0x30, 0x50, 0x90,
    0x90, 0x50, 0x30, 0x10, 0x5C, 0x3C, 0x12, 0x00
};

byte questionMarkSymbolL[16] = { 
    0x00, 0x1F, 0x3F, 0x60, 0x60, 0x60, 0x38, 0x11,
    0x01, 0x01, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00
};

byte questionMarkSymbolR[16] = {
    0x00, 0xF0, 0xF0, 0x18, 0x18, 0x18, 0x30, 0xF0,
    0xC0, 0x80, 0x80, 0x80, 0x00, 0x80, 0x80, 0x00
};

uint8_t mqttXpos = 95;

/* Global Variables */
unsigned long previousPublishMillis;   // previous millis used to control when MQTT is sent
unsigned long currentMillis;        // current millis

float desiredTemp = 20;           // temperature we're shooting for
float upperBound= 3;                // upper bound on temperature tolerance
float lowerBound = 0.5;             // lower bound on temperature tolerance

#define relayPin D3                // pin the controls relay that controls boiler

/**
 * Code from the pubsubclient esp8266 example that connects to the wifi network.
 *  I added code to print output to the screen in addition to the serial port
 *  and added a max attempts before erroring out.
 */
bool setup_wifi() 
{
    #ifdef DEBUG_NETWORK
    clearBody();
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
    #endif
    
    // We start by connecting to a WiFi network
    WiFi.begin(ssid, password);

    int attempts = 0;       // intialize number of attempts
    int maxAttempts = 20;   // max attempts until erroring out
    while (WiFi.status() != WL_CONNECTED) 
    {
        delay(500);
        
        #ifdef DEBUG_NETWORK
        Serial.print(".");
        
        
        oled.print(".");
        oled.display();
        #endif
        if (attempts % 2 == 0) {drawTwoByteSymbol(wifiSymbolL, wifiSymbolR, wifiXpos, 0); oled.display();}
        else { oled.fillRect(wifiXpos, 0, 16, 16, BLACK); oled.display(); }
        
        attempts++;
        
        if (attempts == maxAttempts)
        {
            String failedConnectMsg = "Failed to connect to WiFi network!";
            
            #ifdef DEBUG_NETWORK
            Serial.println(failedConnectMsg);
            
            oled.println();
            oled.print(failedConnectMsg);
            #endif
            
            drawTwoByteSymbol(noWifiSymbolL, noWifiSymbolR, 111, 0);
            oled.display();

            #ifdef DEBUG_NETWORK
            delay(2000);
            #endif
            return false;
        }
    }
    
    // Convey success message

    #ifdef DEBUG_NETWORK
    String connectedMsg = "WiFi connected";
    String ipMsg = "IP address: ";
    
    Serial.println("");
    Serial.println(connectedMsg);
    Serial.println(ipMsg);
    Serial.println(WiFi.localIP());
    
    
    oled.println();    
    oled.println(connectedMsg);
    oled.println(ipMsg);
    oled.println((WiFi.localIP()));
    #endif
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
    #ifdef DEBUG_NETWORK
    clearBody();
    #endif
    
    int attempt = 0;        // initialize number of attempts
    int maxAttempts = 2;    // set max attempts
    
    // Loop until connected or max attempts is reached
    while (!client.connected()) {
        attempt++;
        
        #ifdef DEBUG_NETWORK
        Serial.println("Attempting to connect to MQTT broker at:");
        Serial.println(mqtt_server);
        
        
        oled.println("Attempting to connect");
        oled.setCursor(7, 25);
        oled.println("to MQTT broker at:");
        oled.setCursor(14, 34);
        oled.println(mqtt_server);
        oled.display();
        #endif
        
        // Attempt to connect
        drawTwoByteSymbol(questionMarkSymbolL, questionMarkSymbolR, mqttXpos, 0);
        oled.display();
        
        if (client.connect(clientID))
        {
            #ifdef DEBUG_NETWORK
            Serial.println("connected");
            oled.println("connected");
            delay(2000);    // wait so user can read screen
            #endif
            
            drawTwoByteSymbol(mqttSymbolL, mqttSymbolR, mqttXpos, 0);
            oled.display();
            
        } 
        else 
        {
            String failedMQTT = "failed, rc=" + String(client.state());
            if (attempt == maxAttempts)
            {
                #ifdef DEBUG_NETWORK
                oled.println("Failed to connect to MQTT broker");
                delay(2000);
                #endif
                
                drawTwoByteSymbol(noMqttSymbolL, noMqttSymbolR, mqttXpos, 0);
                oled.display();
                return;
            } 
            else 
            {
                #ifdef DEBUG_NETWORK
                Serial.println(failedMQTT);
                oled.println(failedMQTT);
                #endif

                drawTwoByteSymbol(noMqttSymbolL, noMqttSymbolR, mqttXpos, 0);
                oled.display();
                
                // Wait 5 seconds before retrying
                delay(5000);
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
    unsigned int measurement;
    Wire.beginTransmission(si7021Addr);
    // Send temperature measurement code
    Wire.write(0xF3);
    Wire.endTransmission();
    delay(500);

    // Request 2 bytes of data
    Wire.requestFrom(si7021Addr, 2);

    //Read 2 bytes of data for temperature
    if (Wire.available() == 2)
    {
        unsigned int msb = Wire.read();
        unsigned int lsb = Wire.read();
        // Clear the last to bits of LSB to 00.
        // According to datasheet LSB of RH is always xxxxxx10
        lsb &= 0xFC;
        measurement = msb << 8 | lsb;
    }

    // Convert the data
    float temperature = ((175.72 * measurement) / 65536.0) - 46.85;
    Serial.print("Temperature: ");
    Serial.println(temperature);
    
    return temperature;
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

/** Draw a two byte image to OLED
 *  
 *  \param[in] one - left byte of character to be drawn
 *  \para,[in] two - right byte of character to be drawn
 * 
 */
void drawTwoByteSymbol(byte* left, byte* right, int x, int y)
{
    // clear space where wifi symbol will be drawn
    oled.fillRect(x,y,16,16, BLACK);
    // draw first symbol
    oled.drawBitmap(x,   y, left, 8, 16, WHITE);
    // draw second symbol
    oled.drawBitmap(x+8, y, right, 8, 16, WHITE);
}

/**
 * Toggle boiler based on inputTemp, desiredTemp, and the 
 *  upper and lower bounds aroundt he desiredTemp
 * 
 * \param(in) inputTemper - temperature to compare to desired to
 */
void setBoiler(float inputTemp)
{
    if (inputTemp < desiredTemp - lowerBound) { digitalWrite(relayPin, HIGH); }
    else if (inputTemp > desiredTemp + upperBound) { digitalWrite(relayPin, LOW); }
}


/**
 * setup()
 *  Configure serial, screen, and wifi. Take intial temperature reading.
 */
void setup()
{
    #ifdef DEBUG
    // Open Serial port
    Serial.begin(115200);
    // Send out startup phrase
    Serial.println("Arduino Starting Up...");
    #endif

    // Make relayPin an output and make sure it's LOW
    pinMode(relayPin, OUTPUT);
    digitalWrite(relayPin, LOW);
    
    // Setup display
    oled.begin();  // Switch OLED
    oled.clearDisplay();
    
    //Set text size, color, and position
    oled.setTextSize(2);
    oled.setTextColor(WHITE);
    oled.setCursor(0, 0);

    // Setup si7021 temperature sensor
    Wire.begin(D2, D1);
    Wire.beginTransmission(si7021Addr);
    Wire.write(0xFE); // Write reset command
    Wire.endTransmission();
    delay(300);
    
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
        drawTwoByteSymbol(noMqttSymbolL, noMqttSymbolR, mqttXpos, 0);
    }
    
    // Take initial temperature reading
    float temperature = readAmbientTemperature();
    char temperature_msg[7];
    sprintf(temperature_msg, "%.2f", temperature);

    // Publish initial readings to MQTT
    if (client.connected()) { publishData(topic0, temperature_msg); }
    
    // Set Initial start time 
    previousPublishMillis = millis();  // initial start time

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
            setBoiler(temperature); // change this later to the temperature from the broker
        }
        else
        {   // something is wrong. we're not connected to the mqtt broker      
            
            drawTwoByteSymbol(noMqttSymbolL, noMqttSymbolR, mqttXpos, 0);

            // Not connected so control boiler based on local temp this pass.
            setBoiler(temperature);

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
