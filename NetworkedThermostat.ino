// Uncomment different ones to show different debug messages
#define DEBUG
#define DEBUG_TEMPERATURE       // print temperature sensor voltage and calculated temperature to serial
//#define DEBUG_NETWORK

/* define timer values */
#define MQTT_PERIOD 180000              // how often a message is published to MQTT
#define TEMPERATURE_UPDATE_PERIOD 60000 // how often the temperature is updates on the OLED

/* define ADC constants */
#define MAX_ADC_READING 1023        // max num` on analog to digital converter
#define ADC_REF_VOLTAGE 3.3         // max voltage that could appear on ADC in V

/* Configure PubSub */
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// set wifi ssid, password, and the ip address of the mqtt broker
const char* ssid = "home-automation";
const char* password = "AutomateTheHome";
const char* mqtt_server = "192.168.2.201";

WiFiClient espClient;
PubSubClient client(espClient);

const char* clientID = "NEM 01";
const char* temperatureTopic = "NEM 01/temperature";
const char* relayTopic = "NEM 01/relay";

const char* relayON = "ON";
const char* relayOFF = "OFF";

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
unsigned long previousPublishMillis;    // previous millis used to control when MQTT is sent
unsigned long previousScreenMillis;     // previous millis used to control when temperature is update on the OLED
unsigned long currentMillis;            // current millis

float temperature;
float currentSetTemp;               // current temperature this is set (C)
float previousSetTemp;
float upperBound= 3;                // upper bound on temperature tolerance (C)
float lowerBound = 0.5;             // lower bound on temperature tolerance (C)

#define relayPin D3                 // pin the controls relay that controls boiler
#define upPin D6                    // pin for "up" on temperature select
#define downPin D5                  // pin for "down" on temperature select

boolean currentUpBtnState;
boolean previousUpBtnState;

boolean currentDownBtnState;
boolean previousDownBtnState;

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
 *  Updates global variable temperature
 */
void readAmbientTemperature()
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
    temperature = ((175.72 * measurement) / 65536.0) - 46.85;
    Serial.print("Temperature: ");
    Serial.println(temperature);
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
void printTempToOLED(uint8_t location)
{
    // Default things to print
    int roundedTemp = temperature + 0.5;
    String line = "Temp:" + String(roundedTemp);
    uint8_t yPos = 16;
    
    if (location == 1)
    {
        roundedTemp = currentSetTemp;
        line = "Set: "+ String(roundedTemp);
        yPos = 32;
    }

    uint8_t endOfLabel = line.length()*12-32;
    uint8_t toEndOfLine = 127-endOfLabel ;

    // clear everything after the label
    oled.fillRect(endOfLabel, yPos, toEndOfLine, 16, BLACK);
    // set text size and resest cursor position
    oled.setTextSize(2);
    oled.setCursor(0, yPos);
    // print label plus temperature to screen
    oled.print(line);
    // print degree symbol followed by C
    oled.drawBitmap(line.length()*12, yPos, degreeSymbol, 8, 8, WHITE);
    oled.println(" C");
    // Update display
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
    #ifdef DEBUG
    Serial.println("Checking relay status..");
    #endif
    
    if (inputTemp < currentSetTemp - lowerBound) 
    { 
        #ifdef DEBUG
        Serial.println("Relay ON");
        #endif
        digitalWrite(relayPin, HIGH);
        if (client.connected()) { publishData(relayTopic, relayON); }
    }
    else if (inputTemp > currentSetTemp + upperBound) 
    { 
        #ifdef DEBUG
        Serial.println("Relay OFF");
        #endif
        digitalWrite(relayPin, LOW);
        if (client.connected()) { publishData(relayTopic, relayOFF); }
    }
    else
    {
        #ifdef DEBUG
        Serial.println("currentSetTemp - lowerBound < inputTemp < currentSetTemp + upperBound");
        #endif
    }
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
    Serial.println("Networked Thermostat Starting Up...");
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
    readAmbientTemperature();
    char temperature_msg[6];
    sprintf(temperature_msg, "%.1f", temperature);

    // Publish initial readings to MQTT
    if (client.connected()) { publishData(temperatureTopic, temperature_msg); }

    // Set Initial start time 
    previousPublishMillis = millis();  // initial start time
    previousScreenMillis = millis();

    // Initialize variables for setting preferred temperature
    currentSetTemp = 18;
    previousSetTemp = currentSetTemp;
    
    currentDownBtnState = digitalRead(downPin);
    previousDownBtnState = currentDownBtnState;

    currentUpBtnState = digitalRead(upPin);
    previousUpBtnState = currentUpBtnState;

    // Print the current temperature and the default preferred temperature to OLED
    clearBody();
    printTempToOLED(0);
    printTempToOLED(1);
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
    
    if (currentMillis - previousScreenMillis >= TEMPERATURE_UPDATE_PERIOD)
    {
        // Get temperature reading
        readAmbientTemperature();
        printTempToOLED(0);
        previousScreenMillis = currentMillis;
    }
    
    if ((currentMillis - previousPublishMillis >= MQTT_PERIOD))
    {
        // Check MQTT connection
        reconnect(); // connection might have dropped between last time and now
        if (client.connected())
        {   // we're connected. yay.
            drawTwoByteSymbol(mqttSymbolL, mqttSymbolR, mqttXpos, 0);
            Serial.println("Should be publishing...");
            // Publish temperature
            char temperature_msg[7];
            sprintf(temperature_msg, "%.1f", temperature);
            publishData(temperatureTopic, temperature_msg);
            Serial.println("Should have published...");
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
                #ifdef DEBUG_NETWORK
                Serial.print("Connected to ");
                Serial.print(ssid) ;
                Serial.println(" but not mqtt broker");
                Serial.print("IP Address: ");
                Serial.println(WiFi.localIP());
                #endif

                #ifdef DEBUG
                Serial.println("Not connected to mqtt but am connected to wifi");
                #endif
                
                drawTwoByteSymbol(noMqttSymbolL, noMqttSymbolR, 95, 0);
            }
            else 
            {   // we're not connected to wifi. can't get to the broker with it!

                #ifdef DEBUG
                Serial.println("Not connect to wifi and not connected to mqtt broker");
                #endif
                
                drawTwoByteSymbol(noWifiSymbolL, noWifiSymbolR, 111, 0);

                // Let's try connecting to wifi again
                setup_wifi();
            }
        }

        // Network connectivity doesn't matter for stuff beyond this point
        previousPublishMillis = currentMillis;
    }
    
    // check up button status for set temperature
    boolean released = false;
    currentUpBtnState = digitalRead(upPin);
    if (currentUpBtnState == HIGH) // buttons are active low.
    {
        if (previousUpBtnState == LOW)
        { // button was released after being held down
            released = true;
        }
    }
    previousUpBtnState = currentUpBtnState;
    
    if (released)
    {
        #ifdef DEBUG
        Serial.println("Up button was pressed, then released");
        #endif
        previousSetTemp = currentSetTemp;
        currentSetTemp++;
    }
    
    // reinitialized released and check down button
    released = false;
    currentDownBtnState = digitalRead(downPin);
    if (currentDownBtnState == HIGH) // buttons are active low.
    {
        if (previousDownBtnState == LOW)
        { // button was released after being held down
            released = true;
        }
    }
    previousDownBtnState = currentDownBtnState;
    
    if (released)
    {
        #ifdef DEBUG
        Serial.println("Down button was pressed, then released");
        #endif
        
        previousSetTemp = currentSetTemp;
        currentSetTemp--;
    }
    
    printTempToOLED(1);
    
    if (abs(previousSetTemp - currentSetTemp) > 0.5)
    {
        #ifdef DEBUG
        Serial.println("Set temperature changed!");
        #endif
        
        if (client.connected()) { setBoiler(temperature); }
        else { setBoiler(temperature); }
        
        previousSetTemp = currentSetTemp;
    }
    
}
