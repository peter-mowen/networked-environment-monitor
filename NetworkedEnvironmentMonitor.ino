#include <MCP3008.h>
// Configure MCP3008 ADC
#define CS_PIN                      D8
#define CLOCK_PIN                   D5
#define MOSI_PIN                    D7
#define MISO_PIN                    D6

MCP3008 adc(CLOCK_PIN, MOSI_PIN, MISO_PIN, CS_PIN);

//#define BUTTON_PIN              7               // pin number for button input

// define timer values
#define LCD_TIMEOUT             5000            // milliseconds until screen turns off
#define LCD_PERIOD              1000            // in milliseconds
#define MQTT_PERIOD             1*LCD_PERIOD    // how often a message is published to MQTT

// define ADC values
#define MAX_ADC_READING         1024            // max num` on analog to digital converter
#define ADC_REF_VOLTAGE         3.3             // max voltage that could appear on ADC in V

#define DEBUG

//PubSub Stuff
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

const char* ssid = "tuguestwireless";// 
const char* password = "";
const char* mqtt_server = "10.33.46.65";//"192.168.1.10";

WiFiClient espClient;
PubSubClient client(espClient);
const char* clientID = "Peter's BR";
const char* topic0 = "24/temperature";
const char* topic1 = "24/light level";

// OLED libraries and constants
#include <ArducamSSD1306.h>    // Modification of Adafruit_SSD1306 for ESP8266 compatibility
#include <Adafruit_GFX.h>   // Needs a little change in original Adafruit library (See README.txt file)
#include <Wire.h>           // For I2C comm, but needed for not getting compile error

#define OLED_RESET  16  // Pin 15 -RESET digital signal

#define LOGO16_GLCD_HEIGHT 16
#define LOGO16_GLCD_WIDTH  16

ArducamSSD1306 oled(OLED_RESET); // FOR I2C

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

// Global Variables
unsigned long previousMillisMQTT;
unsigned long currentMillis;

unsigned long startMillisLCD;

int currentButtonState = 0;
int previousButtonState = 0;

void setup()
{
    // Open Serial port. Set Baud rate to 115200
    Serial.begin(115200);
    // Send out startup phrase
    Serial.println("Arduino Starting Up...");
    
    // SSD1306 Init
    oled.begin();  // Switch OLED
    oled.clearDisplay();
    
    //Set text size, color, and position
    clearAndUpdateTitle("Starting");
    // initialize pin to listen for button press
    //pinMode(BUTTON_PIN, INPUT);

    /*
    //Setup MQTT
    setup_wifi();
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
    if (!client.connected()) { reconnect(); }
    */
    // initial temperature reading
    float temperature = readAmbientTemperature();
    char temperature_msg[7];
    sprintf(temperature_msg, "%.2f", temperature);

    // initial light level reading
    double lightLevel = readLightLevel();
    char lightLevel_msg[7];
    sprintf(lightLevel_msg, "%.2f", lightLevel);
    
    publishData(topic0, temperature_msg);
    publishData(topic1, lightLevel_msg);
    previousMillisMQTT = millis();  // initial start time

    clearAndUpdateTitle(clientID);
    printDataToOLED(temperature);
    //delay(LCD_TIMEOUT);
    //turnOffLCD();
}

void loop()
{
    //double ldrResistance = readLightLevel();
    // package data to send to serial
    //String data1 = String("ldr resistance: " + String(ldrResistance));

    /*
    if (!client.connected()) { reconnect(); }
    client.loop();
    */
    currentMillis = millis();
    
    if ((currentMillis - previousMillisMQTT >= MQTT_PERIOD))
    {
        // Get temperature reading
        float temperature = readAmbientTemperature();
        char temperature_msg[7];
        sprintf(temperature_msg, "%.2f", temperature);
        //publishData(topic0, temperature_msg);
        
        // Get light level reading
        double lightLevel = readLightLevel();
        char lightLevel_msg[7];
        sprintf(lightLevel_msg, "%.2f", lightLevel);
        //publishData(topic1, lightLevel_msg);
        
        previousMillisMQTT = currentMillis;
        printDataToOLED(temperature);
    }
    
    /* TODO: move this to its own function
    currentButtonState = digitalRead(BUTTON_PIN);
    //Serial.println(String(currentButtonState));
    //Serial.println(String(currentButtonState) + " " + String(previousButtonState));
    if ((currentButtonState == HIGH)&&(previousButtonState == LOW))
    {
        previousButtonState = currentButtonState;
        digitalWrite(LCD_POWER_PIN, HIGH);
        printDataToOLED(temperature);
        startMillisLCD = millis();  // initial start time
    } else if ((currentButtonState == LOW)&&(previousButtonState == HIGH))
    {
        //Serial.println(String(currentMillis - startMillisLCD));
        if (currentMillis - startMillisLCD >= LCD_TIMEOUT)
        {
              turnOffLCD();
              startMillisLCD = millis();
              previousButtonState = currentButtonState;
        }
    }
    */
}

/*
 * MQTT Functions
 */
void setup_wifi() 
{
    String connecting = "Connecting to: ";
    delay(10);
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print(connecting);
    Serial.println(ssid);

    oled.setTextSize(1);
    oled.setCursor(0,16);
    oled.println(connecting);
    oled.println(ssid);
    oled.display();
    WiFi.begin(ssid, password);

    int attempts = 0;
    int maxAttempts = 20;
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
            haltOnError(failedConnectMsg);
        }
    }
    
    Serial.println("");
    oled.println("");
    oled.display();
    String connectedMsg = "WiFi connected";
    Serial.println(connectedMsg);
    oled.println(connectedMsg);
    String ipMsg = "IP address: ";
    Serial.println(ipMsg);
    Serial.println(WiFi.localIP());
    oled.println(ipMsg);
    oled.println((WiFi.localIP()));
    oled.display();
    delay(2000);
}

void callback(char* topic, byte* payload, unsigned int length)
{
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for (int i = 0; i < length; i++) { Serial.print((char)payload[i]); }
    Serial.println();
}

void reconnect() {
    // Loop until we're reconnected
    oled.fillRect(0,16,127,127, BLACK);
    oled.setTextSize(1);
    oled.setCursor(0,16);
    int attempt = 0;
    int maxAttempts = 2;
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
            delay(2000);
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
                + "  failed, rc= " + String(client.state()));
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

/*
 * Environment Monitor Functions
 */
float readAmbientTemperature()
{
    float thermistorVoltage = readSensor(0); 
    float temperature = (thermistorVoltage - 0.5)*100;    // [degrees C]
    return temperature;
}



double readLightLevel()
{
    double resistorVoltage = readSensor(1);   // [V]
    double ldrVoltage = MAX_ADC_READING - resistorVoltage;
    double ldrResistance = (ldrVoltage/resistorVoltage)*10000; //10k is resistor value
    return ldrResistance;
}

float readSensor(int channelNum)
{   
    int sum = 0;
    int numOfSensorReads = 10;
    for (int i = 0; i< numOfSensorReads ; i++) { 
        sum += adc.readADC(channelNum); 
        delay(100);}
    int sensorVal = sum / numOfSensorReads;
    #ifdef DEBUG
    Serial.println("sensorVal = " + String(sensorVal));
    #endif
    float voltage = ( (float)sensorVal / MAX_ADC_READING ) * ADC_REF_VOLTAGE;   // [V]
    return voltage;
}

void publishData(const char* topic, const char* msg)
{
    client.publish(topic, msg);
    #ifdef DEBUG
        Serial.println("topic: " + String(topic) + " , msg: " + String(msg));
    #endif
}

void printDataToOLED(float temperature)
{
    int roundedTemp = temperature + 0.5;
    String displayTemp = "Temp: "+ String(roundedTemp);

    // Update OLED display
    clearBody(); // sets cursor to begining of body
    oled.setTextSize(2);
    oled.print(displayTemp);
    oled.drawBitmap(displayTemp.length()*12, 16, degreeSymbol, 8, 8, WHITE);
    oled.println(" C");
    oled.display();
    /*    
    lcd.setCursor(0,1);

    double ldrDouble = ldrResistance;
    
    String prefix = " ";
    if (ldrResistance > 1000000)
    {
        ldrDouble = (ldrResistance) / 1000000.0;
        prefix = String("M");
    } else if (ldrResistance > 1000)
    {
        ldrDouble = (ldrResistance) / 1000.0;
        prefix = String("k");
    } else if (ldrResistance < 0)
    {
        prefix = "OL";
    }
    //Serial.println(prefix);
    if (prefix.equals("OL"))
    {
        lcd.print(String("LDR: INF"));
        lcd.write(byte(1));
    } else if (!prefix.equals(" "))
    {
        lcd.print(String(String("LDR: ")  + String(ldrDouble, 2) + prefix));
        lcd.write(byte(1));
    } else
    {
        lcd.print(String(String("LDR: ")  + String(ldrDouble, 2)));
        lcd.write(byte(1));
    }
    */
}

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

/*
 * Display control functions
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

 void clearBody()
 {
    oled.fillRect(0,16,127,127, BLACK);
    oled.setCursor(0,16);
    
 }
