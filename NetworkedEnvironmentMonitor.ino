// define pins
#define ANALOG_IN                 A0              // Pin number for input from the multiplexer
#define BIT_0                     0              // least significant bit
#define BIT_1                     2              //
#define BIT_2                     14              // most significant bit

//#define BUTTON_PIN              7               // pin number for button input

// define timer values
#define LCD_TIMEOUT             5000            // milliseconds until screen turns off
#define LCD_PERIOD              1000            // in milliseconds
#define MQTT_PERIOD             1*LCD_PERIOD    // how often a message is published to MQTT

// define ADC values
#define MAX_ADC_READING         1024            // max num` on analog to digital converter
#define ADC_REF_VOLTAGE         3.3             // max voltage that could appear on ADC in V
#define ADC_OFFSET              31              // See comment below
/*
 * Have to add an ADC_OFFSET because the A0 pin on the esp8266 is not as accurate as the 
 * Arduino. I found the above value by setting up a voltage divider using two 1k resistors
 * and found the sensor value for the voltage between the two. On my multimeter, it read
 * 1.65V, which is half of 3.3V. The ADC pin should have read 512 at that point, but it 
 * did not. Took the difference between the ADC pin value and 512 and set it as the offset.
 * This brought the value close enough to what the Arduino was reading.
 */

#define NUM_OF_DATA              1                // how many sets of data can be sent

#define DEBUG

//PubSub Stuff
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

const char* ssid = "LouisTheHome";// "CCP WLAN"
const char* password = "1nTheEventOfFireLookDirectlyAtFire";
const char* mqtt_server = "192.168.1.10";

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
    oled.setTextSize(2);
    oled.setTextColor(WHITE);
    oled.setCursor(0, 0);
    oled.println("Starting..");
    oled.display();
    // initialize pin to listen for button press
    //pinMode(BUTTON_PIN, INPUT);

    //Setup MQTT
    setup_wifi();
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
    if (!client.connected()) { reconnect(); }

    // initialize the multiplexer control pins;
    pinMode(BIT_0, OUTPUT);
    digitalWrite(BIT_0, LOW);
    pinMode(BIT_1, OUTPUT);
    digitalWrite(BIT_1, LOW);
    pinMode(BIT_2, OUTPUT);
    digitalWrite(BIT_2, LOW);
    
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

    
    printDataToOLED(temperature);
    //delay(LCD_TIMEOUT);
    //turnOffLCD();
}

void loop()
{
    //double ldrResistance = readLightLevel();
    // package data to send to serial
    //String data1 = String("ldr resistance: " + String(ldrResistance));

    if (!client.connected()) { reconnect(); }
    client.loop();
    
    currentMillis = millis();
    
    if ((currentMillis - previousMillisMQTT >= MQTT_PERIOD))
    {
        // Get temperature reading
        float temperature = readAmbientTemperature();
        char temperature_msg[7];
        sprintf(temperature_msg, "%.2f", temperature);
        publishData(topic0, temperature_msg);
        
        // Get light level reading
        double lightLevel = readLightLevel();
        char lightLevel_msg[7];
        sprintf(lightLevel_msg, "%.2f", lightLevel);
        publishData(topic1, lightLevel_msg);
        
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
    
    while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    oled.print(".");
    oled.display();
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
    oled.setTextColor(WHITE, BLACK);
    oled.setCursor(0,16);
    
    while (!client.connected()) {
        Serial.println("Attempting to connect to MQTT broker at:");
        Serial.println(mqtt_server);
        oled.println("Attempting to connect");
        oled.setCursor(7, 23);
        oled.println("to MQTT broker at:");
        oled.setCursor(7, 30);
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
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}

/*
 * Environment Monitor Functions
 */
float readAmbientTemperature()
{
    float thermistorVoltage = readSensor(0); // select pin 0 on cd4051b 
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

float readSensor(int input)
{   
    // select analog input on the cd4051b
    switch(input)
    {
        case 0:
            selectMultiplexerChannel(0);
            break;
        case 1:
            selectMultiplexerChannel(1);
            break;
    }
    delay(1); // debounce time
    int sum = 0;
    int numOfSensorReads = 10;
    for (int i = 0; i< numOfSensorReads ; i++) { sum += analogRead(ANALOG_IN) - ADC_OFFSET; }
    int sensorVal = sum / numOfSensorReads;
    #ifdef DEBUG
    Serial.println("sensorVal = " + String(sensorVal));
    #endif
    float voltage = ( (float)sensorVal / MAX_ADC_READING ) * ADC_REF_VOLTAGE;   // [V]
    return voltage;
}

void selectMultiplexerChannel(int input)
{
    digitalWrite(BIT_0, bitRead(input, 0));
    digitalWrite(BIT_1, bitRead(input, 1));
    digitalWrite(BIT_2, bitRead(input, 2));
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
    // Clear the buffer.
    oled.clearDisplay();
    //Set text size, color, and position
    oled.setTextSize(2);
    oled.setTextColor(WHITE);
    oled.setCursor(0,0);
    oled.println(clientID);
    // Update OLED display
    oled.setCursor(0,16);
    int roundedTemp = temperature + 0.5;
    String displayTemp = "Temp: "+ String(roundedTemp);
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

/*
void turnOffLCD(){
    lcd.clear();
    digitalWrite(LCD_POWER_PIN, LOW);
}
*/
