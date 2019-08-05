// define pins
#define THERMISTOR_PIN          A0              // Pin number for input from thermistor
//#define LDR_PIN                 A1              // Pin number for light dependent resistor
#define BUTTON_PIN              7               // pin number for button input
//#define LCD_POWER_PIN           8               // Pin number to toggle power to LCD screen

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

const char* ssid = "CCP WLAN"; //"LouisTheHome";
const char* password = ""; //"1nTheEventOfFireLookDirectlyAtFire";
const char* mqtt_server = "10.4.136.255";

WiFiClient espClient;
PubSubClient client(espClient);
const char* clientID = "Monitor01";
const char* topic0 = "Living Room/temp";

// OLED libraries and constants
#include <ArducamSSD1306.h>    // Modification of Adafruit_SSD1306 for ESP8266 compatibility
#include <Adafruit_GFX.h>   // Needs a little change in original Adafruit library (See README.txt file)
#include <Wire.h>           // For I2C comm, but needed for not getting compile error

#define OLED_RESET  16  // Pin 15 -RESET digital signal

#define LOGO16_GLCD_HEIGHT 16
#define LOGO16_GLCD_WIDTH  16

ArducamSSD1306 display(OLED_RESET); // FOR I2C
/*
byte degreeSymbol[8] = {
    B00111,
    B00101,
    B00111,
    B00000,
    B00000,
    B00000,
    B00000,
    B00000,
};

byte omega[8] = {
    B01110,
    B10001,
    B10001,
    B10001,
    B10001,
    B01010,
    B01010,
    B11011,
};
*/

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
    display.begin();  // Switch OLED
    // Clear the buffer.
    display.clearDisplay();
    //Set text size, color, and position
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
     
    // initialize pin to listen for button press
    //pinMode(BUTTON_PIN, INPUT);
    
    // print startup message to LCD display
    display.println(clientID);
    display.display();

    //Setup MQTT
    setup_wifi();
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
    
    // initial reading
    float temperature = readTemperature();
    char msg[7];
    sprintf(msg, "%.2f", temperature);
    publishData(topic0, msg);
    previousMillisMQTT = millis();  // initial start time
    
    //printDataToLCD(temperature);
    delay(LCD_TIMEOUT);
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
        // Get sensor reading
        float temperature = readTemperature();
        char msg[7];
        sprintf(msg, "%.2f", temperature);
        publishData(topic0, msg);
        previousMillisMQTT = currentMillis;
        //printDataToLCD(temperature);
    }
    
    /* TODO: move this to its own function
    currentButtonState = digitalRead(BUTTON_PIN);
    //Serial.println(String(currentButtonState));
    //Serial.println(String(currentButtonState) + " " + String(previousButtonState));
    if ((currentButtonState == HIGH)&&(previousButtonState == LOW))
    {
        previousButtonState = currentButtonState;
        digitalWrite(LCD_POWER_PIN, HIGH);
        printDataToLCD(temperature);
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
    delay(10);
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    
    WiFi.begin(ssid, password);
    
    while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    }
    
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
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
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (client.connect("Thermometer01")){
            Serial.println("connected");
            // Once connected, publish an announcement...
            const char* heartbeat = "heartbeat";
            client.publish( heartbeat, clientID);
            // ... and resubscribe
        } else {
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
float readTemperature()
{
    int sum = 0;
    int numOfSensorReads = 10;
    for (int i = 0; i< numOfSensorReads ; i++)
    {
        sum += analogRead(THERMISTOR_PIN) - ADC_OFFSET;
    }
    int thermistorSensorVal = sum / numOfSensorReads;
    //Serial.println(String(thermistorSensorVal));
    //convert the ADC reading to thermistorVoltage
    float thermistorVoltage = ( (float)thermistorSensorVal / MAX_ADC_READING ) * ADC_REF_VOLTAGE;   // [V]
    //Serial.println(String(thermistorVoltage));
    float temperature = (thermistorVoltage - 0.5)*100;    // [degrees C]
    return temperature;
}
/*
double readLightLevel()
{
    int ldrSensorVal = analogRead(LDR_PIN);

    // figure out ldrResistance
    double resistorVoltage = ( (double)ldrSensorVal / MAX_ADC_READING ) * ADC_REF_VOLTAGE;   // [V]
    double ldrVoltage = 5.0 - resistorVoltage;
    double ldrResistance = (ldrVoltage/resistorVoltage)*10000; //10k is resistor value
    return ldrResistance;
}
*/
void publishData(const char* topic, const char* msg)
{
    client.publish(topic, msg);
    #ifdef DEBUG
    Serial.println("topic: " + String(topic) + " , msg: " + String(msg));
    #endif
}
/*
void printDataToLCD(int temperature)
{
    lcd.createChar(0, degreeSymbol);
    //lcd.createChar(1, omega);

    // Update LCD display
    lcd.clear();
    lcd.print(String(String("Temp: ") + String((int)temperature)));
    lcd.write(byte(0));
    lcd.print(String("C"));
    
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
}
*/
/*
void turnOffLCD(){
    lcd.clear();
    digitalWrite(LCD_POWER_PIN, LOW);
}
*/
