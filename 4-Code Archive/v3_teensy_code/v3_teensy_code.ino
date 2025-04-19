#include <array>
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_SSD1306.h>
#include "Adafruit_MCP9808.h"
#include "MS5837.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

// Constants and configurations
#define KILL_SWITCH_PIN 32
#define KILL_LIGHTS_PIN 39
// #define RELAY_PIN 39
#define LEAK_DETECT_PIN 26
#define TFT_CS 10  // (CS)
#define TFT_RST 30 // (RESET)，d5
#define TFT_DC 29  // (DC), d4

// Global Variables
MS5837 sensor;
Adafruit_SSD1306 display(4);
Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);

bool leak = false;
bool surface = true;

int firstWirePin = 22;
int secondWirePin = 21;
int firstVal = 0;
int secondVal = 0;

int lastKillSwitchState = -1;

Servo servos[8];
byte servoPins[] = {2, 3, 4, 5, 6, 7, 8, 9};

char inputBuffer[32];
int bufferPosition = 0;
int displayCounter = 0;
int outerSwitch;
int lightPWM;

// Temperature Sensors (0x18, 0x19, 0x1A, I2C_Bus_1)
Adafruit_MCP9808 tempsensor1 = Adafruit_MCP9808();
Adafruit_MCP9808 tempsensor2 = Adafruit_MCP9808();
Adafruit_MCP9808 tempsensor3 = Adafruit_MCP9808();

Adafruit_BME280 bme = Adafruit_BME280(); 

// SD Card
const int SDSelect = BUILTIN_SDCARD;

// Function Prototypes
void configServo();
void configDepthSensor();
bool readLeakSensor();
std::array<float, 3> readTempSensors();
void floatToSurface();
void handleSerialInput();
void processInput(char *input);
bool isValidServoCommand(int servoNum, int val);
String handleDepthCommand();
void handleVoltageCommand();
void setServo(int servoNum, int val);
void updateDisplay();
void displayText(String text);

// Setup Function
void setup() {
    Serial.begin(115200); // Use standard baud rate
    Serial.println("Initializing...");

    Wire.begin();
    // 初始化 BME280
    if (!bme.begin(0x76)) {  // 0x76 是 BME280 的 I2C 地址；0x77 是备用地址
      Serial.println("Could not find a valid BME280 sensor, check wiring!");
      while (1); // 卡住程序
    }

    // Initialize TFT display
    tft.begin();
    tft.setRotation(1);
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);
    tft.setCursor(50, 100);
    tft.println("Hello World!");
    Serial.println("TFT Initialized");

    configServo();
    configDepthSensor();

    // if (!sensor.init()) {
    //     Serial.println("MS5837 sensor initialization failed");
    // } else {
    //     Serial.println("MS5837 sensor initialized successfully");
    // }


    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

    pinMode(KILL_SWITCH_PIN, INPUT_PULLUP);
    pinMode(KILL_LIGHTS_PIN, OUTPUT);
    digitalWrite(KILL_LIGHTS_PIN, HIGH);

    pinMode(LEAK_DETECT_PIN, INPUT);

    // SD.begin(SDSelect);
}

// Loop Function
void loop() {
    tft.fillRect(50, 100, 200, 30, ILI9341_BLACK); // Clear "Hello World" area
    tft.setCursor(50, 100);

    handleSerialInput();
    // leak = readLeakSensor();


    // BME280 once per second
    readBME280Data();
    delay(1000); 

    // Detect Kill Switch's state
    int killSwitchState = digitalRead(KILL_SWITCH_PIN);
    if (killSwitchState != lastKillSwitchState) { // State change
        Serial.print("Kill Switch State Changed: ");
        Serial.println(killSwitchState == HIGH ? "ON" : "OFF");
        lastKillSwitchState = killSwitchState; // Renew state

        // Change Relay State when kill switch is on
        if (killSwitchState == HIGH) {
            Serial.println("Relay is off.");
            // Relay off
            digitalWrite(KILL_LIGHTS_PIN, LOW);
        } else {
            Serial.println("Relay is on.");
            digitalWrite(KILL_LIGHTS_PIN, HIGH);
        }
    }

    
    

}

// Configurations
void configServo() {
    for (int i = 0; i < 8; i++) {
        servos[i].attach(servoPins[i]);
        servos[i].writeMicroseconds(1500); // Neutral position
    }
}

void configDepthSensor() {
    Wire.begin();
    sensor.setModel(MS5837::MS5837_02BA);
    sensor.init();
    sensor.setFluidDensity(997); // Freshwater density in kg/m^3
}

// Sensor Readings
bool readLeakSensor() {
    leak = digitalRead(LEAK_DETECT_PIN);
    Serial.println(leak);
    return leak;
}

std::array<float, 3> readTempSensors() {
    return {tempsensor1.readTempC(), tempsensor2.readTempC(), tempsensor3.readTempC()};
}

// Serial Input Handlers
void handleSerialInput() {
    while (Serial.available() > 0) {
        char inChar = (char)Serial.read();
        if (inChar == '\n' || inChar == '\r') {
            inputBuffer[bufferPosition] = '\0';
            processInput(inputBuffer);
            bufferPosition = 0;
        } else if (bufferPosition < sizeof(inputBuffer) - 1) {
            inputBuffer[bufferPosition++] = inChar;
        }
    }
}


// Process input for Servo
// void processInput(char *input) {
//     if (strcmp(input, "depth") == 0) {
//         handleDepthCommand();
//     } else if (strcmp(input, "voltage") == 0) {
//         handleVoltageCommand();
//     } else {
//         int servoNum, val;
//         outerSwitch = digitalRead(KILL_SWITCH_PIN);
//         if (sscanf(input, "%d %d", &servoNum, &val) == 2 && outerSwitch == LOW && !leak) {
//             if (isValidServoCommand(servoNum, val)) {
//                 setServo(servoNum, val);
//                 Serial.println("Successfully set servo");
//             } else {
//                 Serial.println("Invalid command");
//             }
//         } else {
//             Serial.println("Invalid format");
//         }
//     }
// }

void processInput(char *input) {
    String inputStr = String(input);
    inputStr.toLowerCase(); // Convert input to lowercase for easier matching

    // Check for temperature, humidity, or pressure commands
    if (inputStr.startsWith("temperature") || inputStr.startsWith("t")) {
        float temperature = bme.readTemperature(); // Read temperature from BME280
        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.println(" °C");
    } else if (inputStr.startsWith("humidity") || inputStr.startsWith("h")) {
        float humidity = bme.readHumidity(); // Read humidity from BME280
        Serial.print("Humidity: ");
        Serial.print(humidity);
        Serial.println(" %");
    } else if (inputStr.startsWith("pressure") || inputStr.startsWith("p")) {
        float pressure = bme.readPressure() / 100.0F; // Read pressure from BME280 in hPa
        Serial.print("Pressure: ");
        Serial.print(pressure);
        Serial.println(" hPa");
    }
    // Check for "depth" or "voltage" commands
    else if (strcmp(input, "depth") == 0) {
        handleDepthCommand(); // Call the depth command handler
    } else if (strcmp(input, "voltage") == 0) {
        handleVoltageCommand(); // Call the voltage command handler
    }
    // Check if input is a servo control command
    else {
        int servoNum, val;
        outerSwitch = digitalRead(KILL_SWITCH_PIN); // Read the kill switch state
        if (sscanf(input, "%d %d", &servoNum, &val) == 2 && outerSwitch == LOW && !leak) {
            // Validate and execute servo command
            if (isValidServoCommand(servoNum, val)) {
                setServo(servoNum, val); // Set the servo position
                Serial.println("Successfully set servo");
            } else {
                Serial.println("Invalid servo command");
            }
        } else {
            Serial.println("Invalid input format or no valid command matched");
        }
    }
}


bool isValidServoCommand(int servoNum, int val) {
    return val >= 1100 && val <= 1900 && servoNum >= 2 && servoNum <= 9;
}

// Command Handlers
String handleDepthCommand() {
    configDepthSensor();
    sensor.read();
    return String(sensor.pressure(), 2) + " mbar \n" +
           String(sensor.temperature(), 2) + " °C \n" +
           String(sensor.depth(), 2) + " m";
}

void handleVoltageCommand() {
    firstVal = analogRead(firstWirePin);
    secondVal = analogRead(secondWirePin);
    double current = (firstVal * 120.0) / 1024;
    double voltage = (secondVal * 60.0) / 1024;
    Serial.println(String(current, 2) + " A " + String(voltage, 2) + " V");
}

void setServo(int servoNum, int val) {
    if (servoNum >= 2 && servoNum <= 9) {
        servos[servoNum - 2].writeMicroseconds(val);
    }
}

// BME280 Sensor: Temperature, Humidity, Pressure
void readBME280Data() {
    float temperature = bme.readTemperature(); // temp
    float humidity = bme.readHumidity();       // humidity
    float pressure = bme.readPressure() / 100.0F; // pressure


    // Serial.print("Temperature: ");
    // Serial.print(temperature);
    // Serial.println(" C");

    // Serial.print("Humidity: ");
    // Serial.print(humidity);
    // Serial.println(" %");

    // Serial.print("Pressure: ");
    // Serial.print(pressure);
    // Serial.println(" hPa");

    // tft print
    tft.print("Temperature: ");
    tft.print(temperature);
    tft.println(" C");

    tft.print("Humidity: ");
    tft.print(humidity);
    tft.println(" %");

    tft.print("Pressure: ");
    tft.print(pressure);
    tft.println("hPa");
}

// Display Update
void updateDisplay() {
    firstVal = analogRead(firstWirePin);
    secondVal = analogRead(secondWirePin);

    double current = (firstVal * 120.0) / 1024;
    double voltage = (secondVal * 60.0) / 1024;

    sensor.read();
    outerSwitch = digitalRead(KILL_SWITCH_PIN);

    if (outerSwitch == 1) {
        configServo();
        lightPWM = !lightPWM;
        digitalWrite(KILL_LIGHTS_PIN, lightPWM ? LOW : HIGH);
    } else {
        digitalWrite(KILL_LIGHTS_PIN, HIGH);
    }

    String text = String(voltage) + " V, Switch: " + String(outerSwitch) + "\n" +
                  String(sensor.depth(), 2) + " m, " +
                  String(sensor.temperature(), 2) + " °C, " +
                  String(sensor.pressure(), 2) + " mbar";

    displayText(text);
}

void displayText(String text) {
    display.clearDisplay();
    display.setTextSize(1.5);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.println(text);
    display.display();
}