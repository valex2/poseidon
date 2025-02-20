#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_SSD1306.h>
#include "Adafruit_MCP9808.h"
#include "MS5837.h"

// Constants and configurations
#define KILL_SWITCH_PIN 33
#define KILL_LIGHTS_PIN 39
#define LEAK_DETECT_PIN 14

MS5837 sensor;
Adafruit_SSD1306 display(4);
Adafruit_BME280 bme_bulkhead;  // BME280 sensor at 0x76 (bulkhead)
Adafruit_BME280 bme_mid;       // BME280 sensor at 0x77 (mid enclosure)

bool leak = false;
bool surface = true;
int firstWirePin = 22;
int secondWirePin = 21;
int firstVal = 0;
int secondVal = 0;
Servo servos[8];
byte servoPins[] = {2, 3, 4, 5, 6, 7, 8, 9};
char inputBuffer[32];
int bufferPosition = 0;
int displayCounter = 0;
int outerSwitch;
int lightPWM;

// MCP9808 sensors labeled accordingly
Adafruit_MCP9808 tempsensor_IMU = Adafruit_MCP9808();  // 0x18 → IMU Temp
Adafruit_MCP9808 tempsensor_ESC = Adafruit_MCP9808();  // 0x19 → ESC Temp
Adafruit_MCP9808 tempsensor_Orin = Adafruit_MCP9808(); // 0x1A → Orin Temp

// SD Card
const int SDSelect = BUILTIN_SDCARD;

//--------------------------------------------------
// Setup Function
//--------------------------------------------------
void setup() {
    Serial.begin(9600);
    Wire.begin();  // Initialize I2C bus

    configServo();
    configDepthSensor();
    
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    pinMode(KILL_SWITCH_PIN, INPUT);
    pinMode(KILL_LIGHTS_PIN, OUTPUT);
    digitalWrite(KILL_LIGHTS_PIN, HIGH);
    pinMode(LEAK_DETECT_PIN, INPUT);

    if (!tempsensor_IMU.begin(0x18)) Serial.println("Error: IMU Temp sensor not found!");
    else { tempsensor_IMU.setResolution(3); tempsensor_IMU.wake(); }

    if (!tempsensor_ESC.begin(0x19)) Serial.println("Error: ESC Temp sensor not found!");
    else { tempsensor_ESC.setResolution(3); tempsensor_ESC.wake(); }

    if (!tempsensor_Orin.begin(0x1A)) Serial.println("Error: Orin Temp sensor not found!");
    else { tempsensor_Orin.setResolution(3); tempsensor_Orin.wake(); }

    // Initialize both BME280 sensors
    if (!bme_bulkhead.begin(0x76)) Serial.println("Error: Bulkhead BME280 not found!");
    else Serial.println("Bulkhead BME280 initialized successfully!");

    if (!bme_mid.begin(0x77)) Serial.println("Error: Mid Enclosure BME280 not found!");
    else Serial.println("Mid Enclosure BME280 initialized successfully!");

    if (!SD.begin(SDSelect)) Serial.println("Error: SD card not detected!");
}

//--------------------------------------------------
// Main Loop
//--------------------------------------------------
void loop() {
    handleSerialInput();
    readLeakSensor();
    
    static unsigned long lastDisplayUpdate = 0;
    if (millis() - lastDisplayUpdate >= 1000) {  // Update every second
        updateDisplay();
        logTemperatureData();
        lastDisplayUpdate = millis();
    }
}

//--------------------------------------------------
// Temperature Sensor Read Functions
//--------------------------------------------------
void readTempSensors(float temps[3]) {
    temps[0] = tempsensor_IMU.readTempC();
    temps[1] = tempsensor_ESC.readTempC();
    temps[2] = tempsensor_Orin.readTempC();
}


//--------------------------------------------------
// SD Card Logging (includes both BME280 sensors)
//--------------------------------------------------
void logTemperatureData() {
    String dataString = "";
    unsigned long currentTime = millis();
    dataString += String(currentTime) + ",";

    // Read MCP9808 sensors
    float temps[3];
    readTempSensors(temps);
    dataString += String(temps[0], 2) + "," + String(temps[1], 2) + "," + String(temps[2], 2);

    // Read Bulkhead BME280
    float bme_bulk_Temp = bme_bulkhead.readTemperature();
    float bme_bulk_Pressure = bme_bulkhead.readPressure() / 100.0;  // Convert to hPa
    float bme_bulk_Humidity = bme_bulkhead.readHumidity();

    // Read Mid Enclosure BME280
    float bme_mid_Temp = bme_mid.readTemperature();
    float bme_mid_Pressure = bme_mid.readPressure() / 100.0;
    float bme_mid_Humidity = bme_mid.readHumidity();

    dataString += "," + String(bme_bulk_Temp, 2) + "," + String(bme_bulk_Pressure, 2) + "," + String(bme_bulk_Humidity, 2);
    dataString += "," + String(bme_mid_Temp, 2) + "," + String(bme_mid_Pressure, 2) + "," + String(bme_mid_Humidity, 2);

    File dataFile = SD.open("datalog.txt", FILE_WRITE);
    if (dataFile) {
        dataFile.println(dataString);
        dataFile.close();
        Serial.println(dataString);  // Print to serial
    } else {
        Serial.println("Error opening datalog.txt");
    }
}

//--------------------------------------------------
// Display Update Function (includes both BME280 sensors)
//--------------------------------------------------
void updateDisplay() {
    firstVal = analogRead(firstWirePin);
    secondVal = analogRead(secondWirePin);
    double current = (firstVal * 120.0) / 1024;
    double voltage = (secondVal * 60.0) / 1024;
    sensor.read();

    // Read BME280 sensor values
    float bme_bulk_Temp = bme_bulkhead.readTemperature();
    float bme_bulk_Pressure = bme_bulkhead.readPressure() / 100.0;
    float bme_bulk_Humidity = bme_bulkhead.readHumidity();

    float bme_mid_Temp = bme_mid.readTemperature();
    float bme_mid_Pressure = bme_mid.readPressure() / 100.0;
    float bme_mid_Humidity = bme_mid.readHumidity();

    outerSwitch = digitalRead(KILL_SWITCH_PIN);
    if (outerSwitch == 1) {
        configServo();
        if (lightPWM == 1) {
            digitalWrite(KILL_LIGHTS_PIN, LOW);
            lightPWM = 0;
        } else {
            digitalWrite(KILL_LIGHTS_PIN, HIGH);
            lightPWM = 1;
        }
    } else {
        digitalWrite(KILL_LIGHTS_PIN, HIGH);
    }

    String text = String(voltage) + " V, Switch: " + String(outerSwitch) + "\n" +
                  "Depth: " + String(sensor.depth(), 2) + " m\n" +
                  "MS5837 Temp: " + String(sensor.temperature(), 2) + " °C\n" +
                  "IMU Temp: " + String(tempsensor_IMU.readTempC(), 2) + " °C\n" +
                  "ESC Temp: " + String(tempsensor_ESC.readTempC(), 2) + " °C\n" +
                  "Orin Temp: " + String(tempsensor_Orin.readTempC(), 2) + " °C\n" +
                  "Bulkhead BME280: " + String(bme_bulk_Temp, 2) + " °C, " +
                  String(bme_bulk_Pressure, 2) + " hPa, " +
                  String(bme_bulk_Humidity, 2) + " %\n" +
                  "Mid Enclosure BME280: " + String(bme_mid_Temp, 2) + " °C, " +
                  String(bme_mid_Pressure, 2) + " hPa, " +
                  String(bme_mid_Humidity, 2) + " %";

    displayText(text);
}
//--------------------------------------------------
// OLED Display Update
//--------------------------------------------------
void displayText(String text) {
    display.clearDisplay();
    display.setTextSize(1.5);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.println(text);
    display.display();
}

//--------------------------------------------------
// Servo Configuration
//--------------------------------------------------
void configServo() {
    for (int i = 0; i < 8; i++) {
        servos[i].attach(servoPins[i]);
        servos[i].writeMicroseconds(1500); // Neutral position
    }
}

//--------------------------------------------------
// Depth Sensor Configuration
//--------------------------------------------------
void configDepthSensor() {
    Wire.begin();
    sensor.setModel(MS5837::MS5837_02BA);
    sensor.init();
    sensor.setFluidDensity(997); // kg/m^3 for freshwater
}

//--------------------------------------------------
// Leak Sensor Reading
//--------------------------------------------------
void readLeakSensor() {
    if (!leak) {
        leak = digitalRead(LEAK_DETECT_PIN);
    } else if (leak && surface) {
        floatToSurface();
        surface = false;
    }
}

//--------------------------------------------------
// Leak Handling: Float to Surface
//--------------------------------------------------
void floatToSurface() {
    Serial.println("leak detected!! surfacing for 20 seconds");
    for (int i = 0; i < 4; i++) {
        servos[i].writeMicroseconds(1650);
    }
    for (int i = 4; i < 8; i++) {
        servos[i].writeMicroseconds(1500);
    }
    for (int i = 20; i >= 0; i--) {
        Serial.println(i);
        delay(1000);
    }
    for (int i = 0; i < 8; i++) {
        servos[i].writeMicroseconds(1500); // neutral position
    }
}

//--------------------------------------------------
// Serial Input Handling
//--------------------------------------------------
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

//--------------------------------------------------
// Process Serial Commands
//--------------------------------------------------
void processInput(char* input) {
    if (strcmp(input, "depth") == 0) {
        handleDepthCommand();
    } else if (strcmp(input, "voltage") == 0) {
        handleVoltageCommand();
    } else {
        int servoNum, val;
        outerSwitch = digitalRead(KILL_SWITCH_PIN);
        if (sscanf(input, "%d %d", &servoNum, &val) == 2 && outerSwitch == LOW && !leak) {
            if (isValidServoCommand(servoNum, val)) {
                setServo(servoNum, val);
                // Serial.println("Sucessfully set servo");
            } else {
                Serial.println("Invalid command");
            }
        } else {
            Serial.println("Invalid format");
        }
    }
}

//--------------------------------------------------
// Validate Servo Commands
//--------------------------------------------------
bool isValidServoCommand(int servoNum, int val) {
    return val >= 1100 && val <= 1900 && servoNum >= 0 && servoNum <= 7;
}

//--------------------------------------------------
// Handle Depth Command
//--------------------------------------------------
void handleDepthCommand() {
    sensor.read();
    Serial.println(String(sensor.pressure(), 2) + " mbar " + String(sensor.temperature(), 2) + " °C " + String(sensor.depth(), 2) + " m");
}

//--------------------------------------------------
// Handle Voltage Command
//--------------------------------------------------
void handleVoltageCommand() {
    firstVal = analogRead(firstWirePin);
    secondVal = analogRead(secondWirePin);
    double current = (firstVal * 120.0) / 1024;
    double voltage = (secondVal * 60.0) / 1024;
    Serial.println(String(current, 2) + " A " + String(voltage, 2) + " V");
}

//--------------------------------------------------
// Set Servo Position
//--------------------------------------------------
void setServo(int servoNum, int val) {
    // Remap input values based on your custom mapping
    int remappedServo;
    switch (servoNum) {
        case 1: remappedServo = 2; break;
        case 5: remappedServo = 3; break;
        case 6: remappedServo = 4; break;
        case 2: remappedServo = 5; break;
        case 7: remappedServo = 6; break;
        case 4: remappedServo = 7; break;
        case 3: remappedServo = 8; break;
        case 0: remappedServo = 9; break;
        default: 
            Serial.println("Invalid servo number"); 
            return;
    }
    if (remappedServo >= 2 && remappedServo <= 9) {
        servos[remappedServo - 2].writeMicroseconds(val);
        Serial.print("Set servo ");
        Serial.print(servoNum);
        Serial.print(" to ");
        Serial.println(val);
    } else {
        Serial.println("Remapped servo number is out of range");
    }
}
