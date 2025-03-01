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
#define KILL_SWITCH_PIN 26
#define KILL_LIGHTS_PIN 39
#define LEAK_DETECT_PIN 33
#define LIGHT_PIN 37

MS5837 sensor;
Adafruit_SSD1306 display(4);
Adafruit_BME280 bme_bulkhead;  // Bulkhead sensor (I2C addr 0x76)
Adafruit_BME280 bme_mid;       // Mid enclosure sensor (I2C addr 0x77)

bool leak = false;
bool surface = true;
int firstWirePin = 22;
int secondWirePin = 21;
int firstVal = 0;
int secondVal = 0;
Servo servos[8];
byte servoPins[] = {0, 1, 2, 3, 4, 5, 6, 7};
char inputBuffer[32];
int bufferPosition = 0;
int outerSwitch;

bool systemError = false;  // Global error flag

// MCP9808 sensors
Adafruit_MCP9808 tempsensor_IMU = Adafruit_MCP9808();   // I2C addr 0x18
Adafruit_MCP9808 tempsensor_ESC = Adafruit_MCP9808();   // I2C addr 0x19
Adafruit_MCP9808 tempsensor_Orin = Adafruit_MCP9808();  // I2C addr 0x1A

// SD Card
const int SDSelect = BUILTIN_SDCARD;

// Function prototypes
void testMotors();
bool isError();

//--------------------------------------------------
// Setup Function
//--------------------------------------------------
void setup() {
    Serial.begin(9600);
    Wire.begin();

    configServo();
    configDepthSensor();
    
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    pinMode(KILL_SWITCH_PIN, INPUT);
    pinMode(KILL_LIGHTS_PIN, OUTPUT);
    digitalWrite(KILL_LIGHTS_PIN, HIGH);
    pinMode(LEAK_DETECT_PIN, INPUT);

    pinMode(LIGHT_PIN, OUTPUT);
    // LED controlled in loop using analogWrite

    // Initialize temperature sensors – set error flag on failure
    if (!tempsensor_IMU.begin(0x18)) { systemError = true; } 
    else { tempsensor_IMU.setResolution(3); tempsensor_IMU.wake(); }

    if (!tempsensor_ESC.begin(0x19)) { systemError = true; } 
    else { tempsensor_ESC.setResolution(3); tempsensor_ESC.wake(); }

    if (!tempsensor_Orin.begin(0x1A)) { systemError = true; } 
    else { tempsensor_Orin.setResolution(3); tempsensor_Orin.wake(); }

    // Initialize BME280 sensors – print only on successful init
    if (!bme_bulkhead.begin(0x76)) { systemError = true; } 
    else { Serial.println("Bulkhead BME280 initialized successfully!"); }

    if (!bme_mid.begin(0x77)) { systemError = true; } 
    else { Serial.println("Mid Enclosure BME280 initialized successfully!"); }

    // Initialize SD card (set error flag if not present)
    if (!SD.begin(SDSelect)) { systemError = true; }

    // // After initialization, briefly test each motor
    // testMotors();
}

//--------------------------------------------------
// Main Loop
//--------------------------------------------------
void loop() {
    handleSerialInput();
    readLeakSensor();
    
    updateDisplay();
    logTemperatureData();
    
    // LED control:
    // If there is no error, drive the LED at half brightness;
    // if an error condition is detected, blink rapidly.
    if (!isError()) {
        analogWrite(LIGHT_PIN, 128); // 50% brightness
    } else {
        static unsigned long lastBlink = 0;
        static bool blinkState = false;
        if (millis() - lastBlink >= 200) {  // blink every 200 ms
            blinkState = !blinkState;
            lastBlink = millis();
        }
        analogWrite(LIGHT_PIN, blinkState ? 255 : 0);
    }
}

//--------------------------------------------------
// Check for error conditions
//--------------------------------------------------
bool isError() {
    // Error if any sensor failed init, leak detected, or kill switch active.
    return systemError || leak || (digitalRead(KILL_SWITCH_PIN) == HIGH);
}

//--------------------------------------------------
// Test Motors Function: Cycle each motor to 1600 and back to 1500
//--------------------------------------------------
void testMotors() {
    for (int i = 0; i < 8; i++) {
        servos[i].writeMicroseconds(1600);
        delay(200);
        servos[i].writeMicroseconds(1500);
        delay(200);
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
// SD Card Logging (both BME280 sensors)
//--------------------------------------------------
void logTemperatureData() {
    String dataString = "";
    unsigned long currentTime = millis();
    dataString += String(currentTime) + ",";

    float temps[3];
    readTempSensors(temps);
    dataString += String(temps[0], 2) + "," + String(temps[1], 2) + "," + String(temps[2], 2);

    float bme_bulk_Temp = bme_bulkhead.readTemperature();
    float bme_bulk_Pressure = bme_bulkhead.readPressure() / 100.0;
    float bme_bulk_Humidity = bme_bulkhead.readHumidity();

    float bme_mid_Temp = bme_mid.readTemperature();
    float bme_mid_Pressure = bme_mid.readPressure() / 100.0;
    float bme_mid_Humidity = bme_mid.readHumidity();

    dataString += "," + String(bme_bulk_Temp, 2) + "," + String(bme_bulk_Pressure, 2) + "," + String(bme_bulk_Humidity, 2);
    dataString += "," + String(bme_mid_Temp, 2) + "," + String(bme_mid_Pressure, 2) + "," + String(bme_mid_Humidity, 2);

    File dataFile = SD.open("datalog.txt", FILE_WRITE);
    if (dataFile) {
        dataFile.println(dataString);
        dataFile.close();
    }
}

//--------------------------------------------------
// Display Update Function (both BME280 sensors)
//--------------------------------------------------
void updateDisplay() {
    firstVal = analogRead(firstWirePin);
    secondVal = analogRead(secondWirePin);
    double voltage = (secondVal * 60.0) / 1024;
    sensor.read();

    float bme_bulk_Temp = bme_bulkhead.readTemperature();
    float bme_mid_Temp = bme_mid.readTemperature();

    outerSwitch = digitalRead(KILL_SWITCH_PIN);
    if (outerSwitch == HIGH) {
         configServo();  // Reinitialize servos if kill switch is active
    }

    String text = String(voltage) + " V\n" +
                  "Depth: " + String(sensor.depth(), 2) + " m\n" +
                  "MS5837 Temp: " + String(sensor.temperature(), 2) + " C\n" +
                  "IMU Temp: " + String(tempsensor_IMU.readTempC(), 2) + " C\n" +
                  "ESC Temp: " + String(tempsensor_ESC.readTempC(), 2) + " C\n" +
                  "Orin Temp: " + String(tempsensor_Orin.readTempC(), 2) + " C\n" +
                  "Bulkhead: " + String(bme_bulk_Temp, 2) + " C\n" +
                  "Mid: " + String(bme_mid_Temp, 2) + " C";
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
// Depth Sensor Configuration & Test
//--------------------------------------------------
void configDepthSensor() {
    // Loop until sensor initializes – minimal printing.
    while (!sensor.init()) {
        delay(5000);
    }
    Serial.println("Depth sensor (MS5837) initialized successfully!");
    sensor.setModel(MS5837::MS5837_02BA);
    sensor.setFluidDensity(997);  // Freshwater density
}

//--------------------------------------------------
// Test Depth Sensor Function (triggered via serial command)
//--------------------------------------------------
void testDepthSensor() {
    sensor.read();
    // Minimal output to reduce printing
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
    // Adjust servos for surfacing without printing
    for (int i = 0; i < 4; i++) {
        servos[i].writeMicroseconds(1650);
    }
    for (int i = 4; i < 8; i++) {
        servos[i].writeMicroseconds(1500);
    }
    for (int i = 20; i >= 0; i--) {
        delay(1000);
    }
    for (int i = 0; i < 8; i++) {
        servos[i].writeMicroseconds(1500); // Return to neutral
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
    // Command to test depth sensor
    if (strcmp(input, "depth") == 0) {
        testDepthSensor();
    }
    // Command for voltage (minimal output)
    else if (strcmp(input, "voltage") == 0) {
        // (Optional: insert voltage handling if desired)
    }
    // Command to set all servos to a given PWM value (e.g., "all 1600")
    else if (strncmp(input, "all", 3) == 0) {
        int pwmVal;
        if (sscanf(input + 3, "%d", &pwmVal) == 1 &&
            pwmVal >= 1100 && pwmVal <= 1900 &&
            digitalRead(KILL_SWITCH_PIN) == LOW && !leak) {
            for (int i = 0; i < 8; i++) {
                servos[i].writeMicroseconds(pwmVal);
            }
        }
    }
    // Command to reset all servos to off (neutral 1500)
    else if (strcmp(input, "off") == 0) {
        for (int i = 0; i < 8; i++) {
            servos[i].writeMicroseconds(1500);
        }
    }
    // Otherwise, check for individual servo command "servoNumber PWMValue"
    else {
        int servoNum, val;
        outerSwitch = digitalRead(KILL_SWITCH_PIN);
        if (sscanf(input, "%d %d", &servoNum, &val) == 2 &&
            outerSwitch == LOW && !leak) {
            if (isValidServoCommand(servoNum, val)) {
                setServo(servoNum, val);
            }
        }
    }
}

//--------------------------------------------------
// Validate Servo Commands
//--------------------------------------------------
bool isValidServoCommand(int servoNum, int val) {
    return (val >= 1100 && val <= 1900 && servoNum >= 0 && servoNum <= 7);
}

//--------------------------------------------------
// Set Servo Position (with remapping)
//--------------------------------------------------
void setServo(int servoNum, int val) {
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
        default: return;
    }
    if (remappedServo >= 2 && remappedServo <= 9) {
        servos[remappedServo - 2].writeMicroseconds(val);
    }
}
