#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
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
void setup() {
    Serial.begin(9600);
    configServo();
    configDepthSensor();
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    pinMode(KILL_SWITCH_PIN, INPUT);
    pinMode(KILL_LIGHTS_PIN, OUTPUT);
    digitalWrite(KILL_LIGHTS_PIN, HIGH);
    pinMode(LEAK_DETECT_PIN, INPUT);
}
void loop() {
    handleSerialInput();
    readLeakSensor();
    if (displayCounter >= 80) {
        updateDisplay();
        displayCounter = 0;
    }
    displayCounter++;
}
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
    sensor.setFluidDensity(997); // kg/m^3 for freshwater
}
void readLeakSensor() {
    if (!leak) {
        leak = digitalRead(LEAK_DETECT_PIN);
    } else if (leak && surface) {
        floatToSurface();
        surface = false;
    }
}
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
                Serial.println("Sucessfully set servo");
            } else {
                Serial.println("Invalid command");
            }
        } else {
            Serial.println("Invalid format");
        }
    }
}
bool isValidServoCommand(int servoNum, int val) {
    return val >= 1100 && val <= 1900 && servoNum >= 2 && servoNum <= 9;
}
void handleDepthCommand() {
    sensor.read();
    Serial.println(String(sensor.pressure(), 2) + " mbar " + String(sensor.temperature(), 2) + " °C " + String(sensor.depth(), 2) + " m");
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
void updateDisplay() {
    firstVal = analogRead(firstWirePin);
    secondVal = analogRead(secondWirePin);
    double current = (firstVal * 120.0) / 1024;
    double voltage = (secondVal * 60.0) / 1024;
    sensor.read();
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
