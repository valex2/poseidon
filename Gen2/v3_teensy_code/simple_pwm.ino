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
// 

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

// Tempsensors (0x18, 0x19, 0x1A, I2C_Bus_1)
Adafruit_MCP9808 tempsensor1 = Adafruit_MCP9808();
Adafruit_MCP9808 tempsensor2 = Adafruit_MCP9808();
Adafruit_MCP9808 tempsensor3 = Adafruit_MCP9808();

// SD Card
const int SDSelect = BUILTIN_SDCARD;

// Light PWM
// Servo lightServo; 

void setup() {
    Serial.begin(9600);
    configServo();
    configDepthSensor();
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    pinMode(KILL_SWITCH_PIN, INPUT);
    pinMode(KILL_LIGHTS_PIN, OUTPUT);
    digitalWrite(KILL_LIGHTS_PIN, HIGH);
    pinMode(LEAK_DETECT_PIN, INPUT);

    tempsensor1.begin(0x18);
    tempsensor1.setResolution(3);
    tempsensor1.wake();

    tempsensor2.begin(0x19);
    tempsensor2.setResolution(3);
    tempsensor2.wake();

    tempsensor3.begin(0x1A);
    tempsensor3.setResolution(3);
    tempsensor3.wake();

    SD.begin(SDSelect);

}
void loop() {
    handleSerialInput();
    readLeakSensor();
    if (displayCounter >= 80) {
        updateDisplay();

        logTemperatureData();
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
void readTempSensors(float temps[3]) {
    temps[0] = tempsensor1.readTempC();
    temps[1] = tempsensor2.readTempC();
    temps[2] = tempsensor3.readTempC();
}
void logTemperatureData() {
  // Make a string for assembling the data to log:
  String dataString = "";

  // Get the current time in milliseconds since the program started
  unsigned long currentTime = millis();

  // Append the current time to the data string
  dataString += String(currentTime) + ",";

  // Array to store temperature readings from sensors
  float temps[3];
  readTempSensors(temps);

  // Append each temperature to the data string
  for (int i = 0; i < 3; i++) {
    dataString += String(temps[i], 2);  // Format to 2 decimal places
    if (i < 2) {  // Add a comma after the first two values
      dataString += ",";
    }
  }

  // Open the file for writing
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // If the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // Print to the serial port too:
    // Serial.println(dataString);
  } else {
    // If the file isn't open, pop up an error:
    // Serial.println("error opening datalog.txt");
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
    return val >= 1100 && val <= 1900 && servoNum >= 0 && servoNum <= 7;
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
            return; // Exit if an invalid servo number is entered
    }

    // Check if remapped servo is in range
    if (remappedServo >= 2 && remappedServo <= 9) {
        servos[remappedServo - 2].writeMicroseconds(val);
        Serial.print("Set servo ");
        Serial.print(remappedServo);
        Serial.print(" to ");
        Serial.println(val);
    } else {
        Serial.println("Remapped servo number is out of range");
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
