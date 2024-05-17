#include <Servo.h>
#include <Wire.h>
#include "MS5837.h"
MS5837 sensor;
int firstWirePin = 22;
int secondWirePin = 21;
int firstVal = 0;
int secondVal = 0;

Servo servo[8];
byte servoPins[] = {2, 3, 4, 5, 6, 7, 8, 9};
char inputBuffer[32]; // Buffer to store incoming serial data
int bufferPosition = 0; // Position in the buffer

void setup() {
  Serial.begin(9600);
  config_servo();
  config_depth_sensor();
}

void config_servo() {
  for (int i = 0; i < 8; i++) {
    servo[i].attach(servoPins[i]);
    servo[i].writeMicroseconds(1500); // Default neutral position
  }
}

void config_depth_sensor(){
  Wire.begin();
  sensor.setModel(MS5837::MS5837_02BA); // model number for Bar02
  sensor.init();
  sensor.setFluidDensity(997); // kg/m^3 (997 freshwater, 1029 for seawater)
}

void loop() {
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    if (inChar == '\n' || inChar == '\r') { // End of one command, handle both carriage return and newline
      inputBuffer[bufferPosition] = '\0'; // Null-terminate the string
      process_input(inputBuffer);
      bufferPosition = 0; // Reset buffer for the next command
    } else {
      if (bufferPosition < sizeof(inputBuffer) - 1) { // Prevent buffer overflow
        inputBuffer[bufferPosition++] = inChar;
      }
    }
  }
}

void process_input(char *input) {
  if (strcmp(input, "depth") == 0) {
    handle_depth_command();
  } else if (strcmp(input, "voltage") == 0) {
    handle_voltage_command();
  } else {
    // Handle numerical servo control
    int servoNum, val;
    if (sscanf(input, "%d %d", &servoNum, &val) == 2) {
      if (val >= 1100 && val <= 1900 && servoNum >= 2 && servoNum <= 9) {
        set_servo(servoNum, val);
      } else {
        Serial.println("not valid");
      }
    } else {
      Serial.println("Invalid format");
    }
  }
}

void handle_depth_command() {
  sensor.read();
  
  Serial.println(String(sensor.pressure(), 2) + "mbar " + String(sensor.temperature(), 2) + "C " + String(sensor.depth(), 2) + "mDeep ");
}

void handle_voltage_command() {
  firstVal = analogRead(firstWirePin);
  secondVal = analogRead(secondWirePin);
  double current = (firstVal * 120.0) / 1024;
  double voltage = (secondVal * 60.0) / 1024;
  Serial.println(String(current, 2) + "A " + String(voltage, 2) + "V");
}

void set_servo(int servoNum, int val) {
  if (servoNum < 2 || servoNum > 9) return; // Out of valid servo range
  servo[servoNum - 2].writeMicroseconds(val); // Adjust servo position
}
