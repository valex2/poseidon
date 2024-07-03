#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_SSD1306.h>
#include "MS5837.h"
MS5837 sensor;
Adafruit_SSD1306 display(4);
// #define BME_SCK 13
// #define BME_MISO 12
// #define BME_MOSI 11
// #define BME_CS 10
// #define SEALEVELPRESSURE_HPA (1013.25)
// Adafruit_BME280 bme; // I2C once the internal humidity sensor is working

int firstWirePin = 22;
int secondWirePin = 21;
int firstVal = 0;
int secondVal = 0;

Servo servo[8];
byte servoPins[] = {2, 3, 4, 5, 6, 7, 8, 9};
char inputBuffer[32]; // Buffer to store incoming serial data
int bufferPosition = 0; // Position in the buffer
int k = 0; // for display updating
int outerSwitch;
int light_pwm;
int switch_debounce = 0;


void setup() {
  Serial.begin(9600);
  config_servo();
  config_depth_sensor();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // start the display
  pinMode(10, INPUT);
  pinMode(12, OUTPUT);
  digitalWrite(12, 1);
  // unsigned status;
  // status = bme.begin();
  // if (!status) {
  //       Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
  //       Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
  //       Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
  //       Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
  //       Serial.print("        ID of 0x60 represents a BME 280.\n");
  //       Serial.print("        ID of 0x61 represents a BME 680.\n");
  //       while (1) delay(10);
  // }
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
  if (k >= 80) {
    firstVal = analogRead(firstWirePin); // current sensor
    secondVal = analogRead(secondWirePin);
    double current = (firstVal * 120.0) / 1024;
    double voltage = (secondVal * 60.0) / 1024;

    sensor.read(); // presure sensor

    outerSwitch = digitalRead(10);

    if (outerSwitch == 1) {
      if (switch_debounce == 0) {
        config_servo();
        switch_debounce = 0; // set to one to implement debouncing
      }
      if (light_pwm == 1) {
        digitalWrite(12, 0);
        light_pwm = 0;
      } else {
        digitalWrite(12, 1);
        light_pwm = 1;
      }
    } else {
      digitalWrite(12, 1);
      switch_debounce = 0;
    }

    String text = String(voltage) + " V " + "Switch: " + String(outerSwitch) + "\n" + String(sensor.depth(), 2) + " mDeep\n";
    displayText(text);
    k = 0;
  }

  k = k + 1;
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

void displayText(String text) {
  display.clearDisplay();
  display.setTextSize(1.5);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println(text);
  display.display();
}
