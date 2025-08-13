#include <Servo.h>
#include <Wire.h>

// Servos
Servo servos[8];   // Array to store servo objects
// const int servoPins[8] = {2, 4, 3, 7, 0, 6, 5, 1};  // Update these pin numbers as needed
const int servoPins[8] = {4, 3, 0, 2, 1, 6, 5, 7};
int lastThrusterPWM[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};   

// Serial setup
char inputBuffer[64]; // Buffer to store incoming serial data
int bufferPosition = 0; // Position in the buffer
const int DIGITS = 8;

// Depth sensor
#include "MS5837.h"
MS5837 sensor;

// Battery sensor
const int voltagePin = 40;
const int currentPin = 41;

// Indicators
const int greenIndicatorLedPin = 37; // LED1
// const int redIndicatorLedPin= 36;  // LED2
const int killSwitchPin = 26;
bool firstIteration = false;

// External lumen lights
Servo lightServo;
int lumenPin = 8;
int lightVal = 1100;
int lightCycles = 5;

// SD Card
#include <SD.h>
int sd_loop_counter = 0;
const int chipSelect = BUILTIN_SDCARD;
unsigned long loopIterationCounter = 0; // for perioidic SD logging
int sdLoggingFrequency = 20000; // TODO: make this human readable

void setup() {
    // Begin serial and I2C (via wire)
    Serial.begin(9600);
    Wire.begin();
    
    // Initialize all servos
    config_servos();

    // Initilize depth sensor
    config_depth();

    // Initilize voltage/current sensor
    config_battery();

    // Initilize indicator
    config_indicator();

    // Initilize lumen
    config_lumen();

    // Initilize SD card
    config_sd_card();
    
    Serial.println("Initilize Complete");
}

void config_servos() {
    for (int i = 0; i < 8; i++) {
        servos[i].attach(servoPins[i]);   // Attach servo
        servos[i].writeMicroseconds(1500);  // Start at neutral
    }
}

void config_depth() {
    sensor.setModel(MS5837::MS5837_02BA); // model number for Bar02
    sensor.init();
    sensor.setFluidDensity(997); // kg/m^3 (997 freshwater, 1029 for seawater)
}

void config_battery() {
    pinMode(currentPin, INPUT);
    pinMode(voltagePin, INPUT);
}

void config_indicator() {
    pinMode(greenIndicatorLedPin, OUTPUT);
    pinMode(killSwitchPin, INPUT);
    digitalWrite(greenIndicatorLedPin, HIGH);
}

void config_lumen() {
  lightServo.attach(lumenPin);
  lightServo.writeMicroseconds(1100); // Turn off the light
}

void config_sd_card() {
  SD.begin(chipSelect);
  write_data_sd("Configuring");
}

void loop() {
    //SD Card Logs
    loopIterationCounter++;
    if (loopIterationCounter % sdLoggingFrequency == 0) {
        logPeriodicData();
    }

    // Indicator (kill switch) logic
    int killSwitch = digitalRead(killSwitchPin); // Read the value from the pin (0, loose = nominal or 1, tighten = kill)
    if (killSwitch == 0 && firstIteration) {
        digitalWrite(greenIndicatorLedPin, HIGH);

        // Set servo to previous calues
        servos[0].writeMicroseconds(lastThrusterPWM[0]);
        servos[1].writeMicroseconds(lastThrusterPWM[1]);
        servos[2].writeMicroseconds(lastThrusterPWM[2]);
        servos[3].writeMicroseconds(lastThrusterPWM[3]);
        servos[4].writeMicroseconds(lastThrusterPWM[4]);
        servos[5].writeMicroseconds(lastThrusterPWM[5]);
        servos[6].writeMicroseconds(lastThrusterPWM[6]);
        servos[7].writeMicroseconds(lastThrusterPWM[7]);

        firstIteration = false;
    } else if (killSwitch == 1) { // If KILLED, reset motors and blink red LED
        //Serial.println("KILLED");
        config_servos();
        digitalWrite(greenIndicatorLedPin, LOW);
        firstIteration = true;
        return; // Skip further processing when killed
    } 

    while (Serial.available() > 0) {
        char inChar = (char)Serial.read();
        if (inChar == '\n' || inChar == '\r') { // End of one command
            inputBuffer[bufferPosition] = '\0'; // Null-terminate the string
            process_input(inputBuffer);
            bufferPosition = 0; // Reset buffer for the next command
        } else {
            if (bufferPosition < (int)sizeof(inputBuffer) - 1) { // Prevent buffer overflow
                inputBuffer[bufferPosition++] = inChar;
            }
        }
    }
}

void process_input(char *input) {
  int s0, s1, s2, s3, s4, s5, s6, s7;
  int servoNum, val;

  if (sscanf(input, "%d %d %d %d %d %d %d %d", &s0, &s1, &s2, &s3, &s4, &s5, &s6, &s7) == 8) {   // Setting servos
    // Set servos
    servos[0].writeMicroseconds(s0);
    servos[1].writeMicroseconds(s1);
    servos[2].writeMicroseconds(s2);
    servos[3].writeMicroseconds(s3);
    servos[4].writeMicroseconds(s4);
    servos[5].writeMicroseconds(s5);
    servos[6].writeMicroseconds(s6);
    servos[7].writeMicroseconds(s7);

    // Record last thruster value
    lastThrusterPWM[0] = s0; 
    lastThrusterPWM[1] = s1;
    lastThrusterPWM[2] = s2;
    lastThrusterPWM[3] = s3;
    lastThrusterPWM[4] = s4;
    lastThrusterPWM[5] = s5;
    lastThrusterPWM[6] = s6;
    lastThrusterPWM[7] = s7;

    // THIS MUST BE KEPT THE SAME IN ORDER TO WORK WITH ORIN CODE
    float pressure, temperature, depth;
    handle_depth_command(pressure, temperature, depth);
    float current, voltage;
    handle_battery_command(current, voltage);

    // Print those data for orin
    Serial.println("> pressure:" + String(pressure, DIGITS) + " temperature:" + String(temperature, DIGITS) + " depth:" + String(depth, DIGITS) + " current:" + String(current, DIGITS) + " voltage:" + String(voltage, DIGITS));
  
  } else if (strcmp(input, "batt") == 0) {   // Debugging battery
    Serial.println("TESTING BATTERY");
    float current, voltage;
    handle_battery_command(current, voltage);
    Serial.println("Current:" + String(current, DIGITS) + " voltage:" + String(voltage, DIGITS));

  } else if (sscanf(input, "%d %d", &servoNum, &val) == 2) {   // Turn on single servo
     Serial.println("TESTING SINGLE SERVO");
    if (val >= 1100 && val <= 1900 && servoNum >= 0 && servoNum <= 7) {
        servos[servoNum].writeMicroseconds(val);
        lastThrusterPWM[servoNum] = val;
    } else {
        Serial.println("Invalid command");
    }

  } else if (strcmp(input, "depth") == 0) {   // Debugging depth sensor
    Serial.println("TESTING DEPTH SENSOR");
    float pressure, temperature, depth;
    handle_depth_command(pressure, temperature, depth);
    Serial.println("Pressure: " + String(pressure, DIGITS) + " mbar, Temperature: " + String(temperature, DIGITS) + " °C, Depth: " + String(depth, DIGITS) + " m");

  } else if (strcmp(input, "test") == 0) {   // Debugging all servos
    Serial.println("RUNNING SERVO TEST SCRIPT");
    test_servos();

  } else if (sscanf(input, "light %d", &lightVal) == 1) {   // Turn on lumen lights
    if (lightVal >= 1100 && lightVal <= 1900) {
      lightServo.writeMicroseconds(lightVal);
      Serial.println("Light set to " + String(lightVal));
    } else {
      Serial.println("Invalid light value. Please enter a value between 1100 and 1900.");
    }

  } else if (sscanf(input, "light gradient %d", &lightCycles) == 1) {   // Graient lumen lights
    Serial.println("LIGHT GRADIENT SET");
    gradient_lumen_light(lightCycles);

  } else if (strcmp(input, "transfer") == 0) {   // SD Card Transfer
    transfer_sd_log();

  } else if (strcmp(input, "delete") == 0) {   // SD Card Delete
    delete_sd_log();

  } else {
    Serial.println(input);

  }
}

void handle_depth_command(float& pressure, float& temperature, float& depth) {
  sensor.read();
  pressure = sensor.pressure(); // mbar
  temperature = sensor.temperature(); // C
  depth = sensor.depth(); // m
}

void handle_battery_command(float& current, float& voltage) {
  current = (analogRead(currentPin) * 120.0) / 1024; // A
  voltage = (analogRead(voltagePin) * 60.0) / 1024; // V
}

void test_servos() {
  for (int i = 0; i < 8; i++) {
    // Set the current servo to 1550 microseconds
    servos[i].writeMicroseconds(1550);
    lastThrusterPWM[i] = 1550;
    delay(2000);  // wait 500 ms for the servo to move
    // Return the servo to its neutral position (1500 microseconds)
    servos[i].writeMicroseconds(1500);
    lastThrusterPWM[i] = 1500;
    delay(500);  // wait 500 ms before moving to the next servo
  }
}

void logPeriodicData() {
  float current, voltage;
  handle_battery_command(current, voltage);

  float pressure, temperature, depth;
  handle_depth_command(pressure, temperature, depth);
  
  String dataString = "current:" + String(current, DIGITS) +
                  " voltage:" + String(voltage, DIGITS) +
                  " servo:"; 
  for (int i = 0; i < 8; i++) {
    dataString += "PWM:" + String(lastThrusterPWM[i]);
    if (i < 7)
      dataString += ",";
  }
  
  dataString += " pressure:" + String(pressure, DIGITS) +
                " temperature:" + String(temperature, DIGITS) +
                " depth:" + String(depth, DIGITS);

  dataString += " killSwitch:" + String(digitalRead(killSwitchPin));

  write_data_sd(dataString);
}

void transfer_sd_log(){
  if (SD.exists("datalog.txt")) {
    Serial.println("Transfering datalog.txt ...");
    // Open the file for reading
    File dataFile = SD.open("datalog.txt", FILE_READ);
    if (dataFile) {
      // Read the file and send its content to the Serial port
      while (dataFile.available()) {
        Serial.write(dataFile.read());
      }
      dataFile.close();
      Serial.println("\nFile transfer complete. Save the output on your local device as datalog.txt.");
    } else {
      Serial.println("Error opening datalog.txt for reading.");
    }
  } else {
    Serial.println("datalog.txt does not exist.");
  }
}

void delete_sd_log() {
  if (SD.exists("datalog.txt")) {
      // Remove the file
      if (SD.remove("datalog.txt")) {
        Serial.println("datalog.txt deleted.");
      } else {
        Serial.println("Error deleting datalog.txt.");
      }
      // Recreate the file
      File dataFile = SD.open("datalog.txt", FILE_WRITE);
      if (dataFile) {
        dataFile.close();
        Serial.println("datalog.txt recreated.");
      } else {
        Serial.println("Error recreating datalog.txt.");
      }
    } else {
      Serial.println("datalog.txt does not exist.");
    }
}

void write_data_sd(String dataString) {
  // Get the timestamp
  String timestamp = getTimestamp();
  // open the file.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  // if the file is available, write to it
  if (dataFile) {
    dataFile.println(timestamp + " -> " + dataString);
    dataFile.close();
    //Serial.println("logged data!!");
    //Serial.println(timestamp + " -" + dataString);
  } else {
    // if the file isn't open, pop up an error:
    Serial.println("error opening datalog.txt");
  }
}

void gradient_lumen_light(int cycles) {
  for (int i = 0; i < cycles; i++) {
    for (int lightVal = 1100; lightVal <=1900; lightVal++) {
      lightServo.writeMicroseconds(lightVal);
      delay(1);
    }
    for (int lightVal = 1900; lightVal >=1100; lightVal--) {
      lightServo.writeMicroseconds(lightVal);
      delay(1);
    }
  }
}

String getTimestamp() {
  unsigned long milliseconds = millis();
  unsigned long seconds = milliseconds / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;

  milliseconds %= 1000;
  seconds %= 60;
  minutes %= 60;

  // Format the timestamp
  String timestamp = "";
  if (hours < 10) timestamp += "0"; // Add leading zero for hours
  timestamp += String(hours) + ":";
  if (minutes < 10) timestamp += "0"; // Add leading zero for minutes
  timestamp += String(minutes) + ":";
  if (seconds < 10) timestamp += "0"; // Add leading zero for seconds
  timestamp += String(seconds) + ".";
  if (milliseconds < 100) timestamp += "0"; // Add leading zeros for milliseconds
  if (milliseconds < 10) timestamp += "0";
  timestamp += String(milliseconds);

  return timestamp;
}