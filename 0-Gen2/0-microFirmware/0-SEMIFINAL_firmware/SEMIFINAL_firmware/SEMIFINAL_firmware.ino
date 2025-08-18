#include <Servo.h>
#include <Wire.h>

// Servos
Servo servos[8];   // Array to store servo objects
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
const int killSwitchPin = 26;

// ===== Kill switch ISR state =====
volatile bool isKilled = false;               // Latched when ISR fires
constexpr int KILL_ASSERTED_LEVEL = HIGH; // killed when HIGH (flip if this changes)
volatile unsigned long killISRMicros = 0;

constexpr unsigned long KILL_ISR_DEBOUNCE_US = 3000; // ~3 ms
volatile unsigned long _lastKillIsrUs = 0;

bool killMsgPrinted = false;

// External lumen lights
Servo lightServo;
int lumenPin = 8;
int lightVal = 1100;
int lightCycles = 5;

// Neopixels
// static int pixelUpdateCounter = 0;  // persists between calls
#include <Adafruit_NeoPixel.h>
#define PIN            27          // Pin where NeoPixel strip is connected
#define NUMPIXELS      200        // Total number of pixels
Adafruit_NeoPixel strip(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// SD Card
#include <SD.h>
int sd_loop_counter = 0;
const int chipSelect = BUILTIN_SDCARD;
unsigned long loopIterationCounter = 0; // for perioidic SD logging
int sdLoggingFrequency = 20000; 

// dropper
Servo dropper;
const int dropperPin = 21;  // change this depending
const int dropperMinUS = 800;     // SER-201X spec
const int dropperMaxUS = 2200;    // SER-201X spec
float dropperHalfRangeDeg = 70.0f;   // default ±70° per datasheet (use 100.0f if reprogrammed)
float dropperDeg = 0;

// torpedo
Servo torpedo;
const int torpedoPin = 20;
const int torpedoMinUS = 800;     // SER-201X spec
const int torpedoMaxUS = 2200;    // SER-201X spec
float torpedoHalfRangeDeg = 70.0f;   // default ±70° per datasheet (use 100.0f if reprogrammed)
float torpedoDeg = 0;

// ======= Forward declarations =======
void applyNeutralAll();
void killISR();

void setup() {
    // Begin serial and I2C (via wire)
    Serial.begin(9600);
    Wire.begin();
    
    config_servos();
    config_dropper(); 
    config_torpedo();
    config_depth();
    config_battery();
    config_indicator();   // sets up interrupt
    config_lumen();
    config_neopixels();
    config_sd_card();
    
    Serial.println("Initialize Complete");
}

void config_servos() {
    for (int i = 0; i < 8; i++) {
        servos[i].attach(servoPins[i]);   // Attach servo
        servos[i].writeMicroseconds(1500);  // Start at neutral
    }
}

void config_neopixels() {
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  int brightness = 40; // Set brightness level
  setBlueBreathing(brightness); // Breathing animation for visual indicator
}

void setBlueBreathing(uint8_t brightness) {
  // Generate dynamic shades of blue
  uint8_t blue = brightness;
  uint8_t green = brightness / 4;   // Add a hint of green for cyan tones
  uint8_t red = brightness / 8;    // Very slight purple tint at higher brightness

  uint32_t color = strip.Color(red, green, blue);

  for (int i = 0; i < NUMPIXELS; i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

// Map degrees (-HALF_RANGE..+HALF_RANGE) to microseconds (dropperMinUS..dropperMaxUS)
int degToUS_generic(float deg, float halfRangeDeg, int usMin, int usMax) {
  if (deg < -halfRangeDeg) deg = -halfRangeDeg;
  if (deg > +halfRangeDeg) deg = +halfRangeDeg;
  float t = (deg + halfRangeDeg) / (2.0f * halfRangeDeg); // 0..1
  return (int)(usMin + t * (usMax - usMin) + 0.5f);
}

// Specific wrappers
inline int dropperDegToUS(float d)  { return degToUS_generic(d,  dropperHalfRangeDeg,  dropperMinUS,  dropperMaxUS); }
inline int torpedoDegToUS(float d)  { return degToUS_generic(d,  torpedoHalfRangeDeg,  torpedoMinUS,  torpedoMaxUS); }


void config_dropper() {
  dropper.attach(dropperPin, dropperMinUS, dropperMaxUS);
  dropper.writeMicroseconds(dropperDegToUS(0));
}

void config_torpedo() {
  torpedo.attach(torpedoPin, torpedoMinUS, torpedoMaxUS);
  torpedo.writeMicroseconds(torpedoDegToUS(0)); // neutral/safe
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
  pinMode(killSwitchPin, INPUT); // use INPUT_PULLUP + invert logic if needed

  // Seed state from live pin so LED matches reality at boot
  int ks = digitalRead(killSwitchPin);
  isKilled = (ks == KILL_ASSERTED_LEVEL);
  digitalWrite(greenIndicatorLedPin, isKilled ? LOW : HIGH);

  attachInterrupt(digitalPinToInterrupt(killSwitchPin), killISR, CHANGE);
}


void config_lumen() {
  lightServo.attach(lumenPin);
  lightServo.writeMicroseconds(1100); // Turn off the light
}

void config_sd_card() {
  SD.begin(chipSelect);
  write_data_sd("Configuring");
}

// ===== Interrupt Service Routine =====
void killISR() {
  unsigned long nowUs = micros();
  if (nowUs - _lastKillIsrUs < KILL_ISR_DEBOUNCE_US) return;
  _lastKillIsrUs = nowUs;

  int ks = digitalRead(killSwitchPin);
  bool asserted = (ks == KILL_ASSERTED_LEVEL);

  isKilled = asserted;
  killISRMicros = nowUs;

  #ifdef digitalWriteFast
    digitalWriteFast(greenIndicatorLedPin, asserted ? LOW : HIGH);
  #else
    digitalWrite(greenIndicatorLedPin, asserted ? LOW : HIGH);
  #endif
}

// Apply neutral to all 8 outputs and remember values
void applyNeutralAll() {
  for (int i = 0; i < 8; i++) {
    servos[i].writeMicroseconds(1500);
    lastThrusterPWM[i] = 1500;
  }
  // Also neutralize these:
  dropper.writeMicroseconds(dropperDegToUS(14));
  torpedo.writeMicroseconds(torpedoDegToUS(0));
}

void loop() {
    digitalWrite(greenIndicatorLedPin, isKilled ? LOW : HIGH);
    
    if (isKilled) {
      applyNeutralAll();

      // Print once when entering killed state
      if (!killMsgPrinted) {
        Serial.println(String("[KILLED] ISR at us=") + killISRMicros + ". Outputs forced to 1500us.");
        write_data_sd("KILL asserted -> neutralized all outputs");
        killMsgPrinted = true;
      }

      // Periodic logging still active
      loopIterationCounter++;
      if (loopIterationCounter % sdLoggingFrequency == 0) {
        logPeriodicData();
      }

      // Avoid serial buffer buildup (optional)
      while (Serial.available() > 0) (void)Serial.read();
      return;
    } else {
      // If we just left killed state, note it once
      if (killMsgPrinted) {
        write_data_sd("KILL cleared -> normal operation");
        Serial.println("[READY] Kill cleared by switch.");
        killMsgPrinted = false;
      }
    }

    // ===== Normal operation (not killed) =====
    // SD Card periodic logs
    loopIterationCounter++;
    if (loopIterationCounter % sdLoggingFrequency == 0) {
        logPeriodicData();
    }

    // Indicator LED reflects nominal (on)
    digitalWrite(greenIndicatorLedPin, HIGH);

    // Handle serial input
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

  // Droppers
  } else if (strcmp(input, "pearl-harbor") == 0) {   // dropper left
    int us = dropperDegToUS(22);
    dropper.writeMicroseconds(us);
  } else if (strcmp(input, "iwo-jima") == 0) {   // dropper right
    int us = dropperDegToUS(-22);
    dropper.writeMicroseconds(us);
  } else if (sscanf(input, "dropperPosition %f", &dropperDeg) == 1) {  // dropperDeg is a float
    // Clamp for safety
    if (dropperDeg < -dropperHalfRangeDeg) dropperDeg = -dropperHalfRangeDeg;
    if (dropperDeg > +dropperHalfRangeDeg) dropperDeg = +dropperHalfRangeDeg;

    int us = dropperDegToUS(dropperDeg);
    dropper.writeMicroseconds(us);

    Serial.print("Commanded: ");
    Serial.print(dropperDeg, 1);
    Serial.print(" dropperDeg  ->  ");
    Serial.print(us);
    Serial.println(" us");
  
  // Torpedos
  } else if (strcmp(input, "hiroshima") == 0) {
    int us = torpedoDegToUS(-9);
    torpedo.writeMicroseconds(us);
  } else if (strcmp(input, "nagasaki") == 0) {
    int us = torpedoDegToUS(37);
    torpedo.writeMicroseconds(us);
  } else if (strcmp(input, "enola-gay") == 0) {
    int us = torpedoDegToUS(14);
    torpedo.writeMicroseconds(us);
  } else if (sscanf(input, "torpedoPosition %f", &torpedoDeg) == 1) {
    if (torpedoDeg < -torpedoHalfRangeDeg) torpedoDeg = -torpedoHalfRangeDeg;
    if (torpedoDeg > +torpedoHalfRangeDeg) torpedoDeg = +torpedoHalfRangeDeg;
    int us = torpedoDegToUS(torpedoDeg);
    torpedo.writeMicroseconds(us);
    Serial.print("Commanded: ");
    Serial.print(torpedoDeg, 1);
    Serial.print(" torpedoDeg  ->  ");
    Serial.print(us);
    Serial.println(" us");
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

  dataString += " killSwitchPin:" + String(digitalRead(killSwitchPin)) +
                " isKilled:" + String(isKilled ? 1 : 0);

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
