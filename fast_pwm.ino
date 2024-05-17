#include <Servo.h>

Servo servo[8];
byte servoPins[] = {2, 3, 4, 5, 6, 7, 8, 9};
char inputBuffer[32]; // Buffer to store incoming serial data
int bufferPosition = 0; // Position in the buffer

void setup() {
  Serial.begin(9600);
  config_servo();
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

void config_servo() {
  for (int i = 0; i < 8; i++) {
    servo[i].attach(servoPins[i]);
    servo[i].writeMicroseconds(1500); // Default neutral position
  }
  pinMode(LED_BUILTIN, OUTPUT);
}

void process_input(char *input) {
  int servoNum, val;
  if (sscanf(input, "%d %d", &servoNum, &val) == 2) { // Successfully parsed both numbers
    if (val >= 1100 && val <= 1900 && servoNum >= 2 && servoNum <= 9) {
      set_servo(servoNum, val);
    } else {
      Serial.println("not valid"); // Input out of range
    }
  } else {
    Serial.println("Invalid format"); // Input does not match expected format
  }
}

void set_servo(int servoNum, int val) {
  if (servoNum < 2 || servoNum > 9) return; // Out of valid servo range
  servo[servoNum - 2].writeMicroseconds(val); // Adjust servo position
}
