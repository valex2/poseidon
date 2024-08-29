#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MCP9808.h>
#include "MS5837.h"

// The setup function runs once when you press reset or power the board
void setup() {
  // Initialize Serial4 for communication at 9600 baud
  Serial4.begin(9600);

  // Wait for the serial port to connect - only needed for native USB
  while (!Serial4) {
    ; // wait for serial port to connect
  }
}

// The loop function runs over and over again forever
void loop() {
  // Define the word to send
  const char* word = "Print this!";

  // Send the word over Serial4
  Serial4.println(word);

  // Wait for a bit before sending the next word
  delay(1000); // Delay of 1 second

  // do something else
  const char* word2 = "Now print this!";

  Serial4.println(word2);

  delay(1000);
}
