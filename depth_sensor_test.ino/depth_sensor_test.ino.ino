#include <Wire.h>
#include "MS5837.h"

// this is a small script for verifying that we're recieving accurate data from our depth sensor.
MS5837 sensor;

void setup() {
  Serial.begin(9600);
  Wire.begin();           // Start I²C communication

  // Uncomment and set the model if required (for example, MS5837_02BA):
  sensor.setModel(MS5837::MS5837_02BA);
  
  // Initialize the sensor; this returns true if the sensor is detected.
  while (!sensor.init()) {
    Serial.println("MS5837 sensor not detected!");
    delay(500);
    Serial.println("Retrying");
  }
  
  // Set the fluid density for depth calculation:
  // 997 for freshwater, 1029 for seawater.
  sensor.setFluidDensity(997);
}

void loop() {
  // Read sensor data (pressure, temperature, and depth)
  sensor.read();
  
  // Retrieve sensor readings:
  float pressure    = sensor.pressure();   // Pressure in mbar
  float temperature = sensor.temperature(); // Temperature in °C
  float depth       = sensor.depth();       // Depth in meters

  // Print the results to the Serial Monitor
  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.print(" mbar, Temperature: ");
  Serial.print(temperature);
  Serial.print(" °C, Depth: ");
  Serial.print(depth);
  Serial.println(" m");

  delay(1000); // Wait 1 second before reading again
}
