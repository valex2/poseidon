#include <Wire.h>
#include "MS5837.h"

MS5837 sensor;
bool sensorInitialized = false;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  Serial.println("Initializing MS5837 sensor...");

  if (sensor.init()) {
    sensor.setModel(MS5837::MS5837_30BA);  // or MS5837_02BA depending on your sensor
    sensor.setFluidDensity(997);           // Set to 1029 for seawater, 997 for freshwater
    sensorInitialized = true;
    Serial.println("Sensor initialized successfully.");
  } else {
    Serial.println("Sensor initialization failed!");
  }
}

void loop() {
  if (sensorInitialized) {
    sensor.read();

    float temperature = sensor.temperature(); // °C
    float pressure = sensor.pressure();       // mbar
    float depth = sensor.depth();             // meters

    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print(" °C, Pressure: ");
    Serial.print(pressure);
    Serial.print(" mbar, Depth: ");
    Serial.print(depth);
    Serial.println(" m");

  } else {
    Serial.println("Sensor not initialized. Check wiring and power.");
  }

  delay(1000);
}
