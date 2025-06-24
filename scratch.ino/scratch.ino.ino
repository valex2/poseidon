#include <Wire.h>
#include <Adafruit_MCP9808.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// Create MCP9808 instances for both sensors
Adafruit_MCP9808 tempSensor1 = Adafruit_MCP9808();
Adafruit_MCP9808 tempSensor2 = Adafruit_MCP9808();

// BME280 instance
Adafruit_BME280 bme;

// I2C addresses
#define MCP9808_ADDR1 0x18
#define MCP9808_ADDR2 0x19
#define BME280_ADDR   0x77

void setup() {
  Serial.begin(115200);
  while (!Serial);  // Wait for Serial

  Wire.begin();

  // Initialize MCP9808 sensors
  if (!tempSensor1.begin(MCP9808_ADDR1)) {
    Serial.println("Couldn't find MCP9808 sensor at 0x18!");
  }

  if (!tempSensor2.begin(MCP9808_ADDR2)) {
    Serial.println("Couldn't find MCP9808 sensor at 0x19!");
  }

  // Initialize BME280
  if (!bme.begin(BME280_ADDR)) {
    Serial.println("Couldn't find BME280 sensor at 0x77!");
  }

  Serial.println("Sensors initialized.\n");
}

void loop() {
  // MCP9808 temperature readings
  float temp1 = tempSensor1.readTempC();
  float temp2 = tempSensor2.readTempC();

  // BME280 readings
  float tempBME = bme.readTemperature();
  float pressure = bme.readPressure() / 100.0F;  // hPa
  float humidity = bme.readHumidity();

  Serial.println("==== Sensor Readings ====");

  Serial.print("MCP9808 @ 0x18: ");
  Serial.print(temp1);
  Serial.println(" °C");

  Serial.print("MCP9808 @ 0x19: ");
  Serial.print(temp2);
  Serial.println(" °C");

  Serial.print("BME280 @ 0x77 - Temp: ");
  Serial.print(tempBME);
  Serial.print(" °C, Humidity: ");
  Serial.print(humidity);
  Serial.print(" %, Pressure: ");
  Serial.print(pressure);
  Serial.println(" hPa");

  Serial.println("=========================\n");

  delay(1000);  // Delay 1 second between readings
}
