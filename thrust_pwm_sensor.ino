#include <Servo.h>
#include <Wire.h>
#include "MS5837.h"
MS5837 sensor;
Servo servo[8];
byte servoPins[] = {2, 3, 4, 5, 6, 7, 8, 9};
int firstWirePin = 14;
int secondWirePin = 15;
int firstVal = 0;
int secondVal = 0;
void setup() {
  Serial.begin(9600);
  config_servo();
  config_depth_sensor();
  delay(7000); // delay to allow the ESC to recognize the stopped signal
}
void loop() {
  Serial.println("Enter servoNum and PWM signal value 1100 to 1900 (Ex: 7 1100)");
  while (Serial.available() <= 0){
    firstVal = analogRead(firstWirePin);
    secondVal = analogRead(secondWirePin);
    double current = (firstVal * 120.0) / 1024;
    double voltage = (secondVal * 60.0) / 1024;
    sensor.read();
    Serial.println(String(firstVal) + "     current:" + String(current, 2) + "A    voltage: " + String(voltage, 2) + "V    pressure: " + String(sensor.pressure(), 2) + "mbar    temperature: " + String(sensor.temperature(), 2) + "C    depth: " + String(sensor.depth(), 2) + "m    altitude: " + String(sensor.altitude(), 2) + "m above mean sea level");
  }
  String input = Serial.readString();
  int servoNum = (input.substring(0,1)).toInt();
  int val = (input.substring(2)).toInt();
  if (val < 1100 || val > 1900 || servoNum < 2 || servoNum > 9) {
    Serial.println("not valid");
    val = 1500;
    servoNum = -1;
  } else {
    set_servo(servoNum, val);
  }
}
void config_servo() {
  for (int i = 0; i < 8; i++)
  {
    servo[i].attach(servoPins[i]);
    servo[i].writeMicroseconds(1500);
  }
  pinMode(LED_BUILTIN, OUTPUT);
}
void config_depth_sensor(){
  Wire.begin();
  sensor.setModel(MS5837::MS5837_02BA); // model number for Bar02
  sensor.init();
  sensor.setFluidDensity(997); // kg/m^3 (997 freshwater, 1029 for seawater)
}
void set_servo(int servoNum, int val) {
  if (servoNum == -1) {
    return;
  }
  Serial.println("Set");
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(100);                      // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  delay(100);
  servo[servoNum-2].writeMicroseconds(val); // Send signal to ESC
}
