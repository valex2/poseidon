#include <Servo.h>
Servo servo[8];
byte servoPins[] = {2, 3, 4, 5, 6, 7, 8, 9};
void setup() {
  Serial.begin(9600);
  config_servo();
  delay(7000); // delay to allow the ESC to recognize the stopped signal
}
void loop() {
  Serial.println("Enter servoNum and PWM signal value 1100 to 1900 (Ex: 7 1100)");
  while (Serial.available() <= 0){
  }
  String input = Serial.readString();
  int servoNum = (input.substring(0,1)).toInt();
  int val = (input.substring(2)).toInt();
  if (val < 1100 || val > 1900 || servoNum < 0 || servoNum > 8) {
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
}
void set_servo(int servoNum, int val) {
  if (servoNum == -1) {
    return;
  }
  servo[servoNum].writeMicroseconds(val); // Send signal to ESC
}
