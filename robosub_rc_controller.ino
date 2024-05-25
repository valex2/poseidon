// what the diff channels are
// 1 = aileron (probs don't need this because seems more for flying?)
// 2 = elevator (move forward)
// 3 = throttle (acceleration)
// 4 = rudder (direction L/R)


// #include <PinChangeInterrupt.h>
// #include <PinChangeInterruptBoards.h>
// #include <PinChangeInterruptPins.h>
// #include <PinChangeInterruptSettings.h>

#include "attachInterruptEx.h"

#define ELEVATOR_INTERRUPT   3 // right hand up down
#define THROTTLE_INTERRUPT   4 // left hand up down
#define RUDDER_INTERRUPT   2 // left hand right left
#define DEAD_ZONE_LOW   -20 // dead zone defined such that boat stops when not using the controller
#define DEAD_ZONE_HIGH  20

// motor class that will be used to control 
class Motor {
  public:
    Motor(unsigned int IN1, unsigned int IN2, unsigned int EN);
    void speed(int);
  private:
    unsigned int IN1_local;
    unsigned int IN2_local;
    unsigned int EN_local;
};


 Motor::Motor(unsigned int IN1, unsigned int IN2, unsigned int EN) {
  IN1_local = IN1;
  IN2_local = IN2;
  EN_local = EN;
  pinMode(IN1_local, OUTPUT);
  pinMode(IN2_local, OUTPUT);
  pinMode(EN_local, OUTPUT);
}

void Motor::speed(int speed){

  if (speed < 0){
    digitalWrite(IN1_local, LOW);
    digitalWrite(IN2_local, HIGH);
    speed = -speed;
  } else {
    digitalWrite(IN1_local, HIGH);
    digitalWrite(IN2_local, LOW);
  }
  
  analogWrite(EN_local, speed);
}

// define the 4 motors (MAKE SURE PINS ALIGN)
Motor left_front_motor(A1,A3, 5);
Motor right_front_motor(A0,A2,6);
Motor left_back_motor(8,11,10);
Motor right_back_motor(12,13,9);


// remote control variables
volatile long CurrentTime = 0;

volatile long StartTimeElevator = 0;
volatile long PulsesElevator = 0;
volatile long PulsesElevatorMapped = 0;

volatile long StartTimeRudder = 0;
volatile long PulsesRudder = 0;
volatile long PulsesRudderMapped = 0;

volatile long StartTimeThrottle = 0;
volatile long PulsesThrottle = 0;
volatile long PulsesThrottleMapped = 0;


void setup() {
  //pinMode(interruptPin, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(interruptPin), blink, CHANGE);

  pinMode(ELEVATOR_INTERRUPT, INPUT_PULLUP);
  pinMode(THROTTLE_INTERRUPT, INPUT_PULLUP);
  pinMode(RUDDER_INTERRUPT, INPUT_PULLUP);

  // attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(THROTTLE_INTERRUPT), PulseTimerThrottle, CHANGE);
  // attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ELEVATOR_INTERRUPT), PulseTimerElevator,CHANGE);
  // attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(RUDDER_INTERRUPT), PulseTimerRudder, CHANGE);

  attachInterruptEx(digitalPinToInterrupt(THROTTLE_INTERRUPT), PulseTimerThrottle, CHANGE);
  attachInterruptEx(digitalPinToInterrupt(ELEVATOR_INTERRUPT), PulseTimerElevator,CHANGE);
  attachInterruptEx(digitalPinToInterrupt(RUDDER_INTERRUPT), PulseTimerRudder, CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  // cap values to prevent glitching
  if (PulsesThrottle > 2000) {
    PulsesThrottle = 2000;
  }
  else if (PulsesThrottle < 1000) {
    PulsesThrottle = 1000;
  }
  
  if (PulsesElevator > 2000) {
    PulsesElevator = 2000;
  }
  else if (PulsesElevator < 1000) {
    PulsesElevator = 1000;
  }

  if (PulsesRudder > 2000) {
    PulsesRudder = 2000;
  }
  else if (PulsesRudder < 1000) {
    PulsesRudder = 1000;
  }

  // scale the remote controller information
  PulsesThrottleMapped = map(PulsesThrottle, 1000, 2000, -255, 255);
  PulsesElevatorMapped = map(PulsesElevator, 1000, 2000, -255, 255);
  PulsesRudderMapped = map(PulsesRudder, 1000, 2000, -255, 255);

  // check for dead zone and set speed to zero
  if (PulsesRudderMapped > DEAD_ZONE_LOW && PulsesRudderMapped < DEAD_ZONE_HIGH &&
    PulsesElevatorMapped > DEAD_ZONE_LOW && PulsesElevatorMapped < DEAD_ZONE_HIGH &&
    PulsesRudderMapped > DEAD_ZONE_LOW && PulsesRudderMapped < DEAD_ZONE_HIGH) {
      
    left_front_motor.speed(0);
    right_front_motor.speed(0);
    left_back_motor.speed(0);
    right_back_motor.speed(0);
  }

  // set motor speeds based on pulses (NOT SURE ABOUT THIS MATH RN)
  int left_front_motor_speed = (PulsesElevatorMapped + PulsesRudderMapped);
  int right_front_motor_speed = (PulsesElevatorMapped - PulsesRudderMapped);
  int left_back_motor_speed = (PulsesElevatorMapped + PulsesRudderMapped);
  int right_back_motor_speed = (PulsesElevatorMapped - PulsesRudderMapped);

  // need to keep these in pwm range
  left_front_motor_speed = constrain(left_front_motor_speed, -255, 255);
  right_front_motor_speed = constrain(right_front_motor_speed, -255, 255);
  left_back_motor_speed = constrain(left_back_motor_speed, -255, 255);
  right_back_motor_speed = constrain(right_back_motor_speed, -255, 255);


  left_front_motor.speed(left_front_motor_speed);
  right_front_motor.speed(right_front_motor_speed);
  left_back_motor.speed(left_back_motor_speed);
  right_back_motor.speed(right_back_motor_speed);
}

// these read the pulses from the reciever and convert to "pulses"
void PulseTimerThrottle(){
  CurrentTime = micros();
  if (CurrentTime > StartTimeThrottle){
    PulsesThrottle = CurrentTime - StartTimeThrottle;
    StartTimeThrottle = CurrentTime;
  }

}

void PulseTimerElevator(){
  CurrentTime = micros();
  if (CurrentTime > StartTimeElevator){
    PulsesElevator = CurrentTime - StartTimeElevator;
    StartTimeElevator = CurrentTime;
  }

}

void PulseTimerRudder(){
  CurrentTime = micros();
  if (CurrentTime > StartTimeRudder){
    PulsesRudder = CurrentTime - StartTimeRudder;
    StartTimeRudder = CurrentTime;
  }
}