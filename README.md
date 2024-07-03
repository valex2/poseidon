# poseidon
Stanford Robosub 🏄‍♂️🐳🌊 Electrical Subsystem Repository

BLAME: Vassili -- valex@stanford.edu

[Website](https://www.stanfordrobosub.org/)

# Important notes
## Thrusters
DO NOT RUN THRUSTERS AT FULL POWER FOR THE TIME BEING - 20 to 30% max

DO NOT RUN FOR MORE THAN 10 SECONDS WHILE OUTSIDE OF WATER

## Comms
We are using a Teensy 4.0 that communicates with the Orin over a serial port.

Use serial library in Python, and write as follows:
> portName.write(b"thruster_number delay_value")

### Thruster PWM Assignment
1100 us is full reverse thrust
1500 us is off
1900 us is full forward thrust
