# poseidon
Stanford Robosub 🏄‍♂️🐳🌊 Electrical Repository

Notion linked here: https://www.notion.so/Electrical-Team-Outline-0ae5ed460508490893e823047c2d156e?pvs=4

Reach out to Vassili -- valex@stanford.edu

PWM code -- credit to Ryota
DO NOT RUN THRUSTERS AT FULL POWER FOR THE TIME BEING - 20 to 30% max
DO NOT RUN FOR MORE THAN 10 SECONDS WHILE OUTSIDE OF WATER

-- writiing to the thrusters:
1100 us is full reverse thrust
1500 us is off
1900 us is full forward thrust

PWM pins 2 - 9 on the MEGA are controlled, match these to thrusters

Step by step:
1. Upload code to Arduino
2. Exit Arduino IDE
3. open terminal
   -- on mac: ls /dev/cu*
   -- find the name of the new port that the Arduino is connected to, copy it
4. python script
   -- import serial
   -- portName = serial.Serial("name from 3")
   -- portName.write(b"thruster_number delay_value")
