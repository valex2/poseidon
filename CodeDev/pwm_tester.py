import serial
import time

def main():
    portName = serial.Serial('/dev/cu.usbmodem160447601', baudrate=9600, timeout=1)
    thruster_ids = [i for i in range(8)]  # Thruster IDs from 0 to 7
    
    # Turn all thrusters on
    for thruster in thruster_ids:
        command = f"{thruster} 1600\n"
        print(f"Turning on: {command.strip()}")
        portName.write(command.encode())
    
    time.sleep(2)  # Wait 2 seconds
    
    # Turn all thrusters off
    for thruster in thruster_ids:
        command = f"{thruster} 1500\n"
        print(f"Turning off: {command.strip()}")
        portName.write(command.encode())
    
    time.sleep(0)  # Wait 2 seconds before sweeping individually
    
    # Sweep each thruster individually
    for thruster in thruster_ids:
        command = f"{thruster} 1600\n"
        print(f"Turning on {thruster}: {command.strip()}")
        portName.write(command.encode())
        time.sleep(1)  # Wait 1 second
        
        command = f"{thruster} 1500\n"
        print(f"Turning off {thruster}: {command.strip()}")
        portName.write(command.encode())
        time.sleep(0)  # Small delay before moving to the next one

if __name__ == '__main__':
    main()