import serial
import time

def main():
    portName = serial.Serial('/dev/cu.usbmodem160447601', baudrate=9600, timeout=1)
    thruster_ids = [f"thruster{i}" for i in range(1, 9)]

    for j in range(2):
        if j == 1:
            time.sleep(2) 
            command = "voltage\n"
            print(command)
            portName.write(command.encode())
            data = portName.read(portName.in_waiting)
            print(data.decode('utf-8'))

            time.sleep(2) 
            command = "depth\n"
            print(command)
            portName.write(command.encode())
            data = portName.read(portName.in_waiting)
            print(data.decode('utf-8'))
            
            time.sleep(2) 
            command = "voltage\n"
            print(command)
            portName.write(command.encode())
            data = portName.read(portName.in_waiting)
            print(data.decode('utf-8'))
            
            time.sleep(2)

        for i, thruster in enumerate(thruster_ids):
            if j == 0:
                command = f"{i+2} 1600\n"
            if j == 1:
                command = f"{i+2} 1500\n"
            print(f"command: {command}")
            portName.write(command.encode())
            ##time.sleep(.1)

    time.sleep(2)
    # test different thrusts
    command = "2 1600\n"
    print(command)
    portName.write(command.encode())
    # 3 should not go
    time.sleep(3)
    command = "3 1600\n"
    print(command)
    portName.write(command.encode())
    command = "2 1500\n"
    print(command)
    portName.write(command.encode())
    # swap 
    time.sleep(3)
    command = "3 1500\n"
    print(command)
    portName.write(command.encode())

if __name__ == '__main__':
    main()
