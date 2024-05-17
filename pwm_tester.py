import serial
import time

def main():
    portName = serial.Serial('/dev/cu.usbmodem158479401', baudrate=9600, timeout=1)
    thruster_ids = [f"thruster{i}" for i in range(1, 9)]

    for j in range(2):
        if j == 1:
            time.sleep(5)
        for i, thruster in enumerate(thruster_ids):
            if j == 0:
                command = f"{i+2} 1600\n"
            if j == 1:
                command = f"{i+2} 1500\n"
            print(f"command: {command}")
            portName.write(command.encode())
            ##time.sleep(.1)

if __name__ == '__main__':
    main()
