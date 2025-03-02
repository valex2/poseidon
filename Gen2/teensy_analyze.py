import serial
import serial.tools.list_ports
import os
from datetime import datetime
import time
from tqdm import tqdm
import matplotlib.pyplot as plt
import numpy as np

###### Configuration ######
def find_device_port():
    ports = list(serial.tools.list_ports.comports())
    
    for port in ports:
        print(f"Found port: {port.device} - {port.description}")
        
        if "usbmodem" in port.device:
            print(f"Found Teensy on port: {port.device}")
            return port.device
    return None

device_port = find_device_port()
if device_port:
    print(f"Device found on port: {device_port}")
    SERIAL_PORT = device_port
    BAUD_RATE = 100000
else:
    print("Device not found. Please check your connection.")

# make the output directory the location of the script
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIRECTORY = os.path.join(SCRIPT_DIR, "data")  # Replace with your desired output directory
if not os.path.exists(OUTPUT_DIRECTORY):
    os.makedirs(OUTPUT_DIRECTORY)

def initialize_serial(port, baud_rate): # function to initialize the serial connection
    try:
        ser = serial.Serial(port, baud_rate, timeout=2)
        print(f"Connected to {port} at {baud_rate} baud.")
        return ser
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return None
##############################s

def transfer_file(ser):
    print("Transferring datalog.txt ...")
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_file_path = os.path.join(OUTPUT_DIRECTORY, f"datalog_{timestamp}.txt")
    
    ser.write(b'transfer\n')
    ser.flush()
    
    line_count = 0
    timeout_counter = 0
    
    with open(output_file_path, 'w') as file:
        while True:
            try:
                if ser.in_waiting:
                    line = ser.readline().decode('utf-8').strip()
                    timeout_counter = 0  # Reset timeout counter when we receive data

                    # Check if we've reached the end of the transfer
                    if "File transfer complete" in line:
                        file.write(line)
                        break

                    file.write(line + '\n')
                    line_count += 1

                    if line_count % 100 == 0:
                        print(f"Received {line_count} lines")
                else:
                    time.sleep(0.01)  # Wait if no data
                    timeout_counter += 1
                    if timeout_counter > 1000:  # 10 second timeout
                        print("Warning: No data received for 10 seconds")
                        break
            except serial.SerialException as e:
                print(f"Serial error occurred: {e}")
                break
    
    print(f"Transfer complete! Total lines: {line_count}")
    return output_file_path  # Modified to return the file path

# Function to delete the file
def delete_file(ser):
    # Send the 'delete' command to the Arduino
    ser.write(b'delete\n')

    # Wait for the Arduino to respond
    while True:
        line = ser.readline().decode('utf-8').strip()
        print(line)
        if "datalog.txt recreated" in line or "datalog.txt does not exist" in line:
            break

# Function to send servo commands
def send_servo_command(ser, command):
    # Send the servo command to the Arduino
    ser.write((command + '\n').encode('utf-8'))

    # Wait for the Arduino to respond
    while True:
        line = ser.readline().decode('utf-8').strip()
        print(line)
        if "Invalid command" in line or ">" in line:
            break

def plot_data(data_path):
    # Initialize lists for each measurement
    times = []
    bulkhead_pressure = []
    mid_pressure = []
    depth = []
    current = []
    voltage = []
    servos = []
    imu_temp = []
    esc_temp = []
    orin_temp = []
    bulkhead_temp = []
    mid_temp = []
    bulkhead_humidity = []
    mid_humidity = []
    operational_flags = []
    kill_switch_flags = []

    # Read file lines
    with open(data_path, 'r') as file:
        lines = file.readlines()

    for line in lines:
        if '->' not in line:
            continue

        # Parse the timestamp
        timestamp_str = line.split('->')[0].strip()
        try:
            timestamp = datetime.strptime(timestamp_str, '%H:%M:%S.%f')
        except Exception:
            continue

        # Set default values for this line
        line_bulkhead_pressure = np.nan
        line_mid_pressure = np.nan
        line_depth = np.nan
        line_current = np.nan
        line_voltage = np.nan
        line_servos = [np.nan] * 8
        line_imu_temp = np.nan
        line_esc_temp = np.nan
        line_orin_temp = np.nan
        line_bulkhead_temp = np.nan
        line_mid_temp = np.nan
        line_bulkhead_humidity = np.nan
        line_mid_humidity = np.nan
        line_operational = np.nan
        line_kill_switch = np.nan

        # Get data part (everything after the arrow)
        data_part = line.split('->', 1)[1].strip()

        # Tokenize the data part into individual measurements
        measurements = []
        cur_measurement = []
        for token in data_part.split():
            # Start a new measurement when a token with ':' is encountered and there is an existing measurement
            if ':' in token and cur_measurement:
                measurements.append(' '.join(cur_measurement))
                cur_measurement = [token]
            else:
                cur_measurement.append(token)
        if cur_measurement:
            measurements.append(' '.join(cur_measurement))

        # Process each measurement token
        for meas in measurements:
            if ':' not in meas:
                continue
            key, value = meas.split(':', 1)
            key = key.strip()
            value = value.strip()

            if key == 'current':
                try:
                    line_current = float(value)
                except:
                    line_current = np.nan
            elif key == 'voltage':
                try:
                    line_voltage = float(value)
                except:
                    line_voltage = np.nan
            elif key == 'servo':
                # Value example: "PWM:1500,PWM:1500,..." → extract numeric parts
                servo_vals = []
                for token in value.split(','):
                    token = token.strip()
                    if token:
                        parts = token.split(':')
                        if len(parts) > 1:
                            try:
                                servo_vals.append(float(parts[1]))
                            except:
                                servo_vals.append(np.nan)
                # Pad or truncate to exactly 8 values
                if len(servo_vals) < 8:
                    servo_vals += [np.nan] * (8 - len(servo_vals))
                elif len(servo_vals) > 8:
                    servo_vals = servo_vals[:8]
                line_servos = servo_vals
            elif key == 'temps':
                # Example: "IMU_Temp:24.06250000,ESC_Temp:25.62500000,Orin_Temp:24.00000000,
                # Bulkhead_BME:26.80999947,MID_BME:23.92000008,..."
                temp_dict = {}
                for token in value.split(','):
                    token = token.strip()
                    if token:
                        try:
                            tkey, tval = token.split(':', 1)
                            temp_dict[tkey] = float(tval)
                        except:
                            continue
                line_imu_temp = temp_dict.get("IMU_Temp", np.nan)
                line_esc_temp = temp_dict.get("ESC_Temp", np.nan)
                line_orin_temp = temp_dict.get("Orin_Temp", np.nan)
                line_bulkhead_temp = temp_dict.get("Bulkhead_BME", np.nan)
                line_mid_temp = temp_dict.get("MID_BME", np.nan)
            elif key == 'pressure':
                # Example: "Bulkhead_BME:1011.38031006,MID_BME:1011.94891357,
                # EXTpressure:0.00000000,EXTtemperature:20.00000000,EXTdepth:-10.36080742,"
                tokens = value.split(',')
                for token in tokens:
                    token = token.strip()
                    if token.startswith("Bulkhead_BME"):
                        try:
                            _, p_str = token.split(':', 1)
                            line_bulkhead_pressure = float(p_str)
                        except:
                            line_bulkhead_pressure = np.nan
                    elif token.startswith("MID_BME"):
                        try:
                            _, p_str = token.split(':', 1)
                            line_mid_pressure = float(p_str)
                        except:
                            line_mid_pressure = np.nan
                    elif token.startswith("EXTdepth"):
                        try:
                            _, d_str = token.split(':', 1)
                            line_depth = float(d_str)
                        except:
                            line_depth = np.nan
            elif key == 'humidity':
                # Example: "Bulkhead_BME:30.58789062,MID_BME:35.72363281,"
                tokens = value.split(',')
                for token in tokens:
                    token = token.strip()
                    if token.startswith("Bulkhead_BME"):
                        try:
                            _, h_str = token.split(':', 1)
                            line_bulkhead_humidity = float(h_str)
                        except:
                            line_bulkhead_humidity = np.nan
                    elif token.startswith("MID_BME"):
                        try:
                            _, h_str = token.split(':', 1)
                            line_mid_humidity = float(h_str)
                        except:
                            line_mid_humidity = np.nan
            elif key == 'internalStates':
                # Example: "Operational:1,KillSwitch:1,"
                tokens = value.split(',')
                for token in tokens:
                    token = token.strip()
                    if token:
                        try:
                            flag_key, flag_val = token.split(':', 1)
                            if flag_key == "Operational":
                                line_operational = int(flag_val)
                            elif flag_key == "KillSwitch":
                                line_kill_switch = int(flag_val)
                        except:
                            continue

        # Append the values for this line
        times.append(timestamp)
        bulkhead_pressure.append(line_bulkhead_pressure)
        mid_pressure.append(line_mid_pressure)
        depth.append(line_depth)
        current.append(line_current)
        voltage.append(line_voltage)
        servos.append(line_servos)
        imu_temp.append(line_imu_temp)
        esc_temp.append(line_esc_temp)
        orin_temp.append(line_orin_temp)
        bulkhead_temp.append(line_bulkhead_temp)
        mid_temp.append(line_mid_temp)
        bulkhead_humidity.append(line_bulkhead_humidity)
        mid_humidity.append(line_mid_humidity)
        operational_flags.append(line_operational)
        kill_switch_flags.append(line_kill_switch)

    if times:
        base_time = times[0]
        times_seconds = [(t - base_time).total_seconds() for t in times]
    else:
        times_seconds = []

    # Convert elapsed seconds to minutes
    times_minutes = [s / 60.0 for s in times_seconds]

    times_plot = times_minutes

    # Convert servos list to a 2D NumPy array.
    # Ensure each servo reading is exactly length 8.
    servos_fixed = [s if len(s)==8 else s + [np.nan]*(8-len(s)) for s in servos]
    servos_array = np.array(servos_fixed)  # Now shape (N, 8)

    # Create a panel plot using a 4x2 grid with figure size that fits your screen (18x12 inches)
    plt.figure(figsize=(18, 9))

    # Subplot 1: Pressure (Bulkhead and MID)
    plt.subplot(4, 2, 1)
    plt.plot(times_plot, bulkhead_pressure, 'b-', label='Bulkhead Pressure')
    plt.plot(times_plot, mid_pressure, 'c-', label='MID Pressure')
    plt.title('Pressure over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Pressure (hPa)')
    plt.legend()
    plt.grid(True)

    # Subplot 2: Temperature (all channels)
    plt.subplot(4, 2, 2)
    plt.plot(times_plot, imu_temp, label='IMU Temp')
    plt.plot(times_plot, esc_temp, label='ESC Temp')
    plt.plot(times_plot, orin_temp, label='Orin Temp')
    plt.plot(times_plot, bulkhead_temp, label='Bulkhead Temp')
    plt.plot(times_plot, mid_temp, label='MID Temp')
    plt.title('Temperature over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Temperature (°C)')
    plt.legend()
    plt.grid(True)

    # Subplot 3: Humidity (Bulkhead and MID)
    plt.subplot(4, 2, 3)
    plt.plot(times_plot, bulkhead_humidity, 'b-', label='Bulkhead Humidity')
    plt.plot(times_plot, mid_humidity, 'c-', label='MID Humidity')
    plt.title('Humidity over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Humidity (%)')
    plt.legend()
    plt.grid(True)

    # Subplot 4: Current
    plt.subplot(4, 2, 4)
    plt.plot(times_plot, current, 'r-')
    plt.title('Current over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Current (A)')
    plt.grid(True)

    # Subplot 5: Voltage
    plt.subplot(4, 2, 5)
    plt.plot(times_plot, voltage, 'm-')
    plt.title('Voltage over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Voltage (V)')
    plt.grid(True)

    # Subplot 6: Servo Positions
    plt.subplot(4, 2, 6)
    for i in range(8):
        plt.plot(times_plot, servos_array[:, i], label=f'Servo {i+1}')
    plt.title('Servo Positions over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Position')
    plt.legend()
    plt.grid(True)

    # Subplot 7: Depth (EXTdepth)
    plt.subplot(4, 2, 7)
    plt.plot(times_plot, depth, 'g-')
    plt.title('Depth over Time (EXTdepth)')
    plt.xlabel('Time (s)')
    plt.ylabel('Depth')
    plt.grid(True)

    # Subplot 8: Internal States (Operational and KillSwitch)
    plt.subplot(4, 2, 8)
    plt.plot(times_plot, operational_flags, 'ko-', label='Operational')
    plt.plot(times_plot, kill_switch_flags, 'ro-', label='KillSwitch')
    plt.title('Internal States over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('State')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()

def main():
    # Initialize serial connection
    ser = initialize_serial(SERIAL_PORT, BAUD_RATE)
    if not ser:
        return

    # Interactive command loop
    while True:
        # Prompt the user for input
        command = input("Enter command ('transfer', 'transfer plot delete (tpd)', 'delete', 'exit'): ").strip()

        if command.lower() == "exit":
            break
        elif command.lower() == "transfer":
            print("moving to transfer file")
            transfer_file(ser)
        elif command.lower() == "transfer plot delete" or command.lower() == "tpd":
            file_path = transfer_file(ser)
            plot_data(file_path)
            delete_file(ser)
        elif command.lower() == "delete":
            delete_file(ser)
        else:
            ser.write((command + '\n').encode('utf-8'))
            print(f"Sent command: {command}")
            time.sleep(0.5)
            # Read all available data before continuing
            while ser.in_waiting:
                line = ser.readline().decode('utf-8').strip()
                print(line)
            continue

    # Close the serial connection
    ser.close()
    print("Serial connection closed.")

if __name__ == "__main__":
    main()