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
    print("Device not found. Only local plotting will be available.")
    SERIAL_PORT = None  # Mark that no device is connected
    BAUD_RATE = None

# Make the output directory relative to the script location
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIRECTORY = os.path.join(SCRIPT_DIR, "data")
if not os.path.exists(OUTPUT_DIRECTORY):
    os.makedirs(OUTPUT_DIRECTORY)

def initialize_serial(port, baud_rate):
    try:
        ser = serial.Serial(port, baud_rate, timeout=2)
        print(f"Connected to {port} at {baud_rate} baud.")
        return ser
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return None

##############################
def transfer_file(ser):
    if ser is None:
        print("No device connected; cannot transfer file.")
        return None

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
                    timeout_counter = 0  # Reset timeout counter when data is received

                    # Check for transfer completion message
                    if "File transfer complete" in line:
                        file.write(line)
                        break

                    file.write(line + '\n')
                    line_count += 1

                    if line_count % 100 == 0:
                        print(f"Received {line_count} lines")
                else:
                    time.sleep(0.01)
                    timeout_counter += 1
                    if timeout_counter > 1000:
                        print("Warning: No data received for 10 seconds")
                        break
            except serial.SerialException as e:
                print(f"Serial error occurred: {e}")
                break
    
    print(f"Transfer complete! Total lines: {line_count}")
    return output_file_path

def delete_file(ser):
    if ser is None:
        print("No device connected; cannot delete file on device.")
        return
    ser.write(b'delete\n')
    while True:
        line = ser.readline().decode('utf-8').strip()
        print(line)
        if "datalog.txt recreated" in line or "datalog.txt does not exist" in line:
            break

def send_servo_command(ser, command):
    if ser is None:
        print("No device connected; cannot send servo command.")
        return
    ser.write((command + '\n').encode('utf-8'))
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

    with open(data_path, 'r') as file:
        lines = file.readlines()

    for line in lines:
        if '->' not in line:
            continue

        # Parse timestamp (assumed format: HH:MM:SS.microseconds)
        timestamp_str = line.split('->')[0].strip()
        try:
            timestamp = datetime.strptime(timestamp_str, '%H:%M:%S.%f')
        except Exception:
            continue

        # Default values for measurements
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

        data_part = line.split('->', 1)[1].strip()

        # Tokenize the data part
        measurements = []
        cur_measurement = []
        for token in data_part.split():
            if ':' in token and cur_measurement:
                measurements.append(' '.join(cur_measurement))
                cur_measurement = [token]
            else:
                cur_measurement.append(token)
        if cur_measurement:
            measurements.append(' '.join(cur_measurement))

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
                if len(servo_vals) < 8:
                    servo_vals += [np.nan] * (8 - len(servo_vals))
                elif len(servo_vals) > 8:
                    servo_vals = servo_vals[:8]
                line_servos = servo_vals
            elif key == 'temps':
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

    # Compute linear time axis with rollover handling
    if times:
        times_seconds = []
        cumulative_offset = 0
        last_value = None
        for t in times:
            current_value = t.hour * 3600 + t.minute * 60 + t.second + t.microsecond/1e6
            if last_value is not None and current_value < last_value:
                cumulative_offset += 24 * 3600
            times_seconds.append(current_value + cumulative_offset)
            last_value = current_value
    else:
        times_seconds = []

    times_minutes = [s / 60.0 for s in times_seconds]
    times_plot = times_minutes

    servos_fixed = [s if len(s)==8 else s + [np.nan]*(8-len(s)) for s in servos]
    servos_array = np.array(servos_fixed)

    plt.figure(figsize=(18, 9))

    # Subplot 1: Pressure
    plt.subplot(4, 2, 1)
    plt.plot(times_plot, bulkhead_pressure, 'b-', label='Bulkhead Pressure')
    plt.plot(times_plot, mid_pressure, 'c-', label='MID Pressure')
    plt.title('Pressure over Time')
    plt.xlabel('Time (min)')
    plt.ylabel('Pressure (hPa)')
    plt.legend()
    plt.grid(True)

    # Subplot 2: Temperature
    plt.subplot(4, 2, 2)
    plt.plot(times_plot, imu_temp, label='IMU Temp')
    plt.plot(times_plot, esc_temp, label='ESC Temp')
    plt.plot(times_plot, orin_temp, label='Orin Temp')
    plt.plot(times_plot, bulkhead_temp, label='Bulkhead Temp')
    plt.plot(times_plot, mid_temp, label='MID Temp')
    plt.title('Temperature over Time')
    plt.xlabel('Time (min)')
    plt.ylabel('Temperature (°C)')
    plt.legend()
    plt.grid(True)

    # Subplot 3: Humidity
    plt.subplot(4, 2, 3)
    plt.plot(times_plot, bulkhead_humidity, 'b-', label='Bulkhead Humidity')
    plt.plot(times_plot, mid_humidity, 'c-', label='MID Humidity')
    plt.title('Humidity over Time')
    plt.xlabel('Time (min)')
    plt.ylabel('Humidity (%)')
    plt.legend()
    plt.grid(True)

    # Subplot 4: Current
    plt.subplot(4, 2, 4)
    plt.plot(times_plot, current, 'r-')
    plt.title('Current over Time')
    plt.xlabel('Time (min)')
    plt.ylabel('Current (A)')
    plt.grid(True)

    # Subplot 5: Voltage
    plt.subplot(4, 2, 5)
    plt.plot(times_plot, voltage, 'm-')
    plt.title('Voltage over Time')
    plt.xlabel('Time (min)')
    plt.ylabel('Voltage (V)')
    plt.grid(True)

    # Subplot 6: Servo Positions
    plt.subplot(4, 2, 6)
    for i in range(8):
        plt.plot(times_plot, servos_array[:, i], label=f'Servo {i+1}')
    plt.title('Servo Positions over Time')
    plt.xlabel('Time (min)')
    plt.ylabel('Position')
    plt.legend()
    plt.grid(True)

    # Subplot 7: Depth
    plt.subplot(4, 2, 7)
    plt.plot(times_plot, depth, 'g-')
    plt.title('Depth over Time (EXTdepth)')
    plt.xlabel('Time (min)')
    plt.ylabel('Depth')
    plt.grid(True)

    # Subplot 8: Internal States
    plt.subplot(4, 2, 8)
    plt.plot(times_plot, operational_flags, 'ko-', label='Operational')
    plt.plot(times_plot, kill_switch_flags, 'ro-', label='KillSwitch')
    plt.title('Internal States over Time')
    plt.xlabel('Time (min)')
    plt.ylabel('State')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()

def list_and_plot_files():
    files = os.listdir(OUTPUT_DIRECTORY)
    if not files:
        print("No files found in data folder.")
        return
    print("Available data files:")
    for i, filename in enumerate(files):
        print(f"[{i}] {filename}")
    try:
        index = int(input("Enter the index number of the file to plot: "))
        if index < 0 or index >= len(files):
            print("Invalid index.")
            return
        selected_file = os.path.join(OUTPUT_DIRECTORY, files[index])
        print(f"Plotting file: {selected_file}")
        plot_data(selected_file)
    except Exception as e:
        print("Error:", e)

def main():
    ser = None
    # Try to initialize serial only if a device was found
    if SERIAL_PORT is not None:
        ser = initialize_serial(SERIAL_PORT, BAUD_RATE)
    
    # Interactive command loop
    while True:
        command = input("Enter command ('transfer', 'transfer plot delete (tpd)', 'delete', 'list', 'exit'): ").strip()

        if command.lower() == "exit":
            break
        elif command.lower() == "transfer":
            if ser:
                transfer_file(ser)
            else:
                print("No device connected; cannot perform transfer.")
        elif command.lower() in ["transfer plot delete", "tpd"]:
            if ser:
                file_path = transfer_file(ser)
                if file_path:
                    plot_data(file_path)
                    delete_file(ser)
            else:
                print("No device connected; cannot perform transfer plot delete.")
        elif command.lower() == "delete":
            if ser:
                delete_file(ser)
            else:
                print("No device connected; cannot perform delete.")
        elif command.lower() == "list":
            list_and_plot_files()
        else:
            if ser:
                ser.write((command + '\n').encode('utf-8'))
                print(f"Sent command: {command}")
                time.sleep(0.5)
                while ser.in_waiting:
                    line = ser.readline().decode('utf-8').strip()
                    print(line)
            else:
                print("No device connected; command ignored.")
                continue

    if ser:
        ser.close()
    print("Serial connection closed.")

if __name__ == "__main__":
    main()
