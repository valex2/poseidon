import serial

# See page 126/276 (pg. 112 in the manual)

# Define the serial port and baud rate (115200)
ser = serial.Serial('COM1', 115200)  # Use the appropriate COM port and baud rate

# Send data over the serial port
data = b'+++'
ser.write(data)

# Close the serial port
ser.close()
