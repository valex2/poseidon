import matplotlib.pyplot as plt
import sys
import argparse

def plot_temperature_data(file_path):
    """
    Reads temperature data from a file and plots temperature vs. time for each sensor.
    
    Args:
    - file_path (str): The path to the data log file.
    """
    
    # Initialize lists to store temperature data and time
    time_data = []
    temp1_data = []
    temp2_data = []
    temp3_data = []

    # Read the file and extract data
    with open(file_path, 'r') as file:
        for line in file:
            data = line.strip().split(',')
            if len(data) == 4:
                time_data.append(int(data[0]) / 1000)  # Convert milliseconds to seconds
                temp1_data.append(float(data[1]))
                temp2_data.append(float(data[2]))
                temp3_data.append(float(data[3]))

    # Plot the temperature data
    plt.figure(figsize=(10, 6))
    
    # convert time to minutes
    time_data = [x/60 for x in time_data]
    plt.plot(time_data, temp1_data, label='Ethernet/Buck Boost', marker='o')
    plt.plot(time_data, temp2_data, label='Power Board', marker='o')
    plt.plot(time_data, temp3_data, label='IMU/Orin', marker='o')

    # Adding titles and labels
    plt.title('MBARI Internal Temperature Profiles')
    plt.xlabel('Time (Minutes)')
    plt.ylabel('Temperature (°C)')
    plt.legend()
    plt.grid(True)
    plt.savefig('MBARI_temperature_profiles.png')
    plt.show()

def main():
    # Command line argument parser
    parser = argparse.ArgumentParser(description='Plot temperature data from a log file.')
    parser.add_argument('file_path', type=str, help='Path to the data log file.')
    
    # Parse the arguments
    args = parser.parse_args()
    
    # Call the function to plot the data
    plot_temperature_data(args.file_path)

if __name__ == "__main__":
    main()
