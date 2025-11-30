import mpu6050
import time
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

# --- Plotting Configuration ---
MAX_SAMPLES = 100  # Number of samples to display on the plot
# --------------------------

# Initialize data deques for plotting
time_data = deque(maxlen=MAX_SAMPLES)
angle1_data = deque(maxlen=MAX_SAMPLES)
angle2_data = deque(maxlen=MAX_SAMPLES)
avg_angle_data = deque(maxlen=MAX_SAMPLES)

# Initialize the MPU6050 sensors on different I2C addresses.
# Make sure one of your sensors has the AD0 pin pulled to VCC for address 0x69.
try:
    # Note: The second argument to mpu6050 might be a bus number.
    # Using different addresses is the more standard approach.
    mpu6050_1 = mpu6050.mpu6050(0x68) 
    mpu6050_2 = mpu6050.mpu6050(0x68 , 2)
    print("Successfully initialized both IMUs.")
except OSError as e:
    print(f"Failed to initialize IMUs: {e}")
    print("Please check I2C connections and addresses (run 'sudo i2cdetect -y 1').")
    print("Ensure one IMU is at 0x68 and the other at 0x69 (AD0 pin to VCC).")
    exit()


# Define a function to read and process the sensor data
def read_and_process_data():
    try:
        # Read the accelerometer values from both sensors
        accel_1 = mpu6050_1.get_accel_data()
        accel_2 = mpu6050_2.get_accel_data()

        # Calculate individual angles
        angle_1 = math.atan(accel_1['y'] / (accel_1['z'] + 1e-6)) * 180 / math.pi
        angle_2 = math.atan(accel_2['y'] / (accel_2['z'] + 1e-6)) * 180 / math.pi

        # Calculate average angle from the two individual angles
        avg_angle = (angle_1 + angle_2) / 2

        return angle_1, angle_2, avg_angle
    except OSError as e:
        print(f"A sensor reading error occurred: {e}")
        return None, None, None


# --- Matplotlib Setup ---
fig, ax = plt.subplots()
line1, = ax.plot([], [], lw=2, label='IMU 1 Angle (0x68)')
line2, = ax.plot([], [], lw=2, label='IMU 2 Angle (0x69)')
line_avg, = ax.plot([], [], lw=2, linestyle='--', color='red', label='Average Angle')
ax.legend()
ax.set_xlabel('Time (s)')
ax.set_ylabel('Angle (degrees)')
ax.set_title('Real-time IMU Angles')
ax.grid(True)

start_time = time.time()

# This function is called periodically by FuncAnimation
def update(frame):
    # Read the sensor data
    angle_1, angle_2, avg_angle = read_and_process_data()

    if angle_1 is not None:
        current_time = time.time() - start_time
        
        # Append new data
        time_data.append(current_time)
        angle1_data.append(angle_1)
        angle2_data.append(angle_2)
        avg_angle_data.append(avg_angle)

        # Update plot data
        line1.set_data(time_data, angle1_data)
        line2.set_data(time_data, angle2_data)
        line_avg.set_data(time_data, avg_angle_data)

        # Re-scale the axes
        ax.relim()
        ax.autoscale_view()
        
        # Print the latest values to the console
        print(f"Time: {current_time:.2f}s | Angle 1: {angle_1:.2f} | Angle 2: {angle_2:.2f} | Avg Angle: {avg_angle:.2f}")

    # blit=True requires the update function to return an iterable of all the artists that were modified
    return line1, line2, line_avg,

# Set up plot to call update() function periodically
# The interval is in milliseconds. 100ms = 10Hz update rate.
ani = FuncAnimation(fig, update, blit=True, interval=100)

try:
    print("Displaying plot. Close the plot window to stop the script.")
    plt.show()
except KeyboardInterrupt:
    print("Program stopped by user.")
finally:
    print("Script finished.")
