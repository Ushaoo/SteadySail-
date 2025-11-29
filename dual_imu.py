import mpu6050
import time
import math


try:
    mpu6050_1 = mpu6050.mpu6050(0x68,1)
    mpu6050_2 = mpu6050.mpu6050(0x68,2)
    print("Successfully initialized both IMUs.")
except OSError as e:
    print(f"Failed to initialize IMUs: {e}")
    print("Please check I2C connections and addresses (run 'sudo i2cdetect -y 1').")
    exit()


# Define a function to read and average the sensor data
def read_and_average_sensor_data():
    # Read the accelerometer values from both sensors
    accel_1 = mpu6050_1.get_accel_data()
    accel_2 = mpu6050_2.get_accel_data()

    # Read the gyroscope values from both sensors
    gyro_1 = mpu6050_1.get_gyro_data()
    gyro_2 = mpu6050_2.get_gyro_data()

    # Read temperature from both sensors
    temp_1 = mpu6050_1.get_temp()
    temp_2 = mpu6050_2.get_temp()

    # Calculate average accelerometer data
    avg_accel = {
        'x': (accel_1['x'] + accel_2['x']) / 2,
        'y': (accel_1['y'] + accel_2['y']) / 2,
        'z': (accel_1['z'] + accel_2['z']) / 2,
    }

    # Calculate average gyroscope data
    avg_gyro = {
        'x': (gyro_1['x'] + gyro_2['x']) / 2,
        'y': (gyro_1['y'] + gyro_2['y']) / 2,
        'z': (gyro_1['z'] + gyro_2['z']) / 2,
    }

    # Calculate average temperature
    avg_temp = (temp_1 + temp_2) / 2

    return avg_accel, avg_gyro, avg_temp

# Start a while loop to continuously read the sensor data
while True:
    try:
        # Read the averaged sensor data
        accelerometer_data, gyroscope_data, temperature = read_and_average_sensor_data()
        
        acc_x = accelerometer_data['x']
        acc_y = accelerometer_data['y']
        acc_z = accelerometer_data['z']

        # Calculate angle from averaged accelerometer data
        # Added a small epsilon to the denominator to avoid division by zero
        angle = math.atan(acc_y / (acc_z + 1e-6)) * 180 / math.pi

        # Print the averaged sensor data
        print(f"Avg Accelerometer: x={acc_x:.2f}, y={acc_y:.2f}, z={acc_z:.2f}")
        print(f"Avg Gyroscope:     x={gyroscope_data['x']:.2f}, y={gyroscope_data['y']:.2f}, z={gyroscope_data['z']:.2f}")
        print(f"Avg Temperature:   {temperature:.2f} C")
        print(f"Calculated Angle:  {angle:.2f} degrees")
        print("-" * 20)

        # Wait for a short period
        time.sleep(0.1)

    except KeyboardInterrupt:
        print("Program stopped by user.")
        break
    except OSError as e:
        print(f"A sensor reading error occurred: {e}")
        print("Please check sensor connections. Retrying in 1 second...")
        time.sleep(1)
