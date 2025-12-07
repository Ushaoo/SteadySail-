import mpu6050
import time
import math

# Create a new Mpu6050 object
mpu6050 = mpu6050.mpu6050(0x68)

# Define a function to read the sensor data
def read_sensor_data():
    # Read the accelerometer values
    accelerometer_data = mpu6050.get_accel_data()

    # Read the gyroscope values
    gyroscope_data = mpu6050.get_gyro_data()

    # Read temp
    temperature = mpu6050.get_temp()

    return accelerometer_data, gyroscope_data, temperature

# Start a while loop to continuously read the sensor data
while True:

    # Read the sensor data
    accelerometer_data, gyroscope_data, temperature = read_sensor_data()
    acc_x =  accelerometer_data['x']
    acc_y =  accelerometer_data['y']
    acc_z =  accelerometer_data['z']

    if acc_z == 0:
        acc_z = 0.0001  # Prevent division by zero
    angle = math.atan(acc_y/acc_z) * 180 / math.pi


    # Print the sensor data
    print("Accelerometer data:", accelerometer_data)
    print("Acc x:", acc_x)
    print("Acc y:", acc_y)
    print("Acc z:", acc_z)
    print("Angle:", angle)
    print("Gyroscope data:", gyroscope_data)
    print("Temp:", temperature)

    # Wait for 1 second
    time.sleep(0.05)