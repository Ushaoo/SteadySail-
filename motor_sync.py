from motor_test import PCA9685
import mpu6050
import time
import math

mode = 4
motor_channels = [0, 1, 2, 3, 4, 5, 6]

motor_channel_1 = 0
motor_channel_2 = 1
motor_channel_3 = 2
motor_channel_4 = 3
motor_channel_5 = 4
motor_channel_6 = 5

# Initialize the PCA9685 module for motor control
pwm = PCA9685(address=0x40, debug=True, bus_num=1)
pwm.setPWMFreq(50)  # Set frequency to 50 Hz

# Initialize the MPU6050 IMU sensor
# imu = mpu6050.mpu6050(0x68)

base_pulse = 1500  # Neutral pulse width (1500 us for most ESCs)
motor_pulse = 1700  # Test pulse width to run the motor

try:
    # Initialize all motors to neutral
    for ch in motor_channels:
        pwm.setServoPulse(ch, base_pulse)
    time.sleep(3)

    if mode == 1:
        while True:
            for ch in motor_channels:
                print(f"Running motor on channel {ch} for 5 seconds...")
                pwm.setServoPulse(ch, motor_pulse)
                time.sleep(2)
                pwm.setServoPulse(ch, base_pulse)
                print(f"Stopped motor on channel {ch}.")
                time.sleep(1)  # Optional: wait 1 second before next motor


    if mode == 2:
        while True:
            
        
            # pwm.setServoPulse(motor_channel_1, motor_pulse)
            # pwm.setServoPulse(motor_channel_2, 3000 - motor_pulse)
            # pwm.setServoPulse(motor_channel_3, motor_pulse)
            # pwm.setServoPulse(motor_channel_4, motor_pulse)
            # pwm.setServoPulse(motor_channel_5, 0)

            # print("up")
            
            # time.sleep(5)


            motor_pulse = 3000 - motor_pulse  
            # pwm.setServoPulse(motor_channel_1, motor_pulse)
            # pwm.setServoPulse(motor_channel_2, 3000 - motor_pulse)
            # pwm.setServoPulse(motor_channel_3, motor_pulse)
            # pwm.setServoPulse(motor_channel_4, motor_pulse)
            # pwm.setServoPulse(motor_channel_5, 0)
            # print("down")
            
            # time.sleep(5)


            pwm.setServoPulse(motor_channel_1, motor_pulse)
            pwm.setServoPulse(motor_channel_2, 3000 - motor_pulse)
            pwm.setServoPulse(motor_channel_3, 3000 - motor_pulse)
            pwm.setServoPulse(motor_channel_4, 3000 - motor_pulse)
            pwm.setServoPulse(motor_channel_5, 0)
            print("tilt1")
            
            time.sleep(8)
            
            motor_pulse = 3000 - motor_pulse 
            pwm.setServoPulse(motor_channel_1, motor_pulse)
            pwm.setServoPulse(motor_channel_2, 3000 - motor_pulse)
            pwm.setServoPulse(motor_channel_3, 3000 - motor_pulse)
            pwm.setServoPulse(motor_channel_4, 3000 - motor_pulse)
            pwm.setServoPulse(motor_channel_5, 0)
            print("tilt2")
            time.sleep(8)


    if mode == 3:
        while True:
            # pwm.setServoPulse(motor_channel_1, 1500)
            # print("1500")
            # time.sleep(0.1)
            pwm.setServoPulse(motor_channel_3, 1700)
            pwm.setServoPulse(motor_channel_1, 1700)

            # pwm.setServoPulse(motor_channel_2, 3000 - 1700)
            # pwm.setServoPulse(motor_channel_4, 1700)

            pwm.setServoPulse(motor_channel_5, 2000)
            
            
            print("1500")
            time.sleep(3)


    if mode == 4:
        while True:
            for i in range(1000, 2100, 100):
                pwm.setServoPulse(3, 3000 - i)
                pwm.setServoPulse(4, i)
                print(f"Set all motors to {i} us")
                time.sleep(1)

except KeyboardInterrupt:
    print("Stopping all motors...")
    for ch in motor_channels:
        pwm.setServoPulse(ch, base_pulse)
    time.sleep(0.5)
    for ch in motor_channels:
        pwm.setServoPulse(ch, 0)