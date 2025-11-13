#!/usr/bin/env python3

from motor_test import PCA9685
import mpu6050
import time
import math
from imu_data import simple_imu_logger

mode = False

angle_low = 5
angle_high = 10
angle_capsized = 35
 
motor_channel_1 = 0
# motor_channel_2 = 1
# motor_channel_3 = 2
# motor_channel_4 = 3
# motor_channel_5 = 4
# motor_channel_6 = 5



 
class PIDController:
    def __init__(self, kp, ki, kd, target_value=0):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.target = target_value
        
        self.previous_error = 0
        self.integral = 0
        
    def gyro_update(self, current_value, dt):
        # Calculate error
        error = self.target - current_value
        
        # Proportional term
        proportional = self.kp * error
        
        # Integral term
        self.integral += error * dt
        integral = self.ki * self.integral
        
        # Derivative term
        derivative = self.kd * (error - self.previous_error) / dt
        
        # Calculate PID output
        output = proportional + integral + derivative
        
        # Store error for next iteration
        self.previous_error = error
        
        return output
    
    def acc_update(self, current_value, dt):
        # Calculate error
        error = self.target - current_value
        
        # Proportional term
        proportional = self.kp * error
        
        # Integral term
        self.integral += error * dt
        integral = self.ki * self.integral
        
        # Derivative term
        derivative = self.kd * (error - self.previous_error) / dt
        
        # Calculate PID output
        output = proportional + integral + derivative
        
        # Store error for next iteration
        self.previous_error = error
        
        return output
    
    def angacc_update(self, current_value, dt):
        # Calculate error
        error = self.target - current_value
        
        # Proportional term
        proportional = self.kp * error
        
        # Integral term
        self.integral += error * dt
        integral = self.ki * self.integral
        
        # Derivative term
        derivative = self.kd * (error - self.previous_error) / dt
        
        # Calculate PID output
        output = proportional + integral + derivative
        
        # Store error for next iteration
        self.previous_error = error
        
        return output
    
    def set_gyro_target(self, target):
        self.target = target

    def set_acc_target(self, target):
        self.target = target

    def set_angacc_target(self, target):
        self.target = target
        
    def reset(self):
        self.previous_error = 0
        self.integral = 0

def main():
    # Initialize the PCA9685 module for motor control
    pwm = PCA9685(address=0x40, debug=True, bus_num=1)
    pwm.setPWMFreq(50)  # Set frequency to 50 Hz
    
    # Initialize the MPU6050 IMU sensor
    imu = mpu6050.mpu6050(0x68)
    
    motor_channel = 0  # The channel connected to the motor (0-15)
    
    # PID Controller setup
    # These values may need tuning based on your specific system
    
    gyro_kp = 294.0   # Proportional gain
    gyro_ki = 0.08   # Integral gain  0.8
    gyro_kd = 0.0   # Derivative gain
    gyro_target = 0.0  # Target angular velocity (rad/s) 
    
    # PID for acc
    acc_kp = 29.0   # Proportional gain
    acc_ki = 0.000    # Integral gain  
    acc_kd = 0.008   # Derivative gain
    acc_target = 0.0  # Target 

    # PID for angular acceleration
    angacc_kp = 292.0   # Proportional gain
    angacc_ki = 0.000    # Integral gain  
    angacc_kd = 0.008   # Derivative gain
    angacc_target = 0.0  # Target 
    

    gyro_pid = PIDController(gyro_kp, gyro_ki, gyro_kd, gyro_target)
    acc_pid = PIDController(acc_kp, acc_ki, acc_kd, acc_target)
    angacc_pid = PIDController(angacc_kp, angacc_ki, angacc_kd, angacc_target)
    
    # Motor parameters
    base_pulse = 1500  # Neutral pulse width (1500 us for most ESCs)
    min_pulse = 1000   # Minimum pulse width
    max_pulse = 2000   # Maximum pulse width
    
    print("Starting IMU-based motor control with PID...")
    print("Press Ctrl+C to stop")
    
    try:
        # Initialize motor (send neutral signal)
        pwm.setServoPulse(motor_channel_1, base_pulse)
        # pwm.setServoPulse(motor_channel_2, base_pulse)
        # pwm.setServoPulse(motor_channel_3, base_pulse)
        # pwm.setServoPulse(motor_channel_4, base_pulse)
        # pwm.setServoPulse(motor_channel_5, base_pulse)
        # pwm.setServoPulse(motor_channel_6, base_pulse)
        time.sleep(3)
        
        previous_time = time.time()

        parameter = 0
        
        while True:
            if mode:

                #simple_imu_logger(sample_rate=100) 
                current_time = time.time()
                dt = current_time - previous_time
                
                # Read IMU data
                gyro_data = imu.get_gyro_data()
                acc_data = imu.get_accel_data()

                acc_x =  acc_data['x']
                acc_y =  acc_data['y']
                acc_z =  acc_data['z']

                angle = math.atan(acc_y/acc_z) * 180 / math.pi
                
                # Get angular velocity around Z-axis (yaw) in rad/s
                angular_velocity_x = math.radians(gyro_data['x'])
                
                # Calculate PID output

                # if gyro_or_acc:            
                #     pid_output = pid.update(angular_velocity_x, dt)
                # else:
                #     pid_output = pid.update(angle, dt)
                angle_abs = abs(angle)

                if angle_abs < angle_low:
                    pid_output = gyro_pid.gyro_update(angular_velocity_x, dt)
                    #pid_output = pid.update(angle, dt)

                elif angle_abs >= angle_low and angle_abs <= angle_high:
                    parameter = (angle_abs - angle_low)/(angle_high - angle_low)
                    
                    pid_output = (1 - parameter) * gyro_pid.gyro_update(angular_velocity_x, dt) + parameter * acc_pid.acc_update(angle, dt)
                    # pid_output = pid.update(angle, dt)

                elif angle_abs > angle_high:
                    pid_output = acc_pid.acc_update(angle, dt)

                elif angle_abs > angle_capsized:
                    pid_output = acc_pid.acc_update(angle, dt) * 2000

                else:
                    pid_output = gyro_pid.gyro_update(angular_velocity_x, dt)


            else:

                current_time = time.time()
                dt = current_time - previous_time
                
                # Read IMU data
                gyro_data = imu.get_gyro_data()
                acc_data = imu.get_accel_data()

                acc_x =  acc_data['x']
                acc_y =  acc_data['y']
                acc_z =  acc_data['z']

                angle = math.atan(acc_y/acc_z) * 180 / math.pi
                
                # Get angular velocity around Z-axis (yaw) in rad/s
                angular_velocity_x = math.radians(gyro_data['x'])
                # Angular acceleration based PID control
                # Calculate angular acceleration (change in angular velocity over time)
                if 'previous_angular_velocity_x' not in locals():
                    previous_angular_velocity_x = angular_velocity_x
                    angular_acceleration_x = 0.0
                else:
                    angular_acceleration_x = (angular_velocity_x - previous_angular_velocity_x) / dt
                    previous_angular_velocity_x = angular_velocity_x
                
                # Use angular acceleration PID controller
                pid_output = angacc_pid.angacc_update(angular_acceleration_x, dt)

            
            # Convert PID output to motor pulse width
            # Clamp the output to prevent excessive motor speeds
            motor_adjustment = max(min(pid_output, 500), -500)  # Limit to ±500 us
            motor_pulse = int(base_pulse + motor_adjustment)
            
            # Ensure pulse width is within safe limits
            motor_pulse = max(min(motor_pulse, max_pulse), min_pulse)
            
            # Set motor speed
            pwm.setServoPulse(motor_channel_1, motor_pulse)
            # pwm.setServoPulse(motor_channel_2, 3000 - motor_pulse)
            # pwm.setServoPulse(motor_channel_3, motor_pulse)


            # pwm.setServoPulse(motor_channel_4, motor_pulse)
            # pwm.setServoPulse(motor_channel_5, 3000 - motor_pulse)
            # pwm.setServoPulse(motor_channel_6, 3000 - motor_pulse)

            # Print debug information
            print(f"Angular Vel X: {angular_velocity_x:.3f} rad/s, "
                  f"Angle: {angle:.3f} deg, "
                  f"PID Output: {pid_output:.2f}, "
                  f"Motor Pulse: {motor_pulse} us"
                  f"parameter = {parameter}")
            
            previous_time = current_time
            time.sleep(0.01)  # 100 Hz control loop
            
    except KeyboardInterrupt:
        print("\nMotor control interrupted by user.")
        pwm.setServoPulse(motor_channel_1, base_pulse)
        # pwm.setServoPulse(motor_channel_2, base_pulse)
        # pwm.setServoPulse(motor_channel_3, base_pulse)
        # pwm.setServoPulse(motor_channel_4, base_pulse)
        # pwm.setServoPulse(motor_channel_5, base_pulse)
        # pwm.setServoPulse(motor_channel_6, base_pulse)
        time.sleep(0.5)
        pwm.setServoPulse(motor_channel_1, 0)  # Stop the motor
        # pwm.setServoPulse(motor_channel_2, 0)  # Stop the motor
        # pwm.setServoPulse(motor_channel_3, 0)  # Stop the motor
        # pwm.setServoPulse(motor_channel_4, 0)  # Stop the motor
        # pwm.setServoPulse(motor_channel_5, 0)  # Stop the motor
        # pwm.setServoPulse(motor_channel_6, 0)  # Stop the motor

if __name__ == "__main__":
    main()

# Additional utility functions for advanced control

def calibrate_imu(imu, samples=100):
    """Calibrate the IMU by collecting baseline readings"""
    print(f"Calibrating IMU with {samples} samples...")
    
    gyro_offsets = {'x': 0, 'y': 0, 'z': 0}
    
    for i in range(samples):
        gyro_data = imu.get_gyro_data()
        gyro_offsets['x'] += gyro_data['x']
        gyro_offsets['y'] += gyro_data['y'] 
        gyro_offsets['z'] += gyro_data['z']
        time.sleep(0.01)
        
    # Calculate averages
    gyro_offsets['x'] /= samples
    gyro_offsets['y'] /= samples
    gyro_offsets['z'] /= samples
    
    print(f"IMU Calibration complete. Offsets: {gyro_offsets}")
    return gyro_offsets

def stabilized_motor_control():
    """Advanced version with IMU calibration and stabilization"""
    # Initialize hardware
    pwm = PCA9685(address=0x40, debug=True, bus_num=1)
    pwm.setPWMFreq(50)
    imu = mpu6050.mpu6050(0x68)
    
    # Calibrate IMU
    gyro_offsets = calibrate_imu(imu)
    
    # PID setup for stabilization (keeping angular velocity at 0)
    pid_roll = PIDController(kp=30.0, ki=2.0, kd=8.0, target_value=0.0)
    pid_pitch = PIDController(kp=30.0, ki=2.0, kd=8.0, target_value=0.0)
    
    
    base_pulse = 1500
    
    print("Starting stabilized motor control...")
    print("The system will try to maintain stability using IMU feedback")
    print("Press Ctrl+C to stop")
    
    try:
        pwm.setServoPulse(motor_channel_1, base_pulse)
        # pwm.setServoPulse(motor_channel_2, base_pulse)
        # pwm.setServoPulse(motor_channel_3, base_pulse)
        # pwm.setServoPulse(motor_channel_4, base_pulse)
        # pwm.setServoPulse(motor_channel_5, base_pulse)
        # pwm.setServoPulse(motor_channel_6, base_pulse)
        time.sleep(2)
        
        previous_time = time.time()
        
        while True:
            current_time = time.time()
            dt = current_time - previous_time
            
            # Read and correct gyro data
            gyro_data = imu.get_gyro_data()
            corrected_roll = math.radians(gyro_data['x'] - gyro_offsets['x'])
            corrected_pitch = math.radians(gyro_data['y'] - gyro_offsets['y'])
            
            # Calculate PID outputs for stabilization
            roll_output = pid_roll.update(corrected_roll, dt)
            pitch_output = pid_pitch.update(corrected_pitch, dt)
            
            # Combine outputs (you might want to weight these differently)
            combined_output = (roll_output + pitch_output) / 2
            
            # Apply to motor
            motor_adjustment = max(min(combined_output, 400), -400)
            motor_pulse = int(base_pulse + motor_adjustment)
            motor_pulse = max(min(motor_pulse, 2000), 1000)
            
            pwm.setServoPulse(motor_channel, motor_pulse)
            pwm.setServoPulse(1, motor_pulse)
            
            print(f"Roll: {math.degrees(corrected_roll):.2f}°/s, "
                  f"Pitch: {math.degrees(corrected_pitch):.2f}°/s, "
                  f"Motor: {motor_pulse} us")
            
            previous_time = current_time
            time.sleep(0.02)
            
    except KeyboardInterrupt:
        print("\nStabilized control stopped.")
        pwm.setServoPulse(motor_channel_1, base_pulse)
        # pwm.setServoPulse(motor_channel_2, base_pulse)
        # pwm.setServoPulse(motor_channel_3, base_pulse)
        # pwm.setServoPulse(motor_channel_4, base_pulse)
        # pwm.setServoPulse(motor_channel_5, base_pulse)
        # pwm.setServoPulse(motor_channel_6, base_pulse)
        time.sleep(0.5)
        pwm.setServoPulse(motor_channel_1, 0)
        # pwm.setServoPulse(motor_channel_2, 0)
        # pwm.setServoPulse(motor_channel_3, 0)
        # pwm.setServoPulse(motor_channel_4, 0)
        # pwm.setServoPulse(motor_channel_5, 0)
        # pwm.setServoPulse(motor_channel_6, 0)