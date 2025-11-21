#!/usr/bin/env python3

from motor_test import PCA9685
import mpu6050
import time
import math
import sys
from imu_data import simple_imu_logger

# 导入系统辨识模块
try:
    from system_identification import run_system_identification
    SYSTEM_IDENTIFICATION_AVAILABLE = True
except ImportError:
    SYSTEM_IDENTIFICATION_AVAILABLE = False
    print("警告: system_identification模块不可用,系统辨识功能将不可用")

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
    def __init__(self, kp, ki, kd, target_value=0, derivative_filter_coef=0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.target = target_value
        self.derivative_filter_coef = derivative_filter_coef  # 微分滤波系数
        
        self.previous_error = 0
        self.integral = 0
        self.filtered_derivative = 0  # 滤波后的微分项
        
    def update(self, current_value, dt, error_source='gyro'):
        """统一的PID更新函数,带微分滤波"""
        error = self.target - current_value
        
        # 比例项
        proportional = self.kp * error
        
        # 积分项（带抗饱和）
        self.integral += error * dt
        integral = self.ki * self.integral
        
        # 微分项（带滤波）
        derivative_raw = (error - self.previous_error) / dt
        # 一阶低通滤波
        self.filtered_derivative = (self.derivative_filter_coef * derivative_raw + 
                                  (1 - self.derivative_filter_coef) * self.filtered_derivative)
        derivative = self.kd * self.filtered_derivative
        
        # PID输出
        output = proportional + integral + derivative
        
        self.previous_error = error
        return output
    
    # 简化的设置目标函数
    def set_target(self, target):
        self.target = target

def main():
    # 检查是否运行系统辨识模式
    '''if len(sys.argv) > 1 and sys.argv[1] == 'identify':
        if SYSTEM_IDENTIFICATION_AVAILABLE:
            run_system_identification()
        else:
            print("错误: 系统辨识功能不可用,请确保system_identification.py在相同目录下")
        return'''

    # 正常控制模式
    # Initialize the PCA9685 module for motor control
    pwm = PCA9685(address=0x40, debug=True, bus_num=1)
    pwm.setPWMFreq(50)  # Set frequency to 50 Hz
    
    # Initialize the MPU6050 IMU sensor
    imu = mpu6050.mpu6050(0x68)
    
    motor_channel = 0  # The channel connected to the motor (0-15)
    
    # PID Controller setup
    # 这些值可以通过系统辨识来优化
    
    # 改进的PID参数（使用带滤波的PID）
    # 角速度PID - 调整后的参数
    gyro_kp = 294.0
    gyro_ki = 0.08  
    gyro_kd = 2.0  # 增加了适度的微分项
    gyro_filter = 0.2   # 微分滤波系数
    
    # 角度PID
    angle_kp = 29.0
    angle_ki = 0.000
    angle_kd = 1.0  # 调整了微分增益
    angle_filter = 0.15
    
    # 角加速度PID
    angacc_kp = 292.0
    angacc_ki = 0.000  
    angacc_kd = 3.0
    angacc_filter = 0.25  # 更强的滤波，因为角加速度噪声更大

    gyro_pid = PIDController(gyro_kp, gyro_ki, gyro_kd, 0.0, gyro_filter)
    acc_pid = PIDController(angle_kp, angle_ki, angle_kd, 0.0, angle_filter)
    angacc_pid = PIDController(angacc_kp, angacc_ki, angacc_kd, 0.0, angacc_filter)
    
    # Motor parameters
    base_pulse = 1500  # Neutral pulse width (1500 us for most ESCs)
    min_pulse = 1000   # Minimum pulse width
    max_pulse = 2000   # Maximum pulse width
    
    print("Starting IMU-based motor control with PID...")
    print("Press Ctrl+C to stop")
    if SYSTEM_IDENTIFICATION_AVAILABLE:
        print("Run with 'identify' argument for system identification: python motor_control.py identify")
    
    try:
        # Initialize motor (send neutral signal)
        pwm.setServoPulse(motor_channel_1, base_pulse)
        time.sleep(3)
        
        previous_time = time.time()
        previous_angular_velocity_x = 0
        parameter = 0
        
        while True:
            current_time = time.time()
            dt = current_time - previous_time
            if dt <= 0: 
                dt = 0.01  # 防止除零
            
            # Read IMU data
            gyro_data = imu.get_gyro_data()
            acc_data = imu.get_accel_data()

            # 计算角度和角速度
            angle = math.atan2(acc_data['y'], acc_data['z']) * 180 / math.pi
            angular_velocity_x = math.radians(gyro_data['x'])
            
            # 计算角加速度
            angular_acceleration_x = (angular_velocity_x - previous_angular_velocity_x) / dt
            previous_angular_velocity_x = angular_velocity_x
            
            angle_abs = abs(angle)
            
            if mode:
                # 模式1：基于角度的混合控制
                if angle_abs < angle_low:
                    pid_output = gyro_pid.update(angular_velocity_x, dt)
                elif angle_abs <= angle_high:
                    parameter = (angle_abs - angle_low) / (angle_high - angle_low)
                    gyro_output = gyro_pid.update(angular_velocity_x, dt)
                    angle_output = acc_pid.update(angle, dt)
                    pid_output = (1 - parameter) * gyro_output + parameter * angle_output
                elif angle_abs > angle_capsized:
                    # 倾覆保护 - 更强力的恢复
                    pid_output = acc_pid.update(angle, dt) * 1.5  # 适度增强，避免过度
                else:
                    pid_output = acc_pid.update(angle, dt)
            else:
                # 模式2：基于角加速度的控制
                pid_output = angacc_pid.update(angular_acceleration_x, dt)
            
            # Convert PID output to motor pulse width
            # Clamp the output to prevent excessive motor speeds
            motor_adjustment = max(min(pid_output, 500), -500)  # Limit to ±500 us
            motor_pulse = int(base_pulse + motor_adjustment)
            
            # Ensure pulse width is within safe limits
            motor_pulse = max(min(motor_pulse, max_pulse), min_pulse)
            
            # Set motor speed
            pwm.setServoPulse(motor_channel_1, motor_pulse)

            # Print debug information
            print(f"Angle: {angle:6.2f}°, "
                  f"AngVel: {angular_velocity_x:6.3f} rad/s, "
                  f"AngAcc: {angular_acceleration_x:6.3f} rad/s², "
                  f"PID Out: {pid_output:7.2f}, "
                  f"Motor Pulse: {motor_pulse} us")
            
            previous_time = current_time
            time.sleep(0.01)  # 100 Hz control loop
            
    except KeyboardInterrupt:
        print("\nMotor control interrupted by user.")
        pwm.setServoPulse(motor_channel_1, base_pulse)
        time.sleep(0.5)
        pwm.setServoPulse(motor_channel_1, 0)  # Stop the motor

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