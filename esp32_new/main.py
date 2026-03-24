#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESP32 MicroPython - 前馈+双IMU融合 电机控制
基于树莓派的 feedforward_dual_imu.py 改写
完整兼容 MicroPython 环境
"""

import time
import math
import json
from machine import I2C, Pin, PWM, Timer, UART
from collections import deque
import struct
import micropython

# ==================== 项目导入 ====================
try:
    from lib.mpu6050 import MPU6050
    from lib.pca9685 import PCA9685
except ImportError:
    print("Warning: 驱动库未找到，请检查 /lib 文件夹")
    MPU6050 = None
    PCA9685 = None

# ==================== 配置导入 ====================
try:
    from config import *
except ImportError:
    print("Warning: 配置文件未找到，使用默认配置")
    # 默认配置
    I2C_SCL_PIN = 22
    I2C_SDA_PIN = 21
    I2C_FREQ = 400000
    PID_KP = 20.0
    PID_KI = 1.0
    PID_KD = 0.0
    ENABLE_DATA_LOGGING = True

# ==================== 配置常数 ====================
LEFT_THRUSTER = 0
RIGHT_THRUSTER = 1

BASE_PULSE = 1500
MIN_PULSE = 1000
MAX_PULSE = 2000
THRUST_SCALE = 0.55

ALPHA_ACC = 0.98
ALPHA_EMA = 0.15
DT = 0.01

ANGLE_DEADZONE = 1.0
ANGLE_DEADZONE_SOFT = 3.0
FEEDFORWARD_PARAM = 0.28
FEEDBACK_PARAM = 0.5
MASS = 80.0
WIDTH = 0.6
G = 9.81


def apply_deadzone_smooth(value, deadzone_core, deadzone_soft):
    """非线性死区插值函数"""
    abs_val = abs(value)
    
    if abs_val < deadzone_core:
        return 0.0
    elif abs_val >= deadzone_soft:
        return value
    else:
        t = (abs_val - deadzone_core) / (deadzone_soft - deadzone_core)
        smooth_factor = t * t * (3 - 2 * t)
        return (1 if value > 0 else -1) * abs_val * smooth_factor


class PID2DOF:
    """2自由度PID控制器"""
    
    def __init__(self, Kp, Ki, Kd, b=1.0, c=0.0, dt=DT):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.b = b
        self.c = c
        self.dt = dt
        self.integral = 0.0

    def update_gains(self, Kp, Ki, Kd):
        """动态更新PID增益"""
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def compute(self, setpoint, measured, omega_filtered):
        """计算PID输出"""
        error = setpoint - measured
        
        proportional = self.Kp * (self.b * setpoint - measured)
        self.integral += error * self.dt
        integral_term = self.Ki * self.integral
        derivative = self.Kd * (self.c * 0 - omega_filtered)
        
        output = proportional + integral_term + derivative
        return output

    def reset(self):
        """重置积分器"""
        self.integral = 0.0


class DualIMUFusion:
    """双IMU融合算法"""
    
    def __init__(self):
        self.q0, self.q1, self.q2, self.q3 = 1.0, 0.0, 0.0, 0.0
        self.bias_gx = [0.0, 0.0]
        self.bias_gy = [0.0, 0.0]
        self.bias_gz = [0.0, 0.0]

    def set_gyro_bias(self, bias_g1, bias_g2):
        """设置初始的陀螺仪零偏"""
        self.bias_gx = [bias_g1.get('x', 0), bias_g2.get('x', 0)]
        self.bias_gy = [bias_g1.get('y', 0), bias_g2.get('y', 0)]
        self.bias_gz = [bias_g1.get('z', 0), bias_g2.get('z', 0)]
        print("陀螺仪零偏已设置")

    def update(self, ax1, ay1, az1, gx1, gy1, gz1, ax2, ay2, az2, gx2, gy2, gz2, dt):
        """双IMU融合更新"""
        # 加速度互补
        acc1_mag = math.sqrt(ax1**2 + ay1**2 + az1**2)
        acc2_mag = math.sqrt(ax2**2 + ay2**2 + az2**2)
        
        if acc1_mag > 0.1:
            ax1, ay1, az1 = ax1/acc1_mag, ay1/acc1_mag, az1/acc1_mag
        if acc2_mag > 0.1:
            ax2, ay2, az2 = ax2/acc2_mag, ay2/acc2_mag, az2/acc2_mag
            
        ax = ALPHA_ACC * ax1 + (1-ALPHA_ACC) * ax2
        ay = ALPHA_ACC * ay1 + (1-ALPHA_ACC) * ay2
        az = ALPHA_ACC * az1 + (1-ALPHA_ACC) * az2
        
        # 陀螺仪零偏修正
        gx1_corrected = gx1 - self.bias_gx[0]
        gy1_corrected = gy1 - self.bias_gy[0]
        gz1_corrected = gz1 - self.bias_gz[0]
        
        gx2_corrected = gx2 - self.bias_gx[1]
        gy2_corrected = gy2 - self.bias_gy[1]
        gz2_corrected = gz2 - self.bias_gz[1]
        
        # 平均角速度
        gx = (gx1_corrected + gx2_corrected) / 2
        gy = (gy1_corrected + gy2_corrected) / 2
        gz = (gz1_corrected + gz2_corrected) / 2
        
        # 四元数更新
        self._update_quaternion(gx, gy, gz, dt)
        self._correct_quaternion_with_accel(ax, ay, az)
        
        # 返回欧拉角
        roll, pitch, yaw = self._quaternion_to_euler()
        return roll, pitch, yaw

    def _update_quaternion(self, gx, gy, gz, dt):
        """四元数积分"""
        gx_rad = math.radians(gx)
        gy_rad = math.radians(gy)
        gz_rad = math.radians(gz)
        
        dq0 = 0.5 * (-self.q1 * gx_rad - self.q2 * gy_rad - self.q3 * gz_rad) * dt
        dq1 = 0.5 * (self.q0 * gx_rad + self.q2 * gz_rad - self.q3 * gy_rad) * dt
        dq2 = 0.5 * (self.q0 * gy_rad - self.q1 * gz_rad + self.q3 * gx_rad) * dt
        dq3 = 0.5 * (self.q0 * gz_rad + self.q1 * gy_rad - self.q2 * gx_rad) * dt
        
        self.q0 += dq0
        self.q1 += dq1
        self.q2 += dq2
        self.q3 += dq3
        
        norm = math.sqrt(self.q0**2 + self.q1**2 + self.q2**2 + self.q3**2)
        if norm > 0:
            self.q0 /= norm
            self.q1 /= norm
            self.q2 /= norm
            self.q3 /= norm

    def _correct_quaternion_with_accel(self, ax, ay, az):
        """使用加速度修正四元数"""
        gx_acc = math.atan2(ay, math.sqrt(ax**2 + az**2))
        gy_acc = -math.atan2(ax, math.sqrt(ay**2 + az**2))
        
        alpha = 0.01
        roll, pitch, _ = self._quaternion_to_euler()
        roll = roll * (1 - alpha) + gx_acc * alpha
        pitch = pitch * (1 - alpha) + gy_acc * alpha
        
        self._euler_to_quaternion(roll, pitch, 0)

    def _quaternion_to_euler(self):
        """四元数转欧拉角"""
        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3
        
        sinr_cosp = 2 * (q0 * q1 + q2 * q3)
        cosr_cosp = 1 - 2 * (q1**2 + q2**2)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2 * (q0 * q2 - q3 * q1)
        if abs(sinp) >= 1:
            pitch = (math.pi / 2) if sinp > 0 else (-math.pi / 2)
        else:
            pitch = math.asin(sinp)
        
        siny_cosp = 2 * (q0 * q3 + q1 * q2)
        cosy_cosp = 1 - 2 * (q2**2 + q3**2)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

    def _euler_to_quaternion(self, roll, pitch, yaw):
        """欧拉角转四元数"""
        roll_rad = math.radians(roll)
        pitch_rad = math.radians(pitch)
        yaw_rad = math.radians(yaw)
        
        cy = math.cos(yaw_rad * 0.5)
        sy = math.sin(yaw_rad * 0.5)
        cp = math.cos(pitch_rad * 0.5)
        sp = math.sin(pitch_rad * 0.5)
        cr = math.cos(roll_rad * 0.5)
        sr = math.sin(roll_rad * 0.5)
        
        self.q0 = cy * cp * cr + sy * sp * sr
        self.q1 = cy * cp * sr - sy * sp * cr
        self.q2 = sy * cp * sr + cy * sp * cr
        self.q3 = sy * cp * cr - cy * sp * sr


class SafeMotorController:
    """安全电机控制器"""
    
    def __init__(self, pwm):
        self.pwm = pwm
        self.error_count = 0
        self.total_errors = 0
        self.last_pulse = {LEFT_THRUSTER: BASE_PULSE, RIGHT_THRUSTER: BASE_PULSE}
    
    def set_pulse(self, channel, pulse):
        """设置PWM脉宽"""
        try:
            self.pwm.setServoPulse(channel, pulse)
            self.last_pulse[channel] = pulse
            self.error_count = 0
            return True
        except Exception as e:
            self.error_count += 1
            self.total_errors += 1
            print(f"电机设置失败: {e}")
            return False
    
    def set_both_motors(self, pulse_left, pulse_right, invert_left=True):
        """同时设置两个电机"""
        if invert_left:
            pulse_left_actual = 3000 - pulse_left
        else:
            pulse_left_actual = pulse_left
        
        success_left = self.set_pulse(LEFT_THRUSTER, pulse_left_actual)
        success_right = self.set_pulse(RIGHT_THRUSTER, pulse_right)
        return success_left and success_right
    
    def emergency_stop(self):
        """紧急停止"""
        print("执行紧急停止...")
        try:
            self.pwm.setServoPulse(LEFT_THRUSTER, BASE_PULSE)
            self.pwm.setServoPulse(RIGHT_THRUSTER, BASE_PULSE)
            time.sleep(0.1)
            self.pwm.setServoPulse(LEFT_THRUSTER, 0)
            self.pwm.setServoPulse(RIGHT_THRUSTER, 0)
            print("电机已停止")
            return True
        except Exception as e:
            print(f"停止失败: {e}")
            return False


class DataLogger:
    """数据记录器 (缓冲写入)"""
    
    def __init__(self):
        self.enabled = False
        self.file = None
        self.buffer = []
        self.buffer_size = 100
        self.sample_count = 0
        
    def start_logging(self, prefix="feedforward_esp32"):
        """开始记录数据"""
        timestamp = int(time.time())
        filename = f"{prefix}_{timestamp}.csv"
        
        try:
            self.file = open(filename, 'w')
            headers = ['timestamp', 'roll_deg', 'pitch_deg', 'yaw_deg',
                      'omega_raw', 'omega_filtered', 'tau_ff', 'tau_pid',
                      'tau_total', 'pwm_left', 'pwm_right']
            self.file.write(','.join(headers) + '\n')
            
            self.enabled = True
            self.buffer = []
            print(f"开始记录数据: {filename}")
        except Exception as e:
            print(f"启动记录失败: {e}")
            self.enabled = False
            
    def stop_logging(self):
        """停止记录数据"""
        if self.enabled and self.file:
            self._flush_to_disk()
            self.file.close()
            self.enabled = False
            print(f"记录完成，共 {self.sample_count} 样本")
    
    def log_data(self, **kwargs):
        """记录一帧数据"""
        if not self.enabled:
            return
            
        row = [f"{kwargs.get(k, 0):.4f}" if isinstance(kwargs.get(k, 0), float) 
               else str(kwargs.get(k, 0)) for k in
               ['timestamp', 'roll', 'pitch', 'yaw', 'omega_raw', 
                'omega_filtered', 'tau_ff', 'tau_pid', 'tau_total', 
                'pwm_left', 'pwm_right']]
        
        self.buffer.append(','.join(row) + '\n')
        self.sample_count += 1
        
        if len(self.buffer) >= self.buffer_size:
            self._flush_to_disk()
    
    def _flush_to_disk(self):
        """写入缓冲区到磁盘"""
        if self.file and self.buffer:
            try:
                for line in self.buffer:
                    self.file.write(line)
                self.file.flush()
                self.buffer = []
            except Exception as e:
                print(f"写入失败: {e}")


class FeedforwardDualIMUController:
    """前馈+双IMU融合 控制器"""
    
    def __init__(self, i2c):
        self.i2c = i2c
        self.running = False
        
        # 初始化硬件
        try:
            self.imu1 = MPU6050(i2c, address=0x68)
            self.imu2 = MPU6050(i2c, address=0x69)
            print("✓ IMU 初始化成功")
        except Exception as e:
            print(f"✗ IMU 初始化失败: {e}")
            self.imu1 = None
            self.imu2 = None
        
        try:
            self.pwm = PCA9685(i2c, address=0x40)
            self.pwm.setPWMFreq(50)
            print("✓ PCA9685 初始化成功")
        except Exception as e:
            print(f"✗ PCA9685 初始化失败: {e}")
            self.pwm = None
        
        # 初始化控制器
        self.motor_controller = SafeMotorController(self.pwm) if self.pwm else None
        self.pid = PID2DOF(PID_KP, PID_KI, PID_KD)
        self.fusion = DualIMUFusion()
        self.data_logger = DataLogger()
        self.omega_filtered = 0.0
        
        # 加载校准数据
        self._load_calibration()
    
    def _load_calibration(self):
        """加载校准参数"""
        try:
            with open('calibration_imu1.json') as f:
                cal1 = json.load(f)
            with open('calibration_imu2.json') as f:
                cal2 = json.load(f)
            
            bias1 = cal1.get('gyro_bias', {'x': 0, 'y': 0, 'z': 0})
            bias2 = cal2.get('gyro_bias', {'x': 0, 'y': 0, 'z': 0})
            self.fusion.set_gyro_bias(bias1, bias2)
            print("✓ 校准参数已加载")
        except Exception as e:
            print(f"⚠ 加载校准失败: {e}")
    
    def calibrate_imu(self):
        """IMU 校准"""
        if not self.imu1 or not self.imu2:
            print("✗ IMU 未初始化")
            return False
        
        print("开始校准 (200 样本)...")
        gx1_sum, gy1_sum, gz1_sum = 0, 0, 0
        gx2_sum, gy2_sum, gz2_sum = 0, 0, 0
        
        for i in range(200):
            try:
                gyro1 = self.imu1.get_gyro()
                gyro2 = self.imu2.get_gyro()
                
                gx1_sum += gyro1.get('x', 0)
                gy1_sum += gyro1.get('y', 0)
                gz1_sum += gyro1.get('z', 0)
                gx2_sum += gyro2.get('x', 0)
                gy2_sum += gyro2.get('y', 0)
                gz2_sum += gyro2.get('z', 0)
                
                time.sleep_ms(10)
            except Exception as e:
                print(f"采样失败: {e}")
        
        bias1 = {'x': gx1_sum/200, 'y': gy1_sum/200, 'z': gz1_sum/200}
        bias2 = {'x': gx2_sum/200, 'y': gy2_sum/200, 'z': gz2_sum/200}
        
        self.fusion.set_gyro_bias(bias1, bias2)
        
        try:
            with open('calibration_imu1.json', 'w') as f:
                json.dump({'gyro_bias': bias1}, f)
            with open('calibration_imu2.json', 'w') as f:
                json.dump({'gyro_bias': bias2}, f)
            print("✓ 校准完成")
            return True
        except Exception as e:
            print(f"✗ 保存校准失败: {e}")
            return False
    
    def read_imu(self):
        """读取 IMU 数据"""
        if not self.imu1 or not self.imu2:
            return None
        
        try:
            return {
                'accel1': self.imu1.get_accel(), 'gyro1': self.imu1.get_gyro(),
                'accel2': self.imu2.get_accel(), 'gyro2': self.imu2.get_gyro()
            }
        except Exception as e:
            print(f"IMU 读取失败: {e}")
            return None
    
    def control_step(self, imu_data):
        """单步控制"""
        if not imu_data:
            return None
        
        # 解包数据
        accel1, gyro1 = imu_data['accel1'], imu_data['gyro1']
        accel2, gyro2 = imu_data['accel2'], imu_data['gyro2']
        
        # IMU 融合
        roll, pitch, yaw = self.fusion.update(
            accel1['x'], accel1['y'], accel1['z'],
            gyro1['x'], gyro1['y'], gyro1['z'],
            accel2['x'], accel2['y'], accel2['z'],
            gyro2['x'], gyro2['y'], gyro2['z'],
            DT
        )
        
        # 角速度和滤波
        omega_raw = (gyro1['y'] + gyro2['y']) / 2
        self.omega_filtered = ALPHA_EMA * omega_raw + (1 - ALPHA_EMA) * self.omega_filtered
        
        # 控制
        tau_ff = -FEEDFORWARD_PARAM * MASS * G * WIDTH * math.sin(math.radians(roll)) / 2
        tau_pid = self.pid.compute(0.0, roll, self.omega_filtered)
        tau_total = tau_ff + FEEDBACK_PARAM * tau_pid
        
        # 死区
        if abs(roll) < ANGLE_DEADZONE:
            tau_total = 0.0
        elif abs(roll) < ANGLE_DEADZONE_SOFT:
            t = (abs(roll) - ANGLE_DEADZONE) / (ANGLE_DEADZONE_SOFT - ANGLE_DEADZONE)
            tau_total *= t * t * (3 - 2 * t)
        
        # PWM
        pwm_delta = tau_total * THRUST_SCALE
        pwm_left = max(MIN_PULSE, min(MAX_PULSE, int(BASE_PULSE + pwm_delta)))
        pwm_right = max(MIN_PULSE, min(MAX_PULSE, int(BASE_PULSE - pwm_delta)))
        
        # 设置电机
        if self.motor_controller:
            self.motor_controller.set_both_motors(pwm_left, pwm_right)
        
        # 记录
        self.data_logger.log_data(
            timestamp=time.time(),
            roll=roll, pitch=pitch, yaw=yaw,
            omega_raw=omega_raw, omega_filtered=self.omega_filtered,
            tau_ff=tau_ff, tau_pid=tau_pid, tau_total=tau_total,
            pwm_left=pwm_left, pwm_right=pwm_right
        )
        
        return {'roll': roll, 'pwm_left': pwm_left, 'pwm_right': pwm_right}
    
    def start(self):
        """启动"""
        self.running = True
        if ENABLE_DATA_LOGGING:
            self.data_logger.start_logging()
        print("✓ 控制器已启动")
    
    def stop(self):
        """停止"""
        self.running = False
        if self.motor_controller:
            self.motor_controller.emergency_stop()
        self.data_logger.stop_logging()
        print("✓ 控制器已停止")
    
    def run(self, frequency=100):
        """主循环"""
        self.start()
        loop_time = 1.0 / frequency
        
        try:
            while self.running:
                start_time = time.time()
                
                imu_data = self.read_imu()
                if imu_data:
                    result = self.control_step(imu_data)
                    if result:
                        print(f"Roll: {result['roll']:6.2f}° | PWM: {result['pwm_left']:4d} {result['pwm_right']:4d}")
                
                elapsed = time.time() - start_time
                if elapsed < loop_time:
                    time.sleep(loop_time - elapsed)
                    
        except KeyboardInterrupt:
            print("✓ 控制循环被中断")
        finally:
            self.stop()


def main():
    """主程序"""
    print("\n" + "="*50)
    print("ESP32 MicroPython - 前馈双IMU控制器")
    print("="*50 + "\n")
    
    # 初始化 I2C
    print(f"初始化 I2C...")
    i2c = I2C(1, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=I2C_FREQ)
    
    # 创建控制器
    print("初始化控制器...")
    controller = FeedforwardDualIMUController(i2c)
    
    # 校准
    print("\n开始 IMU 校准...")
    controller.calibrate_imu()
    
    # 运行
    print("\n启动控制循环 (100Hz)...")
    controller.run(frequency=100)


if __name__ == '__main__':
    main()
