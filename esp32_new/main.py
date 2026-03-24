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


class ThrusterRotationController:
    """推进器旋转控制器 - 阶段1.1
    
    功能：根据平衡需求转矩计算推进器旋转角
    当前模式：旋转功能框架，推进器暂时保持竖直（θ=0）
    
    硬件假设：
    - 推进器通过舵机旋转，使用 PCA9685 独立通道
    - LEFT_ROTATION_SERVO / RIGHT_ROTATION_SERVO 通道
    - 旋转范围：±45° （脉宽 1000-2000μs 对应 -45 到 +45°）
    """
    
    def __init__(self, pwm, enabled=False):
        """初始化旋转控制器
        
        Args:
            pwm: PCA9685 对象
            enabled: 是否启用旋转功能（默认关闭，用于向后兼容）
        """
        self.pwm = pwm
        self.enabled = enabled
        
        # 推进器配置参数
        self.max_rotation_angle = 45.0      # 最大旋转角度 (度)
        self.rotation_pulse_min = 1000      # 对应 -45°
        self.rotation_pulse_max = 2000      # 对应 +45°
        self.rotation_pulse_center = 1500   # 对应 0° (竖直)
        
        # 旋转舵机通道 (PCA9685 通道)
        # 假设 0-1 是推力ESC, 2-3 是旋转舵机
        self.left_rotation_channel = 2
        self.right_rotation_channel = 3
        
        # 船舶物理参数
        self.thruster_distance = WIDTH      # 推进器间距 (m)
        self.max_thrust_force = 50.0        # 最大推力 (N) - 需要根据实际标定
        
        # 状态存储
        self.last_rotation_left = 0.0
        self.last_rotation_right = 0.0
        self.rotation_rate_limit = 90.0     # 旋转速率限制 (deg/s) - 防止过快驱动
        self.last_update_time = time.time()
        
        if self.enabled:
            print("✓ 推进器旋转控制器已启用（框架模式）")
        else:
            print("⚠ 推进器旋转控制器已禁用（保持向后兼容）")
    
    def torque_to_angle(self, tau_total):
        """[阶段1.2] 根据所需转矩计算旋转角
        
        物理模型：τ = F × r × sin(θ)
        其中 F 为推力，r 为推进器间距，θ 为旋转角
        
        当前阶段：仅返回框架，不实际使用
        """
        if not self.enabled:
            return 0.0
        
        # 简化假设：使用最大推力计算理论旋转角
        # 实际应用时需要根据当前推力 F_actual 动态计算
        if abs(tau_total) < 1e-6:
            return 0.0
        
        # τ = F × (r/2) × sin(θ) × 2 (两个推进器)
        # θ = arcsin(τ / (F × r))
        try:
            sin_theta = tau_total / (self.max_thrust_force * self.thruster_distance)
            # 限制在 [-1, 1] 范围内
            sin_theta = max(-1.0, min(1.0, sin_theta))
            angle_rad = math.asin(sin_theta)
            angle_deg = math.degrees(angle_rad)
            
            # 限制在最大旋转角范围
            angle_deg = max(-self.max_rotation_angle, min(self.max_rotation_angle, angle_deg))
            return angle_deg
        except:
            return 0.0
    
    def angle_to_pulse(self, angle_deg):
        """将旋转角 (度) 转换为 PWM 脉宽 (μs)
        
        线性映射：-45° ↔ 1000μs, 0° ↔ 1500μs, +45° ↔ 2000μs
        """
        # 限制角度范围
        angle_deg = max(-self.max_rotation_angle, min(self.max_rotation_angle, angle_deg))
        
        # 线性插值
        pulse = self.rotation_pulse_center + (angle_deg / self.max_rotation_angle) * 500
        return int(pulse)
    
    def set_rotation_angle(self, angle_left, angle_right):
        """设置推进器旋转角度（度）
        
        Args:
            angle_left: 左推进器旋转角 (-45 ~ +45°)
            angle_right: 右推进器旋转角 (-45 ~ +45°)
        
        Returns:
            success: 是否成功设置
        """
        if not self.enabled or not self.pwm:
            return False
        
        try:
            # 应用旋转速率限制 (防止舵机突变)
            current_time = time.time()
            dt = current_time - self.last_update_time
            if dt > 0:
                max_delta = self.rotation_rate_limit * dt
                angle_left = max(
                    self.last_rotation_left - max_delta,
                    min(self.last_rotation_left + max_delta, angle_left)
                )
                angle_right = max(
                    self.last_rotation_right - max_delta,
                    min(self.last_rotation_right + max_delta, angle_right)
                )
            
            # 转换为 PWM 脉宽
            pulse_left = self.angle_to_pulse(angle_left)
            pulse_right = self.angle_to_pulse(angle_right)
            
            # 设置 PWM
            self.pwm.setServoPulse(self.left_rotation_channel, pulse_left)
            self.pwm.setServoPulse(self.right_rotation_channel, pulse_right)
            
            # 更新状态
            self.last_rotation_left = angle_left
            self.last_rotation_right = angle_right
            self.last_update_time = current_time
            
            return True
        except Exception as e:
            print(f"设置旋转角失败: {e}")
            return False
    
    def set_rotation_pulse(self, pulse_left, pulse_right):
        """直接设置 PWM 脉宽（用于调试）
        
        Args:
            pulse_left: 左舵机脉宽 (1000-2000μs)
            pulse_right: 右舵机脉宽 (1000-2000μs)
        """
        if not self.enabled or not self.pwm:
            return False
        
        try:
            pulse_left = max(self.rotation_pulse_min, min(self.rotation_pulse_max, pulse_left))
            pulse_right = max(self.rotation_pulse_min, min(self.rotation_pulse_max, pulse_right))
            
            self.pwm.setServoPulse(self.left_rotation_channel, pulse_left)
            self.pwm.setServoPulse(self.right_rotation_channel, pulse_right)
            return True
        except Exception as e:
            print(f"设置旋转脉宽失败: {e}")
            return False
    
    def neutral_position(self):
        """回到中立位置（竖直）"""
        return self.set_rotation_angle(0.0, 0.0)


class PropulsionLayer:
    """推进层控制器 - 阶段1.3
    
    功能：在保持平衡的前提下实现推进
    策略：优先保证平衡 > 利用剩余能力推进
    """
    
    def __init__(self, enabled=False):
        """初始化推进层
        
        Args:
            enabled: 是否启用推进功能
        """
        self.enabled = enabled
        
        # 推进参数
        self.propulsion_mode = "disabled"      # disabled, forward, backward, custom
        self.speed_target = 0.0                # 目标速度 (0.0 ~ 1.0)
        self.direction_target = 0.0            # 目标方向 (度)
        self.max_speed = 0.5                   # 最大速度系数
        
        # 约束参数
        self.balance_priority = 0.7            # 平衡优先级 (0.0=全推进, 1.0=全平衡)
        self.max_roll_allowed = 3.0            # 推进时允许的最大翻滚角
        self.min_thrust_required = 10.0        # 推进所需最小推力
        
        # 状态
        self.current_speed = 0.0
        self.current_direction = 0.0
        self.available_force = 0.0             # 可用于推进的剩余力
        
        if self.enabled:
            print("✓ 推进层已启用")
        else:
            print("⚠ 推进层已禁用")
    
    def set_propulsion_target(self, speed=0.0, direction=0.0, mode="custom"):
        """设置推进目标
        
        Args:
            speed: 目标速度 (0.0 ~ 1.0)
            direction: 目标方向 (度)
            mode: 推进模式
        """
        if not self.enabled:
            return False
        
        self.speed_target = max(0.0, min(1.0, speed))
        self.direction_target = direction % 360.0
        self.propulsion_mode = mode
        
        return True
    
    def calculate_force_allocation(self, tau_balance, roll_angle, current_thrust_left, current_thrust_right):
        """计算力分配 - 在平衡和推进之间找到最优平衡
        
        Args:
            tau_balance: 平衡所需转矩 (N·m)
            roll_angle: 当前翻滚角 (度)
            current_thrust_left: 当前左推力 (N)
            current_thrust_right: 当前右推力 (N)
        
        Returns:
            force_alloc: {'rotation_left': angle, 'rotation_right': angle, 
                         'thrust_left': force, 'thrust_right': force}
        """
        if not self.enabled or self.propulsion_mode == "disabled":
            return None
        
        # 检查平衡约束
        if abs(roll_angle) > self.max_roll_allowed:
            # 翻滚角过大，停止推进
            self.available_force = 0.0
            return None
        
        # 计算可用于推进的能力
        # 根据翻滚角动态调整优先级
        roll_ratio = abs(roll_angle) / self.max_roll_allowed
        adjusted_priority = self.balance_priority + roll_ratio * 0.2
        
        self.available_force = (1.0 - adjusted_priority) * (current_thrust_left + current_thrust_right) / 2
        
        # 根据推进模式分配力
        if self.propulsion_mode == "forward":
            # 前进：两推进器对称
            propulsion_thrust = self.available_force * self.speed_target
            return {
                'rotation_left': 0.0,
                'rotation_right': 0.0,
                'thrust_delta': propulsion_thrust
            }
        
        elif self.propulsion_mode == "backward":
            # 后退：两推进器对称
            propulsion_thrust = -self.available_force * self.speed_target
            return {
                'rotation_left': 0.0,
                'rotation_right': 0.0,
                'thrust_delta': propulsion_thrust
            }
        
        elif self.propulsion_mode == "custom":
            # 自定义方向：需要旋转推进器
            propulsion_thrust = self.available_force * self.speed_target
            rotation_angle = self.direction_target / 90.0 * 45.0  # 映射到旋转角范围
            
            return {
                'rotation_left': rotation_angle,
                'rotation_right': rotation_angle,
                'thrust_delta': propulsion_thrust
            }
        
        return None
    
    def disable_propulsion(self):
        """禁用推进"""
        self.propulsion_mode = "disabled"
        self.speed_target = 0.0
        self.current_speed = 0.0


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
        # 初始化推进器旋转控制器 (阶段1.2 - enabled 由配置决定)
        self.rotation_controller = ThrusterRotationController(self.pwm, enabled=ROTATION_ENABLED) if self.pwm else None
        # 初始化推进层 (阶段1.3 - enabled 由配置决定)
        self.propulsion_layer = PropulsionLayer(enabled=PROPULSION_ENABLED)
        self.pid = PID2DOF(PID_KP, PID_KI, PID_KD)
        self.fusion = DualIMUFusion()
        self.data_logger = DataLogger()
        self.omega_filtered = 0.0
        
        # 状态跟踪
        self.current_thrust_left = 0.0
        self.current_thrust_right = 0.0
        
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
        """单步控制 - 集成平衡、旋转、推进"""
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
        
        # PWM - 基础平衡控制
        pwm_delta = tau_total * THRUST_SCALE
        pwm_left = max(MIN_PULSE, min(MAX_PULSE, int(BASE_PULSE + pwm_delta)))
        pwm_right = max(MIN_PULSE, min(MAX_PULSE, int(BASE_PULSE - pwm_delta)))
        
        # 转换 PWM 为推力 (估计值)
        # 简化模型: 推力 ∝ (脉宽 - BASE_PULSE)
        thrust_left = (pwm_left - BASE_PULSE) / THRUST_SCALE
        thrust_right = (pwm_right - BASE_PULSE) / THRUST_SCALE
        self.current_thrust_left = thrust_left
        self.current_thrust_right = thrust_right
        
        # 初始化旋转和推进参数
        rotation_left = 0.0
        rotation_right = 0.0
        propulsion_thrust_left = 0.0
        propulsion_thrust_right = 0.0
        propulsion_applied = False
        
        # 阶段1.3: 推进层处理
        if self.propulsion_layer and self.propulsion_layer.enabled:
            force_alloc = self.propulsion_layer.calculate_force_allocation(
                tau_total, roll, self.current_thrust_left, self.current_thrust_right
            )
            
            if force_alloc:
                propulsion_applied = True
                rotation_left = force_alloc.get('rotation_left', 0.0)
                rotation_right = force_alloc.get('rotation_right', 0.0)
                
                # 添加推进推力
                thrust_delta = force_alloc.get('thrust_delta', 0.0)
                propulsion_thrust_left = thrust_delta
                propulsion_thrust_right = thrust_delta
                
                # 计算最终 PWM (基础 + 推进)
                pwm_left = max(MIN_PULSE, min(MAX_PULSE, int(pwm_left + propulsion_thrust_left * THRUST_SCALE)))
                pwm_right = max(MIN_PULSE, min(MAX_PULSE, int(pwm_right + propulsion_thrust_right * THRUST_SCALE)))
        
        # 阶段1.2: 推进器旋转控制 (如果没有推进层分配旋转角)
        if not propulsion_applied and self.rotation_controller and self.rotation_controller.enabled:
            # 根据转矩计算旋转角（对称旋转）
            rotation_angle = self.rotation_controller.torque_to_angle(tau_total)
            rotation_left = rotation_angle
            rotation_right = rotation_angle
        
        # 设置电机 - 推力
        if self.motor_controller:
            self.motor_controller.set_both_motors(pwm_left, pwm_right)
        
        # 设置推进器旋转角 (旋转舵机)
        if self.rotation_controller:
            self.rotation_controller.set_rotation_angle(rotation_left, rotation_right)
        
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
        # 将旋转舵机回到中立位置
        if self.rotation_controller:
            self.rotation_controller.neutral_position()
        self.data_logger.stop_logging()
        print("✓ 控制器已停止")
    
    def enable_rotation_control(self, enable=True):
        """启用/禁用推进器旋转控制 (用于阶段1.2)
        
        Args:
            enable: True 启用旋转, False 禁用（保持竖直）
        """
        if self.rotation_controller:
            self.rotation_controller.enabled = enable
            status = "已启用" if enable else "已禁用"
            print(f"✓ 推进器旋转控制{status}")
            if enable:
                self.rotation_controller.neutral_position()
        else:
            print("✗ 旋转控制器未初始化")
    
    def set_rotation_angle(self, angle_left, angle_right):
        """手动设置推进器旋转角 (用于调试)
        
        Args:
            angle_left: 左推进器旋转角 (-45 ~ +45°)
            angle_right: 右推进器旋转角 (-45 ~ +45°)
        """
        if self.rotation_controller:
            return self.rotation_controller.set_rotation_angle(angle_left, angle_right)
        return False
    
    def enable_propulsion_control(self, enable=True):
        """启用/禁用推进功能 (用于阶段1.3)
        
        Args:
            enable: True 启用推进, False 禁用
        """
        if self.propulsion_layer:
            self.propulsion_layer.enabled = enable
            status = "已启用" if enable else "已禁用"
            print(f"✓ 推进功能{status}")
            if not enable:
                self.propulsion_layer.disable_propulsion()
        else:
            print("✗ 推进层未初始化")
    
    def set_propulsion_target(self, speed=0.0, direction=0.0, mode="custom"):
        """设置推进目标
        
        Args:
            speed: 目标速度 (0.0 ~ 1.0)
            direction: 目标方向 (度, 0=前进)
            mode: 推进模式 (forward, backward, custom)
        """
        if self.propulsion_layer:
            return self.propulsion_layer.set_propulsion_target(speed, direction, mode)
        return False
    
    def set_propulsion_mode(self, mode="disabled"):
        """设置推进模式
        
        Args:
            mode: disabled, forward, backward, custom
        """
        if self.propulsion_layer:
            if mode in ["disabled", "forward", "backward", "custom"]:
                self.propulsion_layer.propulsion_mode = mode
                print(f"✓ 推进模式设置为: {mode}")
                return True
            else:
                print(f"✗ 无效的推进模式: {mode}")
                return False
        return False
    
    def set_propulsion_priority(self, balance_priority=0.7):
        """设置平衡 vs 推进的优先级
        
        Args:
            balance_priority: 0.0=全推进, 1.0=全平衡 (默认0.7)
        """
        if self.propulsion_layer:
            self.propulsion_layer.balance_priority = max(0.0, min(1.0, balance_priority))
            print(f"✓ 平衡优先级设置为: {self.propulsion_layer.balance_priority:.2f}")
        return False
    
    def set_propulsion_constraints(self, max_roll=3.0, min_thrust=10.0):
        """设置推进约束条件
        
        Args:
            max_roll: 推进时允许的最大翻滚角 (度)
            min_thrust: 推进所需最小推力 (N)
        """
        if self.propulsion_layer:
            self.propulsion_layer.max_roll_allowed = max_roll
            self.propulsion_layer.min_thrust_required = min_thrust
            print(f"✓ 推进约束设置: max_roll={max_roll}°, min_thrust={min_thrust}N")
        return False
    
    def get_system_status(self):
        """获取系统状态
        
        Returns:
            status_dict: 包含各模块状态的字典
        """
        status = {
            'rotation_enabled': self.rotation_controller.enabled if self.rotation_controller else False,
            'propulsion_enabled': self.propulsion_layer.enabled if self.propulsion_layer else False,
            'rotation_left': self.rotation_controller.last_rotation_left if self.rotation_controller else 0.0,
            'rotation_right': self.rotation_controller.last_rotation_right if self.rotation_controller else 0.0,
            'propulsion_mode': self.propulsion_layer.propulsion_mode if self.propulsion_layer else 'disabled',
            'propulsion_speed': self.propulsion_layer.current_speed if self.propulsion_layer else 0.0,
            'thrust_left': self.current_thrust_left,
            'thrust_right': self.current_thrust_right,
        }
        return status
    
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
