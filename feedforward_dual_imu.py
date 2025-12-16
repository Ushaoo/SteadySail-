#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
前馈+双IMU融合 电机控制
- 使用 PCA9685 驱动电机（参考 motor_test.py）
- 双IMU融合姿态估计（参考 dual_imu.py）
- UDP数据发送（参考 data_sender.py）
"""

import time
import math
import numpy as np
import csv
from datetime import datetime
from math import sin, radians
from collections import deque
from threading import Thread, Event, Lock
import json

# 导入自定义模块
from motor_test import PCA9685
import mpu6050

# 导入数据发送器
from data_sender import DataSender

# 导入Web PID调节器
try:
    from web_pid_tuner import ParameterManager, WebPIDTuner, HAS_FLASK
except ImportError:
    HAS_FLASK = False
    ParameterManager = None
    WebPIDTuner = None
    print("Warning: web_pid_tuner not available")

# ==================== IMU 配置 ====================
# 每个IMU的XYZ轴是否需要反向
IMU1_INVERT_X = True
IMU1_INVERT_Y = True
IMU1_INVERT_Z = False

IMU2_INVERT_X = False
IMU2_INVERT_Y = False
IMU2_INVERT_Z = False

CALIBRATION_SAMPLES = 200  # 用于校准的采样次数

# ==================== 融合参数 ====================
ALPHA_ACC = 0.98          # 加速度互补滤波系数（越大越信低频IMU）
WEIGHT_DYNAMIC = 0.8      # 动态时更信低噪声那颗
BIAS_UPDATE_RATE = 0.001  # 静止时零偏更新速度

# ==================== 电机通道定义 ====================
LEFT_THRUSTER = 12
RIGHT_THRUSTER = 8

# ==================== 物理参数 ====================
# ==================== 物理参数 ====================
MASS = 80.0  # kg
WIDTH = 0.6  # m
INERTIA = MASS * (WIDTH / 2)**2 / 3  # 近似转动惯量 ≈2.4 kg·m²
G = 9.81  # m/s²
K_SELF = 100.0  # 自回中刚度系数（实验调谐）
DT = 0.01  # 循环时间 s

# ==================== FF / FB 调参 ====================
# ==================== FF / FB 调参 ====================
FEEDFORWARD_PARAM = 0.28  # 前馈力矩缩放
FEEDBACK_PARAM = 0.5  # PID反馈力矩缩放

# ==================== 电机PWM参数 ====================
BASE_PULSE = 1500  # 中性脉宽 (us)
MIN_PULSE = 1000   # 最小脉宽 (us)
MAX_PULSE = 2000   # 最大脉宽 (us)
THRUST_SCALE = 0.55  # 力矩到PWM的缩放系数

# ==================== 控制阈值 ====================
ANGLE_THRESH = 0.8  # 度，启动补偿
PRED_HORIZON = 0.05  # s，预测时域

# ==================== 滤波参数 ====================
ALPHA_EMA = 0.15  # EMA滤波系数（0.1振动大，0.3响应快）
OMEGA_DEADZONE = 3.0  # 角速度死区 (deg/s)

# ==================== 数据发送配置 ====================
ENABLE_DATA_SENDER = True  # 是否启用UDP数据发送

# ==================== Web调参配置 ====================
ENABLE_WEB_TUNER = True  # 是否启用Web PID调节器
WEB_TUNER_PORT = 5000  # Web服务器端口
WEB_TUNER_HOST = '0.0.0.0'  # Web服务器监听地址

# ==================== 数据记录配置 ====================
ENABLE_DATA_LOGGING = False  # 是否启用CSV数据记录（运行时询问）

# ==================== I2C容错配置 ====================
I2C_RETRY_COUNT = 3       # I2C通信失败时的重试次数
I2C_RETRY_DELAY = 0.01    # 重试间隔 (秒)
I2C_RESET_ON_FAIL = True  # 多次失败后是否尝试重置PCA9685

# ==================== PID参数 ====================
# ==================== PID参数 ====================
PID_KP = 20.0   # 比例增益
PID_KI = 1.0    # 积分增益
PID_KD = 0.0    # 微分增益
PID_B = 0.8     # 比例权重（2DOF）
PID_C = 0.0     # 微分权重（2DOF，0抑制setpoint kick）

# ==================== 死区参数 ====================
ANGLE_DEADZONE = 1.0  # 角度死区核心 (deg)
ANGLE_DEADZONE_SOFT = 3.0  # 角度死区软边界 (deg)，在此范围内平滑过渡
OMEGA_DEADZONE_SOFT = 6.0  # 角速度死区软边界 (deg/s)


class SafeMotorController:
    """带重试机制的安全电机控制器"""
    
    def __init__(self, pwm, retry_count=I2C_RETRY_COUNT, retry_delay=I2C_RETRY_DELAY):
        self.pwm = pwm
        self.retry_count = retry_count
        self.retry_delay = retry_delay
        self.error_count = 0
        self.total_errors = 0
        self.last_pulse = {LEFT_THRUSTER: BASE_PULSE, RIGHT_THRUSTER: BASE_PULSE}
    
    def set_pulse(self, channel, pulse):
        """安全设置PWM脉宽，带重试机制"""
        for attempt in range(self.retry_count):
            try:
                self.pwm.setServoPulse(channel, pulse)
                self.last_pulse[channel] = pulse
                self.error_count = 0  # 成功后重置连续错误计数
                return True
            except OSError as e:
                self.error_count += 1
                self.total_errors += 1
                if attempt < self.retry_count - 1:
                    time.sleep(self.retry_delay)
                else:
                    print(f"I2C通信失败 (channel={channel}, pulse={pulse}): {e}")
                    print(f"连续错误: {self.error_count}, 总错误: {self.total_errors}")
        
        # 如果多次失败，尝试重置PCA9685
        if I2C_RESET_ON_FAIL and self.error_count >= self.retry_count:
            self._try_reset()
        
        return False
    
    def _try_reset(self):
        """尝试重置PCA9685"""
        print("尝试重置PCA9685...")
        try:
            # 重新初始化MODE1寄存器
            self.pwm.write(0x00, 0x00)
            time.sleep(0.05)
            self.pwm.setPWMFreq(50)
            time.sleep(0.05)
            print("PCA9685重置成功")
            self.error_count = 0
        except Exception as e:
            print(f"PCA9685重置失败: {e}")
    
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
        """紧急停止 - 多次尝试将电机设为中性"""
        print("执行紧急停止...")
        for _ in range(5):  # 多次尝试
            try:
                self.pwm.setServoPulse(LEFT_THRUSTER, BASE_PULSE)
                self.pwm.setServoPulse(RIGHT_THRUSTER, BASE_PULSE)
                time.sleep(0.1)
                self.pwm.setServoPulse(LEFT_THRUSTER, 0)
                self.pwm.setServoPulse(RIGHT_THRUSTER, 0)
                print("电机已停止")
                return True
            except OSError:
                time.sleep(0.1)
        print("警告: 无法正常停止电机!")
        return False
    
    def get_status(self):
        """获取状态信息"""
        return {
            'error_count': self.error_count,
            'total_errors': self.total_errors,
            'last_pulse_left': self.last_pulse.get(LEFT_THRUSTER, 0),
            'last_pulse_right': self.last_pulse.get(RIGHT_THRUSTER, 0)
        }


class DataLogger:
    """IMU和控制数据记录器"""
    
    def __init__(self):
        self.enabled = False
        self.file = None
        self.writer = None
        self.sample_count = 0
        self.start_time = None
        
    def start_logging(self, prefix="feedforward_dual_imu", params=None):
        """开始记录数据，文件名中包含所有关键参数
        
        Args:
            prefix: 文件名前缀
            params: 参数字典，包含 PID、前馈、反馈等参数
        """
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 如果提供了参数，在文件名中包含所有关键参数
        if params:
            # 提取所有参数
            kp = params.get('pid_kp', 0)
            ki = params.get('pid_ki', 0)
            kd = params.get('pid_kd', 0)
            ff = params.get('feedforward_param', 0)
            fb = params.get('feedback_param', 0)
            mass = params.get('mass', 0)
            width = params.get('width', 0)
            dz = params.get('angle_deadzone', 0)
            dz_soft = params.get('angle_deadzone_soft', 0)
            omega_dz = params.get('omega_deadzone_soft', 0)
            thrust = params.get('thrust_scale', 0)
            motor_left_inv = params.get('motor_left_invert', False)
            motor_right_inv = params.get('motor_right_invert', False)
            
            # 构造参数字符串（使用简洁格式，按调整频率分组）
            # PID参数组
            pid_str = f"Kp{kp:.1f}_Ki{ki:.3f}_Kd{kd:.1f}"
            # 前馈反馈参数组
            ctrl_str = f"FF{ff:.2f}_FB{fb:.2f}"
            # 死区参数组
            dz_str = f"DZ{dz:.1f}_{dz_soft:.1f}_ODZ{omega_dz:.1f}"
            # 物理参数组
            phys_str = f"M{mass:.0f}_W{width:.2f}"
            # 电机参数组
            motor_str = f"TS{thrust:.2f}"
            if motor_left_inv or motor_right_inv:
                motor_inv = f"_L{int(motor_left_inv)}R{int(motor_right_inv)}"
            else:
                motor_inv = ""
            
            param_str = f"_{pid_str}_{ctrl_str}_{dz_str}_{phys_str}_{motor_str}{motor_inv}"
            filename = f"{prefix}{param_str}_{timestamp}.csv"
        else:
            filename = f"{prefix}_{timestamp}.csv"
        
        try:
            self.file = open(filename, 'w', newline='')
            self.writer = csv.writer(self.file)
            # 写入表头
            headers = [
                'timestamp',
                'roll_deg', 'pitch_deg', 'yaw_deg',
                'omega_raw', 'omega_filtered', 'alpha',
                'tau_ff', 'tau_pid', 'tau_total',
                'pwm_left', 'pwm_right',
                'deadzone_factor',
                'accel1_x', 'accel1_y', 'accel1_z',
                'gyro1_x', 'gyro1_y', 'gyro1_z',
                'accel2_x', 'accel2_y', 'accel2_z',
                'gyro2_x', 'gyro2_y', 'gyro2_z'
            ]
            self.writer.writerow(headers)
            
            self.enabled = True
            self.sample_count = 0
            self.start_time = time.time()
            print(f"开始记录数据到: {filename}")
            
        except Exception as e:
            print(f"启动数据记录失败: {e}")
            self.enabled = False
            
    def stop_logging(self):
        """停止记录数据"""
        if self.enabled and self.file:
            total_time = time.time() - self.start_time
            avg_rate = self.sample_count / total_time if total_time > 0 else 0
            
            self.file.close()
            self.enabled = False
            
            print(f"\n数据记录已停止")
            print(f"总样本数: {self.sample_count}")
            print(f"持续时间: {total_time:.2f} 秒")
            print(f"平均采样率: {avg_rate:.2f} Hz")
    
    def log_data(self, timestamp, roll, pitch, yaw,
                 omega_raw, omega_filtered, alpha,
                 tau_ff, tau_pid, tau_total,
                 pwm_left, pwm_right, deadzone_factor,
                 accel_1=None, gyro_1=None, accel_2=None, gyro_2=None):
        """记录一帧数据"""
        if not self.enabled:
            return
            
        try:
            row = [
                f"{timestamp:.4f}",
                f"{roll:.4f}", f"{pitch:.4f}", f"{yaw:.4f}",
                f"{omega_raw:.4f}", f"{omega_filtered:.4f}", f"{alpha:.4f}",
                f"{tau_ff:.4f}", f"{tau_pid:.4f}", f"{tau_total:.4f}",
                pwm_left, pwm_right,
                f"{deadzone_factor:.4f}"
            ]
            
            # IMU1数据
            if accel_1 and gyro_1:
                row.extend([
                    f"{accel_1['x']:.4f}", f"{accel_1['y']:.4f}", f"{accel_1['z']:.4f}",
                    f"{gyro_1['x']:.4f}", f"{gyro_1['y']:.4f}", f"{gyro_1['z']:.4f}"
                ])
            else:
                row.extend(['', '', '', '', '', ''])
            
            # IMU2数据
            if accel_2 and gyro_2:
                row.extend([
                    f"{accel_2['x']:.4f}", f"{accel_2['y']:.4f}", f"{accel_2['z']:.4f}",
                    f"{gyro_2['x']:.4f}", f"{gyro_2['y']:.4f}", f"{gyro_2['z']:.4f}"
                ])
            else:
                row.extend(['', '', '', '', '', ''])
            
            self.writer.writerow(row)
            self.sample_count += 1
            
            # 每500个样本输出一次进度
            if self.sample_count % 500 == 0:
                elapsed = time.time() - self.start_time
                current_rate = self.sample_count / elapsed
                print(f"已记录 {self.sample_count} 样本, 采样率: {current_rate:.1f} Hz")
                
        except Exception as e:
            print(f"数据写入错误: {e}")




class ParameterSync:
    """Web参数同步管理器"""
    
    def __init__(self, param_manager):
        self.param_manager = param_manager
        self.lock = Lock()
        
    def get_control_params(self):
        """获取控制参数的快照"""
        if self.param_manager is None:
            return None
            
        params = self.param_manager.get_current_params()
        return {
            'pid_kp': params['pid']['kp'],
            'pid_ki': params['pid']['ki'],
            'pid_kd': params['pid']['kd'],
            'feedforward_param': params.get('feedforward_param', FEEDFORWARD_PARAM),
            'feedback_param': params.get('feedback_param', FEEDBACK_PARAM),
            'mass': params.get('mass', MASS),
            'width': params.get('width', WIDTH),
            'angle_deadzone': params.get('angle_deadzone', ANGLE_DEADZONE),
            'angle_deadzone_soft': params.get('angle_deadzone_soft', ANGLE_DEADZONE_SOFT),
            'omega_deadzone_soft': params.get('omega_deadzone_soft', OMEGA_DEADZONE_SOFT),
            'motor_left_invert': params.get('motor_left_invert', False),
            'motor_right_invert': params.get('motor_right_invert', False),
            'thrust_scale': params.get('thrust_scale', THRUST_SCALE),
        }


def apply_deadzone_smooth(value, deadzone_core, deadzone_soft):
    """
    非线性死区插值函数
    
    - |value| < deadzone_core: 输出0（完全死区）
    - deadzone_core <= |value| < deadzone_soft: 平滑过渡（三次插值）
    - |value| >= deadzone_soft: 输出原值
    
    Args:
        value: 输入值
        deadzone_core: 死区核心半径（内部输出为0）
        deadzone_soft: 死区软边界（外部输出原值）
    
    Returns:
        处理后的值
    """
    abs_val = abs(value)
    
    if abs_val < deadzone_core:
        return 0.0
    elif abs_val >= deadzone_soft:
        return value
    else:
        # 三次插值平滑过渡: 0 -> 1 映射到 deadzone_core -> deadzone_soft
        # 使用 smoothstep 函数: 3t^2 - 2t^3
        t = (abs_val - deadzone_core) / (deadzone_soft - deadzone_core)
        smooth_factor = t * t * (3 - 2 * t)  # smoothstep
        return np.sign(value) * abs_val * smooth_factor


class PID2DOF:
    """2自由度PID控制器"""
    
    def __init__(self, Kp, Ki, Kd, b=1.0, c=0.0, dt=DT):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.b = b    # 比例权重
        self.c = c    # 微分权重
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0

    def update_gains(self, Kp, Ki, Kd):
        """动态更新PID增益"""
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def compute(self, setpoint, measured, omega_filtered):
        """计算PID输出
        
        Args:
            setpoint: 目标值（通常为0）
            measured: 测量角度
            omega_filtered: 滤波后的角速度
        
        Returns:
            PID输出力矩
        """
        error = setpoint - measured
        
        # 比例项（带权重b）
        proportional = self.Kp * (self.b * setpoint - measured)
        
        # 积分项
        self.integral += error * self.dt
        integral_term = self.Ki * self.integral
        
        # 微分项（使用滤波角速度，c=0时不跟踪setpoint变化）
        derivative = self.Kd * (self.c * 0 - omega_filtered)  # d(setpoint)/dt = 0
        
        output = proportional + integral_term + derivative
        return output

    def reset(self):
        """重置积分器"""
        self.integral = 0.0
        self.prev_error = 0.0


class DualIMUFusion:
    """双IMU融合算法（从dual_imu.py整合）"""
    
    def __init__(self):
        self.q0, self.q1, self.q2, self.q3 = 1.0, 0.0, 0.0, 0.0
        
        # 两颗IMU的零偏和协方差（在线估计）
        self.bias_gx = [0.0, 0.0]
        self.bias_gy = [0.0, 0.0]
        self.bias_gz = [0.0, 0.0]
        
        self.var_gx = [1.0, 1.0]
        self.var_gy = [1.0, 1.0]
        self.var_gz = [1.0, 1.0]
        
        # 滑动窗口估计噪声（100个样本）
        self.gyro_buffer = [deque(maxlen=100), deque(maxlen=100)]

    def set_gyro_bias(self, bias_g1, bias_g2):
        """设置初始的陀螺仪零偏"""
        self.bias_gx = [bias_g1['x'], bias_g2['x']]
        self.bias_gy = [bias_g1['y'], bias_g2['y']]
        self.bias_gz = [bias_g1['z'], bias_g2['z']]
        print(f"陀螺仪零偏已设置: IMU1={bias_g1}, IMU2={bias_g2}")

    def update(self, ax1, ay1, az1, gx1, gy1, gz1, ax2, ay2, az2, gx2, gy2, gz2, dt):
        # Step 1：加速度互补得到最可信的重力方向
        acc1_mag = np.sqrt(ax1**2 + ay1**2 + az1**2)
        acc2_mag = np.sqrt(ax2**2 + ay2**2 + az2**2)
        
        if acc1_mag > 0.1:
            ax1, ay1, az1 = ax1/acc1_mag, ay1/acc1_mag, az1/acc1_mag
        if acc2_mag > 0.1:
            ax2, ay2, az2 = ax2/acc2_mag, ay2/acc2_mag, az2/acc2_mag
            
        ax = ALPHA_ACC * ax1 + (1-ALPHA_ACC) * ax2
        ay = ALPHA_ACC * ay1 + (1-ALPHA_ACC) * ay2
        az = ALPHA_ACC * az1 + (1-ALPHA_ACC) * az2
        
        mag = np.sqrt(ax**2 + ay**2 + az**2)
        if mag > 0.01:
            ax, ay, az = ax/mag, ay/mag, az/mag

        # Step 2：角速度最优加权
        for i, (gx, gy, gz) in enumerate([(gx1, gy1, gz1), (gx2, gy2, gz2)]):
            self.gyro_buffer[i].append([gx, gy, gz])
            if len(self.gyro_buffer[i]) == 100:
                arr = np.array(self.gyro_buffer[i])
                self.var_gx[i] = np.var(arr[:, 0])
                self.var_gy[i] = np.var(arr[:, 1])
                self.var_gz[i] = np.var(arr[:, 2])

        inv_var_gx0 = 1 / (self.var_gx[0] + 1e-9)
        inv_var_gy0 = 1 / (self.var_gy[0] + 1e-9)
        inv_var_gz0 = 1 / (self.var_gz[0] + 1e-9)
        inv_var_gx1 = 1 / (self.var_gx[1] + 1e-9)
        inv_var_gy1 = 1 / (self.var_gy[1] + 1e-9)
        inv_var_gz1 = 1 / (self.var_gz[1] + 1e-9)

        w1_total_inv_var = inv_var_gx0 + inv_var_gy0 + inv_var_gz0
        w2_total_inv_var = inv_var_gx1 + inv_var_gy1 + inv_var_gz1
        total_inv_var = w1_total_inv_var + w2_total_inv_var
        
        if total_inv_var < 1e-9:
            w1 = 0.5
        else:
            w1 = w1_total_inv_var / total_inv_var
        w2 = 1.0 - w1
        
        acc_error = abs(mag - 1.0)
        if acc_error > 0.3:
            if w1_total_inv_var > w2_total_inv_var:
                w1 = WEIGHT_DYNAMIC
            else:
                w1 = 1 - WEIGHT_DYNAMIC
            w2 = 1 - w1

        gx = w1 * gx1 + w2 * gx2
        gy = w1 * gy1 + w2 * gy2
        gz = w1 * gz1 + w2 * gz2

        # Step 3：Mahony 更新
        gx, gy, gz = np.radians(gx), np.radians(gy), np.radians(gz)

        vx = 2*(self.q1*self.q3 - self.q0*self.q2)
        vy = 2*(self.q0*self.q1 + self.q2*self.q3)
        vz = self.q0**2 - self.q1**2 - self.q2**2 + self.q3**2

        ex = ay*vz - az*vy
        ey = az*vx - ax*vz
        ez = ax*vy - ay*vx

        Kp = 2.0 + 25.0 * acc_error
        gx += Kp * ex
        gy += Kp * ey
        gz += Kp * ez

        self.q0 += 0.5*dt * (-self.q1*gx - self.q2*gy - self.q3*gz)
        self.q1 += 0.5*dt * (self.q0*gx + self.q2*gz - self.q3*gy)
        self.q2 += 0.5*dt * (self.q0*gy - self.q1*gz + self.q3*gx)
        self.q3 += 0.5*dt * (self.q0*gz + self.q1*gy - self.q2*gx)

        norm = np.sqrt(self.q0**2 + self.q1**2 + self.q2**2 + self.q3**2)
        if norm > 1e-9:
            self.q0 /= norm
            self.q1 /= norm
            self.q2 /= norm
            self.q3 /= norm

        return self.quat_to_euler()

    def quat_to_euler(self):
        roll = np.arctan2(2*(self.q0*self.q1 + self.q2*self.q3), 1-2*(self.q1**2 + self.q2**2))
        pitch = np.arcsin(np.clip(2*(self.q0*self.q2 - self.q3*self.q1), -1.0, 1.0))
        yaw = np.arctan2(2*(self.q0*self.q3 + self.q1*self.q2), 1-2*(self.q2**2 + self.q3**2))
        return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)


def calibrate_imus(mpu1, mpu2, samples=CALIBRATION_SAMPLES):
    """校准双IMU"""
    print(f"开始IMU校准，请保持设备静止（Z轴向上）... 将采集 {samples} 个样本。")
    
    bias_g1 = {'x': 0, 'y': 0, 'z': 0}
    bias_a1 = {'x': 0, 'y': 0, 'z': 0}
    bias_g2 = {'x': 0, 'y': 0, 'z': 0}
    bias_a2 = {'x': 0, 'y': 0, 'z': 0}

    for i in range(samples):
        try:
            g1_raw = mpu1.get_gyro_data()
            a1_raw = mpu1.get_accel_data()
            g2_raw = mpu2.get_gyro_data()
            a2_raw = mpu2.get_accel_data()

            bias_g1['x'] += g1_raw['x'] * (-1 if IMU1_INVERT_X else 1)
            bias_g1['y'] += g1_raw['y'] * (-1 if IMU1_INVERT_Y else 1)
            bias_g1['z'] += g1_raw['z'] * (-1 if IMU1_INVERT_Z else 1)
            bias_a1['x'] += a1_raw['x'] * (-1 if IMU1_INVERT_X else 1)
            bias_a1['y'] += a1_raw['y'] * (-1 if IMU1_INVERT_Y else 1)
            bias_a1['z'] += a1_raw['z'] * (-1 if IMU1_INVERT_Z else 1)

            bias_g2['x'] += g2_raw['x'] * (-1 if IMU2_INVERT_X else 1)
            bias_g2['y'] += g2_raw['y'] * (-1 if IMU2_INVERT_Y else 1)
            bias_g2['z'] += g2_raw['z'] * (-1 if IMU2_INVERT_Z else 1)
            bias_a2['x'] += a2_raw['x'] * (-1 if IMU2_INVERT_X else 1)
            bias_a2['y'] += a2_raw['y'] * (-1 if IMU2_INVERT_Y else 1)
            bias_a2['z'] += a2_raw['z'] * (-1 if IMU2_INVERT_Z else 1)

            if (i + 1) % 100 == 0:
                print(f"已采集 {i + 1}/{samples}...")
            time.sleep(0.002)
        except OSError:
            print("校准期间读取传感器错误，跳过此样本")

    bias_g1 = {k: v / samples for k, v in bias_g1.items()}
    bias_a1 = {k: v / samples for k, v in bias_a1.items()}
    bias_g2 = {k: v / samples for k, v in bias_g2.items()}
    bias_a2 = {k: v / samples for k, v in bias_a2.items()}
    
    bias_a1['z'] = bias_a1['z'] - 9.8
    bias_a2['z'] = bias_a2['z'] - 9.8

    print("校准完成！")
    print(f"陀螺仪零偏 IMU1: x={bias_g1['x']:.3f}, y={bias_g1['y']:.3f}, z={bias_g1['z']:.3f}")
    print(f"陀螺仪零偏 IMU2: x={bias_g2['x']:.3f}, y={bias_g2['y']:.3f}, z={bias_g2['z']:.3f}")
    print(f"加速度计零偏 IMU1: x={bias_a1['x']:.3f}, y={bias_a1['y']:.3f}, z={bias_a1['z']:.3f}")
    print(f"加速度计零偏 IMU2: x={bias_a2['x']:.3f}, y={bias_a2['y']:.3f}, z={bias_a2['z']:.3f}")
    return bias_g1, bias_a1, bias_g2, bias_a2


def read_dual_imu(mpu1, mpu2, bias_g1, bias_a1, bias_g2, bias_a2):
    """读取并处理双IMU数据"""
    accel_1_raw = mpu1.get_accel_data()
    gyro_1_raw = mpu1.get_gyro_data()
    accel_2_raw = mpu2.get_accel_data()
    gyro_2_raw = mpu2.get_gyro_data()

    # 应用轴向反转和零偏校正
    accel_1 = {
        'x': accel_1_raw['x'] * (-1 if IMU1_INVERT_X else 1) - bias_a1['x'],
        'y': accel_1_raw['y'] * (-1 if IMU1_INVERT_Y else 1) - bias_a1['y'],
        'z': accel_1_raw['z'] * (-1 if IMU1_INVERT_Z else 1) - bias_a1['z'],
    }
    gyro_1 = {
        'x': gyro_1_raw['x'] * (-1 if IMU1_INVERT_X else 1) - bias_g1['x'],
        'y': gyro_1_raw['y'] * (-1 if IMU1_INVERT_Y else 1) - bias_g1['y'],
        'z': gyro_1_raw['z'] * (-1 if IMU1_INVERT_Z else 1) - bias_g1['z'],
    }
    accel_2 = {
        'x': accel_2_raw['x'] * (-1 if IMU2_INVERT_X else 1) - bias_a2['x'],
        'y': accel_2_raw['y'] * (-1 if IMU2_INVERT_Y else 1) - bias_a2['y'],
        'z': accel_2_raw['z'] * (-1 if IMU2_INVERT_Z else 1) - bias_a2['z'],
    }
    gyro_2 = {
        'x': gyro_2_raw['x'] * (-1 if IMU2_INVERT_X else 1) - bias_g2['x'],
        'y': gyro_2_raw['y'] * (-1 if IMU2_INVERT_Y else 1) - bias_g2['y'],
        'z': gyro_2_raw['z'] * (-1 if IMU2_INVERT_Z else 1) - bias_g2['z'],
    }
    
    return accel_1, gyro_1, accel_2, gyro_2


def main():
    """主函数"""
    global ENABLE_DATA_LOGGING
    
    # ==================== 初始化Web调参 ====================
    param_manager = None
    web_tuner = None
    param_sync = None
    
    if ENABLE_WEB_TUNER and HAS_FLASK:
        try:
            param_manager = ParameterManager()
            
            # 初始化Web调参中的统一PID参数
            param_manager.current_params['pid']['kp'] = PID_KP
            param_manager.current_params['pid']['ki'] = PID_KI
            param_manager.current_params['pid']['kd'] = PID_KD
            # 添加前馈反馈参数
            param_manager.current_params['feedforward_param'] = FEEDFORWARD_PARAM
            param_manager.current_params['feedback_param'] = FEEDBACK_PARAM
            # 添加物理参数
            param_manager.current_params['mass'] = MASS
            param_manager.current_params['width'] = WIDTH
            # 添加死区参数
            param_manager.current_params['angle_deadzone'] = ANGLE_DEADZONE
            param_manager.current_params['angle_deadzone_soft'] = ANGLE_DEADZONE_SOFT
            param_manager.current_params['omega_deadzone_soft'] = OMEGA_DEADZONE_SOFT
            # 添加电机控制参数
            param_manager.current_params['motor_left_invert'] = False
            param_manager.current_params['motor_right_invert'] = False
            
            web_tuner = WebPIDTuner(param_manager, port=WEB_TUNER_PORT, host=WEB_TUNER_HOST)
            web_tuner.start()
            param_sync = ParameterSync(param_manager)
            
            print(f"Web PID调节器已启动: http://{WEB_TUNER_HOST}:{WEB_TUNER_PORT}")
        except Exception as e:
            print(f"Web调参初始化失败: {e}")
            print("将继续使用配置文件中的参数运行")
    elif ENABLE_WEB_TUNER and not HAS_FLASK:
        print("Flask未安装，Web调参功能已禁用")
        print("如需启用Web调参，请运行: pip install flask")
    
    # ==================== 初始化硬件 ====================
    print("初始化PCA9685电机驱动...")
    pwm = PCA9685(address=0x40, debug=True, bus_num=1)
    pwm.setPWMFreq(50)  # 50Hz for ESC
    
    print("初始化双IMU...")
    try:
        mpu1 = mpu6050.mpu6050(0x68)
        mpu2 = mpu6050.mpu6050(0x68, 2)  # 第二个IMU在bus 2
        print("双IMU初始化成功")
    except OSError as e:
        print(f"IMU初始化失败: {e}")
        print("请检查I2C连接和地址 (运行 'sudo i2cdetect -y 1')")
        return

    # ==================== 校准IMU ====================
    bias_g1, bias_a1, bias_g2, bias_a2 = calibrate_imus(mpu1, mpu2)
    
    # ==================== 初始化融合算法 ====================
    fusion = DualIMUFusion()
    fusion.set_gyro_bias(bias_g1, bias_g2)
    
    # ==================== 初始化PID控制器 ====================
    pid = PID2DOF(Kp=PID_KP, Ki=PID_KI, Kd=PID_KD, b=PID_B, c=PID_C)
    print(f"PID初始化: Kp={PID_KP}, Ki={PID_KI}, Kd={PID_KD}, b={PID_B}, c={PID_C}")
    
    # ==================== 初始化数据发送器 ====================
    data_sender = None
    if ENABLE_DATA_SENDER:
        data_sender = DataSender()
        data_sender.start()
        print("数据发送器已启动")
    
    # ==================== 初始化数据记录器 ====================
    data_logger = DataLogger()
    
    # 如果启用了Web调参，在param_manager中设置data_logger引用
    if param_manager:
        param_manager.set_data_logger(data_logger)
        print("数据记录器已添加到Web调参管理器")
        print("在Web界面中使用'Start Recording'按钮来启动数据记录")
    
    # ==================== 初始化安全电机控制器 ====================
    motor_ctrl = SafeMotorController(pwm)
    
    # ==================== 初始化电机（发送中性信号）====================
    print("初始化电机...")
    motor_ctrl.set_pulse(LEFT_THRUSTER, BASE_PULSE)
    motor_ctrl.set_pulse(RIGHT_THRUSTER, BASE_PULSE)
    time.sleep(3)  # ESC初始化等待
    
    print("开始前馈控制...")
    print("按 Ctrl+C 停止")
    
    # ==================== 主控制循环 ====================
    prev_filtered_omega = 0.0
    filtered_omega = 0.0
    prev_time = time.time()
    control_start = prev_time
    
    try:
        while True:
            current_time = time.time()
            dt = current_time - prev_time
            if dt < DT:
                time.sleep(DT - dt)
                current_time = time.time()
                dt = current_time - prev_time
            prev_time = current_time
            
            # 步骤0: 获取实时参数（如果启用Web调参）
            current_feedforward = FEEDFORWARD_PARAM
            current_feedback = FEEDBACK_PARAM
            current_mass = MASS
            current_width = WIDTH
            current_angle_deadzone = ANGLE_DEADZONE
            current_angle_deadzone_soft = ANGLE_DEADZONE_SOFT
            current_omega_deadzone_soft = OMEGA_DEADZONE_SOFT
            current_thrust_scale = THRUST_SCALE
            motor_left_invert = False
            motor_right_invert = False
            
            if param_sync:
                try:
                    params = param_sync.get_control_params()
                    if params:
                        pid.update_gains(params['pid_kp'], params['pid_ki'], params['pid_kd'])
                        current_feedforward = params['feedforward_param']
                        current_feedback = params['feedback_param']
                        current_mass = params.get('mass', MASS)
                        current_width = params.get('width', WIDTH)
                        current_angle_deadzone = params['angle_deadzone']
                        current_angle_deadzone_soft = params['angle_deadzone_soft']
                        current_omega_deadzone_soft = params['omega_deadzone_soft']
                        current_thrust_scale = params.get('thrust_scale', THRUST_SCALE)
                        motor_left_invert = params.get('motor_left_invert', False)
                        motor_right_invert = params.get('motor_right_invert', False)
                except Exception as e:
                    print(f"参数同步错误: {e}")
            print(f"\rKp={pid.Kp:.2f} Ki={pid.Ki:.4f} Kd={pid.Kd:.2f} | Feedforward={current_feedforward:.2f} Feedback={current_feedback:.2f}    ", end='')
            print(current_mass, current_width, current_angle_deadzone)
            # 步骤1: 读取双IMU数据
            try:
                accel_1, gyro_1, accel_2, gyro_2 = read_dual_imu(
                    mpu1, mpu2, bias_g1, bias_a1, bias_g2, bias_a2
                )
            except OSError as e:
                print(f"传感器读取错误: {e}")
                continue
            
            # 步骤2: 融合姿态
            roll, pitch, yaw = fusion.update(
                accel_1['x'], accel_1['y'], accel_1['z'],
                gyro_1['x'], gyro_1['y'], gyro_1['z'],
                accel_2['x'], accel_2['y'], accel_2['z'],
                gyro_2['x'], gyro_2['y'], gyro_2['z'],
                dt
            )
            
            # 使用roll角作为主要控制变量
            theta = roll  # 角度（度）
            
            # 融合角速度（简单平均）
            raw_omega = (gyro_1['x'] + gyro_2['x']) / 2.0  # deg/s
            
            # EMA滤波角速度
            filtered_omega = ALPHA_EMA * raw_omega + (1 - ALPHA_EMA) * filtered_omega
            
            # 计算角加速度（从滤波后的角速度）
            alpha = (filtered_omega - prev_filtered_omega) / dt if dt > 0 else 0.0
            prev_filtered_omega = filtered_omega
            
            # 死区处理：小角速度使用非线性插值平滑过渡
            omega = apply_deadzone_smooth(filtered_omega, OMEGA_DEADZONE, current_omega_deadzone_soft)
            
            # 如果角速度被衰减，角加速度也相应衰减
            if abs(filtered_omega) > 1e-6:
                omega_ratio = abs(omega / filtered_omega)
            else:
                omega_ratio = 0.0
            alpha = alpha * omega_ratio

            # 步骤3: 估计扰动力矩
            tau_disturb = current_mass * G * (current_width / 2) * sin(radians(theta))

            # 步骤4: 预测
            theta_pred = theta + omega * PRED_HORIZON + 0.5 * alpha * PRED_HORIZON**2

            # 步骤5: 前馈计算所需力矩
            tau_self = -K_SELF * theta  # 自回中补偿
            tau_ff = -tau_disturb - INERTIA * alpha - tau_self  # 前馈力矩
            
            # 预测补偿
            if abs(theta_pred) > ANGLE_THRESH:
                tau_ff *= 1.2
            
            # 步骤6: PID反馈（基于残余误差）
            tau_pid = pid.compute(setpoint=0.0, measured=theta, omega_filtered=omega)
            
            # 总力矩 = 前馈 + PID反馈
            tau_total = current_feedforward * tau_ff - current_feedback * tau_pid
            
            # 全系统死区：使用非线性插值平滑衰减输出
            # 计算角度和角速度的死区因子（0~1）
            if abs(theta) < current_angle_deadzone:
                angle_factor = 0.0
                pid.reset()  # 在核心死区内清积分
            elif abs(theta) < current_angle_deadzone_soft:
                t = (abs(theta) - current_angle_deadzone) / (current_angle_deadzone_soft - current_angle_deadzone)
                angle_factor = t * t * (3 - 2 * t)  # smoothstep
            else:
                angle_factor = 1.0
            
            if abs(omega) < OMEGA_DEADZONE:
                omega_factor = 0.0
            elif abs(omega) < current_omega_deadzone_soft:
                t = (abs(omega) - OMEGA_DEADZONE) / (current_omega_deadzone_soft - OMEGA_DEADZONE)
                omega_factor = t * t * (3 - 2 * t)  # smoothstep
            else:
                omega_factor = 1.0
            
            # 综合死区因子：取较大值（只要有一个量较大就输出）
            deadzone_factor = max(angle_factor, omega_factor)
            tau_total = tau_total * deadzone_factor

            # 步骤7: 映射到PWM脉宽
            # 力矩 -> PWM调整量（每边最大±500，即1000~2000全范围）
            pwm_adjust = tau_total * current_thrust_scale
            pwm_adjust = max(min(pwm_adjust, 500), -500)  # 限幅±500
            
            # 左右电机差动控制（正力矩 -> 左加右减）
            pulse_left = int(BASE_PULSE + pwm_adjust)
            pulse_right = int(BASE_PULSE - pwm_adjust)
            
            # 限幅到有效PWM范围
            pulse_left = max(min(pulse_left, MAX_PULSE), MIN_PULSE)
            pulse_right = max(min(pulse_right, MAX_PULSE), MIN_PULSE)

            # 步骧8: 驱动电机（使用安全控制器）
            # 计算实际的PWM值，考虑Web Tuner中设置的反转标志
            if motor_left_invert:
                pwm_left_actual = pulse_left
            else:
                pwm_left_actual = 3000-pulse_left
            
            if motor_right_invert:
                pwm_right_actual = pulse_right
            else:
                pwm_right_actual = 3000 - pulse_right
            
            motor_ctrl.set_pulse(LEFT_THRUSTER, pwm_left_actual)
            motor_ctrl.set_pulse(RIGHT_THRUSTER, pwm_right_actual)
            
            # 步骧9: 发送数据（如果启用）
            if data_sender:
                timestamp = current_time - control_start
                bangbang_protection = abs(theta) < current_angle_deadzone and abs(omega) < OMEGA_DEADZONE
                data_sender.record(
                    timestamp=timestamp,
                    angle_deg=theta,
                    angular_velocity_deg_s=omega,
                    pwm_left=pulse_left,
                    pwm_right=pulse_right,
                    M_ff=tau_ff,
                    M_fb=tau_pid,  # PID反馈力矩
                    bangbang_protection=bangbang_protection
                )
            
            # 步骤10: 记录数据到CSV（如果启用）
            if data_logger.enabled:
                timestamp = current_time - control_start
                data_logger.log_data(
                    timestamp=timestamp,
                    roll=theta, pitch=pitch, yaw=yaw,
                    omega_raw=raw_omega, omega_filtered=omega, alpha=alpha,
                    tau_ff=tau_ff, tau_pid=tau_pid, tau_total=tau_total,
                    pwm_left=pulse_left, pwm_right=pulse_right,
                    deadzone_factor=deadzone_factor,
                    accel_1=accel_1, gyro_1=gyro_1,
                    accel_2=accel_2, gyro_2=gyro_2
                )
            
            # 打印状态
            print(f"Roll: {theta:+6.2f}° | ω: {omega:+6.2f}°/s | "
                  f"τ_ff: {tau_ff:+7.2f} | τ_pid: {tau_pid:+7.2f} | PWM_L: {pulse_left} | PWM_R: {pulse_right}")

    except KeyboardInterrupt:
        print("\n用户中断，停止控制...")
    finally:
        # 使用安全控制器停止电机
        motor_ctrl.emergency_stop()
        
        # 打印I2C错误统计
        status = motor_ctrl.get_status()
        if status['total_errors'] > 0:
            print(f"\nI2C通信统计: 总错误次数={status['total_errors']}")
        
        # 停止Web调参
        if web_tuner:
            try:
                web_tuner.stop()
            except Exception as e:
                print(f"停止Web调参时出错: {e}")
        
        # 停止数据发送器
        if data_sender:
            data_sender.stop()
        
        # 停止数据记录器
        if data_logger.enabled:
            data_logger.stop_logging()
        
        print("程序已退出")


if __name__ == "__main__":
    main()