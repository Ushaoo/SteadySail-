#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
电机 PID 角度控制测试程序
功能等同于 my_project/main/pwm_capture.c

用途：通过 PID 控制实现精确的电机角度控制
支持：
  - PWM 信号捕获（从舵机反馈）
  - PID 闭环控制
  - 串口命令控制目标角度
"""

import time
import math
from machine import I2C, Pin, Timer, UART
import ustruct as struct

# 导入库
try:
    from lib.pca9685 import PCA9685
except ImportError:
    print("Error: PCA9685 库未找到")
    PCA9685 = None

# ==================== 配置参数 ====================
# I2C 配置
I2C_SCL_PIN = 22
I2C_SDA_PIN = 21
I2C_FREQ = 400000

# 电机 PWM 配置
MOTOR_PWM_GPIO = 2              # PWM 输出引脚
MOTOR_PWM_FREQ_HZ = 50           # 50Hz 伺服频率
MOTOR_PWM_NEUTRAL = 1400         # 中立脉宽 (μs)
MOTOR_PWM_MAX = 1480             # 最大脉宽 (μs)
MOTOR_PWM_MIN = 1320             # 最小脉宽 (μs)

# PWM 输入捕获配置 (角度反馈)
PWM_INPUT_PIN = 4                # PWM 输入引脚 (读取反馈)
CAPTURE_MIN_PERIOD_US = 200      # 最小周期
CAPTURE_MAX_PERIOD_US = 200000   # 最大周期
CAPTURE_STALE_US = 300000        # 数据过期时间

# PID 控制参数
PID_LOOP_MS = 10                 # PID 循环周期 (ms)
PID_KP = 9.8                     # 比例系数
PID_KI = 1.0                     # 积分系数
PID_KD = 0.49                    # 微分系数
PID_DEADBAND_DEG = 3.0           # 死带 (度)
PID_DEADBAND_BLEND_DEG = 3.0     # 死带平滑过渡 (度)
PID_OUTPUT_FILTER_TAU_S = 0.12   # 输出滤波时间常数

# 串口配置
UART_BAUDRATE = 115200

# ==================== 工具函数 ====================
def clampf(value, lo, hi):
    """将浮点数限制在范围内"""
    return max(lo, min(hi, value))

def normalize_angle_360(angle):
    """将角度归一化到 [0, 360)"""
    while angle < 0.0:
        angle += 360.0
    while angle >= 360.0:
        angle -= 360.0
    return angle

def shortest_angle_error(target, current):
    """计算最短角度误差"""
    err = target - current
    while err > 180.0:
        err -= 360.0
    while err < -180.0:
        err += 360.0
    return err

def apply_smooth_deadband(err, deadband, blend):
    """应用平滑死带"""
    abs_err = abs(err)
    sign = 1.0 if err >= 0.0 else -1.0
    
    if abs_err <= deadband:
        return 0.0
    
    if blend <= 0.0 or abs_err >= (deadband + blend):
        return sign * (abs_err - deadband)
    
    # smoothstep 插值
    x = abs_err - deadband
    t = x / blend
    s = t * t * (3.0 - 2.0 * t)  # smoothstep
    return sign * (x * s)

def compute_angle_from_pwm(period_us, high_us):
    """从 PWM 脉宽计算旋转角度"""
    if period_us == 0:
        return 0.0
    duty_raw = (float(high_us) / float(period_us)) * 4119.0 - 16.0
    angle = duty_raw * (360.0 / 4095.0)
    return clampf(angle, 0.0, 360.0)


# ==================== PWM 捕获类 ====================
class PWMCapture:
    """PWM 信号捕获 - 读取伺服反馈信号"""
    
    def __init__(self, pin_num):
        """初始化 PWM 捕获
        
        参数：
            pin_num: GPIO 引脚号
        """
        self.pin = Pin(pin_num, Pin.IN, Pin.PULL_UP)
        self.period_us = 0
        self.high_us = 0
        self.last_update_time = 0
        self.last_rise_time = 0
        self.rise_time = 0
        
        print(f"✓ PWM 捕获初始化完成 (引脚 {pin_num})")
    
    def read_pwm(self):
        """读取 PWM 信号（高电平时间和周期）
        
        返回：
            (period_us, high_us): 周期和高电平时间，或 (0, 0) 如果无效
        """
        # 等待下降沿
        timeout = 100000  # 100ms 超时
        start_time = time.ticks_us()
        
        while self.pin.value() == 1:
            if time.ticks_diff(time.ticks_us(), start_time) > timeout:
                return 0, 0
        
        # 等待上升沿
        start_time = time.ticks_us()
        while self.pin.value() == 0:
            if time.ticks_diff(time.ticks_us(), start_time) > timeout:
                return 0, 0
        
        rise_time = time.ticks_us()
        period_time = time.ticks_diff(rise_time, self.last_rise_time)
        self.last_rise_time = rise_time
        
        # 等待下降沿
        start_time = time.ticks_us()
        while self.pin.value() == 1:
            if time.ticks_diff(time.ticks_us(), start_time) > timeout:
                return 0, 0
        
        fall_time = time.ticks_us()
        high_time = time.ticks_diff(fall_time, rise_time)
        
        self.last_update_time = time.ticks_us()
        
        # 验证 PWM 参数
        if (CAPTURE_MIN_PERIOD_US <= period_time <= CAPTURE_MAX_PERIOD_US and
            high_time > 0):
            self.period_us = period_time
            self.high_us = high_time
            return period_time, high_time
        
        return 0, 0
    
    def get_latest_angle(self):
        """获取最新的角度
        
        返回：
            (valid, angle_deg, period_us, high_us): 有效标志和角度数据
        """
        # 检查数据是否过期
        now = time.ticks_us()
        if time.ticks_diff(now, self.last_update_time) > CAPTURE_STALE_US:
            return False, 0.0, 0, 0
        
        if self.period_us == 0 or self.high_us == 0:
            return False, 0.0, 0, 0
        
        angle_deg = compute_angle_from_pwm(self.period_us, self.high_us)
        return True, angle_deg, self.period_us, self.high_us


# ==================== 电机控制类 ====================
class MotorControl:
    """电机 PWM 输出控制"""
    
    def __init__(self, gpio_pin):
        """初始化电机控制
        
        参数：
            gpio_pin: GPIO 引脚号
        """
        self.pin = Pin(gpio_pin, Pin.OUT)
        self.pwm = PWM(self.pin, freq=MOTOR_PWM_FREQ_HZ, duty_u16=0)
        self.current_duty = 0
        # 计算 PWM 中立点对应的 duty_u16 值
        # duty_u16 范围: 0-65535，对应 0-100%
        # 脉宽范围: 1000-2000 μs，周期: 20000 μs (50Hz)
        self.neutral_u16 = int((MOTOR_PWM_NEUTRAL / 20000.0) * 65535)
        self.max_u16 = int((MOTOR_PWM_MAX / 20000.0) * 65535)
        self.min_u16 = int((MOTOR_PWM_MIN / 20000.0) * 65535)
        
        print(f"✓ 电机控制初始化完成 (引脚 {gpio_pin})")
    
    def set_duty(self, duty_us):
        """设置 PWM 脉宽
        
        参数：
            duty_us: 脉宽 (μs)
        """
        # 限制脉宽范围
        duty_us = clampf(duty_us, MOTOR_PWM_MIN, MOTOR_PWM_MAX)
        # 转换为 duty_u16
        duty_u16 = int((duty_us / 20000.0) * 65535)
        self.pwm.duty_u16(duty_u16)
        self.current_duty = duty_us
    
    def set_angle_command(self, angle_deg):
        """根据目标角度设置 PWM 脉宽（开环控制示例）
        
        参数：
            angle_deg: 目标角度 (0-360°)
        """
        # 简单的线性映射: 0° -> 1000μs, 180° -> 1500μs, 360° -> 2000μs
        angle_deg = clampf(angle_deg, 0.0, 360.0)
        pulse_us = 1000.0 + (angle_deg / 360.0) * 1000.0
        self.set_duty(int(pulse_us))


# ==================== PID 控制器类 ====================
class PIDController:
    """PID 角度控制器 - 实现闭环控制"""
    
    def __init__(self, kp, ki, kd, dt_ms=PID_LOOP_MS):
        """初始化 PID 控制器
        
        参数：
            kp: 比例系数
            ki: 积分系数
            kd: 微分系数
            dt_ms: 控制周期 (ms)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt_ms / 1000.0
        
        self.target_deg = 0.0
        self.prev_err = 0.0
        self.integral = 0.0
        self.filtered_output = 0.0
        self.output_alpha = self.dt / (PID_OUTPUT_FILTER_TAU_S + self.dt)
        
        self.initialized = False
        
        print(f"✓ PID 控制器初始化: Kp={kp}, Ki={ki}, Kd={kd}")
    
    def set_target(self, target_deg):
        """设置目标角度"""
        # self.target_deg = normalize_angle_360(target_deg)
        self.target_deg = target_deg
        self.prev_err = 0.0
        self.integral = 0.0
        self.filtered_output = 0.0
        self.initialized = True
    
    def update(self, current_deg):
        """更新 PID 控制输出
        
        参数：
            current_deg: 当前角度
            
        返回：
            (valid, output): 有效标志和控制输出
        """
        if not self.initialized:
            return False, 0.0
        
        # 计算误差
        err_raw = shortest_angle_error(self.target_deg, current_deg)
        err = apply_smooth_deadband(err_raw, PID_DEADBAND_DEG, PID_DEADBAND_BLEND_DEG)
        
        # 积分（带限幅）
        self.integral += err * self.dt
        self.integral = clampf(self.integral, -120.0, 120.0)
        
        # 微分
        deriv = (err - self.prev_err) / self.dt if self.dt > 0 else 0.0
        self.prev_err = err
        
        # PID 计算
        output_raw = self.kp * err + self.ki * self.integral + self.kd * deriv
        
        # 输出滤波
        self.filtered_output += self.output_alpha * (output_raw - self.filtered_output)
        
        return True, output_raw, self.filtered_output, err_raw, err, deriv


# ==================== 主控制类 ====================
class MotorPIDControlSystem:
    """完整的电机 PID 控制系统"""
    
    def __init__(self):
        """初始化控制系统"""
        print("\n" + "="*60)
        print("电机 PID 角度控制系统初始化")
        print("="*60)
        
        # 初始化 PWM 捕获（读取反馈）
        try:
            self.pwm_capture = PWMCapture(PWM_INPUT_PIN)
        except Exception as e:
            print(f"✗ PWM 捕获初始化失败: {e}")
            self.pwm_capture = None
        
        # 初始化电机控制
        try:
            self.motor = MotorControl(MOTOR_PWM_GPIO)
        except Exception as e:
            print(f"✗ 电机控制初始化失败: {e}")
            self.motor = None
        
        # 初始化 PID 控制器
        self.pid = PIDController(PID_KP, PID_KI, PID_KD)
        
        # 初始化串口
        try:
            self.uart = UART(0, UART_BAUDRATE)
            print(f"✓ 串口初始化完成 (波特率 {UART_BAUDRATE})")
        except Exception as e:
            print(f"✗ 串口初始化失败: {e}")
            self.uart = None
        
        self.running = False
        self.log_interval_ms = 200
        self.last_log_time = 0
        
        print("✓ 系统初始化完成\n")
    
    def read_serial_command(self):
        """从串口读取目标角度命令"""
        if not self.uart:
            return None
        
        try:
            if self.uart.any():
                data = self.uart.readline()
                if data:
                    cmd_str = data.decode().strip()
                    try:
                        target = float(cmd_str)
                        return target
                    except ValueError:
                        print(f"✗ 无效的命令: {cmd_str}")
                        return None
        except Exception as e:
            print(f"✗ 串口读取失败: {e}")
        
        return None
    
    def run_pid_control(self):
        """运行 PID 闭环控制"""
        if not self.motor or not self.pwm_capture:
            print("✗ 电机或 PWM 捕获未正确初始化")
            return
        
        print("\n" + "="*60)
        print("启动 PID 闭环角度控制")
        print("="*60)
        print("通过串口输入目标角度 (0-360°)")
        print("示例: 90 或 270.5")
        print("按 Ctrl+C 停止\n")
        
        self.running = True
        self.motor.set_duty(MOTOR_PWM_NEUTRAL)
        time.sleep(1)
        
        try:
            loop_count = 0
            while self.running:
                loop_count += 1
                loop_start = time.ticks_ms()
                
                # 读取串口命令
                cmd = self.read_serial_command()
                if cmd is not None:
                    target = normalize_angle_360(cmd)
                    self.pid.set_target(target)
                    print(f"新目标角度: {target:.2f}°")
                
                # 读取当前角度
                if self.pwm_capture:
                    valid, angle_deg, period_us, high_us = self.pwm_capture.get_latest_angle()
                else:
                    # 如果没有实际反馈，使用模拟反馈
                    valid, angle_deg = False, 0.0
                
                # 初始化目标角度
                if valid and not self.pid.initialized:
                    self.pid.set_target(angle_deg)
                    print(f"初始目标角度设置为当前角度: {angle_deg:.2f}°")
                
                # 更新 PID 控制
                if valid and self.pid.initialized:
                    is_ok, u_raw, u_filt, err_raw, err, deriv = self.pid.update(angle_deg)
                    
                    if is_ok:
                        # 计算电机 PWM 脉宽
                        duty_cmd = int(MOTOR_PWM_NEUTRAL - u_filt)
                        self.motor.set_duty(duty_cmd)
                        
                        # 定期输出日志
                        now = time.ticks_ms()
                        if now - self.last_log_time >= self.log_interval_ms:
                            print(f"target={self.pid.target_deg:6.2f}° angle={angle_deg:6.2f}° "
                                  f"err_raw={err_raw:6.2f}° err={err:6.2f}° u_raw={u_raw:6.2f} "
                                  f"u_filt={u_filt:6.2f} duty={duty_cmd} period={period_us} high={high_us}")
                            self.last_log_time = now
                else:
                    self.motor.set_duty(MOTOR_PWM_NEUTRAL)
                
                # 控制循环周期
                loop_time = time.ticks_diff(time.ticks_ms(), loop_start)
                sleep_time = max(0, PID_LOOP_MS - loop_time)
                if sleep_time > 0:
                    time.sleep_ms(sleep_time)
        
        except KeyboardInterrupt:
            print("\n✓ 控制被中断")
        finally:
            self.motor.set_duty(MOTOR_PWM_NEUTRAL)
            self.running = False
    
    def run_pwm_capture_test(self):
        """运行 PWM 捕获测试（仅读取反馈）"""
        if not self.pwm_capture:
            print("✗ PWM 捕获未正确初始化")
            return
        
        print("\n" + "="*60)
        print("启动 PWM 捕获监测")
        print("="*60 + "\n")
        
        try:
            while True:
                valid, angle_deg, period_us, high_us = self.pwm_capture.get_latest_angle()
                
                if valid:
                    print(f"角度: {angle_deg:6.2f}° | 周期: {period_us:6d}μs | 高电平: {high_us:6d}μs")
                else:
                    print("未获取到有效的 PWM 信号")
                
                time.sleep(0.05)
        
        except KeyboardInterrupt:
            print("\n✓ 监测停止")


# ==================== 主程序 ====================
def main():
    """主程序"""
    system = MotorPIDControlSystem()
    
    try:
        # 运行 PID 闭环控制
        system.run_pid_control()
        
        # 或者只运行 PWM 捕获测试
        # system.run_pwm_capture_test()
    
    except Exception as e:
        print(f"\n✗ 系统错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
