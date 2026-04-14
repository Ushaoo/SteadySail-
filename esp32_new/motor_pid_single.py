#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
单电机 PID 角度控制测试程序 (单电机优化版)
功能等同于 my_project/main/pwm_capture.c

针对单个推进器/舵机的完整控制系统
- PWM 信号输出 (GPIO2)
- PWM 信号反馈读取 (GPIO4，可选)
- PID 闭环控制
- 串口命令接收

连线：
  GPIO2  → 电机/舵机控制信号
  GPIO4  → 电机反馈信号 (可选)
  GPIO21 → I2C SDA (不需要用到)
  GPIO22 → I2C SCL (不需要用到)
"""

import time
import math
from machine import Pin, PWM, UART

# ==================== 配置参数 ====================

# 电机 PWM 配置
MOTOR_PWM_GPIO = 2              # PWM 输出引脚 (主控信号)
MOTOR_PWM_FREQ_HZ = 50          # 50Hz 伺服频率
MOTOR_PWM_NEUTRAL = 1400        # 中立脉宽 (μs) - 电机停止的位置
MOTOR_PWM_MAX = 1480            # 最大脉宽 (μs) - 最大正向
MOTOR_PWM_MIN = 1320            # 最小脉宽 (μs) - 最大反向

# PWM 输入反馈配置 (用于读取电机位置反馈)
PWM_INPUT_PIN = 4               # PWM 输入引脚 (读取反馈信号)
ENABLE_FEEDBACK = True          # 是否启用反馈(如果你的电机没有反馈，改为 False)
CAPTURE_STALE_US = 300000       # 反馈信号过期时间 (300ms)

# PID 控制参数 (针对单个电机优化)
PID_LOOP_MS = 10                # PID 循环周期 (ms)
PID_KP = 8.0                    # 比例系数 (针对单电机降低一些)
PID_KI = 0.8                    # 积分系数
PID_KD = 0.4                    # 微分系数
PID_DEADBAND_DEG = 2.0          # 死带 (度) - 误差在此范围内时输出为0
PID_DEADBAND_BLEND_DEG = 2.0    # 死带平滑过渡 (度)
PID_OUTPUT_FILTER_TAU_S = 0.1   # 输出滤波时间常数

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


# ==================== PWM 反馈读取类 ====================

class FeedbackReader:
    """PWM 反馈信号读取 - 单电机专用版本"""
    
    def __init__(self, pin_num):
        """初始化反馈读取
        
        参数:
            pin_num: GPIO 引脚号
        """
        self.pin = Pin(pin_num, Pin.IN, Pin.PULL_UP)
        self.period_us = 0
        self.high_us = 0
        self.last_update_time = 0
        self.last_rise_time = 0
        
        print(f"✓ 反馈读取初始化 (GPIO {pin_num})")
    
    def get_latest_angle(self):
        """获取最新的角度
        
        返回:
            (valid, angle_deg): 有效标志和角度
        """
        # 简单的等待 + 计时实现
        timeout_count = 100000  # 约 100ms 超时
        
        try:
            # 等待下降沿
            count = 0
            while self.pin.value() == 1 and count < timeout_count:
                count += 1
            
            if count >= timeout_count:
                return False, 0.0
            
            # 等待上升沿
            count = 0
            while self.pin.value() == 0 and count < timeout_count:
                count += 1
            
            if count >= timeout_count:
                return False, 0.0
            
            rise_time = time.ticks_us()
            
            # 等待下降沿，同时计算脉宽
            pulse_start = time.ticks_us()
            count = 0
            while self.pin.value() == 1 and count < timeout_count:
                count += 1
            
            if count >= timeout_count:
                return False, 0.0
            
            pulse_end = time.ticks_us()
            self.high_us = time.ticks_diff(pulse_end, pulse_start)
            
            # 周期通常是 20000μs (50Hz)
            self.period_us = 20000
            self.last_update_time = time.ticks_us()
            
            if self.high_us > 0 and self.high_us < self.period_us:
                angle = compute_angle_from_pwm(self.period_us, self.high_us)
                return True, angle
            
        except Exception as e:
            print(f"反馈读取错误: {e}")
        
        return False, 0.0


# ==================== 正向电机控制类 ====================

class SingleMotorControl:
    """单电机 PWM 控制"""
    
    def __init__(self, gpio_pin):
        """初始化电机控制
        
        参数:
            gpio_pin: GPIO 引脚号
        """
        try:
            self.pin = Pin(gpio_pin, Pin.OUT)
            self.pwm = PWM(self.pin, freq=MOTOR_PWM_FREQ_HZ, duty_u16=0)
            
            # 计算 PWM 参数
            # duty_u16 范围: 0-65535，对应 0-100%
            # 脉宽范围: 1000-2000 μs，周期: 20000 μs (50Hz)
            self.neutral_u16 = int((MOTOR_PWM_NEUTRAL / 20000.0) * 65535)
            self.max_u16 = int((MOTOR_PWM_MAX / 20000.0) * 65535)
            self.min_u16 = int((MOTOR_PWM_MIN / 20000.0) * 65535)
            
            # 设置初始位置为中立
            self.set_duty(MOTOR_PWM_NEUTRAL)
            
            print(f"✓ 电机控制初始化 (GPIO {gpio_pin})")
            print(f"  中立: {MOTOR_PWM_NEUTRAL}μs | 最小: {MOTOR_PWM_MIN}μs | 最大: {MOTOR_PWM_MAX}μs")
        
        except Exception as e:
            print(f"✗ 电机初始化失败: {e}")
            self.pwm = None
    
    def set_duty(self, duty_us):
        """设置 PWM 脉宽
        
        参数:
            duty_us: 脉宽 (μs)
        """
        if not self.pwm:
            return
        
        # 限制脉宽范围
        duty_us = clampf(duty_us, MOTOR_PWM_MIN, MOTOR_PWM_MAX)
        # 转换为 duty_u16
        duty_u16 = int((duty_us / 20000.0) * 65535)
        self.pwm.duty_u16(duty_u16)


# ==================== PID 控制器 ====================

class SimpleMotorPID:
    """简化版 PID 控制器"""
    
    def __init__(self, kp, ki, kd, dt_ms=PID_LOOP_MS):
        """初始化 PID"""
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
        
        print(f"✓ PID 初始化: Kp={kp}, Ki={ki}, Kd={kd}, dt={self.dt}s")
    
    def set_target(self, target_deg):
        """设置目标角度"""
        self.target_deg = normalize_angle_360(target_deg)
        self.prev_err = 0.0
        self.integral = 0.0
        self.filtered_output = 0.0
        self.initialized = True
    
    def update(self, current_deg):
        """更新 PID 输出
        
        参数:
            current_deg: 当前角度
            
        返回:
            control_output: PID 控制输出 (用于 PWM 脉宽计算)
        """
        if not self.initialized:
            return 0.0
        
        # 计算误差
        err_raw = shortest_angle_error(self.target_deg, current_deg)
        err = apply_smooth_deadband(err_raw, PID_DEADBAND_DEG, PID_DEADBAND_BLEND_DEG)
        
        # 积分 (带限幅)
        self.integral += err * self.dt
        self.integral = clampf(self.integral, -120.0, 120.0)
        
        # 微分
        deriv = (err - self.prev_err) / self.dt if self.dt > 0 else 0.0
        self.prev_err = err
        
        # PID 计算
        output_raw = self.kp * err + self.ki * self.integral + self.kd * deriv
        
        # 输出滤波
        self.filtered_output += self.output_alpha * (output_raw - self.filtered_output)
        
        return self.filtered_output, err_raw


# ==================== 主控制系统 ====================

class SingleMotorPIDSystem:
    """单电机 PID 控制系统"""
    
    def __init__(self):
        """初始化系统"""
        print("\n" + "="*60)
        print("单电机 PID 角度控制系统")
        print("="*60 + "\n")
        
        # 初始化电机
        self.motor = SingleMotorControl(MOTOR_PWM_GPIO)
        
        # 初始化 PID
        self.pid = SimpleMotorPID(PID_KP, PID_KI, PID_KD)
        
        # 初始化反馈 (可选)
        self.feedback = None
        if ENABLE_FEEDBACK:
            try:
                self.feedback = FeedbackReader(PWM_INPUT_PIN)
            except Exception as e:
                print(f"⚠ 反馈初始化失败，将使用假设反馈: {e}")
                ENABLE_FEEDBACK = False
        
        # 初始化串口
        try:
            self.uart = UART(0, UART_BAUDRATE)
            print(f"✓ 串口初始化 (波特率 {UART_BAUDRATE})")
        except Exception as e:
            print(f"⚠ 串口初始化失败: {e}")
            self.uart = None
        
        self.running = False
        self.log_interval_ms = 200
        self.last_log_time = 0
        
        time.sleep(1)
        print("✓ 系统就绪\n")
    
    def read_command(self):
        """从串口读取目标角度"""
        if not self.uart or not self.uart.any():
            return None
        
        try:
            data = self.uart.readline()
            if data:
                cmd_str = data.decode().strip()
                try:
                    return float(cmd_str)
                except ValueError:
                    print(f"✗ 无效命令: {cmd_str}")
        except Exception as e:
            pass
        
        return None
    
    def run(self):
        """运行 PID 闭环控制"""
        print("="*60)
        print("运行 PID 控制")
        print("="*60)
        print("通过串口输入目标角度 (0-360°)")
        print("示例: 90 或 270.5")
        print("Ctrl+C 停止\n")
        
        self.running = True
        time.sleep(1)
        
        try:
            loop_count = 0
            while self.running:
                loop_count += 1
                loop_start = time.ticks_ms()
                
                # 读取串口命令
                cmd = self.read_command()
                if cmd is not None:
                    self.pid.set_target(cmd)
                    print(f">>> 新目标: {cmd:.1f}°")
                
                # 读取反馈
                if ENABLE_FEEDBACK and self.feedback:
                    valid, angle_deg = self.feedback.get_latest_angle()
                else:
                    # 无反馈时，假设电机完美执行
                    valid = self.pid.initialized
                    angle_deg = self.pid.target_deg
                
                # 初始化目标
                if valid and not self.pid.initialized:
                    self.pid.set_target(angle_deg)
                    print(f"初始目标: {angle_deg:.1f}°\n")
                
                # PID 更新
                if valid and self.pid.initialized:
                    u_filt, err_raw = self.pid.update(angle_deg)
                    
                    # 计算 PWM 脉宽
                    duty_cmd = int(MOTOR_PWM_NEUTRAL - u_filt)
                    self.motor.set_duty(duty_cmd)
                    
                    # 定期输出状态
                    now = time.ticks_ms()
                    if now - self.last_log_time >= self.log_interval_ms:
                        print(f"目标: {self.pid.target_deg:6.1f}° | "
                              f"当前: {angle_deg:6.1f}° | "
                              f"误差: {err_raw:6.1f}° | "
                              f"输出: {u_filt:6.1f} | "
                              f"PWM: {duty_cmd}μs")
                        self.last_log_time = now
                else:
                    self.motor.set_duty(MOTOR_PWM_NEUTRAL)
                
                # 周期控制
                elapsed = time.ticks_diff(time.ticks_ms(), loop_start)
                sleep_ms = max(1, PID_LOOP_MS - elapsed)
                time.sleep_ms(sleep_ms)
        
        except KeyboardInterrupt:
            print("\n✓ 已停止")
        finally:
            self.motor.set_duty(MOTOR_PWM_NEUTRAL)
            time.sleep(0.5)


# ==================== 主程序 ====================

def main():
    """主程序入口"""
    system = SingleMotorPIDSystem()
    system.run()


if __name__ == '__main__':
    main()
