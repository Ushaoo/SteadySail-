#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
单电机 + PWM 反馈 PID 角度控制
用途：通过 PWM 反馈信号实现精确的电机角度控制（如同 my_project 中的实现）

硬件连线：
  GPIO2      → 电机 PWM 控制信号（输出）
  GPIO4      → 电机 PWM 反馈信号（输入）
  GND        → 电机 GND
  80
电机反馈原理：
  - PWM 周期固定 20ms (50Hz)
  - 脉宽：1000-2000μs 对应 0-360° 旋转角度
  - GPIO4 捕获脉宽，计算当前角度
"""

import time
import math
from machine import Pin, PWM, UART

# ==================== 配置参数 ====================

# 电机 PWM 控制 (GPIO2 输出)
MOTOR_PWM_GPIO = 2
MOTOR_PWM_FREQ_HZ = 50
MOTOR_PWM_NEUTRAL = 1400         # 中立脉宽 (μs)
MOTOR_PWM_MAX = 1480             # 最大脉宽 (μs)
MOTOR_PWM_MIN = 1320             # 最小脉宽 (μs)

# 反馈信号捕获 (GPIO4 输入)
FEEDBACK_PIN = 4
FEEDBACK_MIN_PERIOD_US = 200
FEEDBACK_MAX_PERIOD_US = 200000
FEEDBACK_STALE_US = 300000       # 超过此时间无数据则认为失效

# PID 控制参数
PID_LOOP_MS = 10
PID_KP = 9.8
PID_KI = 1.0
PID_KD = 0.49
PID_DEADBAND_DEG = 3.0
PID_DEADBAND_BLEND_DEG = 3.0
PID_OUTPUT_FILTER_TAU_S = 0.12

# 自动旋转测试配置
TEST_MODE_ENABLED = True         # 启用自动测试模式
TEST_STEP_DURATION_MS = 3000     # 每个步骤持续时间 (ms)
# 测试序列：正20°→逆40°→正40°→逆40°→循环
TEST_SEQUENCE = [
    20,      # 正旋20度
    340,     # 逆旋40度 (20-40=340)
    20,      # 正旋40度 (340+40=20)
    340,     # 逆旋40度 (20-40=340)
]

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
    
    x = abs_err - deadband
    t = x / blend
    s = t * t * (3.0 - 2.0 * t)  # smoothstep
    return sign * (x * s)

def compute_angle_from_pwm(period_us, high_us):
    """从 PWM 脉宽计算电机角度 (与 pwm_capture.c 同逻辑)
    
    参数:
        period_us: PWM 周期 (μs)
        high_us: PWM 高电平宽度 (μs)
        
    返回:
        角度 (0-360°)
    """
    if period_us <= 0:
        return 0.0
    
    duty_raw = ((float(high_us) / float(period_us)) * 4119.0) - 16.0
    angle = duty_raw * (360.0 / 4095.0)
    return clampf(angle, 0.0, 360.0)


# ==================== PWM 反馈捕获类 ====================

class PWMCapture:
    """PWM 反馈信号捕获 (GPIO4 输入) - 轮询方式（改进版，接近 my_project C 实现）"""
    
    def __init__(self, gpio_pin):
        """初始化 PWM 捕获
        
        参数:
            gpio_pin: GPIO 引脚号 (输入)
        """
        self.pin = Pin(gpio_pin, Pin.IN, Pin.PULL_UP)
        self.period_us = 0
        self.high_us = 0
        self.last_update_time_us = 0
        self.last_rise_time_us = 0
        self.read_count = 0
        
        # 【改进】添加与 C 版本相同的稳定性检查
        self.last_edge_us = 0
        self.prev_period_us = 0
        self.current_period_us = 0
        self.rise_time_us = 0
        
        print(f"✓ PWM 反馈捕获初始化 (GPIO {gpio_pin})")
    
    def try_read_pwm(self):
        """尝试读取一个 PWM 周期（改进版，与 my_project C 实现逻辑相同）
        
        返回: True 表示成功读取
        """
        # 更长的超时：50ms（匹配实际 PWM 周期 ~30-40ms）
        timeout_us = 50000
        CAPTURE_EDGE_GLITCH_US = 10  # 毛刺阈值 (与 C 版本相同)
        
        # 等待下降沿（最多 5ms）
        if self.pin.value() == 1:
            start = time.ticks_us()
            while self.pin.value() == 1:
                if time.ticks_diff(time.ticks_us(), start) > timeout_us:
                    return False
        
        # 等待上升沿（最多 5ms）
        start = time.ticks_us()
        while self.pin.value() == 0:
            if time.ticks_diff(time.ticks_us(), start) > timeout_us:
                return False
        
        now_us = time.ticks_us()
        
        # 【改进】毛刺消除（与 C 版本相同）
        if self.last_edge_us != 0 and time.ticks_diff(now_us, self.last_edge_us) < CAPTURE_EDGE_GLITCH_US:
            self.last_edge_us = now_us
            return False
        self.last_edge_us = now_us
        
        # 【改进】上升沿处理（与 C 版本相同的周期计算逻辑）
        if self.last_rise_time_us != 0:
            period = time.ticks_diff(now_us, self.last_rise_time_us)
            
            # 【改进】周期范围检查
            if FEEDBACK_MIN_PERIOD_US <= period <= FEEDBACK_MAX_PERIOD_US:
                # 【改进】周期稳定性检查（与 C 版本相同的 sanity check）
                if self.prev_period_us == 0 or \
                   (period >= (self.prev_period_us // 2) and period <= (self.prev_period_us * 2)):
                    self.current_period_us = period
                    self.prev_period_us = period
        
        self.last_rise_time_us = now_us
        self.rise_time_us = now_us
        
        # 等待下降沿（最多 5ms）
        start = time.ticks_us()
        while self.pin.value() == 1:
            if time.ticks_diff(time.ticks_us(), start) > timeout_us:
                return False
        
        now_us = time.ticks_us()
        
        # 【改进】毛刺消除
        if self.last_edge_us != 0 and time.ticks_diff(now_us, self.last_edge_us) < CAPTURE_EDGE_GLITCH_US:
            self.last_edge_us = now_us
            return False
        self.last_edge_us = now_us
        
        # 【改进】下降沿处理（与 C 版本相同的脉宽计算）
        if self.rise_time_us != 0 and self.current_period_us != 0:
            h = time.ticks_diff(now_us, self.rise_time_us)
            
            # 【改进】脉宽范围检查（与 C 版本相同）
            if 10 <= h <= self.current_period_us:  # CAPTURE_MIN_HIGH_US = 10μs
                self.period_us = self.current_period_us
                self.high_us = h
                self.last_update_time_us = time.ticks_us()
                self.read_count += 1
                return True
        
        return False
    
    def get_latest_angle(self):
        """读取最新角度（非阻塞）
        
        返回: (有效, 角度, 周期_us, 脉宽_us)
        """
        # 检查数据是否过期
        now_us = time.ticks_us()
        if self.last_update_time_us == 0 or \
           time.ticks_diff(now_us, self.last_update_time_us) > FEEDBACK_STALE_US:
            return False, 0.0, 0, 0
        
        if self.period_us <= 0 or self.high_us <= 0:
            return False, 0.0, 0, 0
        
        # 【调试】输出原始 PWM 参数
        # print(f"[DEBUG] 周期={self.period_us}μs, 脉宽={self.high_us}μs")
        
        # 计算角度
        angle = compute_angle_from_pwm(self.period_us, self.high_us)
        return True, angle, self.period_us, self.high_us


# ==================== 电机控制类 ====================

class MotorControl:
    """电机 PWM 控制"""
    
    def __init__(self, gpio_pin):
        """初始化电机
        
        参数:
            gpio_pin: GPIO 引脚号 (输出)
        """
        try:
            self.pin = Pin(gpio_pin, Pin.OUT)
            self.pwm = PWM(self.pin, freq=MOTOR_PWM_FREQ_HZ, duty_u16=0)
            self.set_duty(MOTOR_PWM_NEUTRAL)
            print(f"✓ 电机控制初始化 (GPIO {gpio_pin})")
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
        
        duty_us = clampf(duty_us, MOTOR_PWM_MIN, MOTOR_PWM_MAX)
        duty_u16 = int((duty_us / 20000.0) * 65535)
        self.pwm.duty_u16(duty_u16)


# ==================== PID 控制器 ====================

class MotorPID:
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
        
        print(f"✓ PID 初始化: Kp={kp}, Ki={ki}, Kd={kd}")
    
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
            (control_output, error_raw)
        """
        if not self.initialized:
            return 0.0, 0.0
        
        # 计算误差
        err_raw = shortest_angle_error(self.target_deg, current_deg)
        err = apply_smooth_deadband(err_raw, PID_DEADBAND_DEG, PID_DEADBAND_BLEND_DEG)
        
        # 积分
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


# ==================== 主系统类 ====================

class MotorPWMFeedbackSystem:
    """电机 + PWM 反馈 PID 控制系统"""
    
    def __init__(self):
        """初始化系统"""
        print("\n" + "="*60)
        print("电机 + PWM 反馈 PID 控制系统")
        print("="*60 + "\n")
        
        # 初始化反馈捕获
        self.feedback = PWMCapture(FEEDBACK_PIN)
        
        # 初始化电机
        self.motor = MotorControl(MOTOR_PWM_GPIO)
        
        # 初始化 PID
        self.pid = MotorPID(PID_KP, PID_KI, PID_KD)
        
        # 初始化串口
        self.uart = None
        try:
            self.uart = UART(0, UART_BAUDRATE)
            print(f"✓ 串口初始化 (波特率 {UART_BAUDRATE})")
        except Exception as e:
            print(f"⚠ 串口初始化失败: {e}")
            self.uart = None
        
        self.last_log_time = 0
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
        except:
            pass
        
        return None
    
    def run(self):
        """运行控制循环"""
        print("="*60)
        print("启动 PID 反馈控制")
        print("="*60)
        
        if TEST_MODE_ENABLED:
            print("\n【自动测试模式】")
            print("测试序列：正旋20° → 逆旋40° → 正旋40° → 逆旋40° → 循环")
            print("="*60 + "\n")
        else:
            print("通过串口输入目标角度 (0-360°)")
            print("示例: 90 或 270.5")
            print("Ctrl+C 停止\n")
        
        time.sleep(1)
        
        try:
            loop_count = 0
            last_read_count_logged = 0
            last_pwm_read_time = 0
            
            # 测试模式相关变量
            test_sequence_index = 0
            test_last_change_time = time.ticks_ms()
            
            while True:
                loop_count += 1
                loop_start = time.ticks_ms()
                
                # 【自动测试模式】每隔一段时间切换目标角度
                if TEST_MODE_ENABLED:
                    now_ms = time.ticks_ms()
                    if time.ticks_diff(now_ms, test_last_change_time) >= TEST_STEP_DURATION_MS:
                        # 切换到下一个目标
                        test_sequence_index = (test_sequence_index + 1) % len(TEST_SEQUENCE)
                        target_angle = TEST_SEQUENCE[test_sequence_index]
                        self.pid.set_target(target_angle)
                        
                        # 显示测试进度
                        direction = "正" if target_angle <= 180 else "逆"
                        print(f"\n>>> 测试步骤 {test_sequence_index+1}: {direction}旋 → 目标 {target_angle}°")
                        
                        test_last_change_time = now_ms
                
                # 【关键】每 50ms 才轮询一次 PWM 信号
                # 这样主循环能快速响应 Shell 输入（~10ms 一个循环）
                now_ms = time.ticks_ms()
                if time.ticks_diff(now_ms, last_pwm_read_time) >= 50:
                    # 尝试读取 PWM 反馈信号（快速超时 5ms）
                    self.feedback.try_read_pwm()
                    last_pwm_read_time = now_ms
                
                # 读取串口命令（测试模式下禁用）
                if not TEST_MODE_ENABLED:
                    cmd = self.read_command()
                    if cmd is not None:
                        self.pid.set_target(cmd)
                        print(f">>> 新目标: {cmd:.1f}°")
                
                # 读取反馈角度
                valid, angle_deg, period_us, high_us = self.feedback.get_latest_angle()
                
                # 初始化目标
                if loop_count == 100 and not self.pid.initialized:
                    if TEST_MODE_ENABLED:
                        # 测试模式：使用第一个测试目标
                        self.pid.set_target(TEST_SEQUENCE[0])
                        print(f"初始目标: {TEST_SEQUENCE[0]:.1f}° (测试模式)\n")
                    elif valid:
                        self.pid.set_target(angle_deg)
                        print(f"初始目标: {angle_deg:.1f}°\n")
                    else:
                        self.pid.set_target(90.0)
                        print(f"初始目标: 90.0° (无反馈，使用默认值)\n")
                
                # PID 更新
                if self.pid.initialized:
                    u_filt, err_raw = self.pid.update(angle_deg)
                    
                    # 计算 PWM 脉宽
                    duty_cmd = int(MOTOR_PWM_NEUTRAL - u_filt)
                    self.motor.set_duty(duty_cmd)
                    
                    # 定期输出状态和调试信息
                    now = time.ticks_ms()
                    if now - self.last_log_time >= 200:
                        status = "✓" if valid else "✗"
                        read_delta = self.feedback.read_count - last_read_count_logged
                        print(f"{status} 目标: {self.pid.target_deg:6.1f}° | "
                              f"当前: {angle_deg:6.1f}° | "
                              f"误差: {err_raw:6.1f}° | "
                              f"输出: {u_filt:6.1f} | "
                              f"PWM: {duty_cmd:4d}μs | "
                              f"周期: {period_us:5d}μs | "
                              f"脉宽: {high_us:4d}μs | "
                              f"读取: +{read_delta}")
                        self.last_log_time = now
                        last_read_count_logged = self.feedback.read_count
                else:
                    self.motor.set_duty(MOTOR_PWM_NEUTRAL)
                
                # 周期控制（尽量保持 10ms）
                elapsed = time.ticks_diff(time.ticks_ms(), loop_start)
                # 如果 PWM 轮询占用了很多时间，就跳过这个周期的 sleep
                if elapsed < PID_LOOP_MS:
                    sleep_ms = PID_LOOP_MS - elapsed
                    time.sleep_ms(sleep_ms)
        
        except KeyboardInterrupt:
            print("\n✓ 已停止")
        finally:
            self.motor.set_duty(MOTOR_PWM_NEUTRAL)
            time.sleep(0.5)
    


# ==================== 主程序 ====================

def main():
    """主程序"""
    system = MotorPWMFeedbackSystem()
    system.run()


if __name__ == '__main__':
    main()
