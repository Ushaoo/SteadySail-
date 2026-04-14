#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
单电机 + 磁编码器 PID 角度控制
用途：通过磁编码器反馈实现精确的电机角度控制

硬件连线：
  GPIO2      → 电机 PWM 信号
  GND        → 电机 GND + 磁编码器 GND
  3.3V       → 磁编码器 VCC
  GPIO21(SDA)→ 磁编码器 SDA (I2C)
  GPIO22(SCL)→ 磁编码器 SCL (I2C)
  
磁编码器类型: AS5600 (I2C 地址 0x36)
"""

import time
import math
from machine import I2C, Pin, PWM, UART

# ==================== 配置参数 ====================

# 电机 PWM 配置
MOTOR_PWM_GPIO = 2              # PWM 输出引脚
MOTOR_PWM_FREQ_HZ = 50          # 50Hz 伺服频率
MOTOR_PWM_NEUTRAL = 1400        # 中立脉宽 (μs)
MOTOR_PWM_MAX = 1480            # 最大脉宽 (μs)
MOTOR_PWM_MIN = 1320            # 最小脉宽 (μs)

# I2C 配置 (磁编码器)
# ESP32-S3 I2C1: GPIO 18 (SDA), GPIO 17 (SCL) - 尝试多个配置
I2C_SCL_PIN = 9       # GPIO 9 (SCL) - I2C0 默认
I2C_SDA_PIN = 8       # GPIO 8 (SDA) - I2C0 默认
I2C_FREQ = 400000
ENCODER_I2C_ADDRESS = 0x36      # AS5600/MT6826S 默认地址 (或 0x76 如需调整)

# PID 控制参数
PID_LOOP_MS = 10
PID_KP = 8.0
PID_KI = 0.8
PID_KD = 0.4
PID_DEADBAND_DEG = 2.0
PID_DEADBAND_BLEND_DEG = 2.0
PID_OUTPUT_FILTER_TAU_S = 0.1

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


# ==================== 磁编码器类 ====================

class AS5600Encoder:
    """AS5600 磁编码器驱动 (I2C)"""
    
    def __init__(self, i2c, address=0x36):
        """初始化编码器
        
        参数:
            i2c: I2C 对象
            address: I2C 地址 (默认 0x36)
        """
        self.i2c = i2c
        self.addr = address
        self.angle_raw = 0
        
        try:
            # 测试连接
            self.i2c.scan()
            print(f"✓ 磁编码器初始化 (地址 0x{self.addr:02X})")
        except Exception as e:
            print(f"✗ 磁编码器初始化失败: {e}")
    
    def read_angle(self):
        """读取旋转角度 (0-360°)
        
        返回: 角度 (度)
        """
        try:
            # AS5600 角度寄存器: 0x0E (2 字节，高字节在前)
            data = self.i2c.readfrom_mem(self.addr, 0x0E, 2)
            # 12 位数据，范围 0-4095 对应 0-360°
            raw_angle = ((data[0] << 8) | data[1]) >> 4
            angle_deg = (raw_angle / 4095.0) * 360.0
            self.angle_raw = raw_angle
            return angle_deg
        except Exception as e:
            print(f"读取编码器失败: {e}")
            return 0.0


# ==================== 电机控制类 ====================

class MotorControl:
    """电机 PWM 控制"""
    
    def __init__(self, gpio_pin):
        """初始化电机
        
        参数:
            gpio_pin: GPIO 引脚号
        """
        try:
            self.pin = Pin(gpio_pin, Pin.OUT)
            self.pwm = PWM(self.pin, freq=MOTOR_PWM_FREQ_HZ, duty_u16=0)
            
            # 计算 PWM 参数 (duty_u16 范围: 0-65535)
            self.neutral_u16 = int((MOTOR_PWM_NEUTRAL / 20000.0) * 65535)
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
            control_output: 控制输出
        """
        if not self.initialized:
            return 0.0
        
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

class MotorEncoderSystem:
    """电机 + 磁编码器 PID 控制系统"""
    
    def __init__(self):
        """初始化系统"""
        print("\n" + "="*60)
        print("电机 + 磁编码器 PID 控制系统")
        print("="*60 + "\n")
        
        # 初始化 I2C (可选，失败时继续以开环模式)
        self.i2c = None
        try:
            self.i2c = I2C(0, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=I2C_FREQ)
            print(f"✓ I2C 初始化完成 (GPIO{I2C_SDA_PIN}/GPIO{I2C_SCL_PIN})")
        except Exception as e:
            print(f"⚠ I2C 初始化失败: {e}")
            print(f"  (GPIO21/GPIO22 可能未正确连接，改用开环模式)")
            print(f"  提示: ESP32-S3 I2C0 可能使用 GPIO8(SDA)/GPIO9(SCL)")
            self.i2c = None
        
        # 初始化编码器 (可选)
        self.encoder = None
        if self.i2c:
            self.encoder = AS5600Encoder(self.i2c, ENCODER_I2C_ADDRESS)
        else:
            print(f"⚠ 磁编码器跳过（I2C 不可用）")
        
        # 初始化电机 (必须成功)
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
        print("启动 PID 控制")
        print("="*60)
        
        if self.encoder:
            print("模式: 闭环 PID（有磁编码器反馈）")
        else:
            print("模式: 开环测试（无磁编码器反馈）")
        
        print("通过串口输入目标角度 (0-360°)")
        print("示例: 90 或 270.5")
        print("Ctrl+C 停止\n")
        
        time.sleep(1)
        
        try:
            loop_count = 0
            while True:
                loop_count += 1
                loop_start = time.ticks_ms()
                
                # 读取串口命令
                cmd = self.read_command()
                if cmd is not None:
                    self.pid.set_target(cmd)
                    print(f">>> 新目标: {cmd:.1f}°")
                
                # 读取编码器角度 (如果可用)
                if self.encoder:
                    angle_deg = self.encoder.read_angle()
                else:
                    angle_deg = 0.0
                
                # 初始化目标
                if loop_count == 100 and not self.pid.initialized:
                    self.pid.set_target(angle_deg if self.encoder else 90.0)
                    print(f"初始目标: {self.pid.target_deg:.1f}°\n")
                
                # PID 更新
                if self.pid.initialized:
                    u_filt, err_raw = self.pid.update(angle_deg)
                    
                    # 计算 PWM 脉宽
                    duty_cmd = int(MOTOR_PWM_NEUTRAL - u_filt)
                    self.motor.set_duty(duty_cmd)
                    
                    # 定期输出状态
                    now = time.ticks_ms()
                    if now - self.last_log_time >= 200:
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
    """主程序"""
    system = MotorEncoderSystem()
    system.run()


if __name__ == '__main__':
    main()
