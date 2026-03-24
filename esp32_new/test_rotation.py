#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
推进器旋转控制测试程序 (阶段1.1 测试)

用途：验证旋转舵机硬件连接和旋转角度映射
不影响主控制程序

使用方法：
1. 连接旋转舵机到 PCA9685 通道 2, 3
2. 运行本程序测试
3. 观察舵机是否正常旋转
"""

import time
import math
from machine import I2C, Pin

# 导入库
try:
    from lib.pca9685 import PCA9685
except ImportError:
    print("Error: PCA9685 库未找到")
    exit()

# I2C 配置
I2C_SCL_PIN = 22
I2C_SDA_PIN = 21
I2C_FREQ = 400000


class RotationTestController:
    """旋转控制测试类 - 独立运行，不需要完整的 IMU 和平衡控制"""
    
    def __init__(self):
        print("初始化 I2C 和 PCA9685...")
        self.i2c = I2C(1, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=I2C_FREQ)
        
        try:
            self.pwm = PCA9685(self.i2c, address=0x40)
            self.pwm.setPWMFreq(50)
            print("✓ PCA9685 初始化成功")
        except Exception as e:
            print(f"✗ PCA9685 初始化失败: {e}")
            self.pwm = None
            return
        
        # 旋转参数
        self.left_channel = 2
        self.right_channel = 3
        self.max_angle = 45.0
        self.pulse_min = 1000
        self.pulse_max = 2000
        self.pulse_center = 1500
    
    def angle_to_pulse(self, angle_deg):
        """将旋转角转换为脉宽"""
        angle_deg = max(-self.max_angle, min(self.max_angle, angle_deg))
        pulse = self.pulse_center + (angle_deg / self.max_angle) * 500
        return int(pulse)
    
    def set_angle(self, angle_left, angle_right):
        """设置旋转角"""
        if not self.pwm:
            return False
        
        pulse_left = self.angle_to_pulse(angle_left)
        pulse_right = self.angle_to_pulse(angle_right)
        
        try:
            self.pwm.setServoPulse(self.left_channel, pulse_left)
            self.pwm.setServoPulse(self.right_channel, pulse_right)
            print(f"✓ 设置旋转: L={angle_left:6.1f}° ({pulse_left}μs) | R={angle_right:6.1f}° ({pulse_right}μs)")
            return True
        except Exception as e:
            print(f"✗ 设置失败: {e}")
            return False
    
    def test_sequence_1_neutral(self):
        """测试 1: 中立位置"""
        print("\n--- 测试 1: 中立位置 (0°) ---")
        self.set_angle(0.0, 0.0)
        time.sleep(1)
    
    def test_sequence_2_sweep(self):
        """测试 2: 扫描范围"""
        print("\n--- 测试 2: 扫描 -45° 到 +45° ---")
        for angle in range(-45, 46, 15):
            self.set_angle(float(angle), float(angle))
            time.sleep(0.5)
        self.set_angle(0.0, 0.0)
        time.sleep(0.5)
    
    def test_sequence_3_differential(self):
        """测试 3: 差异旋转 (模拟转矩)"""
        print("\n--- 测试 3: 差异旋转 (模拟转矩控制) ---")
        print("左旋转 +30°, 右旋转 -30°")
        self.set_angle(30.0, -30.0)
        time.sleep(1)
        
        print("交换方向")
        self.set_angle(-30.0, 30.0)
        time.sleep(1)
        
        print("回到中立")
        self.set_angle(0.0, 0.0)
        time.sleep(0.5)
    
    def test_sequence_4_sine_wave(self):
        """测试 4: 正弦波扫描"""
        print("\n--- 测试 4: 正弦波扫描 ---")
        for i in range(100):
            angle = 30.0 * math.sin(i * 0.1)
            self.set_angle(angle, -angle)
            time.sleep(0.05)
        self.set_angle(0.0, 0.0)
        time.sleep(0.5)
    
    def test_sequence_5_speed_test(self):
        """测试 5: 旋转速度测试"""
        print("\n--- 测试 5: 快速响应测试 ---")
        for _ in range(5):
            self.set_angle(45.0, 45.0)
            time.sleep(0.2)
            self.set_angle(-45.0, -45.0)
            time.sleep(0.2)
        self.set_angle(0.0, 0.0)
        time.sleep(0.5)
    
    def run_full_test(self):
        """运行完整测试序列"""
        print("\n" + "="*60)
        print("ESP32 推进器旋转控制 - 阶段1.1 测试")
        print("="*60)
        
        if not self.pwm:
            print("✗ PWM 控制器未初始化，无法继续")
            return
        
        try:
            # 运行所有测试
            self.test_sequence_1_neutral()
            time.sleep(1)
            
            self.test_sequence_2_sweep()
            time.sleep(1)
            
            self.test_sequence_3_differential()
            time.sleep(1)
            
            self.test_sequence_4_sine_wave()
            time.sleep(1)
            
            self.test_sequence_5_speed_test()
            time.sleep(1)
            
            print("\n" + "="*60)
            print("✓ 测试完成！所有舵机应该能正常工作")
            print("="*60 + "\n")
            
        except KeyboardInterrupt:
            print("\n✓ 测试被中断")
            self.set_angle(0.0, 0.0)


def main():
    """主程序"""
    test = RotationTestController()
    test.run_full_test()


if __name__ == '__main__':
    main()
