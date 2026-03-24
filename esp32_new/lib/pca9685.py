#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESP32 MicroPython - PCA9685 PWM 驱动
用于控制伺服电机和 ESC
完整的寄存器操作实现
"""

from machine import I2C
import time
import math


class PCA9685:
    """PCA9685 16-Channel PWM Driver (MicroPython ESP32 版本)"""
    
    # 寄存器地址
    MODE1 = 0x00
    MODE2 = 0x01
    SUBADR1 = 0x02
    SUBADR2 = 0x03
    SUBADR3 = 0x04
    LED0_ON_L = 0x06
    LED0_ON_H = 0x07
    LED0_OFF_L = 0x08
    LED0_OFF_H = 0x09
    ALL_LED_ON_L = 0xFA
    ALL_LED_ON_H = 0xFB
    ALL_LED_OFF_L = 0xFC
    ALL_LED_OFF_H = 0xFD
    PRESCALE = 0xFE
    
    def __init__(self, i2c, address=0x40, freq=50):
        """
        初始化 PCA9685
        
        Args:
            i2c: machine.I2C 对象
            address: I2C 地址，默认 0x40
            freq: PWM 频率 (Hz)，默认 50 Hz
        """
        self.i2c = i2c
        self.address = address
        self.freq = freq
        
        # 初始化设备
        self._init_device()
    
    def _init_device(self):
        """初始化 PCA9685 设备"""
        try:
            # 重置设备
            self.write_byte(self.MODE1, 0x00)
            time.sleep_ms(50)
            
            # 设置 PWM 频率
            self.setPWMFreq(self.freq)
            
            # 设置 MODE2
            self.write_byte(self.MODE2, 0x04)  # OUTDRV = 1 (推挽输出)
            
            print(f"PCA9685 @ 0x{self.address:02X} 初始化成功 (freq={self.freq}Hz)")
            
        except Exception as e:
            print(f"PCA9685 初始化失败: {e}")
    
    def write_byte(self, reg, value):
        """写入单个字节到寄存器"""
        self.i2c.writeto_mem(self.address, reg, bytes([value]))
    
    def read_byte(self, reg):
        """读取单个字节从寄存器"""
        return self.i2c.readfrom_mem(self.address, reg, 1)[0]
    
    def setPWMFreq(self, freq):
        """
        设置 PWM 频率
        
        Args:
            freq: 频率 (Hz)，范围 24-1526 Hz
        """
        try:
            # 计算 prescale 值
            # prescale = round(25MHz / (4096 * freq)) - 1
            prescale_value = int(round(25000000.0 / (4096.0 * freq)) - 1)
            
            # 限制范围
            if prescale_value < 0x03:
                prescale_value = 0x03
            elif prescale_value > 0xFF:
                prescale_value = 0xFF
            
            # 读取当前 MODE1
            old_mode = self.read_byte(self.MODE1)
            
            # 设置睡眠位以改变 prescale
            new_mode = (old_mode & 0x7F) | 0x10  # 清除 RESTART，设置 SLEEP
            self.write_byte(self.MODE1, new_mode)
            
            # 写入 prescale 值
            self.write_byte(self.PRESCALE, prescale_value)
            
            # 恢复 MODE1
            self.write_byte(self.MODE1, old_mode)
            time.sleep_ms(5)
            
            # 设置 RESTART 位
            self.write_byte(self.MODE1, old_mode | 0x80)
            
            self.freq = freq
            print(f"PWM 频率已设置为 {freq} Hz (prescale={prescale_value})")
            
        except Exception as e:
            print(f"设置 PWM 频率失败: {e}")
    
    def setServoPulse(self, channel, pulse):
        """
        设置伺服脉宽
        
        Args:
            channel: 通道 (0-15)
            pulse: 脉宽 (微秒)，范围 1000-2000
        
        说明:
            - 1000 μs = 0° (或最小)
            - 1500 μs = 中立
            - 2000 μs = 180° (或最大)
        """
        try:
            # 计算对应的 4096 分度值
            # tick = (pulse / 20000) * 4096 (对于 50Hz)
            # 或更通用: tick = (pulse * freq / 1000000) * 4096
            duty = int((pulse * self.freq / 1000000.0) * 4096)
            
            # 限制范围
            if duty < 0:
                duty = 0
            elif duty > 4095:
                duty = 4095
            
            # 计算 ON 和 OFF 时间
            # ON 时间通常从 0 开始
            on = 0
            off = duty
            
            # 设置通道的 PWM
            self.setDutyCycle(channel, on, off)
            
        except Exception as e:
            print(f"设置伺服脉宽失败 (ch={channel}, pulse={pulse}): {e}")
    
    def setPWM(self, channel, on, off):
        """
        直接设置 PWM 的 ON 和 OFF 计数
        
        Args:
            channel: 通道 (0-15)
            on: ON 计数 (0-4095)
            off: OFF 计数 (0-4095)
        """
        try:
            # 计算寄存器地址
            base_reg = self.LED0_ON_L + (channel * 4)
            
            # LED0_ON_L
            self.write_byte(base_reg, on & 0xFF)
            # LED0_ON_H
            self.write_byte(base_reg + 1, (on >> 8) & 0x1F)
            # LED0_OFF_L
            self.write_byte(base_reg + 2, off & 0xFF)
            # LED0_OFF_H
            self.write_byte(base_reg + 3, (off >> 8) & 0x1F)
            
        except Exception as e:
            print(f"setPWM 失败: {e}")
    
    def setDutyCycle(self, channel, on, off):
        """
        设置 PWM 占空比
        
        Args:
            channel: 通道 (0-15)
            on: ON 计数 (0-4095)
            off: OFF 计数 (0-4095)
        """
        self.setPWM(channel, on, off)
    
    def setAllPWM(self, on, off):
        """
        同时设置所有通道的 PWM
        
        Args:
            on: ON 计数 (0-4095)
            off: OFF 计数 (0-4095)
        """
        try:
            self.write_byte(self.ALL_LED_ON_L, on & 0xFF)
            self.write_byte(self.ALL_LED_ON_H, (on >> 8) & 0x1F)
            self.write_byte(self.ALL_LED_OFF_L, off & 0xFF)
            self.write_byte(self.ALL_LED_OFF_H, (off >> 8) & 0x1F)
        except Exception as e:
            print(f"setAllPWM 失败: {e}")
