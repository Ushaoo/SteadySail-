#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESP32 MicroPython - MPU6050 IMU 驱动
完整的寄存器读写实现，支持加速度和陀螺仪
"""

from machine import I2C
import time


class MPU6050:
    """MPU6050 IMU 传感器驱动 (MicroPython ESP32 版本)"""
    
    # MPU6050 寄存器地址
    PWR_MGMT_1 = 0x6B
    SMPLRT_DIV = 0x19
    CONFIG = 0x1A
    GYRO_CONFIG = 0x1B
    ACCEL_CONFIG = 0x1C
    ACCEL_XOUT_H = 0x3B
    ACCEL_XOUT_L = 0x3C
    ACCEL_YOUT_H = 0x3D
    ACCEL_YOUT_L = 0x3E
    ACCEL_ZOUT_H = 0x3F
    ACCEL_ZOUT_L = 0x40
    TEMP_OUT_H = 0x41
    TEMP_OUT_L = 0x42
    GYRO_XOUT_H = 0x43
    GYRO_XOUT_L = 0x44
    GYRO_YOUT_H = 0x45
    GYRO_YOUT_L = 0x46
    GYRO_ZOUT_H = 0x47
    GYRO_ZOUT_L = 0x48
    
    # 配置值
    ACCEL_SCALE = 16384.0  # 2g 范围
    GYRO_SCALE = 131.0     # 250°/s 范围
    
    def __init__(self, i2c, address=0x68):
        """
        初始化 MPU6050
        
        Args:
            i2c: machine.I2C 对象
            address: I2C 地址，默认 0x68
        """
        self.i2c = i2c
        self.address = address
        
        # 初始化传感器
        self._init_sensor()
    
    def _init_sensor(self):
        """初始化传感器配置"""
        try:
            # 唤醒设备 (清除睡眠位)
            self.write_byte(self.PWR_MGMT_1, 0x00)
            time.sleep_ms(100)
            
            # 设置采样率分频 (采样率 = 1000 / (1 + SMPLRT_DIV))
            self.write_byte(self.SMPLRT_DIV, 0x07)  # 125 Hz
            
            # 设置低通滤波
            self.write_byte(self.CONFIG, 0x06)  # 6: 5Hz LPF
            
            # 设置陀螺仪量程 (±250°/s)
            self.write_byte(self.GYRO_CONFIG, 0x00)
            
            # 设置加速度量程 (±2g)
            self.write_byte(self.ACCEL_CONFIG, 0x00)
            
            print(f"MPU6050 @ 0x{self.address:02X} 初始化成功")
            
        except Exception as e:
            print(f"MPU6050 初始化失败: {e}")
    
    def write_byte(self, reg, value):
        """写入单个字节到寄存器"""
        self.i2c.writeto_mem(self.address, reg, bytes([value]))
    
    def read_byte(self, reg):
        """读取单个字节从寄存器"""
        return self.i2c.readfrom_mem(self.address, reg, 1)[0]
    
    def read_word(self, reg_high, reg_low):
        """读取 16-bit 数据 (高字节在前)"""
        high = self.read_byte(reg_high)
        low = self.read_byte(reg_low)
        value = (high << 8) | low
        
        # 转换为有符号整数
        if value & 0x8000:
            value = value - 65536
        
        return value
    
    def get_accel(self):
        """
        获取加速度数据 (单位: m/s²)
        
        Returns:
            dict: {'x': float, 'y': float, 'z': float}
        """
        try:
            accel_x_raw = self.read_word(self.ACCEL_XOUT_H, self.ACCEL_XOUT_L)
            accel_y_raw = self.read_word(self.ACCEL_YOUT_H, self.ACCEL_YOUT_L)
            accel_z_raw = self.read_word(self.ACCEL_ZOUT_H, self.ACCEL_ZOUT_L)
            
            # 转换为 g，然后转换为 m/s²
            accel_x = (accel_x_raw / self.ACCEL_SCALE) * 9.81
            accel_y = (accel_y_raw / self.ACCEL_SCALE) * 9.81
            accel_z = (accel_z_raw / self.ACCEL_SCALE) * 9.81
            
            return {
                'x': accel_x,
                'y': accel_y,
                'z': accel_z
            }
        except Exception as e:
            print(f"读取加速度失败: {e}")
            return {'x': 0, 'y': 0, 'z': 0}
    
    def get_gyro(self):
        """
        获取陀螺仪数据 (单位: °/s)
        
        Returns:
            dict: {'x': float, 'y': float, 'z': float}
        """
        try:
            gyro_x_raw = self.read_word(self.GYRO_XOUT_H, self.GYRO_XOUT_L)
            gyro_y_raw = self.read_word(self.GYRO_YOUT_H, self.GYRO_YOUT_L)
            gyro_z_raw = self.read_word(self.GYRO_ZOUT_H, self.GYRO_ZOUT_L)
            
            # 转换为 °/s
            gyro_x = gyro_x_raw / self.GYRO_SCALE
            gyro_y = gyro_y_raw / self.GYRO_SCALE
            gyro_z = gyro_z_raw / self.GYRO_SCALE
            
            return {
                'x': gyro_x,
                'y': gyro_y,
                'z': gyro_z
            }
        except Exception as e:
            print(f"读取陀螺仪失败: {e}")
            return {'x': 0, 'y': 0, 'z': 0}
    
    def get_temp(self):
        """
        获取温度 (单位: °C)
        
        Returns:
            float: 温度值
        """
        try:
            temp_raw = self.read_word(self.TEMP_OUT_H, self.TEMP_OUT_L)
            # 公式: 温度 = 36.53 + temp_raw / 340
            temp = 36.53 + (temp_raw / 340.0)
            return temp
        except Exception as e:
            print(f"读取温度失败: {e}")
            return 0.0
    
    def get_all(self):
        """
        获取所有数据 (加速度 + 陀螺仪 + 温度)
        
        Returns:
            dict: 包含所有传感器数据
        """
        return {
            'accel': self.get_accel(),
            'gyro': self.get_gyro(),
            'temp': self.get_temp()
        }
