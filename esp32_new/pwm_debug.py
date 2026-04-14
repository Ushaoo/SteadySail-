#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PWM 反馈诊断脚本 - 查看原始 PWM 数据（简化版）
"""

import time
from machine import Pin

# 配置
FEEDBACK_PIN = 4

class PWMDebug:
    """调试 PWM 捕获"""
    
    def __init__(self, gpio_pin):
        self.pin = Pin(gpio_pin, Pin.IN, Pin.PULL_UP)
        self.last_rise_time_us = 0
        self.period_us = 0
        self.high_us = 0
        print(f"✓ PWM 调试工具初始化 (GPIO {gpio_pin})")
    
    def read_one_cycle(self):
        """读取一个完整 PWM 周期"""
        timeout_us = 30000  # 30ms
        
        # 等待下降沿
        if self.pin.value() == 1:
            start = time.ticks_us()
            count = 0
            while self.pin.value() == 1:
                count += 1
                if count > 100000:  # 简单的超时机制
                    return False
        
        # 等待上升沿
        start = time.ticks_us()
        count = 0
        while self.pin.value() == 0:
            count += 1
            if count > 100000:
                return False
        
        rise_time = time.ticks_us()
        
        # 计算周期
        if self.last_rise_time_us > 0:
            period = rise_time - self.last_rise_time_us
        else:
            period = 0
        
        self.last_rise_time_us = rise_time
        
        # 等待下降沿
        count = 0
        while self.pin.value() == 1:
            count += 1
            if count > 100000:
                return False
        
        fall_time = time.ticks_us()
        high = fall_time - rise_time
        
        # 简单验证
        if period > 100 and high > 0 and high < period:
            self.period_us = period
            self.high_us = high
            return True
        
        return False
    
    def print_debug_info(self):
        """输出调试信息"""
        if self.period_us <= 0 or self.high_us <= 0:
            return
        
        # 计算占空比百分比
        duty_pct = (float(self.high_us) / float(self.period_us)) * 100.0
        
        # 按标准 50Hz 伺服计算角度
        duty_raw = ((float(self.high_us) / float(self.period_us)) * 4119.0) - 16.0
        angle = duty_raw * (360.0 / 4095.0)
        if angle < 0:
            angle = 0
        if angle > 360:
            angle = 360
        
        freq = 1000000.0 / float(self.period_us) if self.period_us > 0 else 0
        
        print("┌─ PWM 原始数据 ─────────────────────")
        print("│ 周期: %6d μs  (期望: ~20000 μs)" % self.period_us)
        print("│ 脉宽: %6d μs  (期望: 1000-2000 μs)" % self.high_us)
        print("│ 占空比: %5.1f%%" % duty_pct)
        print("│ 频率: %5.1f Hz  (期望: 50 Hz)" % freq)
        print("│ 计算角度: %6.1f°" % angle)
        print("└─────────────────────────────────────")

def main():
    """主程序"""
    print("\n" + "="*50)
    print("PWM 反馈诊断工具（简化版）")
    print("="*50 + "\n")
    
    debug = PWMDebug(FEEDBACK_PIN)
    
    print("开始读取 PWM 信号...（最多等待 30 秒）\n")
    
    sample_count = 0
    start_time = time.ticks_ms()
    dot_count = 0
    
    while True:
        elapsed_ms = time.ticks_ms() - start_time
        if elapsed_ms >= 30000:  # 30 秒超时
            break
        
        if debug.read_one_cycle():
            sample_count += 1
            if dot_count > 0:
                print()  # 新行
                dot_count = 0
            print("样本 #%d" % sample_count)
            debug.print_debug_info()
            print()
        else:
            print(".")
            dot_count += 1
            time.sleep_ms(100)
    
    print("\n")
    print("收集了 %d 个样本" % sample_count)
    
    if sample_count == 0:
        print("\n警告：未捕获到任何有效 PWM 信号")
        print("请检查：")
        print("1. GPIO4 是否连接了反馈信号")
        print("2. 电机是否在运行")
        print("3. PWM 信号频率是否合理")

if __name__ == '__main__':
    main()

