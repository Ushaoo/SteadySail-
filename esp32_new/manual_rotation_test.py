#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
交互式旋转角度测试程序 - 手动输入角度进行旋转测试

用途: 允许用户手动输入旋转角度，实时看到舵机反应
不需要 IMU，完全独立的硬件测试

使用方法:
  1. 连接旋转舵机到 PCA9685 通道 2, 3
  2. 运行本程序
  3. 在 REPL 中输入旋转角度，实时测试
  
示例:
  >>> test = ManualRotationTest()
  >>> test.run()
  输入角度 (左,右) [例如: 0,0 或 30,-30]: 0,0
  ✓ 设置旋转: L=  0.0° | R=  0.0°
  
  输入角度 (左,右) [例如: 0,0 或 30,-30]: 30,-30
  ✓ 设置旋转: L= 30.0° | R=-30.0°
  
  输入角度 (左,右) [例如: 0,0 或 30,-30]: exit
  ✓ 测试结束
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


class ManualRotationTest:
    """交互式旋转测试 - 手动输入角度"""
    
    def __init__(self):
        """初始化硬件"""
        print("初始化 I2C 和 PCA9685...")
        try:
            self.i2c = I2C(1, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=I2C_FREQ)
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
        
        # 状态
        self.last_angle_left = 0.0
        self.last_angle_right = 0.0
        
        print(f"\n旋转参数:")
        print(f"  左通道: {self.left_channel}")
        print(f"  右通道: {self.right_channel}")
        print(f"  最大角度: ±{self.max_angle}°")
        print(f"  脉宽范围: {self.pulse_min}-{self.pulse_max} μs")
        print(f"  中立脉宽: {self.pulse_center} μs")
    
    def angle_to_pulse(self, angle_deg):
        """将旋转角转换为脉宽 (1000-2000 μs)"""
        # 限制角度范围
        angle_deg = max(-self.max_angle, min(self.max_angle, angle_deg))
        
        # 线性映射: -45° → 1000μs, 0° → 1500μs, +45° → 2000μs
        pulse = self.pulse_center + (angle_deg / self.max_angle) * 500
        return int(pulse)
    
    def pulse_to_angle(self, pulse):
        """将脉宽转换回角度 (用于验证)"""
        # 反向映射
        angle = (pulse - self.pulse_center) / 500 * self.max_angle
        return round(angle, 1)
    
    def set_angle(self, angle_left, angle_right):
        """设置旋转角，返回是否成功"""
        if not self.pwm:
            print("✗ PCA9685 未初始化")
            return False
        
        # 限制角度范围
        angle_left = max(-self.max_angle, min(self.max_angle, angle_left))
        angle_right = max(-self.max_angle, min(self.max_angle, angle_right))
        
        # 转换为脉宽
        pulse_left = self.angle_to_pulse(angle_left)
        pulse_right = self.angle_to_pulse(angle_right)
        
        try:
            # 设置舵机
            self.pwm.setServoPulse(self.left_channel, pulse_left)
            self.pwm.setServoPulse(self.right_channel, pulse_right)
            
            # 保存当前角度
            self.last_angle_left = angle_left
            self.last_angle_right = angle_right
            
            # 打印反馈
            print(f"✓ 设置旋转: L={angle_left:6.1f}° ({pulse_left}μs) | R={angle_right:6.1f}° ({pulse_right}μs)")
            return True
        except Exception as e:
            print(f"✗ 设置失败: {e}")
            return False
    
    def parse_input(self, input_str):
        """解析用户输入，返回 (angle_left, angle_right) 或 None"""
        try:
            # 移除空格
            input_str = input_str.strip()
            
            # 检查退出命令
            if input_str.lower() in ['exit', 'quit', 'q', 'stop']:
                return 'exit'
            
            # 检查特殊命令
            if input_str.lower() in ['help', 'h', '?']:
                return 'help'
            
            if input_str.lower() in ['current', 'now']:
                return 'current'
            
            # 解析两个角度 (逗号分隔)
            parts = input_str.split(',')
            if len(parts) != 2:
                print("✗ 格式错误。请输入: 左角度,右角度 (例如: 0,0 或 30,-30)")
                return None
            
            left = float(parts[0].strip())
            right = float(parts[1].strip())
            
            return (left, right)
        except ValueError:
            print("✗ 输入错误。请输入数字，以逗号分隔 (例如: 30,-30)")
            return None
    
    def run(self):
        """交互式运行"""
        print("\n" + "="*70)
        print("交互式旋转测试 - 手动输入角度")
        print("="*70)
        print("\n使用说明:")
        print("  • 输入: 左角度,右角度 (例如: 0,0 或 30,-30)")
        print("  • 范围: -45° 到 +45°")
        print("  • 特殊命令:")
        print("    - current: 显示当前角度")
        print("    - help: 显示帮助")
        print("    - exit/quit/q: 退出程序")
        print()
        
        # 初始中立位置
        self.set_angle(0.0, 0.0)
        
        # 交互循环
        while True:
            try:
                # 获取用户输入
                user_input = input("\n输入角度 (左,右) [例如: 0,0 或 30,-30]: ")
                result = self.parse_input(user_input)
                
                if result == 'exit':
                    print("✓ 测试结束")
                    break
                elif result == 'help':
                    self.print_help()
                    continue
                elif result == 'current':
                    print(f"当前旋转: L={self.last_angle_left:6.1f}° | R={self.last_angle_right:6.1f}°")
                    continue
                elif result is None:
                    continue
                else:
                    # 设置旋转角
                    left, right = result
                    self.set_angle(left, right)
                    time.sleep(0.1)  # 等待舵机响应
            
            except KeyboardInterrupt:
                print("\n✓ 被中断，测试结束")
                break
            except Exception as e:
                print(f"✗ 错误: {e}")
    
    def print_help(self):
        """打印帮助信息"""
        print("\n" + "-"*70)
        print("帮助信息")
        print("-"*70)
        print("\n【基本用法】")
        print("  输入格式: 左角度,右角度")
        print("  例如:")
        print("    0,0       → 中立位置 (两个舵机都竖直)")
        print("    30,-30    → 左顺时针 30°，右逆时针 30° (模拟转矩)")
        print("    45,45     → 两个舵机都旋转 45° (最大角度)")
        print("    -45,-45   → 两个舵机都旋转 -45°")
        print("    10,0      → 左旋转 10°，右保持中立")
        print()
        print("【特殊命令】")
        print("  current   → 显示当前旋转角")
        print("  help      → 显示本帮助")
        print("  exit/quit → 退出程序")
        print()
        print("【物理范围】")
        print("  • 最大旋转角: ±45°")
        print("  • 脉宽范围: 1000-2000 μs")
        print("  • 中立位置脉宽: 1500 μs")
        print()
        print("【常见测试】")
        print("  1. 中立位置:  输入 0,0")
        print("  2. 单个舵机:  输入 30,0 (只左旋转)")
        print("  3. 对称旋转:  输入 30,30")
        print("  4. 反向旋转:  输入 30,-30 (模拟转矩效果)")
        print("  5. 最大角度:  输入 45,45")
        print("-"*70 + "\n")
    
    def quick_test(self):
        """快速测试序列 (不需要交互)"""
        """自动运行一系列测试，不需要手动输入"""
        print("\n" + "="*70)
        print("快速测试序列 (自动)")
        print("="*70)
        
        test_cases = [
            (0.0, 0.0, "中立位置"),
            (15.0, 15.0, "低角度对称"),
            (30.0, 30.0, "中角度对称"),
            (45.0, 45.0, "最大角度对称"),
            (30.0, -30.0, "反向旋转 (转矩效果)"),
            (45.0, -45.0, "最大反向旋转"),
            (10.0, 0.0, "单边旋转左"),
            (0.0, 20.0, "单边旋转右"),
            (-30.0, 30.0, "交叉旋转"),
            (0.0, 0.0, "回到中立"),
        ]
        
        for angle_left, angle_right, description in test_cases:
            print(f"\n{description}: {angle_left}°, {angle_right}°")
            self.set_angle(angle_left, angle_right)
            time.sleep(1)
        
        print("\n✓ 快速测试完成\n")


# ==================== 主程序 ====================

def main():
    """主函数"""
    print("\n" + "="*70)
    print("推进器旋转测试 - 交互式模式")
    print("="*70)
    
    # 创建测试对象
    test = ManualRotationTest()
    
    # 检查初始化是否成功
    if not test.pwm:
        print("✗ 初始化失败，无法继续")
        return
    
    # 询问用户选择
    print("\n请选择运行模式:")
    print("  1. 交互式 (手动输入角度)")
    print("  2. 快速测试 (自动运行预设测试)")
    print("  3. 显示帮助")
    print("  0. 退出")
    print()
    
    try:
        choice = input("输入选择 (0-3): ").strip()
        
        if choice == '1':
            test.run()
        elif choice == '2':
            test.quick_test()
            print("\n输入任意按键退出...")
            input()
        elif choice == '3':
            test.print_help()
        else:
            print("退出")
    except KeyboardInterrupt:
        print("\n\n✓ 程序已终止")


if __name__ == '__main__':
    main()


# ==================== 快速使用 ====================
# 
# 在 MicroPython REPL 中快速运行:
#
# 方式 1: 运行主程序
#   >>> exec(open('manual_rotation_test.py').read())
#
# 方式 2: 直接创建对象进行测试
#   >>> from manual_rotation_test import ManualRotationTest
#   >>> test = ManualRotationTest()
#   >>> test.run()                    # 交互式
#   >>> test.quick_test()             # 快速测试
#   >>> test.set_angle(30, -30)       # 直接设置
#   >>> test.print_help()             # 查看帮助
