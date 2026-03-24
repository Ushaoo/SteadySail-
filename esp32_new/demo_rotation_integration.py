#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
推进器旋转控制 - 集成演示脚本

用途: 演示如何在主控制程序中启用和使用旋转功能
"""

import time
from machine import I2C, Pin

# 导入主程序
from main import (
    FeedforwardDualIMUController,
    I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQ
)


class RotationIntegrationDemo:
    """旋转功能集成演示"""
    
    def __init__(self):
        print("初始化 I2C 和控制器...")
        self.i2c = I2C(1, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=I2C_FREQ)
        self.controller = FeedforwardDualIMUController(self.i2c)
    
    def demo_1_basic_operation(self):
        """演示 1: 基础平衡（旋转功能禁用）"""
        print("\n" + "="*60)
        print("演示 1: 基础平衡运行 - 旋转功能禁用")
        print("="*60)
        print("系统将正常运行，旋转舵机保持中立位置")
        
        # 确保旋转功能禁用
        self.controller.enable_rotation_control(False)
        
        print("运行 20 秒...")
        self.controller.start()
        start_time = time.time()
        
        try:
            while time.time() - start_time < 20:
                imu_data = self.controller.read_imu()
                if imu_data:
                    result = self.controller.control_step(imu_data)
                    if result and int((time.time() - start_time) * 10) % 10 == 0:
                        print(f"Roll: {result['roll']:6.2f}°")
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("被中断")
        finally:
            self.controller.stop()
        
        print("✓ 演示 1 完成")
    
    def demo_2_enable_rotation_framework(self):
        """演示 2: 启用旋转功能框架"""
        print("\n" + "="*60)
        print("演示 2: 启用旋转功能框架 - 向后兼容")
        print("="*60)
        print("旋转功能已启用，但暂不改变控制逻辑")
        print("旋转舵机会根据平衡转矩微微摆动")
        
        # 启用旋转功能
        self.controller.enable_rotation_control(True)
        
        print("运行 20 秒...")
        self.controller.start()
        start_time = time.time()
        
        try:
            while time.time() - start_time < 20:
                imu_data = self.controller.read_imu()
                if imu_data:
                    result = self.controller.control_step(imu_data)
                    if result and int((time.time() - start_time) * 10) % 10 == 0:
                        rotation = self.controller.rotation_controller
                        print(f"Roll: {result['roll']:6.2f}° | Rotation: L={rotation.last_rotation_left:6.1f}° R={rotation.last_rotation_right:6.1f}°")
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("被中断")
        finally:
            self.controller.stop()
        
        print("✓ 演示 2 完成")
    
    def demo_3_manual_rotation_test(self):
        """演示 3: 手动旋转测试"""
        print("\n" + "="*60)
        print("演示 3: 手动旋转角度测试 - 用于硬件验证")
        print("="*60)
        
        if not self.controller.rotation_controller:
            print("✗ 旋转控制器未初始化")
            return
        
        # 启用旋转
        self.controller.enable_rotation_control(True)
        
        print("测试不同旋转角度...")
        angles = [0, 15, 30, 45, 0, -15, -30, -45, 0]
        
        for angle in angles:
            print(f"设置旋转角: {angle}°")
            self.controller.set_rotation_angle(float(angle), float(angle))
            time.sleep(1)
        
        print("✓ 演示 3 完成")
    
    def demo_4_wave_pattern(self):
        """演示 4: 波形扫描（模拟动态旋转）"""
        print("\n" + "="*60)
        print("演示 4: 波形扫描 - 模拟动态旋转")
        print("="*60)
        
        import math
        
        self.controller.enable_rotation_control(True)
        self.controller.start()
        
        print("运行正弦波扫描 10 秒...")
        start_time = time.time()
        
        try:
            while time.time() - start_time < 10:
                elapsed = time.time() - start_time
                
                # 生成正弦波旋转命令
                angle = 30 * math.sin(elapsed * 0.5)
                
                imu_data = self.controller.read_imu()
                if imu_data:
                    # 临时禁用自动旋转，使用手动命令
                    rotation_ctrl = self.controller.rotation_controller
                    old_enabled = rotation_ctrl.enabled
                    rotation_ctrl.enabled = False
                    
                    # 手动设置旋转角
                    self.controller.control_step(imu_data)
                    self.controller.set_rotation_angle(angle, angle)
                    
                    rotation_ctrl.enabled = old_enabled
                    
                    if int(elapsed * 10) % 10 == 0:
                        print(f"时间: {elapsed:5.1f}s | 旋转角: {angle:6.1f}°")
                
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("被中断")
        finally:
            self.controller.stop()
        
        print("✓ 演示 4 完成")
    
    def demo_5_status_report(self):
        """演示 5: 系统状态报告"""
        print("\n" + "="*60)
        print("演示 5: 系统状态报告")
        print("="*60)
        
        if self.controller.rotation_controller:
            rc = self.controller.rotation_controller
            print(f"旋转控制器状态:")
            print(f"  启用状态: {rc.enabled}")
            print(f"  最大旋转角: {rc.max_rotation_angle}°")
            print(f"  旋转通道: L={rc.left_rotation_channel}, R={rc.right_rotation_channel}")
            print(f"  最后旋转角: L={rc.last_rotation_left:.1f}°, R={rc.last_rotation_right:.1f}°")
            print(f"  脉宽范围: {rc.rotation_pulse_min}-{rc.rotation_pulse_max} μs")
            print(f"  速率限制: {rc.rotation_rate_limit}°/s")
        else:
            print("✗ 旋转控制器未初始化")
        
        print("✓ 演示 5 完成")


def main():
    """运行演示"""
    print("\n" + "="*60)
    print("ESP32 推进器旋转控制 - 集成演示")
    print("="*60 + "\n")
    
    demo = RotationIntegrationDemo()
    
    try:
        # 选择演示
        print("可用演示:")
        print("1. 基础平衡（旋转禁用）")
        print("2. 启用旋转框架")
        print("3. 手动旋转测试")
        print("4. 波形扫描")
        print("5. 状态报告")
        print("\n建议顺序: 5 → 1 → 3 → 2 → 4\n")
        
        # 直接运行全部演示序列
        print("运行完整演示序列...\n")
        
        demo.demo_5_status_report()
        time.sleep(2)
        
        demo.demo_1_basic_operation()
        time.sleep(2)
        
        demo.demo_3_manual_rotation_test()
        time.sleep(2)
        
        demo.demo_2_enable_rotation_framework()
        time.sleep(2)
        
        print("\n" + "="*60)
        print("✓ 所有演示完成！")
        print("="*60 + "\n")
        
    except KeyboardInterrupt:
        print("\n✓ 演示被中断")
    except Exception as e:
        print(f"\n✗ 错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
