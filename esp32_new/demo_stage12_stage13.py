#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
阶段 1.2 & 1.3 完整演示 - 旋转 + 推进功能

展示如何使用新增的旋转和推进功能，以及所有的控制接口
"""

import time
from machine import I2C, Pin

from main import FeedforwardDualIMUController
from config import I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQ


class IntegratedDemo:
    """1.2 & 1.3 集成演示"""
    
    def __init__(self):
        print("初始化控制器...")
        self.i2c = I2C(1, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=I2C_FREQ)
        self.controller = FeedforwardDualIMUController(self.i2c)
    
    def demo_1_basic_balance(self):
        """演示1: 基础平衡（旋转和推进均禁用）"""
        print("\n" + "="*60)
        print("演示1: 基础平衡（无旋转，无推进）")
        print("="*60)
        
        self.controller.enable_rotation_control(False)
        self.controller.enable_propulsion_control(False)
        
        print("运行 15 秒...")
        self.controller.start()
        start_time = time.time()
        
        try:
            while time.time() - start_time < 15:
                imu_data = self.controller.read_imu()
                if imu_data:
                    self.controller.control_step(imu_data)
                    if int((time.time() - start_time) * 2) % 2 == 0:
                        status = self.controller.get_system_status()
                        print(f"[基础] Roll: ? | Rotation: 禁用 | Propulsion: 禁用")
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("被中断")
        finally:
            self.controller.stop()
        
        print("✓ 演示1完成")
    
    def demo_2_enable_rotation_stage12(self):
        """演示2: 启用旋转（阶段1.2）"""
        print("\n" + "="*60)
        print("演示2: 启用旋转控制（阶段1.2）")
        print("="*60)
        
        self.controller.enable_rotation_control(True)
        self.controller.enable_propulsion_control(False)
        
        print("旋转功能已启用，将根据平衡转矩自动调整旋转角")
        print("运行 15 秒...")
        self.controller.start()
        start_time = time.time()
        
        try:
            while time.time() - start_time < 15:
                imu_data = self.controller.read_imu()
                if imu_data:
                    self.controller.control_step(imu_data)
                    if int((time.time() - start_time) * 2) % 2 == 0:
                        status = self.controller.get_system_status()
                        print(f"[旋转] Rotation: L={status['rotation_left']:6.1f}° R={status['rotation_right']:6.1f}° | Propulsion: 禁用")
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("被中断")
        finally:
            self.controller.stop()
        
        print("✓ 演示2完成")
    
    def demo_3_enable_propulsion_stage13(self):
        """演示3: 启用推进（阶段1.3）"""
        print("\n" + "="*60)
        print("演示3: 启用推进功能（阶段1.3）")
        print("="*60)
        
        self.controller.enable_rotation_control(True)
        self.controller.enable_propulsion_control(True)
        
        # 设置推进目标：前进，速度50%
        self.controller.set_propulsion_mode("forward")
        self.controller.set_propulsion_target(speed=0.5, mode="forward")
        
        print("推进功能已启用，设置前进模式，速度 50%")
        print("系统将自动平衡翻滚同时前进")
        print("运行 15 秒...")
        self.controller.start()
        start_time = time.time()
        
        try:
            while time.time() - start_time < 15:
                imu_data = self.controller.read_imu()
                if imu_data:
                    self.controller.control_step(imu_data)
                    if int((time.time() - start_time) * 2) % 2 == 0:
                        status = self.controller.get_system_status()
                        print(f"[推进] Mode: {status['propulsion_mode']} | Speed: {status['propulsion_speed']:.1%} | Thrust: L={status['thrust_left']:6.1f} R={status['thrust_right']:6.1f}")
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("被中断")
        finally:
            self.controller.stop()
        
        print("✓ 演示3完成")
    
    def demo_4_propulsion_modes(self):
        """演示4: 不同的推进模式"""
        print("\n" + "="*60)
        print("演示4: 不同推进模式切换")
        print("="*60)
        
        self.controller.enable_rotation_control(True)
        self.controller.enable_propulsion_control(True)
        
        modes = [
            ("forward", 0.5, "前进模式"),
            ("backward", 0.3, "后退模式"),
            ("forward", 0.7, "快速前进"),
        ]
        
        self.controller.start()
        
        try:
            for mode, speed, description in modes:
                print(f"\n设置: {description} (速度: {speed:.0%})")
                self.controller.set_propulsion_mode(mode)
                self.controller.set_propulsion_target(speed=speed, mode=mode)
                
                start_time = time.time()
                while time.time() - start_time < 10:
                    imu_data = self.controller.read_imu()
                    if imu_data:
                        self.controller.control_step(imu_data)
                        if int((time.time() - start_time) * 2) % 2 == 0:
                            status = self.controller.get_system_status()
                            print(f"  {description}: 推进速度 {status['propulsion_speed']:.0%}")
                    time.sleep(0.01)
        except KeyboardInterrupt:
            print("被中断")
        finally:
            self.controller.stop()
        
        print("✓ 演示4完成")
    
    def demo_5_priority_adjustment(self):
        """演示5: 调整平衡 vs 推进优先级"""
        print("\n" + "="*60)
        print("演示5: 平衡 vs 推进优先级调整")
        print("="*60)
        
        self.controller.enable_rotation_control(True)
        self.controller.enable_propulsion_control(True)
        self.controller.set_propulsion_mode("forward")
        
        priorities = [0.9, 0.7, 0.5]
        
        self.controller.start()
        
        try:
            for priority in priorities:
                print(f"\n设置平衡优先级: {priority:.1f} (0.0=全推进, 1.0=全平衡)")
                self.controller.set_propulsion_priority(priority)
                self.controller.set_propulsion_target(speed=0.5, mode="forward")
                
                start_time = time.time()
                while time.time() - start_time < 8:
                    imu_data = self.controller.read_imu()
                    if imu_data:
                        self.controller.control_step(imu_data)
                        if int((time.time() - start_time) * 2) % 2 == 0:
                            status = self.controller.get_system_status()
                            print(f"  优先级 {priority}: 推进 {status['propulsion_speed']:.0%}")
                    time.sleep(0.01)
        except KeyboardInterrupt:
            print("被中断")
        finally:
            self.controller.stop()
        
        print("✓ 演示5完成")
    
    def demo_6_manual_control(self):
        """演示6: 手动控制接口"""
        print("\n" + "="*60)
        print("演示6: 手动控制接口")
        print("="*60)
        
        print("测试所有可用的控制接口...")
        
        # 测试旋转控制
        print("\n1. 手动设置旋转角...")
        self.controller.set_rotation_angle(30.0, -30.0)
        time.sleep(1)
        self.controller.set_rotation_angle(0.0, 0.0)
        
        # 测试推进目标设置
        print("2. 设置推进目标...")
        self.controller.enable_propulsion_control(True)
        self.controller.set_propulsion_target(speed=0.5, direction=0.0, mode="forward")
        time.sleep(1)
        
        # 测试优先级设置
        print("3. 设置约束条件...")
        self.controller.set_propulsion_constraints(max_roll=2.0, min_thrust=15.0)
        time.sleep(1)
        
        # 获取系统状态
        print("4. 获取系统状态...")
        status = self.controller.get_system_status()
        print(f"   旋转启用: {status['rotation_enabled']}")
        print(f"   推进启用: {status['propulsion_enabled']}")
        print(f"   推进模式: {status['propulsion_mode']}")
        print(f"   当前推力: L={status['thrust_left']:.1f}N, R={status['thrust_right']:.1f}N")
        
        print("✓ 演示6完成")


def main():
    """主程序"""
    print("\n" + "="*60)
    print("ESP32 推进器旋转+推进功能 - 1.2 & 1.3 完整演示")
    print("="*60 + "\n")
    
    demo = IntegratedDemo()
    
    try:
        # 运行所有演示
        print("将依次运行 6 个演示...")
        print("每个演示展示不同的功能和控制接口\n")
        
        input("按 Enter 开始演示1...")
        demo.demo_1_basic_balance()
        
        input("按 Enter 开始演示2...")
        demo.demo_2_enable_rotation_stage12()
        
        input("按 Enter 开始演示3...")
        demo.demo_3_enable_propulsion_stage13()
        
        input("按 Enter 开始演示4...")
        demo.demo_4_propulsion_modes()
        
        input("按 Enter 开始演示5...")
        demo.demo_5_priority_adjustment()
        
        input("按 Enter 开始演示6...")
        demo.demo_6_manual_control()
        
        print("\n" + "="*60)
        print("✓ 所有演示完成！")
        print("="*60)
        
    except KeyboardInterrupt:
        print("\n✓ 演示被中断")
    except Exception as e:
        print(f"\n✗ 错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
