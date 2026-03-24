#!/usr/bin/env python3
"""
一键式MPU6050校准
放在水平桌面运行即可
"""

import time
import json
import mpu6050
import math

def auto_calibrate(address=0x68, bus=None, imu_name="IMU", duration=5):
    """自动校准：只需将IMU水平放置"""
    
    print(f"\n=== 校准 {imu_name} ===")
    print("请将IMU水平放置在桌面上，Z轴向上")
    print("不要移动，保持静止...")
    
    # 等待3秒让用户放置
    for i in range(3, 0, -1):
        print(f"开始校准倒计时: {i}秒", end='\r')
        time.sleep(1)
    print("\n开始校准...")
    
    # 连接IMU
    try:
        if bus:
            imu = mpu6050.mpu6050(address, bus)
        else:
            imu = mpu6050.mpu6050(address)
    except:
        print(f"无法连接到IMU (地址: 0x{address:02x})")
        return None
    
    # 采集数据
    gyro_samples = []
    accel_samples = []
    
    for i in range(100):  # 采集100个样本，约5秒
        try:
            gyro_samples.append(imu.get_gyro_data())
            accel_samples.append(imu.get_accel_data())
        except:
            continue
        
        if i % 20 == 0:
            print(f"进度: {i+1}/100", end='\r')
        time.sleep(0.05)
    
    print("\n计算校准参数...")
    
    # 计算平均值（陀螺仪零偏）
    gyro_avg = {'x': 0, 'y': 0, 'z': 0}
    for g in gyro_samples:
        gyro_avg['x'] += g['x']
        gyro_avg['y'] += g['y']
        gyro_avg['z'] += g['z']
    
    gyro_avg['x'] /= len(gyro_samples)
    gyro_avg['y'] /= len(gyro_samples)
    gyro_avg['z'] /= len(gyro_samples)
    
    # 计算加速度计平均值
    accel_avg = {'x': 0, 'y': 0, 'z': 0}
    for a in accel_samples:
        accel_avg['x'] += a['x']
        accel_avg['y'] += a['y']
        accel_avg['z'] += a['z']
    
    accel_avg['x'] /= len(accel_samples)
    accel_avg['y'] /= len(accel_samples)
    accel_avg['z'] /= len(accel_samples)
    
    # 自动计算加速度计偏移（假设水平放置，Z轴向上）
    # 理论值：X=0, Y=0, Z=1g
    accel_bias = {
        'x': accel_avg['x'],  # X轴应该为0
        'y': accel_avg['y'],  # Y轴应该为0
        'z': accel_avg['z'] - 9.8  # Z轴减去1g
    }
    
    # 计算当前重力幅值，用于验证
    current_magnitude = math.sqrt(accel_avg['x']**2 + accel_avg['y']**2 + accel_avg['z']**2)
    
    print(f"\n校准完成!")
    print(f"陀螺仪零偏: X={gyro_avg['x']:.3f}, Y={gyro_avg['y']:.3f}, Z={gyro_avg['z']:.3f} °/s")
    print(f"加速度计偏移: X={accel_bias['x']:.3f}, Y={accel_bias['y']:.3f}, Z={accel_bias['z']:.3f} g")
    print(f"当前重力测量: {current_magnitude:.3f}g (应为1.000g)")
    
    # 保存校准参数
    calibration = {
        'gyro_bias': gyro_avg,
        'accel_bias': accel_bias,
        'accel_scale': {'x': 1.0, 'y': 1.0, 'z': 1.0},  # 简单起见，不缩放
        'timestamp': time.time(),
        'gravity_magnitude': current_magnitude
    }
    
    # 保存到文件
    filename = f"calibration_{imu_name.lower()}.json"
    with open(filename, 'w') as f:
        json.dump(calibration, f, indent=2)
    
    print(f"参数已保存到: {filename}")
    
    return calibration

def test_calibration(address=0x68, bus=None, imu_name="IMU"):
    """快速测试校准效果"""
    
    print(f"\n测试 {imu_name} 校准效果...")
    
    # 加载校准参数
    try:
        with open(f"calibration_{imu_name.lower()}.json", 'r') as f:
            cal = json.load(f)
    except:
        print("未找到校准文件，请先运行校准")
        return
    
    # 连接IMU
    try:
        if bus:
            imu = mpu6050.mpu6050(address, bus)
        else:
            imu = mpu6050.mpu6050(address)
    except:
        print("无法连接到IMU")
        return
    
    # 测试几组数据
    print("测试5组数据:")
    for i in range(5):
        try:
            gyro = imu.get_gyro_data()
            accel = imu.get_accel_data()
            
            # 应用校准
            gyro_cal = {
                'x': gyro['x'] - cal['gyro_bias']['x'],
                'y': gyro['y'] - cal['gyro_bias']['y'],
                'z': gyro['z'] - cal['gyro_bias']['z']
            }
            
            accel_cal = {
                'x': accel['x'] - cal['accel_bias']['x'],
                'y': accel['y'] - cal['accel_bias']['y'],
                'z': accel['z'] - cal['accel_bias']['z']
            }
            
            # 计算重力幅值
            magnitude = math.sqrt(accel_cal['x']**2 + accel_cal['y']**2 + accel_cal['z']**2)
            
            print(f"  样本{i+1}: 陀螺仪=[{gyro_cal['x']:.2f},{gyro_cal['y']:.2f},{gyro_cal['z']:.2f}]°/s, "
                  f"重力={magnitude:.3f}g")
            
            time.sleep(0.5)
        except:
            print(f"  样本{i+1}: 读取失败")
            continue

def main():
    """主函数"""
    print("=== MPU6050 一键式校准 ===\n")
    
    # 校准第一个IMU（主IMU）
    print("首先校准主IMU (IMU1)...")
    auto_calibrate(address=0x68, bus=1, imu_name="IMU1", duration=5)
    
    # 测试第一个IMU
    test_calibration(address=0x68, bus=1, imu_name="IMU1")
    
    # 询问是否校准第二个IMU
    response = input("\n是否校准第二个IMU? (y/n): ").strip().lower()
    if response == 'y':
        print("\n校准第二个IMU (IMU2)...")
        auto_calibrate(address=0x68, bus=2, imu_name="IMU2", duration=5)
        test_calibration(address=0x68, bus=2, imu_name="IMU2")
    
    print("\n校准完成！现在可以运行主程序了。")

if __name__ == "__main__":
    main()