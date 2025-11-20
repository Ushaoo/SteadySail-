#!/usr/bin/env python3
"""
系统辨识模块
提供Ziegler-Nichols调参方法和系统特性分析功能
"""

import time
import math
import numpy as np

# 尝试导入matplotlib，如果不可用则跳过绘图功能
try:
    import matplotlib.pyplot as plt
    PLOTTING_AVAILABLE = True
except ImportError:
    PLOTTING_AVAILABLE = False
    print("警告: matplotlib不可用，系统辨识将无法绘制图表")


class SystemIdentifier:
    def __init__(self, pwm, imu, motor_channel=0):
        """
        初始化系统辨识器
        
        参数:
            pwm: PCA9685 PWM控制器对象
            imu: MPU6050 IMU传感器对象
            motor_channel: 电机通道号
        """
        self.pwm = pwm
        self.imu = imu
        self.motor_channel = motor_channel
        self.base_pulse = 1500
        
    def step_response_test(self, step_size=100, duration=2.0, sample_rate=100):
        """
        阶跃响应测试
        
        参数:
            step_size: 阶跃脉冲宽度变化量 (微秒)
            duration: 测试持续时间 (秒)
            sample_rate: 采样频率 (Hz)
            
        返回:
            包含测试数据的字典
        """
        print(f"开始阶跃响应测试，阶跃大小: {step_size}us")
        
        # 数据记录
        timestamps = []
        angles = []
        angular_velocities = []
        motor_pulses = []
        
        # 初始稳定
        self.pwm.setServoPulse(self.motor_channel, self.base_pulse)
        time.sleep(1.0)
        
        start_time = time.time()
        sample_interval = 1.0 / sample_rate
        
        try:
            # 记录阶跃前响应
            pre_step_duration = 0.5
            pre_step_end = start_time + pre_step_duration
            
            current_time = start_time
            while current_time < pre_step_end:
                # 读取传感器数据
                gyro_data = self.imu.get_gyro_data()
                acc_data = self.imu.get_accel_data()
                
                angle = math.atan2(acc_data['y'], acc_data['z']) * 180 / math.pi
                angular_velocity = math.radians(gyro_data['x'])
                
                timestamps.append(current_time - start_time)
                angles.append(angle)
                angular_velocities.append(angular_velocity)
                motor_pulses.append(self.base_pulse)
                
                time.sleep(sample_interval)
                current_time = time.time()
            
            # 施加阶跃输入
            step_pulse = self.base_pulse + step_size
            step_time = current_time
            self.pwm.setServoPulse(self.motor_channel, step_pulse)
            print(f"施加阶跃输入: {step_pulse}us")
            
            # 记录阶跃后响应
            while current_time < start_time + duration:
                gyro_data = self.imu.get_gyro_data()
                acc_data = self.imu.get_accel_data()
                
                angle = math.atan2(acc_data['y'], acc_data['z']) * 180 / math.pi
                angular_velocity = math.radians(gyro_data['x'])
                
                timestamps.append(current_time - start_time)
                angles.append(angle)
                angular_velocities.append(angular_velocity)
                motor_pulses.append(step_pulse)
                
                time.sleep(sample_interval)
                current_time = time.time()
                
        except KeyboardInterrupt:
            print("测试被用户中断")
        finally:
            # 恢复中立位置
            self.pwm.setServoPulse(self.motor_channel, self.base_pulse)
        
        return {
            'timestamps': np.array(timestamps),
            'angles': np.array(angles),
            'angular_velocities': np.array(angular_velocities),
            'motor_pulses': np.array(motor_pulses),
            'step_size': step_size,
            'step_time': pre_step_duration
        }
    
    def analyze_step_response(self, response_data):
        """
        分析阶跃响应数据，提取系统特性
        
        参数:
            response_data: 阶跃响应测试数据
            
        返回:
            系统特性参数字典
        """
        timestamps = response_data['timestamps']
        angles = response_data['angles']
        step_time = response_data['step_time']
        
        # 找到阶跃时刻的索引
        step_idx = np.argmax(timestamps >= step_time)
        
        # 提取阶跃后的数据
        post_step_times = timestamps[step_idx:] - step_time
        post_step_angles = angles[step_idx:]
        
        if len(post_step_angles) == 0:
            return None
            
        # 计算稳态值（使用最后20%的数据）
        steady_state_start = int(len(post_step_angles) * 0.8)
        if steady_state_start >= len(post_step_angles):
            steady_state_start = len(post_step_angles) - 1
            
        steady_state_value = np.mean(post_step_angles[steady_state_start:])
        
        # 计算10%和90%的响应点
        initial_value = post_step_angles[0]
        target_10 = initial_value + 0.1 * (steady_state_value - initial_value)
        target_90 = initial_value + 0.9 * (steady_state_value - initial_value)
        
        # 找到对应时间点
        idx_10 = np.argmax(post_step_angles >= target_10)
        idx_90 = np.argmax(post_step_angles >= target_90)
        
        time_10 = post_step_times[idx_10] if idx_10 > 0 and idx_10 < len(post_step_times) else 0
        time_90 = post_step_times[idx_90] if idx_90 > 0 and idx_90 < len(post_step_times) else 0
        
        # 计算系统参数
        rise_time = time_90 - time_10 if time_90 > time_10 else 0.1
        
        settling_time = self._find_settling_time(post_step_times, post_step_angles, 
                                               steady_state_value, tolerance=0.05)
        
        # 计算超调量
        max_value = np.max(post_step_angles)
        min_value = np.min(post_step_angles)
        max_overshoot = max_value - steady_state_value if abs(steady_state_value - initial_value) > 0.1 else 0
        overshoot_percentage = (max_overshoot / abs(steady_state_value - initial_value)) * 100 if abs(steady_state_value - initial_value) > 0.1 else 0
        
        system_params = {
            'steady_state_value': steady_state_value,
            'rise_time': rise_time,
            'settling_time': settling_time,
            'overshoot_percentage': overshoot_percentage,
            'time_10': time_10,
            'time_90': time_90
        }
        
        return system_params
    
    def _find_settling_time(self, times, values, steady_state, tolerance=0.05):
        """找到调节时间（进入±5%误差带的时间）"""
        if len(values) == 0:
            return 0
            
        initial_value = values[0]
        error_band = tolerance * abs(steady_state - initial_value) if abs(steady_state - initial_value) > 0.1 else 1.0
        upper_bound = steady_state + error_band
        lower_bound = steady_state - error_band
        
        # 找到最后一个超出误差带的时间点
        settling_idx = len(values) - 1
        for i in range(len(values)-1, -1, -1):
            if values[i] > upper_bound or values[i] < lower_bound:
                settling_idx = i
                break
        
        return times[settling_idx] if settling_idx < len(times) else times[-1]
    
    def zn_tuning_recommendations(self, system_params, method='step_response'):
        """
        根据系统特性提供Ziegler-Nichols调参建议
        
        参数:
            system_params: 系统特性参数
            method: 调参方法 ('step_response' 或 'oscillation')
            
        返回:
            PID参数建议字典
        """
        if method == 'step_response':
            # 阶跃响应法（需要延迟时间L和时间常数T）
            L = system_params['time_10']  # 使用10%响应时间作为延迟时间近似
            T = system_params['rise_time'] * 2  # 使用上升时间估算时间常数
            
            if L > 0 and T > 0:
                # 系统增益估计 - 使用角度变化与输入脉冲变化的比率
                K = abs(system_params['steady_state_value']) / 100  # 假设100us脉冲产生steady_state_value度的变化
                
                # Ziegler-Nichols阶跃响应法公式
                recommendations = {
                    'P': {'kp': T / (K * L), 'ki': 0, 'kd': 0},
                    'PI': {'kp': 0.9 * T / (K * L), 'ki': 0.3 * L, 'kd': 0},
                    'PID': {'kp': 1.2 * T / (K * L), 'ki': 0.5 * L, 'kd': 0.5 * L}
                }
                return recommendations
                
        return None
    
    def plot_response_analysis(self, response_data, system_params):
        """绘制响应分析图"""
        if not PLOTTING_AVAILABLE:
            print("无法绘制图表: matplotlib不可用")
            return
            
        plt.figure(figsize=(12, 8))
        
        # 角度响应
        plt.subplot(2, 1, 1)
        plt.plot(response_data['timestamps'], response_data['angles'], 'b-', linewidth=2)
        plt.axvline(x=response_data['step_time'], color='r', linestyle='--', label='Step Input')
        plt.axhline(y=system_params['steady_state_value'], color='g', linestyle='--', 
                   label=f'Steady State: {system_params["steady_state_value"]:.2f}°')
        plt.xlabel('Time (s)')
        plt.ylabel('Angle (degrees)')
        plt.title('Step Response - Angle')
        plt.legend()
        plt.grid(True)
        
        # 角速度响应
        plt.subplot(2, 1, 2)
        plt.plot(response_data['timestamps'], response_data['angular_velocities'], 'r-', linewidth=2)
        plt.axvline(x=response_data['step_time'], color='r', linestyle='--')
        plt.xlabel('Time (s)')
        plt.ylabel('Angular Velocity (rad/s)')
        plt.title('Step Response - Angular Velocity')
        plt.grid(True)
        
        plt.tight_layout()
        plt.show()


def estimate_system_characteristics(imu, pwm, motor_channel=0, step_sizes=[50, 100, 150]):
    """
    完整的系统特性估计函数
    
    参数:
        imu: MPU6050 IMU传感器对象
        pwm: PCA9685 PWM控制器对象
        motor_channel: 电机通道号
        step_sizes: 要测试的阶跃大小列表
        
    返回:
        包含最佳测试结果的字典
    """
    print("开始系统特性估计...")
    
    identifier = SystemIdentifier(pwm, imu, motor_channel)
    all_results = {}
    
    for step_size in step_sizes:
        print(f"\n正在进行阶跃大小 {step_size}us 的测试...")
        
        # 执行阶跃响应测试
        response_data = identifier.step_response_test(
            step_size=step_size, 
            duration=3.0,  # 延长测试时间
            sample_rate=100
        )
        
        # 分析响应
        system_params = identifier.analyze_step_response(response_data)
        
        if system_params is None:
            print(f"  阶跃大小 {step_size}us 的分析失败，跳过")
            continue
            
        # 获取调参建议
        zn_recommendations = identifier.zn_tuning_recommendations(system_params)
        
        all_results[step_size] = {
            'response_data': response_data,
            'system_params': system_params,
            'zn_recommendations': zn_recommendations
        }
        
        # 显示结果
        print(f"阶跃大小 {step_size}us 的结果:")
        print(f"  稳态角度: {system_params['steady_state_value']:.2f}°")
        print(f"  上升时间: {system_params['rise_time']:.3f}s")
        print(f"  调节时间: {system_params['settling_time']:.3f}s")
        print(f"  超调量: {system_params['overshoot_percentage']:.1f}%")
        
        if zn_recommendations:
            print("  Ziegler-Nichols建议:")
            for controller_type, params in zn_recommendations.items():
                print(f"    {controller_type}: Kp={params['kp']:.2f}, Ki={params['ki']:.4f}, Kd={params['kd']:.4f}")
    
    if not all_results:
        print("所有测试均失败，无法提供调参建议")
        return None
        
    # 选择最佳结果（通常使用中等阶跃大小的结果）
    best_step = step_sizes[len(step_sizes)//2]  # 选择中间的阶跃大小
    if best_step not in all_results:
        best_step = list(all_results.keys())[0]
        
    best_result = all_results[best_step]
    
    # 绘制最佳结果的图表
    identifier.plot_response_analysis(
        best_result['response_data'], 
        best_result['system_params']
    )
    
    print(f"\n系统特性估计完成！建议使用阶跃大小 {best_step}us 的结果进行调参。")
    return best_result


def run_system_identification():
    """
    独立的系统辨识运行函数
    可以直接从命令行调用此函数进行系统辨识
    """
    # 导入必要的模块
    from motor_test import PCA9685
    import mpu6050
    
    print("运行系统辨识模式...")
    
    # 初始化硬件
    pwm = PCA9685(address=0x40, debug=True, bus_num=1)
    pwm.setPWMFreq(50)
    imu = mpu6050.mpu6050(0x68)
    
    # 执行系统辨识
    result = estimate_system_characteristics(imu, pwm, 0)  # 使用通道0
    
    if result and result['zn_recommendations']:
        print("\n基于Ziegler-Nichols方法的PID参数建议:")
        for controller_type, params in result['zn_recommendations'].items():
            print(f"{controller_type}: Kp={params['kp']:.2f}, Ki={params['ki']:.4f}, Kd={params['kd']:.4f}")
        
        # 询问用户是否要应用这些参数
        response = input("\n是否要应用这些PID参数？(y/n): ")
        if response.lower() == 'y':
            # 这里可以保存参数到文件或直接应用
            print("参数已记录，请在代码中手动更新PID参数")
    
    return result


if __name__ == "__main__":
    # 如果直接运行此文件，执行系统辨识
    run_system_identification()