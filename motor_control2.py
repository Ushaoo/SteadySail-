#!/usr/bin/env python3

from motor_test import PCA9685
import mpu6050
import time
import math
from collections import deque
from queue import SimpleQueue, Empty
from threading import Thread, Event
import matplotlib.widgets as widgets
import json
import csv
from datetime import datetime
# 导入独立的参数调节模块
from web_pid_tuner import ParameterManager, WebPIDTuner
from enhanced_pid import EnhancedPIDController
try:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.widgets import Slider
except ImportError:
    plt = None
    FuncAnimation = None

# ========== 配置参数 ==========
mode = True  # True: gyro+accel PID, False: angaccel PID

ENABLE_PLOTTER = True
ENABLE_DUAL_IMU = False  # 是否启用双IMU
ENABLE_DATA_LOGGING = False  # 是否启用数据记录
ENABLE_WEB_TUNER = True  # 启用Web参数调节器
WEB_TUNER_PORT = 5000

# 绘图显示控制
PLOT_ANGLE = True
PLOT_ANGULAR_VELOCITY = True
PLOT_PID_OUTPUT = False
PLOT_ANGULAR_ACCELERATION = True
PLOT_MOTOR_PULSE = True

PLOT_WINDOW_SEC = 5.0
PLOT_MAX_WINDOW_SEC = 30.0
PLOT_SAMPLE_RATE_HZ = 100.0

# 角度阈值
angle_low = 5
angle_high = 10
angle_capsized = 35

motor_channel_1 = 3
motor_channel_2 = 4

def load_calibrations():
    """加载两个IMU的校准参数"""
    try:
        with open('calibration_imu1.json', 'r') as f:
            cal1 = json.load(f)
        print("IMU1校准已加载")
    except:
        print("IMU1校准文件未找到，使用默认值")
        cal1 = {'gyro_bias': {'x': 0, 'y': 0, 'z': 0}, 'accel_bias': {'x': 0, 'y': 0, 'z': 0}}
    
    try:
        with open('calibration_imu2.json', 'r') as f:
            cal2 = json.load(f)
        print("IMU2校准已加载")
    except:
        print("IMU2校准文件未找到，使用默认值")
        cal2 = {'gyro_bias': {'x': 0, 'y': 0, 'z': 0}, 'accel_bias': {'x': 0, 'y': 0, 'z': 0}}
    
    return cal1, cal2
class EnhancedIMUPIDPlotter:
    """增强版绘图器，整合了原IMUPlotter和DualIMU功能"""
    
    def __init__(self, window_sec, max_window_sec, sample_rate_hz, use_dual_imu=False):
        if plt is None or FuncAnimation is None:
            raise RuntimeError("Matplotlib is required for plotting")
            
        self.window_sec = window_sec
        self.use_dual_imu = use_dual_imu
        max_samples = int(max_window_sec * sample_rate_hz)
        
        # 数据缓冲区
        self.time_data = deque(maxlen=max_samples)
        self.angle_data = deque(maxlen=max_samples)
        self.angvel_data = deque(maxlen=max_samples)
        self.pid_data = deque(maxlen=max_samples)
        self.angaccel_data = deque(maxlen=max_samples)
        self.motorpulse_data = deque(maxlen=max_samples)
        
        # 双IMU数据
        if use_dual_imu:
            self.angle1_data = deque(maxlen=max_samples)
            self.angle2_data = deque(maxlen=max_samples)
            self.avg_angle_data = deque(maxlen=max_samples)
        
        self.queue = SimpleQueue()

    def record(self, timestamp, angle_deg, angular_velocity_deg_s, pid_output, 
               angular_acceleration_deg_s2, motor_pulse, angle1=None, angle2=None, avg_angle=None):
        if self.use_dual_imu:
            self.queue.put((timestamp, angle_deg, angular_velocity_deg_s, pid_output,
                          angular_acceleration_deg_s2, motor_pulse, angle1, angle2, avg_angle))
        else:
            self.queue.put((timestamp, angle_deg, angular_velocity_deg_s, pid_output,
                          angular_acceleration_deg_s2, motor_pulse, None, None, None))

    def run_plotter(self):
        # 配置绘图
        plot_configs = [
            {"label": "Angle (deg)", "color": "tab:blue", "enabled": PLOT_ANGLE, "data": self.angle_data},
            {"label": "Angular Velocity (deg/s)", "color": "tab:orange", "enabled": PLOT_ANGULAR_VELOCITY, "data": self.angvel_data},
            {"label": "PID Output", "color": "tab:green", "enabled": PLOT_PID_OUTPUT, "data": self.pid_data},
            {"label": "Angular Accel (deg/s^2)", "color": "tab:red", "enabled": PLOT_ANGULAR_ACCELERATION, "data": self.angaccel_data},
            {"label": "Motor Pulse (us)", "color": "tab:purple", "enabled": PLOT_MOTOR_PULSE, "data": self.motorpulse_data},
        ]

        enabled_plots = [p for p in plot_configs if p["enabled"]]
        if not enabled_plots:
            print("No plots enabled. Exiting plotter.")
            return

        num_plots = len(enabled_plots)
        fig, axes = plt.subplots(num_plots, 1, sharex=True, figsize=(10, 3 * num_plots))
        if num_plots == 1:
            axes = [axes]

        self.lines = []
        self.value_texts = []
        
        # 创建绘图
        for i, plot_info in enumerate(enabled_plots):
            ax = axes[i]
            if self.use_dual_imu and plot_info["label"] == "Angle (deg)":
                # 角度子图显示三条线
                line1, = ax.plot([], [], color='blue', lw=1.5, label='IMU1 Angle')
                line2, = ax.plot([], [], color='green', lw=1.5, label='IMU2 Angle')
                line_avg, = ax.plot([], [], color='red', lw=2, linestyle='--', label='Avg Angle')
                ax.legend()
                self.lines.extend([line1, line2, line_avg])
            else:
                line, = ax.plot([], [], color=plot_info["color"], lw=1.5)
                self.lines.append(line)
                
            ax.set_ylabel(plot_info["label"])
            ax.grid(True, alpha=0.3, linestyle="--")
            
            value_text = ax.text(0.98, 0.95, '', transform=ax.transAxes, 
                               verticalalignment='top', horizontalalignment='right',
                               fontsize=10, bbox=dict(boxstyle='round,pad=0.3', fc='white', ec='gray', alpha=0.8))
            self.value_texts.append(value_text)

        axes[-1].set_xlabel("Time (s)")
        
        # 添加窗口大小滑块
        slider_ax = fig.add_axes([0.15, 0.01, 0.7, 0.03])
        window_slider = Slider(slider_ax, "Window (s)", 1.0, PLOT_MAX_WINDOW_SEC, 
                              valinit=PLOT_WINDOW_SEC, valstep=0.5)
        
        def update_plot_data():
            """更新绘图数据"""
            updated = False
            while True:
                try:
                    if self.use_dual_imu:
                        sample = self.queue.get_nowait()
                        timestamp, angle_deg, ang_vel, pid_output, ang_accel, motor_pulse, angle1, angle2, avg_angle = sample
                        self.angle1_data.append(angle1)
                        self.angle2_data.append(angle2)
                        self.avg_angle_data.append(avg_angle)
                    else:
                        sample = self.queue.get_nowait()
                        timestamp, angle_deg, ang_vel, pid_output, ang_accel, motor_pulse, _, _, _ = sample
                    
                    self.time_data.append(timestamp)
                    self.angle_data.append(angle_deg)
                    self.angvel_data.append(ang_vel)
                    self.pid_data.append(pid_output)
                    self.angaccel_data.append(ang_accel)
                    self.motorpulse_data.append(motor_pulse)
                    updated = True
                except Empty:
                    break

            if not updated or not self.time_data:
                return False
                
            # 根据窗口大小裁剪数据
            cutoff = self.time_data[-1] - self.window_sec
            while self.time_data and self.time_data[0] < cutoff:
                self.time_data.popleft()
                self.angle_data.popleft()
                self.angvel_data.popleft()
                self.pid_data.popleft()
                self.angaccel_data.popleft()
                self.motorpulse_data.popleft()
                if self.use_dual_imu:
                    self.angle1_data.popleft()
                    self.angle2_data.popleft()
                    self.avg_angle_data.popleft()
                    
            return True

        def update(_frame):
            if not update_plot_data():
                return self.lines + self.value_texts

            line_idx = 0
            for i, plot_info in enumerate(enabled_plots):
                if self.use_dual_imu and plot_info["label"] == "Angle (deg)":
                    # 更新三条角度线
                    self.lines[line_idx].set_data(self.time_data, self.angle1_data)
                    self.lines[line_idx+1].set_data(self.time_data, self.angle2_data)
                    self.lines[line_idx+2].set_data(self.time_data, self.avg_angle_data)
                    # 对该子图重新计算坐标并自适应纵轴
                    ax = axes[i]
                    ax.relim()
                    ax.autoscale_view()
                    # 显示平均角度的最新值
                    if self.avg_angle_data:
                        self.value_texts[i].set_text(f'{self.avg_angle_data[-1]:.2f}')
                    line_idx += 3
                else:
                    line = self.lines[line_idx]
                    data = plot_info["data"]
                    line.set_data(self.time_data, data)
                    ax = line.axes
                    ax.relim()
                    ax.autoscale_view()
                    if data:
                        self.value_texts[i].set_text(f'{data[-1]:.2f}')
                    line_idx += 1

            return self.lines + self.value_texts

        def on_slider_change(val):
            self.window_sec = val
            update_plot_data()

        window_slider.on_changed(on_slider_change)
        fig.subplots_adjust(left=0.1, right=0.98, top=0.96, bottom=0.1, hspace=0.4)
        
        anim = FuncAnimation(fig, update, interval=20, blit=False, cache_frame_data=False)
        fig._imu_anim = anim
        plt.show()

class DataLogger:
    """IMU数据记录器"""
    
    def __init__(self):
        self.enabled = False
        self.file = None
        self.writer = None
        self.sample_count = 0
        self.start_time = None
        
    def start_logging(self):
        """开始记录数据"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"motor_control_imu_{timestamp}.csv"
        
        try:
            self.file = open(filename, 'w', newline='')
            self.writer = csv.writer(self.file)
            # 写入表头
            headers = ['timestamp', 'gyro_x', 'gyro_y', 'gyro_z', 
                      'acc_x', 'acc_y', 'acc_z', 'temp',
                      'angle_deg', 'angular_velocity', 'angular_acceleration',
                      'pid_output', 'motor_pulse']
            self.writer.writerow(headers)
            
            self.enabled = True
            self.sample_count = 0
            self.start_time = time.time()
            print(f"Started logging IMU data to: {filename}")
            
        except Exception as e:
            print(f"Failed to start data logging: {e}")
            self.enabled = False
            
    def stop_logging(self):
        """停止记录数据"""
        if self.enabled and self.file:
            total_time = time.time() - self.start_time
            avg_rate = self.sample_count / total_time if total_time > 0 else 0
            
            self.file.close()
            self.enabled = False
            
            print(f"\nData logging stopped.")
            print(f"Total samples: {self.sample_count}")
            print(f"Duration: {total_time:.2f} seconds")
            print(f"Average rate: {avg_rate:.2f} Hz")
    
    def log_data(self, timestamp, gyro_data, acc_data, temp, 
                angle_deg, angular_velocity, angular_acceleration,
                pid_output, motor_pulse):
        """记录一帧数据"""
        if not self.enabled:
            return
            
        try:
            self.writer.writerow([
                datetime.now().isoformat(),
                gyro_data['x'], gyro_data['y'], gyro_data['z'],
                acc_data['x'], acc_data['y'], acc_data['z'],
                temp,
                angle_deg, angular_velocity, angular_acceleration,
                pid_output, motor_pulse
            ])
            self.sample_count += 1
            
            if self.sample_count % 100 == 0:
                elapsed = time.time() - self.start_time
                current_rate = self.sample_count / elapsed
                print(f"Logged {self.sample_count} samples, Rate: {current_rate:.1f} Hz")
                
        except Exception as e:
            print(f"Error writing data: {e}")
            
# 优化后的双IMU数据处理
def process_dual_imu_data(imu, imu2, previous_angular_velocity, dt, cal1, cal2):
    """
    处理双IMU数据，应用校准
    """
    
    # 读取主IMU数据
    try:
        gyro_data1 = imu.get_gyro_data()
        acc_data1 = imu.get_accel_data()
        
        # 应用校准
        gyro_data1['x'] -= cal1['gyro_bias']['x']
        gyro_data1['y'] -= cal1['gyro_bias']['y']
        gyro_data1['z'] -= cal1['gyro_bias']['z']
        
        acc_data1['x'] -= cal1['accel_bias']['x']
        acc_data1['y'] -= cal1['accel_bias']['y']
        acc_data1['z'] -= cal1['accel_bias']['z']
        
        angle1 = math.atan2(acc_data1['y'], acc_data1['z']) * 180 / math.pi
        gyro_x1 = -math.radians(gyro_data1['x'])
        success1 = True
    except Exception as e:
        print(f"主IMU读取失败: {e}")
        angle1, gyro_x1 = 0, 0
        success1 = False
    
    # 读取副IMU数据
    angle2, gyro_x2, success2 = 0, 0, False
    if imu2 and ENABLE_DUAL_IMU:
        try:
            gyro_data2 = imu2.get_gyro_data()
            acc_data2 = imu2.get_accel_data()
            
            # 应用校准
            gyro_data2['x'] -= cal2['gyro_bias']['x']
            gyro_data2['y'] -= cal2['gyro_bias']['y']
            gyro_data2['z'] -= cal2['gyro_bias']['z']
            
            acc_data2['x'] -= cal2['accel_bias']['x']
            acc_data2['y'] -= cal2['accel_bias']['y']
            acc_data2['z'] -= cal2['accel_bias']['z']
            
            angle2 = math.atan2(-acc_data2['y'], acc_data2['z']) * 180 / math.pi
            gyro_x2 = math.radians(gyro_data2['x'])
            success2 = True
        except Exception as e:
            print(f"副IMU读取失败: {e}")
            success2 = False
   
    # 故障检测和降级策略
    if ENABLE_DUAL_IMU and success1 and success2:
        # 双IMU都正常，进行数据融合
        
        # 1. 角度融合（简单平均）
        angle = (angle1 + angle2) / 2
        
        # 2. 角速度融合（简单平均）
        angular_velocity = (gyro_x1 + gyro_x2) / 2
        
        # 3. 一致性检测
        angle_diff = abs(angle1 - angle2)
        gyro_diff = abs(gyro_x1 - gyro_x2)
        
        # 如果差异过大，可能传感器有问题
        if angle_diff > 15.0 or gyro_diff > 1.0:  # 阈值可调整
            print(f"双IMU数据不一致: 角度差={angle_diff:.1f}°, 角速度差={math.degrees(gyro_diff):.1f}°/s")
            # 可以选择使用更可靠的那个传感器，或使用平均值但标记为可疑
            
        # 4. 角加速度计算（基于融合后的角速度）
        if dt > 0:
            angular_acceleration = (angular_velocity - previous_angular_velocity) / dt
        else:
            angular_acceleration = 0.0
            
        return {
            'angle': angle,
            'angular_velocity': angular_velocity,
            'angular_acceleration': angular_acceleration,
            'sensor_status': 'DUAL_OK',
            'angle1': angle1,
            'angle2': angle2,
            'gyro1': gyro_x1,
            'gyro2': gyro_x2
        }
    
    elif success1:
        # 只有主IMU正常，使用主IMU数据
        angular_velocity = gyro_x1
        angle = angle1
        
        if dt > 0:
            angular_acceleration = (angular_velocity - previous_angular_velocity) / dt
        else:
            angular_acceleration = 0.0
            
        return {
            'angle': angle,
            'angular_velocity': angular_velocity,
            'angular_acceleration': angular_acceleration,
            'sensor_status': 'SINGLE_IMU1',
            'angle1': angle1,
            'angle2': None,
            'gyro1': gyro_x1,
            'gyro2': None
        }
    
    elif success2:
        # 只有副IMU正常，使用副IMU数据
        angular_velocity = gyro_x2
        angle = angle2
        
        if dt > 0:
            angular_acceleration = (angular_velocity - previous_angular_velocity) / dt
        else:
            angular_acceleration = 0.0
            
        return {
            'angle': angle,
            'angular_velocity': angular_velocity,
            'angular_acceleration': angular_acceleration,
            'sensor_status': 'SINGLE_IMU2',
            'angle1': None,
            'angle2': angle2,
            'gyro1': None,
            'gyro2': gyro_x2
        }
    
    else:
        # 两个IMU都失效，使用上次的值
        print("两个IMU都失效!")
        return {
            'angle': 0,  # 应该使用安全值
            'angular_velocity': 0,
            'angular_acceleration': 0,
            'sensor_status': 'FAIL',
            'angle1': None,
            'angle2': None,
            'gyro1': None,
            'gyro2': None
        }
def _control_loop(pwm, imu, imu2, gyro_pid, acc_pid, angacc_pid, plotter, param_manager, data_logger, stop_event,c1,c2):
    motor_channel = 0
    base_pulse = 1500
    min_pulse = 1000
    max_pulse = 2000
    
    # 初始化电机
    pwm.setServoPulse(motor_channel_1, base_pulse)
    pwm.setServoPulse(motor_channel_2, base_pulse)
    time.sleep(3)
    
    control_start = time.time()
    previous_time = control_start
    previous_angular_velocity_x = 0.0
    angular_acceleration_x = 0.0
    parameter = 0
    
    # 获取初始参数
    current_params = param_manager.get_current_params()
    
    # 初始化角度阈值变量（避免作用域问题）
    angle_low = current_params['angle_low']
    angle_high = current_params['angle_high']
    angle_capsized = current_params['angle_capsized']
    current_mode = current_params['mode']
    
    # 诊断输出计数器
    diagnosis_report_counter = 0
    
    while not stop_event.is_set():
        # 1. 首先测量循环开始时间（用于计算准确的dt）
        loop_start_time = time.time()
        current_time = loop_start_time
        dt = current_time - previous_time
        
        # 2. 一次性获取所有当前参数（避免多次读取不一致）
        new_params = param_manager.get_current_params()
        
        # 3. 检查参数是否有变化
        if new_params != current_params:
            print("检测到参数变化，更新PID参数...")
            
            # 更新PID参数
            gyro_params = new_params['gyro']
            acc_params = new_params['acc']
            angacc_params = new_params['angacc']
            
            gyro_pid.update_parameters(gyro_params['kp'], gyro_params['ki'], gyro_params['kd'], gyro_params['target'])
            acc_pid.update_parameters(acc_params['kp'], acc_params['ki'], acc_params['kd'], acc_params['target'])
            angacc_pid.update_parameters(angacc_params['kp'], angacc_params['ki'], angacc_params['kd'], angacc_params['target'])
            
            # 更新本地变量（用于本次循环）
            angle_low = new_params['angle_low']
            angle_high = new_params['angle_high']
            angle_capsized = new_params['angle_capsized']
            current_mode = new_params['mode']
            
            current_params = new_params
            print("PID参数已更新")
        
        # 4. 处理IMU数据（使用统一的dt）
        try:
            imu_data = process_dual_imu_data(imu, imu2, previous_angular_velocity_x, dt,c1,c2)
        except Exception as e:
            print(f"IMU数据处理错误: {e}")
            # 跳过本次循环，保持电机在安全状态
            time.sleep(0.01)
            continue
        
        # 5. 提取融合后的数据
        angle = imu_data['angle']
        angular_velocity_x = imu_data['angular_velocity']
        angular_acceleration_x = imu_data['angular_acceleration']
        sensor_status = imu_data['sensor_status']
        
        # 6. 更新角度历史值（用于下次计算角加速度）
        # 注意：只在这里更新一次
        previous_angular_velocity_x = angular_velocity_x
        
        # 7. 在输出中显示传感器状态
        if sensor_status != 'DUAL_OK' and sensor_status != 'SINGLE_IMU1':
            print(f"传感器状态: {sensor_status}")
        
        # 8. 更新PID控制器的当前角度（用于自适应滤波）
        if ENABLE_FILTER:
            gyro_pid.set_current_angle(angle)
            acc_pid.set_current_angle(angle)
            angacc_pid.set_current_angle(angle)
        
        # 9. PID控制逻辑（使用本地变量，避免多次读取参数）
        angle_abs = abs(angle)
        
        if current_mode:  # Gyro+Accel模式
            if angle_abs < angle_low:
                pid_output = gyro_pid.gyro_update(angular_velocity_x, dt)
            elif angle_abs >= angle_low and angle_abs <= angle_high:
                parameter = (angle_abs - angle_low) / (angle_high - angle_low)
                gyro_out = gyro_pid.gyro_update(angular_velocity_x, dt)
                acc_out = acc_pid.acc_update(angle, dt)
                pid_output = (1 - parameter) * gyro_out + parameter * acc_out
            elif angle_abs > angle_high:
                pid_output = acc_pid.acc_update(angle, dt)
            elif angle_abs > angle_capsized:
                # 紧急情况：增加输出增益
                pid_output = acc_pid.acc_update(angle, dt) * 2000
            else:
                pid_output = gyro_pid.gyro_update(angular_velocity_x, dt)
        else:  # Angular Acceleration模式
            pid_output = angacc_pid.angacc_update(angular_acceleration_x, dt)
        
        # 10. 计算并限制电机输出
        motor_adjustment = max(min(pid_output, 500), -500)
        motor_pulse = int(base_pulse + motor_adjustment)
        motor_pulse = max(min(motor_pulse, max_pulse), min_pulse)
        
        # 11. 输出到电机
        try:
            pwm.setServoPulse(motor_channel_1, 3000 - motor_pulse)
            pwm.setServoPulse(motor_channel_2, 3000 - motor_pulse)
        except Exception as e:
            print(f"电机控制错误: {e}")
        
        # 12. 数据记录
        if data_logger.enabled:
            try:
                temp = imu.get_temp()
                data_logger.log_data(current_time, temp,
                                   angle, math.degrees(angular_velocity_x), 
                                   math.degrees(angular_acceleration_x),
                                   pid_output, motor_pulse)
            except Exception as e:
                print(f"数据记录错误: {e}")
        
        # 13. 绘图数据
        if plotter:
            try:
                angle1 = imu_data.get('angle1')
                angle2 = imu_data.get('angle2')
                
                plotter.record(current_time - control_start,
                             angle,
                             math.degrees(angular_velocity_x),
                             pid_output,
                             math.degrees(angular_acceleration_x),
                             motor_pulse,
                             angle1, angle2, angle)
            except Exception as e:
                print(f"绘图数据记录错误: {e}")
        
        # 14. 定期输出状态（减少输出频率）
        diagnosis_report_counter += 1
        if diagnosis_report_counter >= 100:  # 每100次循环输出一次（约1秒）
            print(f"角度: {angle:.1f}°, "
                  f"角速度: {math.degrees(angular_velocity_x):.1f}°/s, "
                  f"PID输出: {pid_output:.1f}, "
                  f"电机脉冲: {motor_pulse}us")
            diagnosis_report_counter = 0
        
        # 15. 更新时间戳
        previous_time = current_time
        
        # 16. 精确控制循环频率（补偿计算时间）
        loop_elapsed = time.time() - loop_start_time
        sleep_time = max(0.01 - loop_elapsed, 0.001)  # 目标100Hz，最小睡眠1ms
        time.sleep(sleep_time)

def main():
    # 用户输入配置
    global ENABLE_DUAL_IMU, ENABLE_DATA_LOGGING,ENABLE_FILTER,ENABLE_WEB_TUNER
    # 滤波器启用选项
    enable_filter = input("Enable adaptive filtering? (y/n): ").strip().lower()
    ENABLE_FILTER = (enable_filter == 'y')
    # 绘图器启用选项
    enable_logging = input("Enable IMU data logging? (y/n): ").strip().lower()
    ENABLE_DATA_LOGGING = (enable_logging == 'y')
    # 双IMU启用选项
    enable_dual_imu = input("Enable dual IMU? (y/n): ").strip().lower()  
    ENABLE_DUAL_IMU = (enable_dual_imu == 'y')
     # 初始化参数管理器
    param_manager = ParameterManager()
    c1,c2= load_calibrations()
    # 初始化Web PID调节器
    web_tuner = None
    if ENABLE_WEB_TUNER:
        try:
            web_tuner = WebPIDTuner(param_manager, port=WEB_TUNER_PORT)
            if web_tuner.start():
                print(f"Web PID Tuner started on port {WEB_TUNER_PORT}")
            else:
                print("Failed to start Web PID Tuner")
        except Exception as e:
            print(f"Error starting Web PID Tuner: {e}")
            ENABLE_WEB_TUNER = False
    # 初始化硬件
    pwm = PCA9685(address=0x40, debug=True, bus_num=1)
    pwm.setPWMFreq(50)
    
    # 初始化IMU
    imu = mpu6050.mpu6050(0x68)
    imu2 = None
    
    if ENABLE_DUAL_IMU:
        try:
            imu2 = mpu6050.mpu6050(0x68,2)
            print("Dual IMU enabled - using addresses 0x68 and 0x69")
        except Exception as e:
            print(f"Failed to initialize second IMU: {e}")
            ENABLE_DUAL_IMU = False
    
     # 获取初始参数
    initial_params = param_manager.get_current_params()
    gyro_params = initial_params['gyro']
    acc_params = initial_params['acc']
    angacc_params = initial_params['angacc']
    
    # 初始化增强的PID控制器
    gyro_pid = EnhancedPIDController(gyro_params['kp'], gyro_params['ki'], gyro_params['kd'], 
                                   gyro_params['target'], name="Gyro PID")
    acc_pid = EnhancedPIDController(acc_params['kp'], acc_params['ki'], acc_params['kd'], 
                                  acc_params['target'], name="Acc PID")
    angacc_pid = EnhancedPIDController(angacc_params['kp'], angacc_params['ki'], angacc_params['kd'], 
                                     angacc_params['target'], name="AngAcc PID")
    
    print("Starting enhanced motor control system...")
    if ENABLE_WEB_TUNER:
        print(f"Web interface available at: http://<raspberry_pi_ip>:{WEB_TUNER_PORT}")
    
    print("Starting enhanced motor control system...")
    print("Press Ctrl+C to stop")
    
    # 初始化数据记录器
    data_logger = DataLogger()
    if ENABLE_DATA_LOGGING:
        data_logger.start_logging()
    
    # 初始化绘图器
    plotter = None
    if ENABLE_PLOTTER and plt is not None:
        try:
            plotter = EnhancedIMUPIDPlotter(PLOT_WINDOW_SEC, PLOT_MAX_WINDOW_SEC, 
                                          PLOT_SAMPLE_RATE_HZ, ENABLE_DUAL_IMU)
        except RuntimeError as exc:
            print(f"Plotter unavailable: {exc}")
    
    stop_event = Event()
    control_thread = Thread(target=_control_loop, 
                          args=(pwm, imu, imu2, gyro_pid, acc_pid, angacc_pid, 
                                plotter, param_manager, data_logger, stop_event,c1,c2))
    control_thread.start()
    
    if plotter:
        plotter.run_plotter()
    
    try:
        control_thread.join()
    except KeyboardInterrupt:
        print("\nMotor control interrupted by user.")
    finally:
        stop_event.set()
        control_thread.join()
        
        # 安全停止电机
        base_pulse = 1500
        pwm.setServoPulse(motor_channel_1, base_pulse)
        pwm.setServoPulse(motor_channel_2, base_pulse)
        time.sleep(0.5)
        pwm.setServoPulse(motor_channel_1, 0)
        pwm.setServoPulse(motor_channel_2, 0)
        
        # 停止数据记录
        if data_logger.enabled:
            data_logger.stop_logging()

if __name__ == "__main__":
    main()

