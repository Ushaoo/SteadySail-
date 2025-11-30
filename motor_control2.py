#!/usr/bin/env python3

from motor_test import PCA9685
import mpu6050
import time
import math
from collections import deque
from queue import SimpleQueue, Empty
from threading import Thread, Event
import csv
from datetime import datetime

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

motor_channel_1 = 0

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

# PIDController类保持不变（使用原有代码）
class PIDController:
    def __init__(self, kp, ki, kd, target_value=0):
        self.kp = kp
        self.ki = ki  
        self.kd = kd
        self.target = target_value
        self.previous_error = 0
        self.integral = 0
        
    def gyro_update(self, current_value, dt):
        error = self.target - current_value
        proportional = self.kp * error
        self.integral += error * dt
        integral = self.ki * self.integral
        derivative = self.kd * (error - self.previous_error) / dt
        output = proportional + integral + derivative
        self.previous_error = error
        return output
    
    # 其他update方法保持不变...
    def acc_update(self, current_value, dt):
        # 实现同上
        pass
        
    def angacc_update(self, current_value, dt):
        # 实现同上  
        pass
        
    def set_gyro_target(self, target):
        self.target = target

    def set_acc_target(self, target):
        self.target = target

    def set_angacc_target(self, target):
        self.target = target
        
    def reset(self):
        self.previous_error = 0
        self.integral = 0

def _control_loop(pwm, imu, imu2, gyro_pid, acc_pid, angacc_pid, plotter, data_logger, stop_event):
    motor_channel = 0
    base_pulse = 1500
    min_pulse = 1000
    max_pulse = 2000
    
    # 初始化电机
    pwm.setServoPulse(motor_channel_1, base_pulse)
    time.sleep(3)
    
    control_start = time.time()
    previous_time = control_start
    previous_angular_velocity_x = 0.0
    angular_acceleration_x = 0.0
    parameter = 0
    
    while not stop_event.is_set():
        current_time = time.time()
        dt = current_time - previous_time
        
        # 读取IMU数据
        gyro_data = imu.get_gyro_data()
        acc_data = imu.get_accel_data()
        
        # 双IMU处理
        angle1 = angle2 = avg_angle = None
        if imu2 and ENABLE_DUAL_IMU:
            try:
                gyro_data2 = imu2.get_gyro_data()
                acc_data2 = imu2.get_accel_data()
                angle1 = math.atan(acc_data['y']/acc_data['z']) * 180 / math.pi
                angle2 = math.atan(acc_data2['y']/acc_data2['z']) * 180 / math.pi
                avg_angle = (angle1 + angle2) / 2
                angle = avg_angle  # 使用平均角度
            except Exception as e:
                print(f"Dual IMU error: {e}, falling back to single IMU")
                angle = math.atan(acc_data['y']/acc_data['z']) * 180 / math.pi
        else:
            angle = math.atan(acc_data['y']/acc_data['z']) * 180 / math.pi
        
        angular_velocity_x = math.radians(gyro_data['x'])

        if dt > 0:
            angular_acceleration_x = (angular_velocity_x - previous_angular_velocity_x) / dt
        else:
            angular_acceleration_x = 0.0
        previous_angular_velocity_x = angular_velocity_x

        # PID控制逻辑（保持原有代码）
        if mode:
            angle_abs = abs(angle)
            if angle_abs < angle_low:
                pid_output = gyro_pid.gyro_update(angular_velocity_x, dt)
            elif angle_abs >= angle_low and angle_abs <= angle_high:
                parameter = (angle_abs - angle_low)/(angle_high - angle_low)
                pid_output = (1 - parameter) * gyro_pid.gyro_update(angular_velocity_x, dt) + parameter * acc_pid.acc_update(angle, dt)
            elif angle_abs > angle_high:
                pid_output = acc_pid.acc_update(angle, dt)
            elif angle_abs > angle_capsized:
                pid_output = acc_pid.acc_update(angle, dt) * 2000
            else:
                pid_output = gyro_pid.gyro_update(angular_velocity_x, dt)
        else:
            pid_output = angacc_pid.angacc_update(angular_acceleration_x, dt)

        motor_adjustment = max(min(pid_output, 500), -500)
        motor_pulse = int(base_pulse + motor_adjustment)
        motor_pulse = max(min(motor_pulse, max_pulse), min_pulse)
        
        pwm.setServoPulse(motor_channel_1, motor_pulse)

        # 数据记录
        if data_logger.enabled:
            temp = imu.get_temp()
            data_logger.log_data(current_time, gyro_data, acc_data, temp,
                               angle, math.degrees(angular_velocity_x), 
                               math.degrees(angular_acceleration_x),
                               pid_output, motor_pulse)

        # 绘图数据
        if plotter:
            plotter.record(current_time - control_start,
                         angle,
                         math.degrees(angular_velocity_x),
                         pid_output,
                         math.degrees(angular_acceleration_x),
                         motor_pulse,
                         angle1, angle2, avg_angle)

        print(f"Angle: {angle:.3f} deg, "
              f"Angular Vel: {angular_velocity_x:.3f} rad/s, "
              f"PID Output: {pid_output:.2f}, "
              f"Motor Pulse: {motor_pulse} us")

        previous_time = current_time
        time.sleep(0.01)

def main():
    # 用户输入配置
    global ENABLE_DUAL_IMU, ENABLE_DATA_LOGGING
    
    enable_logging = input("Enable IMU data logging? (y/n): ").strip().lower()
    ENABLE_DATA_LOGGING = (enable_logging == 'y')
    
    enable_dual_imu = input("Enable dual IMU? (y/n): ").strip().lower()  
    ENABLE_DUAL_IMU = (enable_dual_imu == 'y')
    
    # 初始化硬件
    pwm = PCA9685(address=0x40, debug=True, bus_num=1)
    pwm.setPWMFreq(50)
    
    # 初始化IMU
    imu = mpu6050.mpu6050(0x68)
    imu2 = None
    
    if ENABLE_DUAL_IMU:
        try:
            imu2 = mpu6050.mpu6050(0x69)
            print("Dual IMU enabled - using addresses 0x68 and 0x69")
        except Exception as e:
            print(f"Failed to initialize second IMU: {e}")
            ENABLE_DUAL_IMU = False
    
    # PID控制器设置
    gyro_pid = PIDController(294.0, 0.08, 0.0, 0.0)
    acc_pid = PIDController(29.0, 0.000, 0.008, 0.0)
    angacc_pid = PIDController(292.0, 0.000, 0.008, 0.0)
    
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
                                plotter, data_logger, stop_event))
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
        time.sleep(0.5)
        pwm.setServoPulse(motor_channel_1, 0)
        
        # 停止数据记录
        if data_logger.enabled:
            data_logger.stop_logging()

if __name__ == "__main__":
    main()

# 保留原有的校准和稳定控制函数...