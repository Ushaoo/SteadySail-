import time
import math
import numpy as np
from collections import deque
from scipy.signal import butter, lfilter
import threading
from queue import SimpleQueue, Empty

# Matplotlib 绘图
try:
    import matplotlib
    # 使用更快的后端（Qt5Agg 或 TkAgg）
    # matplotlib.use('TkAgg')  # 如果有问题可以取消注释
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.widgets import Slider
except ImportError:
    plt = None
    FuncAnimation = None

# 网络数据发送
try:
    from data_sender import DataSender
except ImportError:
    DataSender = None

# ==================== 硬件接口 ====================
from motor_test import PCA9685
import mpu6050

# PCA9685 PWM 驱动器
pwm = None  # 将在 main() 中初始化

# ==================== 绘图配置 ====================
ENABLE_PLOTTER = False       # 是否启用绘图
PLOT_WINDOW_SEC = 5.0         # 绘图窗口时长（秒）
PLOT_MAX_WINDOW_SEC = 30.0    # 最大窗口时长
PLOT_SAMPLE_RATE_HZ = 50.0    # 绘图采样率（降低以提高性能）
PLOT_UPDATE_INTERVAL = 50     # 绘图刷新间隔（ms），增大可提高性能

# ==================== 网络发送配置 ====================
ENABLE_DATA_SENDER = True     # 是否启用网络数据发送
DATA_SENDER_IP = "255.255.255.255"  # 目标IP（广播或电脑IP）
DATA_SENDER_PORT = 5005       # UDP端口

# ==================== IMU 配置 ====================
IMU1_INVERT_X = True
IMU1_INVERT_Y = True
IMU1_INVERT_Z = False

IMU2_INVERT_X = False
IMU2_INVERT_Y = False
IMU2_INVERT_Z = False

CALIBRATION_SAMPLES = 500

# ==================== 物理参数（根据实际测量调整） ====================
I_xx = 200             # 滚转转动惯量 kg·m²
m_total = 95.0            # 总质量 kg
h_cg = 0.25               # 重心高度 m
K_omega_ff = 14.0         # 角速度前馈系数 N·m·s/rad

# 推进器参数
L_thruster = 0.45         # 推进器到中线距离 m
PWM_MIN = 1000            # 倒转满速 μs
PWM_MAX = 2000            # 正转满速 μs
PWM_NEUTRAL = 1500        # 中位（停转）μs
F_MAX_ONE = 180.0         # 单侧最大推力 N（正转时）
F_MIN_ONE = -180.0        # 单侧最小推力 N（倒转时）

# 推进器通道（PCA9685 通道号 0-15）
THRUSTER_LEFT_CHANNEL = 0
THRUSTER_RIGHT_CHANNEL = 1

# 控制频率
CONTROL_FREQ = 100        # Hz（Pi 4B 建议 200-400Hz）
DT = 1.0 / CONTROL_FREQ

# 融合参数
ALPHA_ACC = 0.98          # 加速度互补滤波系数

# ==================== 前馈滤波器 ====================
b_ff, a_ff = butter(2, 40 / (CONTROL_FREQ / 2), 'low')
ff_filter_state = np.zeros(2)

def lowpass_2nd(x_new):
    global ff_filter_state
    y, ff_filter_state = lfilter(b_ff, a_ff, [x_new], zi=ff_filter_state)
    return y[0]

# ==================== 偏置力矩滤波器（超慢低通，截止频率 0.03Hz ≈ 33秒时间常数） ====================
b_bias, a_bias = butter(1, 0.03 / (CONTROL_FREQ / 2), 'low')
bias_filter_state = np.array([0.0])

# ==================== 重力项专用超低通滤波器（6Hz） ====================
b_grav, a_grav = butter(2, 6 / (CONTROL_FREQ / 2), 'low')

class LowPassFilter:
    def __init__(self, b=b_grav, a=a_grav):
        self.b = b
        self.a = a
        self.zi = np.zeros(2)
    
    def filter(self, x):
        y, self.zi = lfilter(self.b, self.a, [x], zi=self.zi)
        return y[0]
    
    def reset(self):
        self.zi = np.zeros(2)

lowpass_gravity = LowPassFilter()

# 3Hz 超狠重力滤波器（用于死区内）
b_grav_3hz, a_grav_3hz = butter(2, 3 / (CONTROL_FREQ / 2), 'low')
lowpass_gravity_3hz = LowPassFilter(b_grav_3hz, a_grav_3hz)

# ==================== 终极防 Bang-Bang 参数 ====================
DEADZONE_ANGLE = 2.8 * np.pi / 180      # ±2.8° 死区（实船最优值）
DEADZONE_OMEGA = 8.0 * np.pi / 180      # ±8°/s
HYSTERESIS_TIME = 0.8                   # 必须在死区内持续 0.8s 才认为稳定

# ==================== 双IMU融合类 ====================
class DualIMUFusion:
    def __init__(self):
        self.q0, self.q1, self.q2, self.q3 = 1.0, 0.0, 0.0, 0.0
        self.var_gx = [1.0, 1.0]
        self.var_gy = [1.0, 1.0]
        self.var_gz = [1.0, 1.0]
        self.gyro_buffer = [deque(maxlen=100), deque(maxlen=100)]
        
        # 上一次的角速度（用于计算角加速度）
        self.last_omega = 0.0
        self.omega_filtered = 0.0

    def update(self, ax1, ay1, az1, gx1, gy1, gz1, ax2, ay2, az2, gx2, gy2, gz2, dt):
        # 加速度归一化
        acc1_mag = np.sqrt(ax1**2 + ay1**2 + az1**2)
        acc2_mag = np.sqrt(ax2**2 + ay2**2 + az2**2)
        
        if acc1_mag > 0.1:
            ax1, ay1, az1 = ax1/acc1_mag, ay1/acc1_mag, az1/acc1_mag
        if acc2_mag > 0.1:
            ax2, ay2, az2 = ax2/acc2_mag, ay2/acc2_mag, az2/acc2_mag
        
        # 融合加速度
        ax = ALPHA_ACC * ax1 + (1-ALPHA_ACC) * ax2
        ay = ALPHA_ACC * ay1 + (1-ALPHA_ACC) * ay2
        az = ALPHA_ACC * az1 + (1-ALPHA_ACC) * az2
        
        mag = np.sqrt(ax**2 + ay**2 + az**2)
        if mag > 0.01:
            ax, ay, az = ax/mag, ay/mag, az/mag

        # 陀螺仪方差估计和加权
        for i, (gx, gy, gz) in enumerate([(gx1, gy1, gz1), (gx2, gy2, gz2)]):
            self.gyro_buffer[i].append([gx, gy, gz])
            if len(self.gyro_buffer[i]) == 100:
                arr = np.array(self.gyro_buffer[i])
                self.var_gx[i] = np.var(arr[:, 0]) + 1e-9
                self.var_gy[i] = np.var(arr[:, 1]) + 1e-9
                self.var_gz[i] = np.var(arr[:, 2]) + 1e-9

        # 计算权重
        w1 = 1.0 / (self.var_gx[0] + self.var_gy[0] + self.var_gz[0])
        w2 = 1.0 / (self.var_gx[1] + self.var_gy[1] + self.var_gz[1])
        total_w = w1 + w2
        w1, w2 = w1/total_w, w2/total_w

        # 融合陀螺仪
        gx = w1 * gx1 + w2 * gx2
        gy = w1 * gy1 + w2 * gy2
        gz = w1 * gz1 + w2 * gz2

        # 转换为弧度
        gx, gy, gz = np.radians(gx), np.radians(gy), np.radians(gz)

        # Mahony滤波
        acc_error = abs(mag - 1.0)
        
        vx = 2*(self.q1*self.q3 - self.q0*self.q2)
        vy = 2*(self.q0*self.q1 + self.q2*self.q3)
        vz = self.q0**2 - self.q1**2 - self.q2**2 + self.q3**2

        ex = ay*vz - az*vy
        ey = az*vx - ax*vz
        ez = ax*vy - ay*vx

        Kp = 2.0 + 25.0 * acc_error
        gx += Kp * ex
        gy += Kp * ey
        gz += Kp * ez

        # 四元数积分
        self.q0 += 0.5*dt * (-self.q1*gx - self.q2*gy - self.q3*gz)
        self.q1 += 0.5*dt * (self.q0*gx + self.q2*gz - self.q3*gy)
        self.q2 += 0.5*dt * (self.q0*gy - self.q1*gz + self.q3*gx)
        self.q3 += 0.5*dt * (self.q0*gz + self.q1*gy - self.q2*gx)

        norm = np.sqrt(self.q0**2 + self.q1**2 + self.q2**2 + self.q3**2)
        if norm > 1e-9:
            self.q0 /= norm
            self.q1 /= norm
            self.q2 /= norm
            self.q3 /= norm

        # 计算欧拉角
        roll = np.arctan2(2*(self.q0*self.q1 + self.q2*self.q3), 1-2*(self.q1**2 + self.q2**2))
        pitch = np.arcsin(np.clip(2*(self.q0*self.q2 - self.q3*self.q1), -1.0, 1.0))
        
        # 计算角速度（融合后，rad/s）
        omega = w1 * np.radians(gy1) + w2 * np.radians(gy2)  # roll 对应 Y 轴角速度
        
        # 低通滤波角速度
        alpha_omega = 0.8
        self.omega_filtered = alpha_omega * self.omega_filtered + (1 - alpha_omega) * omega
        
        # 计算角加速度
        alpha_acc = (self.omega_filtered - self.last_omega) / dt if dt > 0 else 0.0
        self.last_omega = self.omega_filtered

        return np.degrees(roll), np.degrees(pitch), self.omega_filtered, alpha_acc

# ==================== 平滑轨迹生成器（五次多项式） ====================
class SmoothTrajectory:
    def __init__(self):
        self.t0 = 0
        self.duration = 0
        self.theta0 = 0
        self.thetaf = 0
        self.active = False
    
    def start(self, current_rad, target_rad, duration_sec=0.4):
        """从当前角度平滑过渡到目标角度"""
        self.t0 = time.time()
        self.duration = max(duration_sec, 0.15)
        self.theta0 = current_rad
        self.thetaf = target_rad
        self.active = True
    
    def get_ref(self):
        if not self.active:
            return self.thetaf, 0.0, 0.0
        
        t = time.time() - self.t0
        if t >= self.duration:
            self.active = False
            return self.thetaf, 0.0, 0.0
        
        # 五次多项式系数
        d = self.thetaf - self.theta0
        T = self.duration
        
        a0 = self.theta0
        a3 = 10 * d / T**3
        a4 = -15 * d / T**4
        a5 = 6 * d / T**5
        
        theta = a0 + a3*t**3 + a4*t**4 + a5*t**5
        omega = 3*a3*t**2 + 4*a4*t**3 + 5*a5*t**4
        alpha = 6*a3*t + 12*a4*t**2 + 20*a5*t**3
        
        return theta, omega, alpha

# ==================== PID 控制器 ====================
class PIDController:
    def __init__(self, Kp, Ki, Kd, output_min, output_max):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral = 0.0
        self.last_error = 0.0
    
    def compute(self, error, dt):
        self.integral += error * dt
        # 积分限幅
        self.integral = np.clip(self.integral, self.output_min/self.Ki if self.Ki > 0 else -100, 
                                               self.output_max/self.Ki if self.Ki > 0 else 100)
        
        derivative = (error - self.last_error) / dt if dt > 0 else 0.0
        self.last_error = error
        
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        return np.clip(output, self.output_min, self.output_max)
    
    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0

# ==================== 实时绘图器 ====================
class FeedforwardPlotter:
    """实时绘图器，显示角度、角速度、电机输出"""
    
    def __init__(self, window_sec, max_window_sec, sample_rate_hz):
        if plt is None or FuncAnimation is None:
            raise RuntimeError("Matplotlib is required for plotting")
        
        self.window_sec = window_sec
        max_samples = int(max_window_sec * sample_rate_hz)
        
        # 数据缓冲区
        self.time_data = deque(maxlen=max_samples)
        self.angle_data = deque(maxlen=max_samples)
        self.angvel_data = deque(maxlen=max_samples)
        self.pwm_left_data = deque(maxlen=max_samples)
        self.pwm_right_data = deque(maxlen=max_samples)
        
        self.queue = SimpleQueue()
    
    def record(self, timestamp, angle_deg, angular_velocity_deg_s, pwm_left, pwm_right):
        """记录一帧数据（由控制循环调用）"""
        self.queue.put((timestamp, angle_deg, angular_velocity_deg_s, pwm_left, pwm_right))
    
    def run_plotter(self):
        """运行绘图器（在主线程中调用）"""
        fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 9))
        
        # 角度子图 - 预设Y轴范围避免频繁autoscale
        ax_angle = axes[0]
        self.line_angle, = ax_angle.plot([], [], 'b-', lw=1.5, label='Roll Angle')
        ax_angle.set_ylabel('Angle (deg)')
        ax_angle.set_ylim(-45, 45)  # 预设范围
        ax_angle.grid(True, alpha=0.3, linestyle='--')
        ax_angle.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
        self.text_angle = ax_angle.text(0.98, 0.95, '', transform=ax_angle.transAxes,
                                        verticalalignment='top', horizontalalignment='right',
                                        fontsize=10, bbox=dict(boxstyle='round,pad=0.3', fc='white', ec='gray', alpha=0.8))
        
        # 角速度子图
        ax_angvel = axes[1]
        self.line_angvel, = ax_angvel.plot([], [], 'orange', lw=1.5, label='Angular Velocity')
        ax_angvel.set_ylabel('Angular Velocity (deg/s)')
        ax_angvel.set_ylim(-180, 180)  # 预设范围
        ax_angvel.grid(True, alpha=0.3, linestyle='--')
        ax_angvel.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
        self.text_angvel = ax_angvel.text(0.98, 0.95, '', transform=ax_angvel.transAxes,
                                          verticalalignment='top', horizontalalignment='right',
                                          fontsize=10, bbox=dict(boxstyle='round,pad=0.3', fc='white', ec='gray', alpha=0.8))
        
        # 电机PWM输出子图
        ax_pwm = axes[2]
        self.line_pwm_left, = ax_pwm.plot([], [], 'g-', lw=1.5, label='Left PWM')
        self.line_pwm_right, = ax_pwm.plot([], [], 'r-', lw=1.5, label='Right PWM')
        ax_pwm.set_ylabel('Motor PWM (us)')
        ax_pwm.set_xlabel('Time (s)')
        ax_pwm.set_ylim(900, 2100)  # 预设范围
        ax_pwm.grid(True, alpha=0.3, linestyle='--')
        ax_pwm.axhline(y=PWM_NEUTRAL, color='gray', linestyle='--', alpha=0.5)
        ax_pwm.legend(loc='upper left')
        self.text_pwm = ax_pwm.text(0.98, 0.95, '', transform=ax_pwm.transAxes,
                                    verticalalignment='top', horizontalalignment='right',
                                    fontsize=10, bbox=dict(boxstyle='round,pad=0.3', fc='white', ec='gray', alpha=0.8))
        
        # 保存axes引用
        self.axes = axes
        
        # 滑块控制窗口大小
        slider_ax = fig.add_axes([0.15, 0.01, 0.7, 0.02])
        window_slider = Slider(slider_ax, 'Window (s)', 1.0, PLOT_MAX_WINDOW_SEC,
                               valinit=PLOT_WINDOW_SEC, valstep=0.5)
        
        def on_slider_change(val):
            self.window_sec = val
        
        window_slider.on_changed(on_slider_change)
        
        def update_plot_data():
            """从队列更新数据 - 批量处理提高效率"""
            updated = False
            batch_count = 0
            max_batch = 50  # 每帧最多处理50个数据点
            
            while batch_count < max_batch:
                try:
                    sample = self.queue.get_nowait()
                    timestamp, angle, angvel, pwm_l, pwm_r = sample
                    self.time_data.append(timestamp)
                    self.angle_data.append(angle)
                    self.angvel_data.append(angvel)
                    self.pwm_left_data.append(pwm_l)
                    self.pwm_right_data.append(pwm_r)
                    updated = True
                    batch_count += 1
                except Empty:
                    break
            
            if not updated or not self.time_data:
                return False
            
            # 根据窗口裁剪数据
            cutoff = self.time_data[-1] - self.window_sec
            while self.time_data and self.time_data[0] < cutoff:
                self.time_data.popleft()
                self.angle_data.popleft()
                self.angvel_data.popleft()
                self.pwm_left_data.popleft()
                self.pwm_right_data.popleft()
            
            return True
        
        def update(_frame):
            if not update_plot_data():
                return self.line_angle, self.line_angvel, self.line_pwm_left, self.line_pwm_right
            
            t_list = list(self.time_data)
            
            # 更新角度曲线（不调用autoscale，只更新X轴范围）
            self.line_angle.set_data(t_list, list(self.angle_data))
            if t_list:
                self.axes[0].set_xlim(t_list[0], t_list[-1])
                self.text_angle.set_text(f'{self.angle_data[-1]:.2f}°')
            
            # 更新角速度曲线
            self.line_angvel.set_data(t_list, list(self.angvel_data))
            if t_list:
                self.axes[1].set_xlim(t_list[0], t_list[-1])
                self.text_angvel.set_text(f'{self.angvel_data[-1]:.2f}°/s')
            
            # 更新PWM曲线
            self.line_pwm_left.set_data(t_list, list(self.pwm_left_data))
            self.line_pwm_right.set_data(t_list, list(self.pwm_right_data))
            if t_list:
                self.axes[2].set_xlim(t_list[0], t_list[-1])
                self.text_pwm.set_text(f'L:{self.pwm_left_data[-1]:.0f} R:{self.pwm_right_data[-1]:.0f}')
            
            return self.line_angle, self.line_angvel, self.line_pwm_left, self.line_pwm_right
        
        fig.subplots_adjust(left=0.1, right=0.98, top=0.96, bottom=0.08, hspace=0.3)
        fig.suptitle('SUP Feedforward Control Monitor', fontsize=12)
        
        # 使用更大的interval提高性能
        anim = FuncAnimation(fig, update, interval=PLOT_UPDATE_INTERVAL, blit=False, cache_frame_data=False)
        fig._feedforward_anim = anim  # 防止动画被垃圾回收
        plt.show()

# ==================== 全局变量 ====================
fusion = None
traj = None
pid_theta = None
pid_omega = None

# 状态变量（用于显示）
current_roll = 0.0
current_omega = 0.0
M_ff = 0.0
M_fb = 0.0
F_left = 0.0
F_right = 0.0
current_pwm_left = PWM_NEUTRAL
current_pwm_right = PWM_NEUTRAL
running = True

# 轨迹规划控制
last_traj_start = 0.0
traj_active = False

# 偏置力矩学习
bias_torque = 0.0              # 慢速学习的恒定偏置
last_bias_update = 0.0         # 上次偏置更新时间
bias_alpha = 0.015             # 学习速率（每2秒最多累积1.5%）
bias_from_fb_filtered = 0.0    # 超慢滤波偏置

# 防 Bang-Bang 状态
last_enter_deadzone = None     # 进入死区时间
bangbang_protection = False    # 是否处于防护状态

# 绘图器
plotter = None

# 数据发送器
data_sender = None

# IMU 和校准数据
mpu6050_1 = None
mpu6050_2 = None
bias_g1, bias_a1, bias_g2, bias_a2 = None, None, None, None

# ==================== IMU 校准 ====================
def calibrate_imus(samples=CALIBRATION_SAMPLES):
    print(f"开始IMU校准，请保持设备静止（Z轴向上）... 将采集 {samples} 个样本。")
    
    bias_g1 = {'x': 0, 'y': 0, 'z': 0}
    bias_a1 = {'x': 0, 'y': 0, 'z': 0}
    bias_g2 = {'x': 0, 'y': 0, 'z': 0}
    bias_a2 = {'x': 0, 'y': 0, 'z': 0}

    for i in range(samples):
        try:
            g1_raw = mpu6050_1.get_gyro_data()
            a1_raw = mpu6050_1.get_accel_data()
            g2_raw = mpu6050_2.get_gyro_data()
            a2_raw = mpu6050_2.get_accel_data()

            # 先应用轴向反转，再累加
            bias_g1['x'] += g1_raw['x'] * (-1 if IMU1_INVERT_X else 1)
            bias_g1['y'] += g1_raw['y'] * (-1 if IMU1_INVERT_Y else 1)
            bias_g1['z'] += g1_raw['z'] * (-1 if IMU1_INVERT_Z else 1)
            bias_a1['x'] += a1_raw['x'] * (-1 if IMU1_INVERT_X else 1)
            bias_a1['y'] += a1_raw['y'] * (-1 if IMU1_INVERT_Y else 1)
            bias_a1['z'] += a1_raw['z'] * (-1 if IMU1_INVERT_Z else 1)

            bias_g2['x'] += g2_raw['x'] * (-1 if IMU2_INVERT_X else 1)
            bias_g2['y'] += g2_raw['y'] * (-1 if IMU2_INVERT_Y else 1)
            bias_g2['z'] += g2_raw['z'] * (-1 if IMU2_INVERT_Z else 1)
            bias_a2['x'] += a2_raw['x'] * (-1 if IMU2_INVERT_X else 1)
            bias_a2['y'] += a2_raw['y'] * (-1 if IMU2_INVERT_Y else 1)
            bias_a2['z'] += a2_raw['z'] * (-1 if IMU2_INVERT_Z else 1)

            if (i + 1) % 100 == 0:
                print(f"已采集 {i + 1}/{samples}...")
            time.sleep(0.002)
        except OSError:
            print("校准期间读取传感器错误，跳过此样本")

    bias_g1 = {k: v / samples for k, v in bias_g1.items()}
    bias_a1 = {k: v / samples for k, v in bias_a1.items()}
    bias_g2 = {k: v / samples for k, v in bias_g2.items()}
    bias_a2 = {k: v / samples for k, v in bias_a2.items()}
    
    # Z轴零偏 = 测量值 - 9.8
    bias_a1['z'] -= 9.8
    bias_a2['z'] -= 9.8

    print("校准完成！")
    print(f"陀螺仪零偏 IMU1: x={bias_g1['x']:.3f}, y={bias_g1['y']:.3f}, z={bias_g1['z']:.3f}")
    print(f"陀螺仪零偏 IMU2: x={bias_g2['x']:.3f}, y={bias_g2['y']:.3f}, z={bias_g2['z']:.3f}")
    print(f"加速度计零偏 IMU1: x={bias_a1['x']:.3f}, y={bias_a1['y']:.3f}, z={bias_a1['z']:.3f}")
    print(f"加速度计零偏 IMU2: x={bias_a2['x']:.3f}, y={bias_a2['y']:.3f}, z={bias_a2['z']:.3f}")
    return bias_g1, bias_a1, bias_g2, bias_a2

# ==================== 读取IMU数据 ====================
def read_imu_data():
    try:
        accel_1_raw = mpu6050_1.get_accel_data()
        gyro_1_raw = mpu6050_1.get_gyro_data()
        accel_2_raw = mpu6050_2.get_accel_data()
        gyro_2_raw = mpu6050_2.get_gyro_data()

        # 先反转，再减零偏
        accel_1 = {
            'x': accel_1_raw['x'] * (-1 if IMU1_INVERT_X else 1) - bias_a1['x'],
            'y': accel_1_raw['y'] * (-1 if IMU1_INVERT_Y else 1) - bias_a1['y'],
            'z': accel_1_raw['z'] * (-1 if IMU1_INVERT_Z else 1) - bias_a1['z'],
        }
        gyro_1 = {
            'x': gyro_1_raw['x'] * (-1 if IMU1_INVERT_X else 1) - bias_g1['x'],
            'y': gyro_1_raw['y'] * (-1 if IMU1_INVERT_Y else 1) - bias_g1['y'],
            'z': gyro_1_raw['z'] * (-1 if IMU1_INVERT_Z else 1) - bias_g1['z'],
        }
        accel_2 = {
            'x': accel_2_raw['x'] * (-1 if IMU2_INVERT_X else 1) - bias_a2['x'],
            'y': accel_2_raw['y'] * (-1 if IMU2_INVERT_Y else 1) - bias_a2['y'],
            'z': accel_2_raw['z'] * (-1 if IMU2_INVERT_Z else 1) - bias_a2['z'],
        }
        gyro_2 = {
            'x': gyro_2_raw['x'] * (-1 if IMU2_INVERT_X else 1) - bias_g2['x'],
            'y': gyro_2_raw['y'] * (-1 if IMU2_INVERT_Y else 1) - bias_g2['y'],
            'z': gyro_2_raw['z'] * (-1 if IMU2_INVERT_Z else 1) - bias_g2['z'],
        }
        
        return accel_1, gyro_1, accel_2, gyro_2
    except OSError as e:
        print(f"IMU读取错误: {e}")
        return None, None, None, None

# ==================== 推进器控制 ====================
def set_thruster_pwm(left_us, right_us):
    """设置推进器PWM（微秒）"""
    if pwm is not None:
        pwm.setServoPulse(THRUSTER_LEFT_CHANNEL, left_us)
        pwm.setServoPulse(THRUSTER_RIGHT_CHANNEL, right_us)

def stop_thrusters():
    """停止推进器（发送中位信号）"""
    set_thruster_pwm(PWM_NEUTRAL, PWM_NEUTRAL)

# ==================== 主控制循环 ====================
def control_loop():
    global current_roll, current_omega, M_ff, M_fb, F_left, F_right, running
    global current_pwm_left, current_pwm_right
    
    print("控制循环启动...")
    
    # 初始化推进器（发送中位信号）
    stop_thrusters()
    time.sleep(3.0)  # ESC 初始化需要时间
    
    control_start = time.time()
    last_time = control_start
    
    while running:
        loop_start = time.time()
        dt = loop_start - last_time
        last_time = loop_start
        
        # 1. 读取IMU数据
        accel_1, gyro_1, accel_2, gyro_2 = read_imu_data()
        if accel_1 is None:
            time.sleep(DT)
            continue
        
        # 2. 姿态融合
        roll_deg, pitch_deg, omega_rad, alpha_rad = fusion.update(
            accel_1['x'], accel_1['y'], accel_1['z'],
            gyro_1['x'], gyro_1['y'], gyro_1['z'],
            accel_2['x'], accel_2['y'], accel_2['z'],
            gyro_2['x'], gyro_2['y'], gyro_2['z'],
            dt
        )
        
        current_roll = roll_deg
        current_omega = omega_rad
        theta_real = np.radians(roll_deg)
        current_time = loop_start  # 定义当前时间
        
        # 3. 终极防 Bang-Bang 三板斧
        global last_enter_deadzone, bangbang_protection, last_traj_start
        global bias_torque, last_bias_update, bias_from_fb_filtered, bias_filter_state, ff_filter_state
        
        in_deadzone = abs(theta_real) < DEADZONE_ANGLE and abs(omega_rad) < DEADZONE_OMEGA
        
        if in_deadzone:
            if last_enter_deadzone is None:
                last_enter_deadzone = current_time
            
            if current_time - last_enter_deadzone > HYSTERESIS_TIME:
                # 真正进入"静止区" → 所有动态前馈归零，只保留最平滑的重力补偿
                theta_ref = theta_real
                omega_ref = 0.0
                alpha_ref = 0.0
                
                # 重力前馈用超狠滤波（3Hz！）彻底消灭噪声
                raw_gravity = m_total * 9.81 * h_cg * np.sin(theta_real)
                gravity_torque = lowpass_gravity_3hz.filter(raw_gravity)
                
                M_ff_dynamic = gravity_torque   # 只有重力项！其余全关
                bangbang_protection = True
                
                # 强制清零所有学习偏置和PID积分（防止之前累积的"记忆"）
                bias_torque = 0.0
                pid_theta.reset()
                pid_omega.reset()
            else:
                bangbang_protection = False
        else:
            last_enter_deadzone = None
            bangbang_protection = False
        
        # 4. 正常动态前馈（只有不在死区时才启用）
        if not bangbang_protection:
            # 只有偏出死区才重新规划轨迹（杜绝频繁重规划！）
            if last_traj_start == 0.0 or current_time - last_traj_start > 0.4:
                recovery_time = max(0.25, min(0.6, abs(roll_deg) / 20))
                traj.start(theta_real, 0.0, recovery_time)
                last_traj_start = current_time
            
            theta_ref, omega_ref, alpha_ref = traj.get_ref()
            
            # 重力项单独低通 6Hz
            raw_gravity = m_total * 9.81 * h_cg * np.sin(theta_real)
            gravity_torque = lowpass_gravity.filter(raw_gravity)
            
            M_ff_dynamic = (I_xx * alpha_ref +         # 惯性前馈
                            K_omega_ff * omega_ref +    # 阻尼前馈
                            gravity_torque)             # 重力前馈（6Hz滤波）
        else:
            # 在死区内：轨迹强制跟随真实角度，alpha/omega 强制归零
            theta_ref = theta_real
            omega_ref = 0.0
            alpha_ref = 0.0
        
        # 5. 反馈力矩（PID）
        error_theta = theta_ref - theta_real
        error_omega = omega_ref - omega_rad
        
        M_fb_theta = pid_theta.compute(error_theta, dt)
        M_fb_omega = pid_omega.compute(error_omega, dt)
        M_fb = M_fb_theta + M_fb_omega
        
        # 6. 偏置力矩学习（补偿恒定偏差，死区内跳过）
        if not bangbang_protection:
            # 方案A：慢速学习恒定偏置（每2秒更新一次）
            if current_time - last_bias_update > 2.0:
                bias_torque += bias_alpha * M_fb  # 用反馈力矩的长期趋势学习
                bias_torque = np.clip(bias_torque, -90, 90)
                last_bias_update = current_time
            
            # 方案B：超慢低通滤波反馈作为第二偏置（最平滑）
            bias_filtered_arr, bias_filter_state = lfilter(
                b_bias, a_bias, [M_fb], zi=bias_filter_state)
            bias_from_fb_filtered = bias_filtered_arr[0]
        
        # 7. 最终力矩合成
        M_ff = M_ff_dynamic + bias_torque
        
        M_total = M_ff + M_fb             # 反馈只做最后微调
        M_total = np.clip(M_total, -220, 220)  # 安全限幅
        
        # 8. 力矩 → 差动推力
        # 正力矩让左电机正转，右电机倒转
        F_diff_left = +M_total / (2 * L_thruster)
        F_diff_right = -M_total / (2 * L_thruster)
        
        # 共模推力（可用于前进）
        F_common = 0.0
        
        # 推力可以为正（正转）或负（倒转）
        F_left = np.clip(F_common + F_diff_left, F_MIN_ONE, F_MAX_ONE)
        F_right = np.clip(F_common + F_diff_right, F_MIN_ONE, F_MAX_ONE)
        
        # 9. 推力 → PWM
        # F=0 -> 1500us, F=+180 -> 2000us, F=-180 -> 1000us
        pwm_left = int(np.interp(F_left, [F_MIN_ONE, 0, F_MAX_ONE], [PWM_MIN, PWM_NEUTRAL, PWM_MAX]))
        pwm_right = int(np.interp(F_right, [F_MIN_ONE, 0, F_MAX_ONE], [PWM_MIN, PWM_NEUTRAL, PWM_MAX]))
        
        # 更新全局变量
        current_pwm_left = pwm_left
        current_pwm_right = pwm_right
        
        # 10. 输出PWM
        set_thruster_pwm(pwm_left, pwm_right)
        
        # 11. 记录绘图数据
        if plotter is not None:
            plotter.record(
                loop_start - control_start,     # 相对时间（秒）
                roll_deg,                        # 角度（度）
                np.degrees(omega_rad),           # 角速度（度/秒）
                pwm_left,                        # 左电机PWM
                pwm_right                        # 右电机PWM
            )
        
        # 11b. 网络数据发送
        if data_sender is not None:
            data_sender.record(
                loop_start - control_start,     # 相对时间（秒）
                roll_deg,                        # 角度（度）
                np.degrees(omega_rad),           # 角速度（度/秒）
                pwm_left,                        # 左电机PWM
                pwm_right,                       # 右电机PWM
                M_ff,                            # 前馈力矩
                M_fb,                            # 反馈力矩
                1.0 if bangbang_protection else 0.0  # 防护状态
            )
        
        # 12. 精确定时
        elapsed = time.time() - loop_start
        sleep_time = DT - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)
    
    # 停止推进器
    stop_thrusters()
    print("控制循环已停止")

# ==================== 主程序 ====================
def main():
    global fusion, traj, pid_theta, pid_omega, running, pwm, plotter, data_sender
    global mpu6050_1, mpu6050_2, bias_g1, bias_a1, bias_g2, bias_a2
    
    print("=" * 50)
    print("SUP 主动稳定器 - 前馈控制测试")
    print("=" * 50)
    
    # 初始化 PCA9685 PWM 驱动器
    try:
        pwm = PCA9685(address=0x40, debug=False, bus_num=1)
        pwm.setPWMFreq(50)  # ESC 标准频率 50Hz
        print("PCA9685 PWM驱动器初始化成功")
    except Exception as e:
        print(f"PCA9685初始化失败: {e}")
        return
    
    # 初始化IMU
    try:
        mpu6050_1 = mpu6050.mpu6050(0x68)
        mpu6050_2 = mpu6050.mpu6050(0x68, 2)
        print("双IMU初始化成功")
    except OSError as e:
        print(f"IMU初始化失败: {e}")
        print("请检查I2C连接")
        return
    
    # 校准IMU
    bias_g1, bias_a1, bias_g2, bias_a2 = calibrate_imus()
    
    # 初始化融合算法
    fusion = DualIMUFusion()
    
    # 初始化轨迹生成器
    traj = SmoothTrajectory()
    
    # 初始化PID控制器（增益较小，主要靠前馈）
    pid_theta = PIDController(Kp=18.0, Ki=0.0, Kd=8.0, output_min=-80, output_max=80)
    pid_omega = PIDController(Kp=12.0, Ki=25.0, Kd=0.0, output_min=-100, output_max=100)
    
    # 初始化绘图器
    if ENABLE_PLOTTER and plt is not None:
        try:
            plotter = FeedforwardPlotter(PLOT_WINDOW_SEC, PLOT_MAX_WINDOW_SEC, PLOT_SAMPLE_RATE_HZ)
            print("实时绘图器已启用")
        except RuntimeError as e:
            print(f"绘图器初始化失败: {e}")
            plotter = None
    else:
        plotter = None
        print("绘图器已禁用")
    
    # 初始化数据发送器
    if ENABLE_DATA_SENDER and DataSender is not None:
        try:
            data_sender = DataSender(DATA_SENDER_IP, DATA_SENDER_PORT)
            data_sender.start()
            print(f"数据发送器已启用: {DATA_SENDER_IP}:{DATA_SENDER_PORT}")
        except Exception as e:
            print(f"数据发送器初始化失败: {e}")
            data_sender = None
    else:
        data_sender = None
        print("数据发送器已禁用")
    
    print("\n按 Enter 开始控制，按 Ctrl+C 停止...")
    input()
    
    # 启动控制线程
    control_thread = threading.Thread(target=control_loop, daemon=True)
    control_thread.start()
    
    # 如果启用绘图器，在主线程运行（matplotlib 需要在主线程）
    if plotter is not None:
        try:
            plotter.run_plotter()  # 阻塞直到关闭窗口
        except KeyboardInterrupt:
            pass
        finally:
            print("\n正在停止...")
            running = False
    else:
        # 主线程显示状态
        try:
            while running:
                print(f"角度: {current_roll:+6.2f}° | 角速度: {np.degrees(current_omega):+7.2f}°/s | "
                      f"左PWM: {current_pwm_left:4d}us | 右PWM: {current_pwm_right:4d}us")
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n正在停止...")
            running = False
    
    # 等待控制线程结束
    time.sleep(0.5)
    running = False
    control_thread.join(timeout=2.0)
    
    # 停止数据发送器
    if data_sender is not None:
        data_sender.stop()
        print("数据发送器已停止")
    
    # 安全停机
    stop_thrusters()
    time.sleep(0.5)
    if pwm is not None:
        pwm.setServoPulse(THRUSTER_LEFT_CHANNEL, 0)
        pwm.setServoPulse(THRUSTER_RIGHT_CHANNEL, 0)
    print("安全停机完成")

if __name__ == "__main__":
    main()
