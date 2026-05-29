import mpu6050
import time
import math
import numpy as np
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ==================== IMU 配置 ====================
# 每个IMU的XYZ轴是否需要反向
IMU1_INVERT_X = True
IMU1_INVERT_Y = True
IMU1_INVERT_Z = False

IMU2_INVERT_X = False
IMU2_INVERT_Y = False
IMU2_INVERT_Z = False

CALIBRATION_SAMPLES = 500 # 用于校准的采样次数
# =================================================

# ==================== 从 dual_imu_fusion.py 整合的参数 ====================
ALPHA_ACC = 0.98          # 加速度互补滤波系数（越大越信低频IMU）
WEIGHT_DYNAMIC = 0.8      # 动态时更信低噪声那颗
BIAS_UPDATE_RATE = 0.001  # 静止时零偏更新速度
# ========================================================================


# --- Plotting Configuration ---
MAX_SAMPLES = 100  # Number of samples to display on the plot
# --------------------------

# Initialize data deques for plotting
time_data = deque(maxlen=MAX_SAMPLES)
angle1_data = deque(maxlen=MAX_SAMPLES) # Original simple angle
angle2_data = deque(maxlen=MAX_SAMPLES) # Original simple angle
avg_angle_data = deque(maxlen=MAX_SAMPLES) # Original simple average
fused_roll_data = deque(maxlen=MAX_SAMPLES) # Fused roll angle
fused_pitch_data = deque(maxlen=MAX_SAMPLES) # Fused pitch angle


# ==================== 从 dual_imu_fusion.py 整合的类 ====================
class DualIMUFusion:
    def __init__(self):
        self.q0, self.q1, self.q2, self.q3 = 1.0, 0.0, 0.0, 0.0
        
        # 两颗IMU的零偏和协方差（在线估计）
        self.bias_gx = [0.0, 0.0]
        self.bias_gy = [0.0, 0.0]
        self.bias_gz = [0.0, 0.0]
        
        self.var_gx = [1.0, 1.0]   # 初始认为两颗一样
        self.var_gy = [1.0, 1.0]
        self.var_gz = [1.0, 1.0]
        
        # 滑动窗口估计噪声（100个样本）
        self.gyro_buffer = [deque(maxlen=100), deque(maxlen=100)]

    def set_gyro_bias(self, bias_g1, bias_g2):
        """设置初始的陀螺仪零偏"""
        self.bias_gx = [bias_g1['x'], bias_g2['x']]
        self.bias_gy = [bias_g1['y'], bias_g2['y']]
        self.bias_gz = [bias_g1['z'], bias_g2['z']]
        print(f"陀螺仪零偏已设置: IMU1={bias_g1}, IMU2={bias_g2}")

    def update(self, ax1,ay1,az1, gx1,gy1,gz1,  ax2,ay2,az2, gx2,gy2,gz2, dt):
        # Step 1：加速度互补得到最可信的重力方向（去除振动共模）
        acc1_mag = np.sqrt(ax1**2 + ay1**2 + az1**2)
        acc2_mag = np.sqrt(ax2**2 + ay2**2 + az2**2)
        
        # 归一化
        if acc1_mag > 0.1:
            ax1,ay1,az1 = ax1/acc1_mag, ay1/acc1_mag, az1/acc1_mag
        if acc2_mag > 0.1:
            ax2,ay2,az2 = ax2/acc2_mag, ay2/acc2_mag, az2/acc2_mag
            
        # 互补融合加速度
        ax = ALPHA_ACC * ax1 + (1-ALPHA_ACC) * ax2
        ay = ALPHA_ACC * ay1 + (1-ALPHA_ACC) * ay2
        az = ALPHA_ACC * az1 + (1-ALPHA_ACC) * az2
        
        # 重新归一化
        mag = np.sqrt(ax**2 + ay**2 + az**2)
        if mag > 0.01:
            ax, ay, az = ax/mag, ay/mag, az/mag

        # Step 2：角速度最优加权（噪声越小权重越大）
        # 在线估计方差（滑动窗口）
        for i, (gx, gy, gz) in enumerate([(gx1,gy1,gz1), (gx2,gy2,gz2)]):
            self.gyro_buffer[i].append([gx, gy, gz])
            if len(self.gyro_buffer[i]) == 100:
                arr = np.array(self.gyro_buffer[i])
                self.var_gx[i] = np.var(arr[:,0])
                self.var_gy[i] = np.var(arr[:,1])
                self.var_gz[i] = np.var(arr[:,2])

        # 权重 = 1/方差（归一化）
        # 避免除以零
        inv_var_gx0 = 1 / (self.var_gx[0] + 1e-9)
        inv_var_gy0 = 1 / (self.var_gy[0] + 1e-9)
        inv_var_gz0 = 1 / (self.var_gz[0] + 1e-9)
        inv_var_gx1 = 1 / (self.var_gx[1] + 1e-9)
        inv_var_gy1 = 1 / (self.var_gy[1] + 1e-9)
        inv_var_gz1 = 1 / (self.var_gz[1] + 1e-9)

        w1_total_inv_var = inv_var_gx0 + inv_var_gy0 + inv_var_gz0
        w2_total_inv_var = inv_var_gx1 + inv_var_gy1 + inv_var_gz1
        total_inv_var = w1_total_inv_var + w2_total_inv_var
        
        if total_inv_var < 1e-9: # 如果总方差接近0，则平均权重
            w1 = 0.5
        else:
            w1 = w1_total_inv_var / total_inv_var
        w2 = 1.0 - w1
        
        # 动态时进一步偏向噪声小的那个
        acc_error = abs(mag - 1.0)
        if acc_error > 0.3:  # 剧烈运动
            # 比较整体噪声水平
            if w1_total_inv_var > w2_total_inv_var: # IMU1 噪声更小
                w1 = WEIGHT_DYNAMIC
            else:
                w1 = 1 - WEIGHT_DYNAMIC
            w2 = 1 - w1

        # 陀螺仪零偏已在外部处理，直接加权融合
        gx = w1 * gx1 + w2 * gx2
        gy = w1 * gy1 + w2 * gy2
        gz = w1 * gz1 + w2 * gz2

        # Step 3：Mahony 更新（与单IMU完全一致）
        gx, gy, gz = np.radians(gx), np.radians(gy), np.radians(gz)

        # 重力预测与误差
        vx = 2*(self.q1*self.q3 - self.q0*self.q2)
        vy = 2*(self.q0*self.q1 + self.q2*self.q3)
        vz = self.q0**2 - self.q1**2 - self.q2**2 + self.q3**2

        ex = ay*vz - az*vy
        ey = az*vx - ax*vz
        ez = ax*vy - ay*vx

        # 动态Kp（翻板时完全信融合后的陀螺）
        Kp = 2.0 + 25.0 * acc_error
        gx += Kp * ex
        gy += Kp * ey
        gz += Kp * ez

        # 四元数积分
        self.q0 += 0.5*dt * (-self.q1*gx - self.q2*gy - self.q3*gz)
        self.q1 += 0.5*dt * ( self.q0*gx + self.q2*gz - self.q3*gy)
        self.q2 += 0.5*dt * ( self.q0*gy - self.q1*gz + self.q3*gx)
        self.q3 += 0.5*dt * ( self.q0*gz + self.q1*gy - self.q2*gx)

        # 归一化
        norm = np.sqrt(self.q0**2 + self.q1**2 + self.q2**2 + self.q3**2)
        if norm > 1e-9:
            self.q0 /= norm; self.q1 /= norm; self.q2 /= norm; self.q3 /= norm

        return self.quat_to_euler()

    def quat_to_euler(self):
        roll  = np.arctan2(2*(self.q0*self.q1 + self.q2*self.q3), 1-2*(self.q1**2 + self.q2**2))
        pitch = np.arcsin(np.clip(2*(self.q0*self.q2 - self.q3*self.q1), -1.0, 1.0))
        yaw   = np.arctan2(2*(self.q0*self.q3 + self.q1*self.q2), 1-2*(self.q2**2 + self.q3**2))
        return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)
# ========================================================================


# Initialize the MPU6050 sensors on different I2C addresses.
try:
    mpu6050_1 = mpu6050.mpu6050(0x68) 
    mpu6050_2 = mpu6050.mpu6050(0x68, 2) # Corrected address for the second IMU
    print("Successfully initialized both IMUs.")
except OSError as e:
    print(f"Failed to initialize IMUs: {e}")
    print("Please check I2C connections and addresses (run 'sudo i2cdetect -y 1').")
    exit()

# ==================== IMU 校准 ====================
def calibrate_imus(samples=CALIBRATION_SAMPLES):
    print(f"开始IMU校准，请保持设备静止（Z轴向上）... 将采集 {samples} 个样本。")
    
    # 陀螺仪和加速度计的零偏（应用轴向反转后的值）
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

            # 先应用轴向反转，再累加（这样零偏就是反转后的值）
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
    
    # 加速度计X/Y轴零偏就是测量平均值，Z轴零偏 = 测量值 - 期望值(9.8)
    # 校准后静止时应该是 x=0, y=0, z=9.8
    bias_a1['z'] = bias_a1['z'] - 9.8
    bias_a2['z'] = bias_a2['z'] - 9.8

    print("校准完成！")
    print(f"陀螺仪零偏 IMU1: x={bias_g1['x']:.3f}, y={bias_g1['y']:.3f}, z={bias_g1['z']:.3f}")
    print(f"陀螺仪零偏 IMU2: x={bias_g2['x']:.3f}, y={bias_g2['y']:.3f}, z={bias_g2['z']:.3f}")
    print(f"加速度计零偏 IMU1: x={bias_a1['x']:.3f}, y={bias_a1['y']:.3f}, z={bias_a1['z']:.3f}")
    print(f"加速度计零偏 IMU2: x={bias_a2['x']:.3f}, y={bias_a2['y']:.3f}, z={bias_a2['z']:.3f}")
    return bias_g1, bias_a1, bias_g2, bias_a2

# 执行校准
bias_g1, bias_a1, bias_g2, bias_a2 = calibrate_imus()
# =================================================

# Instantiate the fusion algorithm
fusion = DualIMUFusion()
fusion.set_gyro_bias(bias_g1, bias_g2) # 设置初始陀螺仪零偏

# 校准后静止时加速度应该是 (0, 0, 9.8)，对应 roll=0, pitch=0
# 初始化四元数为单位四元数即可（对应零姿态）
print("四元数初始化为单位四元数（零姿态）")

last_update_time = time.time()

# Define a function to read and process the sensor data
def read_and_process_data():
    global last_update_time
    try:
        # Read the accelerometer and gyroscope values from both sensors
        accel_1_raw = mpu6050_1.get_accel_data()
        gyro_1_raw = mpu6050_1.get_gyro_data()
        accel_2_raw = mpu6050_2.get_accel_data()
        gyro_2_raw = mpu6050_2.get_gyro_data()

        # --- 先应用轴向反转，再减去零偏（零偏是反转后的值） ---
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

        # --- Original simple angle calculation ---
        angle_1 = math.atan2(accel_1['y'], accel_1['z']) * 180 / math.pi
        angle_2 = math.atan2(accel_2['y'], accel_2['z']) * 180 / math.pi
        avg_angle = (angle_1 + angle_2) / 2
        
        # --- New fusion algorithm update ---
        current_time = time.time()
        dt = current_time - last_update_time
        last_update_time = current_time

        # 陀螺仪数据已经减去零偏，传入融合算法（算法内部不再减零偏）
        roll, pitch, yaw = fusion.update(
            accel_1['x'], accel_1['y'], accel_1['z'],
            gyro_1['x'], gyro_1['y'], gyro_1['z'],
            accel_2['x'], accel_2['y'], accel_2['z'],
            gyro_2['x'], gyro_2['y'], gyro_2['z'],
            dt
        )

        return angle_1, angle_2, avg_angle, roll, pitch, yaw
    except OSError as e:
        print(f"A sensor reading error occurred: {e}")
        return None, None, None, None, None, None


# --- Matplotlib Setup ---
fig, ax = plt.subplots()
line1, = ax.plot([], [], lw=1, label='IMU 1 Simple Angle')
line2, = ax.plot([], [], lw=1, label='IMU 2 Simple Angle')
line_avg, = ax.plot([], [], lw=2, linestyle=':', color='gray', label='Simple Average Angle')
line_fused_roll, = ax.plot([], [], lw=2, color='red', label='Fused Roll Angle')
line_fused_pitch, = ax.plot([], [], lw=2, color='blue', label='Fused Pitch Angle')

ax.legend()
ax.set_xlabel('Time (s)')
ax.set_ylabel('Angle (degrees)')
ax.set_title('Real-time IMU Angles: Simple vs Fused')
ax.grid(True)

start_time = time.time()

# This function is called periodically by FuncAnimation
def update(frame):
    # Read the sensor data
    angle_1, angle_2, avg_angle, fused_roll, fused_pitch, fused_yaw = read_and_process_data()

    if fused_roll is not None:
        current_time = time.time() - start_time
        
        # Append new data
        time_data.append(current_time)
        angle1_data.append(angle_1)
        angle2_data.append(angle_2)
        avg_angle_data.append(avg_angle)
        fused_roll_data.append(fused_roll)
        fused_pitch_data.append(fused_pitch)

        # Update plot data
        line1.set_data(time_data, angle1_data)
        line2.set_data(time_data, angle2_data)
        line_avg.set_data(time_data, avg_angle_data)
        line_fused_roll.set_data(time_data, fused_roll_data)
        line_fused_pitch.set_data(time_data, fused_pitch_data)

        # --- 手动调整坐标轴 ---
        # X轴
        if time_data:
            ax.set_xlim(time_data[0], time_data[-1]) 

        # Y轴
        all_data = list(angle1_data) + list(angle2_data) + list(fused_roll_data) + list(fused_pitch_data)
        if all_data:
            min_val = min(all_data)
            max_val = max(all_data)
            padding = (max_val - min_val) * 0.1 if (max_val - min_val) > 1e-6 else 1
            ax.set_ylim(min_val - padding, max_val + padding)
        # -------------------------
        
        # Print the latest values to the console
        print(f"Time: {current_time:.2f}s | Simple Avg: {avg_angle:.2f} | Fused Roll: {fused_roll:.2f} | Fused Pitch: {fused_pitch:.2f}")

    # blit=False allows axis labels to update automatically
    return line1, line2, line_avg, line_fused_roll, line_fused_pitch,

# Set up plot to call update() function periodically
# The interval is in milliseconds. 50ms = 20Hz update rate.
ani = FuncAnimation(fig, update, blit=False, interval=50)

try:
    print("Displaying plot. Close the plot window to stop the script.")
    plt.show()
except KeyboardInterrupt:
    print("Program stopped by user.")
finally:
    print("Script finished.")
