# ESP32 MicroPython 项目配置文件

# ==================== I2C 配置 ====================
I2C_SCL_PIN = 22        # GPIO 22 - SCL 时钟线
I2C_SDA_PIN = 21        # GPIO 21 - SDA 数据线
I2C_FREQ = 400000       # I2C 频率 (400 kHz)

# ==================== IMU 配置 ====================
IMU1_ADDRESS = 0x68     # IMU1 I2C 地址
IMU2_ADDRESS = 0x69     # IMU2 I2C 地址
CALIBRATION_SAMPLES = 200  # 校准采样数

# ==================== PCA9685 配置 ====================
PCA9685_ADDRESS = 0x40  # PCA9685 I2C 地址
PWM_FREQ = 50           # PWM 频率 (50 Hz)
PWM_CHANNELS = 16       # PWM 通道数

# ==================== 电机配置 ====================
LEFT_THRUSTER = 0       # 左推进器通道
RIGHT_THRUSTER = 1      # 右推进器通道
BASE_PULSE = 1500       # 中立脉宽 (μs)
MIN_PULSE = 1000        # 最小脉宽 (μs)
MAX_PULSE = 2000        # 最大脉宽 (μs)
THRUST_SCALE = 0.55     # 推力到 PWM 的缩放系数

# ==================== 控制参数 ====================
DT = 0.01               # 控制周期 (s)
CONTROL_FREQUENCY = 100 # 控制循环频率 (Hz)

# ==================== PID 参数 ====================
PID_KP = 20.0           # 比例增益
PID_KI = 1.0            # 积分增益
PID_KD = 0.0            # 微分增益
PID_B = 0.8             # 比例权重 (2-DOF PID)
PID_C = 0.0             # 微分权重 (2-DOF PID)

# ==================== IMU 融合参数 ====================
ALPHA_ACC = 0.98        # 加速度互补滤波系数
ALPHA_EMA = 0.15        # EMA 滤波系数

# ==================== 前馈反馈参数 ====================
FEEDFORWARD_PARAM = 0.28  # 前馈力矩缩放系数
FEEDBACK_PARAM = 0.5      # 反馈力矩缩放系数

# ==================== 物理参数 ====================
MASS = 80.0             # 总质量 (kg)
WIDTH = 0.6             # 船宽 (m)
G = 9.81                # 重力加速度 (m/s²)

# ==================== 死区参数 ====================
ANGLE_DEADZONE = 1.0        # 角度死区核心 (°)
ANGLE_DEADZONE_SOFT = 3.0   # 角度死区软边界 (°)

# ==================== 数据记录配置 ====================
ENABLE_DATA_LOGGING = True   # 是否启用数据记录
CSV_BUFFER_SIZE = 100        # CSV 缓冲区大小 (行)

# ==================== WiFi 配置 (可选) ====================
ENABLE_WIFI = False          # 是否启用 WiFi
WIFI_SSID = "your_ssid"      # WiFi SSID
WIFI_PASSWORD = "your_pass"  # WiFi 密码
UDP_PORT = 5005              # UDP 发送端口
TARGET_IP = "192.168.1.100"  # 目标 IP

# ==================== 调试配置 ====================
DEBUG = True                 # 调试输出
VERBOSE = False              # 详细输出
