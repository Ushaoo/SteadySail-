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

# ==================== 推进器旋转配置 (阶段1.1+) ====================
ROTATION_ENABLED = False              # 是否启用旋转功能（默认关闭）
LEFT_ROTATION_CHANNEL = 2             # 左旋转舵机 PCA9685 通道
RIGHT_ROTATION_CHANNEL = 3            # 右旋转舵机 PCA9685 通道
MAX_ROTATION_ANGLE = 45               # 最大旋转角 (度)
ROTATION_PULSE_MIN = 1000             # 旋转舵机最小脉宽 (μs, 对应 -45°)
ROTATION_PULSE_MAX = 2000             # 旋转舵机最大脉宽 (μs, 对应 +45°)
ROTATION_PULSE_CENTER = 1500          # 旋转舵机中立脉宽 (μs, 对应 0°)
MAX_THRUST_FORCE = 50.0               # 最大推力 (N) - 需根据实际标定
ROTATION_RATE_LIMIT = 90.0            # 旋转速率限制 (deg/s)

# ==================== 推进功能配置 (阶段1.3) ====================
PROPULSION_ENABLED = False             # 是否启用推进功能（默认关闭）
PROPULSION_MODE = "disabled"           # 推进模式: disabled, forward, backward, custom
PROPULSION_SPEED_TARGET = 0.0          # 目标推进速度 (0.0 ~ 1.0)
PROPULSION_DIRECTION = 0.0             # 目标推进方向 (度, 0=前进, 180=后退)
PROPULSION_MAX_SPEED = 0.5             # 最大推进速度系数 (0.0 ~ 1.0)
PROPULSION_PRIORITY = 0.7              # 推进优先级 vs 平衡 (0.0 ~ 1.0)
                                       # 0.0 = 全力平衡, 1.0 = 全力推进

# ==================== 旋转策略配置 (阶段1.2~1.3) ====================
ROTATION_STRATEGY = "automatic"        # 旋转策略: disabled, automatic, manual, custom
ROTATION_SYMMETRIC = True              # 两推进器是否对称旋转
ROTATION_INDEPENDENT = False           # 推进器是否独立旋转角
AUTO_TORQUE_CONVERSION = True          # 是否自动转矩转换为旋转角

# ==================== 约束和优化配置 ====================
BALANCE_PRIORITY_MODE = True           # 平衡优先模式 (平衡 > 推进)
MAX_ROLL_FOR_PROPULSION = 3.0          # 推进时允许的最大翻滚角 (度)
MIN_THRUST_FOR_PROPULSION = 10.0       # 推进时最小推力 (N)
DYNAMIC_PRIORITY_ADJUSTMENT = True     # 动态调整优先级 (基于翻滚角)

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
