# MicroPython 在 ESP32 上的完整指南

## 🎯 概述

**MicroPython** 是一个为微控制器优化的 Python 实现，ESP32 是官方支持的主要平台之一。

### 核心答案: **✅ YES，完全支持！**

---

## 📊 对比表

| 特性 | MicroPython | Arduino C++ | 原生 CircuitPython |
|------|------------|-----------|-----------------|
| **易用性** | ⭐⭐⭐⭐⭐ Python 代码 | ⭐⭐⭐ 需要 C++ | ⭐⭐⭐⭐⭐ 友好 |
| **性能** | ⭐⭐⭐⭐ 足够好 | ⭐⭐⭐⭐⭐ 最快 | ⭐⭐⭐ 较慢 |
| **库生态** | ⭐⭐⭐ 成长中 | ⭐⭐⭐⭐⭐ 最成熟 | ⭐⭐⭐⭐ 丰富 |
| **内存占用** | ⭐⭐⭐⭐ ~1MB | ⭐⭐⭐⭐⭐ ~300KB | ⭐⭐⭐ ~1.5MB |
| **社区支持** | ⭐⭐⭐⭐ 大 | ⭐⭐⭐⭐⭐ 最大 | ⭐⭐⭐⭐ 大 |
| **树莓派代码迁移** | ⭐⭐⭐⭐⭐ 最易 | ⭐ 完全重写 | ⭐⭐⭐⭐ 容易 |

---

## 🚀 MicroPython 在 ESP32 上的优势

### 1. **代码迁移成本最低** ✅
```python
# 树莓派原代码可以直接改为 MicroPython
import time
import json
from machine import I2C, Pin, PWM

# 大部分 Python 代码无需改动！
```

### 2. **硬件支持完美** ✅
- ✅ I2C (`machine.I2C`)
- ✅ SPI (`machine.SPI`)
- ✅ PWM (`machine.PWM`)
- ✅ GPIO (`machine.Pin`)
- ✅ UART (`machine.UART`)
- ✅ WiFi (`network`)
- ✅ 蓝牙 (`ubluetooth`)
- ✅ 文件系统 (`uos`, SPIFFS/LittleFS)

### 3. **内存足够** ✅
```
ESP32 标准配置:
- RAM: 520KB (可用)
- Flash: 4MB (可用 ~3.5MB)
- MicroPython 固件: ~1.2MB
- 剩余空间: ~2.3MB (足以运行你的代码 + 数据)
```

### 4. **开发速度快** ✅
- 不需要编译，直接上传运行
- 可以通过 REPL (Python 交互式终端) 调试
- 支持热重载代码

### 5. **库的可用性** ✅
```python
# 关键库都有 MicroPython 版本:
- json ✅
- time ✅
- math ✅
- collections ✅
- threading (受限但有) ⚠️
- ulab (NumPy 替代品) ✅
- socket (WiFi/UDP) ✅
```

---

## 🔧 安装 MicroPython 到 ESP32

### **步骤 1: 准备工具**
```bash
# 安装 esptool (烧录工具)
pip install esptool

# 下载 MicroPython 固件
# 访问: https://micropython.org/download/esp32/
# 下载最新版本，如: esp32-20240105-v1.22.1.bin
```

### **步骤 2: 擦除 Flash**
```bash
esptool.py --chip esp32 --port COM3 erase_flash
# 或在 Linux: /dev/ttyUSB0
```

### **步骤 3: 烧录固件**
```bash
esptool.py --chip esp32 --port COM3 --baud 460800 write_flash -z 0x1000 esp32-20240105-v1.22.1.bin
```

### **步骤 4: 验证安装**
```bash
# 使用 pyserial 连接 REPL
pip install pyserial

# Windows PowerShell:
python -m serial.tools.miniterm COM3 115200

# 看到 >>> 提示符表示成功！
```

---

## 📝 你的代码迁移计划

### **保留的代码 (无需改动)**

```python
# ✅ 全部保留 - 核心算法
class PID2DOF:
    def compute(self, setpoint, measured, omega_filtered):
        # ... (完全相同)

class DualIMUFusion:
    def update(self, ax1, ay1, az1, gx1, gy1, gz1, ...):
        # ... (完全相同)

def apply_deadzone_smooth(value, deadzone_core, deadzone_soft):
    # ... (完全相同)

# ✅ 全部保留 - 数据处理
class DataLogger:
    def log_data(self, ...):
        # ... (改动极小，见下文)

# ✅ 全部保留 - 基础工具
import json  # MicroPython 支持
import time  # MicroPython 支持
import math  # MicroPython 支持
```

### **需要改动的代码**

#### **1. 导入语句**
```python
# ❌ 树莓派
from motor_test import PCA9685
import mpu6050
from data_sender import DataSender
from web_pid_tuner import ParameterManager, WebPIDTuner

# ✅ MicroPython ESP32
from machine import I2C, Pin, PWM, ADC
import micropython
from network import WLAN, STA_IF
import socket

# ✅ 新增驱动 (需要下载到 ESP32)
import mpu6050_esp32  # 自己写的或从 GitHub 下载
import pca9685_esp32  # 自己写的或从 GitHub 下载
```

#### **2. I2C 初始化**
```python
# ❌ 树莓派
from motor_test import PCA9685
pwm = PCA9685()

# ✅ MicroPython ESP32
from machine import I2C, Pin
i2c = I2C(1, scl=Pin(22), sda=Pin(21), freq=400000)

# 然后初始化 PCA9685 和 MPU6050
pwm = PCA9685(i2c_bus=i2c, address=0x40)
imu1 = MPU6050(i2c_bus=i2c, address=0x68)
imu2 = MPU6050(i2c_bus=i2c, address=0x69)
```

#### **3. NumPy 替换**
```python
# ❌ 原代码
import numpy as np
sqrt_result = np.sqrt(x**2 + y**2 + z**2)

# ✅ MicroPython - 方案 A: 纯 Python
import math
sqrt_result = math.sqrt(x**2 + y**2 + z**2)

# ✅ MicroPython - 方案 B: 使用 ulab (可选)
import ulab.numpy as np
sqrt_result = np.sqrt(np.array([x, y, z]))  # 性能更好
```

#### **4. 文件存储**
```python
# ❌ 树莮派 - 无限制
with open(filename, 'w') as f:
    writer = csv.writer(f)
    # ... 不停写入

# ✅ MicroPython ESP32 - 缓冲存储
class DataLogger:
    def __init__(self):
        self.buffer = []
        self.buffer_size = 500  # 每 500 条记录写一次
    
    def log_data(self, ...):
        self.buffer.append([...])
        if len(self.buffer) >= self.buffer_size:
            self._flush_to_disk()
    
    def _flush_to_disk(self):
        with open('data.csv', 'a') as f:  # 追加模式
            for row in self.buffer:
                f.write(','.join(map(str, row)) + '\n')
        self.buffer.clear()
```

#### **5. Web 调参服务器**
```python
# ❌ 树莮派 - Flask (太重)
from web_pid_tuner import WebPIDTuner

# ✅ MicroPython - 选择方案
# 方案 A: 轻量级 HTTP 服务器
import uasyncio as asyncio

# 方案 B: WiFi + 串口通信 (最推荐)
import network
from machine import UART

# 方案 C: MQTT 云端调参
import umqtt
```

#### **6. 多线程处理**
```python
# ❌ 树莮派
from threading import Thread
def send_data():
    # ...
thread = Thread(target=send_data, daemon=True)
thread.start()

# ✅ MicroPython - 使用协程
import uasyncio as asyncio

async def send_data():
    while True:
        # ...
        await asyncio.sleep(0.05)

# 或使用 Timer
from machine import Timer
timer = Timer(1)
timer.init(period=50, mode=Timer.PERIODIC, callback=send_data_callback)
```

#### **7. WiFi UDP 数据发送**
```python
# ✅ MicroPython UDP 发送
import socket
import network

# 连接 WiFi
sta_if = network.WLAN(network.STA_IF)
sta_if.active(True)
sta_if.connect('WiFi_SSID', 'password')

# 创建 UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(1)

# 发送数据
data = struct.pack('!8d', timestamp, angle, angvel, ...)
sock.sendto(data, ('192.168.1.100', 5005))
```

---

## 📦 所需的 MicroPython 库清单

| 库 | 来源 | 说明 |
|----|------|------|
| `machine` | 内置 | I2C, PWM, GPIO, Timer |
| `time` | 内置 | 时间管理 |
| `json` | 内置 | 参数存储 |
| `math` | 内置 | 数学运算 |
| `socket` | 内置 | 网络通信 |
| `network` | 内置 | WiFi 连接 |
| `struct` | 内置 | 数据打包 |
| `mpu6050` | ⬇️ GitHub | IMU 驱动 |
| `pca9685` | ⬇️ GitHub 或自写 | PWM 驱动 |
| `ulab` | 可选 | NumPy 替代品 |

---

## 🔌 引脚配置 (ESP32)

```python
# I2C 引脚 (标准配置)
I2C_SDA = 21
I2C_SCL = 22

# PWM 引脚 (可选，用于电机直接驱动)
PWM_LEFT = 25
PWM_RIGHT = 26

# UART 引脚 (调试和通信)
UART_TX = 1
UART_RX = 3

# 初始化
from machine import I2C, Pin, UART
i2c = I2C(1, scl=Pin(22), sda=Pin(21), freq=400000)
uart = UART(0, baudrate=115200, tx=Pin(1), rx=Pin(3))
```

---

## 💾 项目文件结构 (MicroPython ESP32)

```
esp32_project/
├── main.py                      # 主程序入口
├── boot.py                      # 启动脚本
├── config.py                    # 配置文件
├── lib/
│   ├── mpu6050.py              # IMU 驱动
│   ├── pca9685.py              # PWM 驱动
│   ├── pid_controller.py        # PID 控制器
│   └── imu_fusion.py            # IMU 融合算法
├── calibration_imu1.json        # 校准参数
├── calibration_imu2.json        # 校准参数
└── pid_presets.json             # PID 预设
```

---

## ⚡ 性能预期

```
你的代码在 ESP32 上的运行能力:

✅ 控制循环频率: 100-200 Hz (足够!)
   (树莮派: 200-500 Hz)

✅ 数据采样率: 150-250 Hz (足够!)
   (树莮派: 250+ Hz)

✅ UDP 发送: 50 Hz (足够!)

✅ CSV 记录: 100 条/秒 (足够!)

⚠️ 实时性: 不如树莮派稳定
   (需要禁用中断，使用任务优先级)

⚠️ 同时运行多个操作时可能出现延迟
   (但对平衡控制影响不大)
```

---

## 🎓 推荐学习资源

1. **MicroPython 官方文档**
   - https://docs.micropython.org/en/latest/esp32/

2. **ESP32 引脚和功能**
   - https://docs.micropython.org/en/latest/esp32/quickref.html

3. **社区驱动库**
   - GitHub: `micropython-esp32-*` 搜索结果
   - PyPI: `micropython-*` 包

4. **开发工具**
   - **Thonny IDE** (最友好的 MicroPython IDE)
   - **VS Code + PyMakr** (高级用户)
   - **Arduino IDE** (用于 Arduino framework)

---

## ✨ 最终建议

### **你应该选择 MicroPython，因为:**

1. ✅ **代码迁移最简单** - 你现有的 Python 代码可以直接复用 70-80%
2. ✅ **学习曲线最平缓** - 不需要学习 C++ 或底层硬件知识
3. ✅ **开发速度最快** - 无需编译，直接上传运行
4. ✅ **调试最容易** - REPL 交互式终端让你快速定位问题
5. ✅ **社区支持最好** - MicroPython 社区活跃，问题容易解决
6. ✅ **性能足够** - 对于你的应用场景，性能完全够用

### **唯一的缺点:**
- ⚠️ 实时性不如原生 Arduino/C++ 代码
- ⚠️ 部分高级库不可用 (如 Flask)
- ⚠️ 调试较难 (但可用串口输出)

---

## 🚀 下一步行动

如果你确定要用 MicroPython，我可以为你生成:

1. **完整的 ESP32 MicroPython 项目模板** ✅
2. **MPU6050 和 PCA9685 的 MicroPython 驱动** ✅
3. **改写后的 feedforward_dual_imu.py** ✅
4. **WiFi + UDP 数据传输代码** ✅
5. **轻量级参数调整界面** (Web 或串口) ✅
6. **烧录和部署指南** ✅

**你想要哪个？** 💻
