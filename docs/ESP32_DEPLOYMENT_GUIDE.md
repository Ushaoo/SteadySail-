# ESP32 MicroPython 部署完整指南

## 📋 目录
1. [系统要求](#系统要求)
2. [准备工作](#准备工作)
3. [烧录 MicroPython 固件](#烧录-micropython-固件)
4. [上传项目文件](#上传项目文件)
5. [配置和运行](#配置和运行)
6. [故障排除](#故障排除)

---

## 系统要求

### 硬件
- **ESP32 开发板** (推荐: ESP32-DevKit-C 或类似)
- **USB 数据线** (A 口转 Micro-USB 或 USB-C)
- **IMU 传感器**: 2 个 MPU6050 (地址 0x68 和 0x69)
- **PWM 驱动板**: PCA9685 (地址 0x40)
- **电机和电源**

### 软件
- **Python 3.6+** (Windows/Linux/Mac)
- **esptool.py** (ESP32 烧录工具)
- **pyserial** (串口通信)
- **MicroPython 固件** (ESP32 版本)

---

## 准备工作

### 1. 安装 Python 工具

在 Windows PowerShell 中运行:

```powershell
pip install esptool pyserial
```

验证安装:
```powershell
esptool.py version
```

### 2. 下载 MicroPython 固件

访问 [MicroPython 官方下载页面](https://micropython.org/download/esp32/)

下载最新的稳定版本，如:
- `esp32-20240105-v1.22.1.bin`

保存到 `D:\Downloads` 或其他位置。

### 3. 识别 ESP32 串口

连接 ESP32 到电脑，在 PowerShell 中查看可用串口:

```powershell
# Windows PowerShell
Get-WmiObject Win32_PnPEntity | Where-Object {$_.Name -match "COM"} | Select-Object Name

# 或者更简单的方法
# 打开设备管理器 -> 端口 -> 查看 COM 号
```

假设 ESP32 连接到 `COM3`。

---

## 烧录 MicroPython 固件

### 步骤 1: 擦除 Flash

```powershell
esptool.py --chip esp32 --port COM3 erase_flash
```

等待完成 (大约 1-2 分钟)。

### 步骤 2: 烧录固件

```powershell
$firmware_path = "D:\Downloads\esp32-20240105-v1.22.1.bin"
esptool.py --chip esp32 --port COM3 --baud 460800 write_flash -z 0x1000 $firmware_path
```

### 步骤 3: 验证烧录

```powershell
esptool.py --chip esp32 --port COM3 read_mac
```

如果输出 MAC 地址，说明烧录成功！

---

## 上传项目文件

### 方案 A: 使用 Thonny IDE (推荐，图形化界面)

#### 下载和安装
1. 访问 [Thonny 官网](https://thonny.org/)
2. 下载并安装 Thonny IDE
3. 打开 Thonny，选择 `Tools` > `Options` > `Interpreter`
4. 选择 `MicroPython (ESP32)`，选择 `COM3`

#### 上传文件

1. **创建项目文件夹** (在 Thonny 中)
2. **创建 lib 文件夹**:
   - 右键 > New Folder > 命名为 `lib`

3. **上传驱动文件到 `/lib`**:
   - 将 `mpu6050_esp32.py` 上传到 `/lib`
   - 将 `pca9685_esp32.py` 上传到 `/lib`

4. **上传配置文件到根目录**:
   - `config_esp32.py` → `/` (根目录)

5. **上传主程序**:
   - `feedforward_dual_imu_esp32.py` → 重命名为 `main.py` 并上传到 `/`

6. **上传启动脚本**:
   - `boot_esp32.py` → 重命名为 `boot.py` 并上传到 `/`

7. **编辑校准数据** (可选):
   - 创建 `calibration_imu1.json`:
     ```json
     {
       "gyro_bias": {"x": 0.0, "y": 0.0, "z": 0.0}
     }
     ```
   - 创建 `calibration_imu2.json`:
     ```json
     {
       "gyro_bias": {"x": 0.0, "y": 0.0, "z": 0.0}
     }
     ```

### 方案 B: 使用命令行工具 (ampy)

```powershell
# 安装 ampy
pip install adafruit-ampy

# 创建 lib 文件夹
ampy --port COM3 mkdir /lib

# 上传驱动
ampy --port COM3 put mpu6050_esp32.py /lib/mpu6050_esp32.py
ampy --port COM3 put pca9685_esp32.py /lib/pca9685_esp32.py

# 上传配置
ampy --port COM3 put config_esp32.py config.py

# 上传主程序 (重命名为 main.py)
ampy --port COM3 put feedforward_dual_imu_esp32.py main.py

# 上传启动脚本 (重命名为 boot.py)
ampy --port COM3 put boot_esp32.py boot.py
```

---

## 配置和运行

### 步骤 1: 修改配置文件

在 Thonny 中打开 `config.py`，修改:

```python
# WiFi 配置
WIFI_CONFIG = {
    'ssid': 'your_wifi_ssid',      # 改为你的 WiFi
    'password': 'your_password',   # 改为你的密码
    'auto_connect': True,
}

# I2C 配置 (如果引脚不同)
I2C_CONFIG = {
    'scl_pin': 22,                 # SCL 引脚
    'sda_pin': 21,                 # SDA 引脚
}

# 电机配置 (如果需要反向)
MOTOR_CONFIG = {
    'left_invert': True,           # 修改电机反向
    'right_invert': False,
}
```

### 步骤 2: 连接硬件

确保以下连接正确:

```
ESP32 引脚 -> I2C 设备
GPIO 22 (SCL) -> PCA9685 SCL -> MPU6050(0x68) SCL -> MPU6050(0x69) SCL
GPIO 21 (SDA) -> PCA9685 SDA -> MPU6050(0x68) SDA -> MPU6050(0x69) SDA

电源连接:
5V -> PCA9685 VCC -> MPU6050(0x68) VCC -> MPU6050(0x69) VCC
GND -> PCA9685 GND -> MPU6050(0x68) GND -> MPU6050(0x69) GND
```

### 步骤 3: 首次运行

#### 使用 REPL (Thonny 的交互式终端)

```python
# 复制以下代码到 REPL

# 导入库
from machine import I2C, Pin
from mpu6050_esp32 import MPU6050

# 初始化 I2C
i2c = I2C(1, scl=Pin(22), sda=Pin(21), freq=400000)

# 扫描 I2C 设备
devices = i2c.scan()
print("发现的 I2C 设备地址:", [hex(addr) for addr in devices])

# 如果看到 0x68, 0x69, 0x40，说明硬件连接正确
```

#### 启动主程序

在 Thonny 中按 F5 运行 `main.py`，或在 REPL 中:

```python
exec(open('main.py').read())
```

### 步骤 4: 监控输出

在 Thonny 底部的 Shell 窗口中查看输出:

```
ESP32 MicroPython - 前馈+双IMU融合 控制器
==================================================
初始化 I2C (SDA=21, SCL=22)...
初始化控制器...
IMU 初始化成功
PCA9685 初始化成功
开始 IMU 校准...
校准完成，参数已保存
启动控制循环...
Roll: -0.23°, PWM: L=1480 R=1520
Roll: 0.15°, PWM: L=1510 R=1490
...
```

---

## 故障排除

### 问题 1: `无法解析导入 "machine"`

**原因**: 在 PC 上运行，不在 ESP32 上运行

**解决方案**: 
- 确保使用 Thonny 的 ESP32 解释器
- 或通过串口在 REPL 中运行

### 问题 2: `I2C 设备不响应`

**原因**: 硬件连接问题或引脚配置错误

**解决方案**:
```python
# 检查 I2C 设备
from machine import I2C, Pin
i2c = I2C(1, scl=Pin(22), sda=Pin(21))
devices = i2c.scan()
print(devices)  # 应该输出: [64, 104]  (0x40, 0x68, 0x69)
```

### 问题 3: `IMU 读取失败`

**原因**: 驱动程序不兼容或 I2C 速率太高

**解决方案**:
```python
# 降低 I2C 速率
i2c = I2C(1, scl=Pin(22), sda=Pin(21), freq=100000)  # 降至 100kHz

# 检查单个 IMU
from mpu6050_esp32 import MPU6050
imu = MPU6050(i2c, address=0x68)
accel = imu.get_accel()
print(accel)
```

### 问题 4: `运行时内存不足`

**原因**: ESP32 内存有限，某些操作占用过多

**解决方案**:
- 减少日志输出
- 缩小数据缓冲区
- 禁用不必要的功能

```python
# 在 main.py 中修改
DATA_CONFIG = {
    'enable_logging': False,  # 禁用日志记录
    'csv_buffer_size': 50,    # 减小缓冲
}
```

### 问题 5: `ESP32 无响应`

**原因**: 程序崩溃或无限循环

**解决方案**:
1. 按 Ctrl+C 中断
2. 重新启动 ESP32:
   ```python
   import machine
   machine.reset()
   ```
3. 检查 `main.py` 的错误

---

## 📁 最终文件结构

```
ESP32 Flash 存储:
/
├── boot.py                      # 启动脚本
├── main.py                      # 主程序 (feedforward_dual_imu_esp32.py 重命名)
├── config.py                    # 配置文件
├── calibration_imu1.json        # IMU1 校准参数
├── calibration_imu2.json        # IMU2 校准参数
├── lib/
│   ├── mpu6050_esp32.py         # MPU6050 驱动
│   └── pca9685_esp32.py         # PCA9685 驱动
└── feedforward_esp32_*.csv      # 数据日志 (运行时生成)
```

---

## 🎯 快速命令参考

```powershell
# 烧录固件
esptool.py --chip esp32 --port COM3 erase_flash
esptool.py --chip esp32 --port COM3 --baud 460800 write_flash -z 0x1000 firmware.bin

# 使用 ampy 上传文件
ampy --port COM3 put mpu6050_esp32.py /lib/mpu6050_esp32.py
ampy --port COM3 ls /

# 查看文件
ampy --port COM3 ls /

# 删除文件
ampy --port COM3 rm main.py
```

---

## ✅ 检查清单

- [ ] 安装了 esptool.py 和 pyserial
- [ ] 下载了 MicroPython 固件
- [ ] 成功烧录固件到 ESP32
- [ ] 上传了所有项目文件
- [ ] 配置了 WiFi SSID 和密码
- [ ] 检查了 I2C 硬件连接
- [ ] 验证了 I2C 设备地址
- [ ] 运行了校准程序
- [ ] 控制循环正常输出

---

## 💡 下一步

1. **调整 PID 参数**: 根据实际测试结果调整 `PID_KP`, `PID_KI`, `PID_KD`
2. **优化前馈参数**: 调整 `FEEDFORWARD_PARAM` 以获得最佳稳定性
3. **数据分析**: 下载 CSV 日志文件进行分析
4. **性能优化**: 监控 CPU 和内存使用情况

---

## 📞 常见问题

**Q: 如何修改控制循环频率?**
A: 在 `main.py` 中修改 `controller.run(frequency=100)` 的参数

**Q: 数据记录到哪里?**
A: SPIFFS 根目录，文件名为 `feedforward_esp32_*.csv`

**Q: 如何下载 CSV 数据?**
A: 使用 `ampy` 或 Thonny 下载文件

**Q: 能否通过 WiFi 调整参数?**
A: 可以，但需要实现一个简单的 HTTP 服务器 (后续添加)

---

**祝你部署顺利！** 🚀
