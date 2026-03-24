# ESP32 MicroPython 完整教程
# 从环境配置到代码运行的一步步指南

## 第一部分：准备开发环境

### 步骤 1: 安装 Python (如果还没有)

访问 https://www.python.org/downloads/ 下载最新版本 Python 3.8+

**Windows 安装时勾选**: "Add Python to PATH"

验证安装:
```bash
python --version
```

### 步骤 2: 创建虚拟环境 (推荐)

在项目目录执行:

**Windows PowerShell:**
```powershell
python -m venv venv
.\venv\Scripts\Activate.ps1
```

**Linux/Mac:**
```bash
python3 -m venv venv
source venv/bin/activate
```

看到命令提示符前有 `(venv)` 表示激活成功

### 步骤 3: 升级 pip

```bash
python -m pip install --upgrade pip
```

### 步骤 4: 安装 ESP32 开发工具

```bash
pip install -r esp32_new/requirements.txt
```

这会安装:
- esptool (固件烧录工具)
- pyserial (串口通信)
- mpremote (代码上传工具)

验证安装:
```bash
esptool.py version
```

应该看到类似输出:
```
esptool.py v3.3.2
```

---

## 第二部分：烧录 MicroPython 固件

### 步骤 5: 下载 MicroPython 固件

访问: https://micropython.org/download/esp32/

下载最新版本，例如: `esp32-20240105-v1.22.1.bin`

保存到易于找到的位置，如 `D:\Downloads\`

### 步骤 6: 连接 ESP32 到电脑

用 USB 数据线连接 ESP32 和电脑

**Windows**: 检查设备管理器，找到 COM 口号，如 COM3, COM4 等

**Linux**: 查看设备列表
```bash
ls /dev/ttyUSB*
```

**Mac**: 查看设备列表
```bash
ls /dev/tty.usbserial*
```

### 步骤 7: 擦除 ESP32 Flash

**Windows PowerShell:**
```powershell
esptool.py --chip esp32 --port COM3 erase_flash
```

**Linux/Mac:**
```bash
esptool.py --chip esp32 --port /dev/ttyUSB0 erase_flash
```

> 将 `COM3` 或 `/dev/ttyUSB0` 替换为你的实际端口

等待完成，应该看到:
```
Erasing flash on chip esp32...
Chip erase completed successfully
```

### 步骤 8: 烧录 MicroPython 固件

**Windows PowerShell:**
```powershell
esptool.py --chip esp32 --port COM3 --baud 460800 write_flash -z 0x1000 "D:\Downloads\esp32-20240105-v1.22.1.bin"
```

**Linux/Mac:**
```bash
esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 460800 write_flash -z 0x1000 ~/Downloads/esp32-20240105-v1.22.1.bin
```

等待完成，应该看到:
```
Wrote xxx bytes at address 0x00001000 in x.x seconds
Leaving...
```

### 步骤 9: 验证固件烧录成功

连接串口终端查看输出:

**使用 Thonny IDE (推荐，最简单)**

1. 下载安装 Thonny: https://thonny.org/
2. 打开 Thonny
3. 点击菜单: Tools → Options → Interpreter
4. 选择: MicroPython (ESP32)
5. 选择串口和波特率 (通常 115200)
6. 点击 REPL 窗口会自动连接

你应该看到:
```
MicroPython v1.22.1 on 2024-01-05; ESP32 module
Type "help()" for more information.
>>>
```

---

## 第三部分：上传项目代码

### 步骤 10: 准备代码文件

确保 `esp32_new/` 文件夹包含:
```
esp32_new/
├── main.py              (主程序)
├── boot.py              (启动脚本)
├── config.py            (配置文件)
└── lib/                 (驱动库)
    ├── mpu6050.py       (IMU 驱动)
    ├── pca9685.py       (PWM 驱动)
    ├── pid_controller.py (PID 控制器)
    ├── imu_fusion.py    (IMU 融合)
    └── wifi_helper.py   (WiFi 辅助)
```

### 步骤 11: 使用 Thonny 上传代码 (推荐方法)

1. 在 Thonny 中打开 `esp32_new/main.py`

2. 点击菜单: File → Properties → Current device location

3. 确保设置为 "MicroPython device"

4. 点击 File → Save 或 Ctrl+S

5. 选择保存到 "MicroPython device" (ESP32)

6. 对所有文件重复此过程

按优先级:
- config.py (配置)
- boot.py (启动)
- lib/ 文件夹中的所有文件
- main.py (主程序)

### 步骤 12: 使用命令行上传代码 (替代方法)

使用 mpremote:

**Windows PowerShell:**
```powershell
# 连接设备
mpremote connect COM3

# 上传单个文件
mpremote cp esp32_new/main.py :/main.py
mpremote cp esp32_new/config.py :/config.py
mpremote cp esp32_new/boot.py :/boot.py

# 上传整个文件夹
mpremote mkdir lib
mpremote cp esp32_new/lib/mpu6050.py :/lib/mpu6050.py
mpremote cp esp32_new/lib/pca9685.py :/lib/pca9685.py
mpremote cp esp32_new/lib/pid_controller.py :/lib/pid_controller.py
mpremote cp esp32_new/lib/imu_fusion.py :/lib/imu_fusion.py
mpremote cp esp32_new/lib/wifi_helper.py :/lib/wifi_helper.py
```

**Linux/Mac:**
```bash
# 连接设备
mpremote connect /dev/ttyUSB0

# 上传文件 (语法相同)
mpremote cp esp32_new/main.py :/main.py
# ... 其他文件
```

### 步骤 13: 上传配置文件

上传 JSON 配置文件:

**使用 Thonny:**
1. File → Open → 选择 `calibration_imu1.json`
2. File → Save → 保存到 "MicroPython device"
3. 文件会保存为 `/calibration_imu1.json`

重复上传:
- `calibration_imu1.json`
- `calibration_imu2.json`
- `pid_presets.json`

---

## 第四部分：配置和运行

### 步骤 14: 配置参数

编辑 `esp32_new/config.py`，根据你的硬件修改:

```python
# WiFi 配置
WIFI_SSID = "Your_WiFi_Name"
WIFI_PASSWORD = "Your_WiFi_Password"

# I2C 地址 (根据实际硬件调整)
IMU1_ADDRESS = 0x68
IMU2_ADDRESS = 0x69
PCA9685_ADDRESS = 0x40

# PID 参数 (根据需要调整)
PID_KP = 20.0
PID_KI = 1.0
PID_KD = 0.0

# 其他参数...
```

然后重新上传 config.py 文件

### 步骤 15: 启动程序

**方法 A: 使用 Thonny (推荐)**

1. Thonny 中打开 REPL 窗口
2. 输入并执行:
```python
>>> import main
```

3. 看到以下输出表示成功:
```
Starting SteadySail control system...
Initializing IMU 1...
Initializing IMU 2...
Initializing PCA9685...
Starting control loop...
```

**方法 B: 使用串口终端**

```bash
# 连接到串口
pyserial-miniterm COM3 115200  # Windows
picocom /dev/ttyUSB0 -b 115200 # Linux

# 在终端中输入
import main

# 或让它自动运行 (需要配置 boot.py)
```

**方法 C: 自动启动 (编辑 boot.py)**

修改 `boot.py`:
```python
# 自动导入主程序
import main
```

这样 ESP32 重启后会自动运行 main.py

### 步骤 16: 监视输出

在 Thonny 的 REPL 窗口或串口终端中查看实时输出:

```
IMU 1 Roll: 5.23 deg, Pitch: -2.15 deg, Yaw: 120.45 deg
IMU 2 Roll: 5.19 deg, Pitch: -2.18 deg, Yaw: 120.50 deg
Fused Roll: 5.21 deg, Pitch: -2.16 deg, Yaw: 120.47 deg
PWM Left: 1523 us, PWM Right: 1487 us
Control Loop Time: 9.8 ms
```

---

## 第五部分：故障排除

### 问题 1: 找不到 COM 口

**症状**: `Serial port not found` 错误

**解决方案**:
1. 检查设备管理器 (Windows) 或 `ls /dev/` (Linux)
2. 检查 USB 线是否正确连接
3. 检查 ESP32 驱动程序是否已安装
4. 尝试不同的 USB 口

### 问题 2: 固件烧录失败

**症状**: `Failed to connect to ESP32` 错误

**解决方案**:
1. 按住 ESP32 上的 BOOT 按钮
2. 然后执行烧录命令
3. 命令执行几秒后放开 BOOT 按钮
4. 或者降低波特率: `--baud 115200`

### 问题 3: IMU 无法读取

**症状**: `I2C communication error` 或 `No IMU found`

**解决方案**:
1. 检查 I2C 接线 (GPIO 21=SDA, GPIO 22=SCL)
2. 检查 config.py 中的地址是否正确
3. 运行 I2C 扫描:
```python
from machine import I2C, Pin
i2c = I2C(1, scl=Pin(22), sda=Pin(21), freq=400000)
devices = i2c.scan()
print("Found devices at:", [hex(d) for d in devices])
```

### 问题 4: PWM 输出异常

**症状**: 电机不动或动作异常

**解决方案**:
1. 检查 PCA9685 I2C 连接
2. 验证 PWM 频率设置 (应为 50 Hz)
3. 在 REPL 中测试 PWM 输出:
```python
from lib.pca9685 import PCA9685
from machine import I2C, Pin
i2c = I2C(1, scl=Pin(22), sda=Pin(21))
pca = PCA9685(i2c)
pca.setPWMFreq(50)
pca.setServoPulse(0, 1500)  # 设置通道 0 为中立位置
```

### 问题 5: WiFi 无法连接

**症状**: `WiFi connection failed` 或超时

**解决方案**:
1. 检查 SSID 和密码是否正确
2. 检查 WiFi 信号强度
3. 尝试重启 WiFi 路由器
4. 检查防火墙设置

### 问题 6: 程序崩溃

**症状**: ESP32 自动重启或冻结

**解决方案**:
1. 检查 REPL 中的错误信息
2. 增加栈大小 (在 boot.py 中)
3. 减少日志输出来节省内存
4. 检查是否有内存泄漏

---

## 第六部分：开发和调试

### 快速测试 I2C 和硬件

在 Thonny REPL 中逐句执行:

```python
# 测试 1: I2C 扫描
from machine import I2C, Pin
i2c = I2C(1, scl=Pin(22), sda=Pin(21), freq=400000)
print("I2C devices:", [hex(d) for d in i2c.scan()])

# 测试 2: IMU 读取
from lib.mpu6050 import MPU6050
imu = MPU6050(i2c, 0x68)
accel = imu.get_accel_data()
print("Accel:", accel)

# 测试 3: PWM 输出
from lib.pca9685 import PCA9685
pca = PCA9685(i2c)
pca.setServoPulse(0, 1500)  # 中立
print("PWM set to 1500 us")
```

### 修改代码后重新运行

1. 在 Thonny 中修改 `main.py`
2. File → Save (保存到 ESP32)
3. 在 REPL 中:
```python
import main  # 重新加载
```

### 添加调试输出

在 main.py 中添加 print 语句:

```python
print(f"Roll: {roll:.2f}, Pitch: {pitch:.2f}")
print(f"PWM Left: {pwm_left}, PWM Right: {pwm_right}")
```

实时查看数据

---

## 第七部分：性能优化

### 减少日志输出

编辑 config.py:
```python
DEBUG = False  # 禁用调试输出
LOG_INTERVAL = 100  # 每 100 次循环输出一次
```

### 优化内存使用

```python
import gc
gc.collect()  # 手动垃圾回收

# 在主循环中定期执行
if frame_count % 1000 == 0:
    gc.collect()
```

### 调整控制循环频率

在 config.py 中:
```python
CONTROL_FREQUENCY = 100  # Hz，降低可以省电
```

---

## 快速参考命令

### 常用 esptool 命令

```bash
# 查看版本
esptool.py version

# 读取 MAC 地址
esptool.py --port COM3 read_mac

# 擦除 Flash
esptool.py --chip esp32 --port COM3 erase_flash

# 烧录固件
esptool.py --chip esp32 --port COM3 write_flash -z 0x1000 firmware.bin

# 读取 Flash 内容
esptool.py --port COM3 read_flash 0x0 0x1000 dump.bin
```

### 常用 mpremote 命令

```bash
# 连接设备
mpremote connect COM3

# 列出文件
mpremote ls

# 上传文件
mpremote cp local_file.py :/device_file.py

# 下载文件
mpremote cp :/device_file.py local_file.py

# 执行命令
mpremote exec "import sys; print(sys.version)"

# 进入 REPL
mpremote repl
```

---

## 完整部署流程总结

1. ✅ 创建虚拟环境
2. ✅ 安装开发工具 (`pip install -r esp32_new/requirements.txt`)
3. ✅ 下载 MicroPython 固件
4. ✅ 擦除 ESP32 Flash
5. ✅ 烧录 MicroPython 固件
6. ✅ 验证固件成功 (查看 REPL)
7. ✅ 上传代码文件
8. ✅ 上传配置文件
9. ✅ 编辑 config.py (WiFi, 地址等)
10. ✅ 在 REPL 中运行 `import main`
11. ✅ 监视输出并调试

---

## 下一步

- 修改 config.py 调整控制参数
- 进行硬件测试
- 采集数据用于分析
- 根据测试结果优化算法

## 相关资源

- MicroPython 文档: https://docs.micropython.org/en/latest/esp32/
- ESP32 引脚: https://docs.micropython.org/en/latest/esp32/quickref.html
- Thonny 官网: https://thonny.org/
- 项目 README: 查看主文件夹的 README.md

---

**祝你开发愉快！有问题随时参考本教程。** 🚀
