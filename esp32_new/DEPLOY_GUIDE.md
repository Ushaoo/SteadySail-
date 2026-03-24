# ESP32 部署快速参考

## 🎯 5 分钟部署指南

### 第一次部署 (首次)

```
1. 下载 MicroPython 固件
   ↓
2. 烧录固件到 ESP32 (esptool)
   ↓
3. 上传所有文件到 ESP32 (Thonny IDE)
   ↓
4. 修改 config.py (WiFi, 参数)
   ↓
5. 通过 boot.py 自动启动或手动运行 main.py
```

### 再次部署 (之后)

```
1. 修改 config.py 或 main.py
2. 保存文件到 ESP32 (Thonny: Save to device)
3. ESP32 自动重启并运行新代码
```

---

## 📋 上传文件清单

### 必需文件 (⭐ 一定要上传)
- [ ] `main.py`
- [ ] `boot.py`
- [ ] `config.py`
- [ ] `calibration_imu1.json`
- [ ] `calibration_imu2.json`
- [ ] `lib/mpu6050.py`
- [ ] `lib/pca9685.py`

### 可选文件
- [ ] `README.md` (文档)
- [ ] `DEPLOY_GUIDE.md` (本文件)

---

## ⚙️ 烧录命令速查

### Windows PowerShell

```powershell
# 擦除 Flash
python -m esptool --chip esp32 --port COM3 erase_flash

# 烧录固件
python -m esptool --chip esp32 --port COM3 --baud 460800 write_flash -z 0x1000 esp32-20240105-v1.22.1.bin

# 验证
python -m esptool --chip esp32 --port COM3 flash_id
```

### Linux / macOS

```bash
# 擦除 Flash
esptool.py --chip esp32 --port /dev/ttyUSB0 erase_flash

# 烧录固件
esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 460800 write_flash -z 0x1000 esp32-20240105-v1.22.1.bin

# 验证
esptool.py --chip esp32 --port /dev/ttyUSB0 flash_id
```

---

## 🔌 硬件接线

### I2C 总线 (SCL/SDA)

```
ESP32 GPIO22 (SCL) ----[4.7kΩ]---- 3.3V
                   -------- IMU1/IMU2/PCA9685 SCL

ESP32 GPIO21 (SDA) ----[4.7kΩ]---- 3.3V
                   -------- IMU1/IMU2/PCA9685 SDA
```

### I2C 地址验证

```python
# 在 ESP32 REPL 中运行
from machine import I2C, Pin
i2c = I2C(1, scl=Pin(22), sda=Pin(21), freq=400000)
print(i2c.scan())  # 应该显示 [40, 68, 69] (十进制)
```

---

## 🔧 常用命令

### Thonny IDE 快捷方式

| 操作 | 快捷键 |
|------|--------|
| 保存文件到 ESP32 | Ctrl+S → "Save to device" |
| 运行代码 | F5 或 Ctrl+Enter |
| 停止运行 | Ctrl+C |
| 打开 REPL | 点击 Shell 标签 |

### REPL 常用命令

```python
# 查看文件列表
import os
os.listdir()

# 删除文件
import os
os.remove('old_file.py')

# 查看内存
import gc
print(gc.mem_free())  # 剩余内存

# 重启
import machine
machine.reset()

# 软件重置
machine.soft_reset()
```

---

## 🐛 调试技巧

### 看不到输出？

1. **检查串口连接**
   ```
   设备管理器 → COM 端口 → 确认 COM 号
   ```

2. **重新连接 REPL**
   ```
   Thonny: 工具 → 选项 → 解释器 → 重新连接
   ```

3. **检查 baud rate**
   ```
   应该是 115200 (默认)
   ```

### 程序卡住？

```python
# 按 Ctrl+C 中断

# 或者用软重启
Ctrl+D (在 REPL 中)
```

### I2C 扫描不到设备？

```python
from machine import I2C, Pin
i2c = I2C(1, scl=Pin(22), sda=Pin(21), freq=100000)  # 降低频率
devices = i2c.scan()
print([hex(addr) for addr in devices])  # 显示十六进制地址
```

---

## 📊 配置文件速查

### config.py 关键参数

```python
# 频率 (影响响应速度)
CONTROL_FREQUENCY = 100  # 100Hz 最佳

# PID (影响稳定性)
PID_KP = 20.0   # ↑ 快速反应，↓ 平稳
PID_KI = 1.0    # ↑ 消除偏差，↓ 减少积分饱和
PID_KD = 0.0    # ↑ 阻尼，↓ 响应速度

# 前馈反馈 (影响功耗和稳定性)
FEEDFORWARD_PARAM = 0.28  # ↑ 功耗↑，稳定↑
FEEDBACK_PARAM = 0.5      # ↑ 响应快，↓ 功耗低

# 死区 (避免抖动)
ANGLE_DEADZONE = 1.0       # ↑ 避免抖动，↓ 更灵敏
ANGLE_DEADZONE_SOFT = 3.0  # ↑ 过渡更柔和
```

---

## ✅ 验证清单

### 烧录完成后

- [ ] 串口能看到启动信息
- [ ] IMU 初始化成功
- [ ] PCA9685 初始化成功
- [ ] 校准完成

### 程序运行中

- [ ] IMU 数据在变化 (不是全 0)
- [ ] PWM 输出在变化 (不是固定值)
- [ ] CSV 文件被创建
- [ ] 推进器有响应

---

## 🆘 故障排查表

| 现象 | 可能原因 | 解决方案 |
|------|--------|--------|
| 无串口输出 | USB 驱动/连接 | 检查 COM 口，重新插拔 |
| I2C 扫描失败 | 接线错误 | 检查 GPIO21/22 连接 |
| IMU 数据为 0 | 地址错误或供电 | 运行 i2c.scan()，检查电压 |
| ESP32 重启 | 内存不足或 I2C 冲突 | 降低频率，禁用 WiFi |
| 推进器不动 | PCA9685 失败 | 检查 PWM 地址和供电 |
| 数据文件满 | SPIFFS 已满 | 删除旧 CSV，扩大分区 |

---

## 📦 MicroPython 固件下载

访问: https://micropython.org/download/esp32/

推荐版本:
- **最新稳定版**: `esp32-20240105-v1.22.1.bin`
- **通用版**: `esp32-idf3-20231226-v1.22.bin`

---

## 🎓 进阶技巧

### 持久化日志

```python
# 在 config.py 中设置
ENABLE_DATA_LOGGING = True  # 自动保存到 CSV
```

### 远程监控 (WiFi)

```python
# 在 config.py 中设置
ENABLE_WIFI = True
WIFI_SSID = "your_network"
WIFI_PASSWORD = "password"
```

### 动态调参

```python
# 在 REPL 中修改参数
from main import controller
controller.pid.update_gains(25.0, 1.5, 0.5)  # 新的 KP, KI, KD
```

---

**最后更新**: 2026-03-24
**维护者**: SteadySail Team
