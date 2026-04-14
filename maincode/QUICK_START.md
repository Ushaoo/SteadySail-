# 🚀 首次启动快速指南 (5 分钟)

**假设你什么都不知道，按照这个指南走完 5 步就能让系统运行！**

---

## ⏱ 时间规划

```
步骤 1: 环境检查 (1 分钟)
步骤 2: 硬件连接 (2 分钟)
步骤 3: 编译烧录 (1 分钟)
步骤 4: 验证运行 (1 分钟)
总计: 5 分钟
```

---

## ✅ 步骤 1: 环境检查 (1 分钟)

### 1.1 检查 VS Code

打开 VS Code，检查左侧扩展栏:

```
① 按 Ctrl+Shift+X 打开扩展
② 搜索 "ESP-IDF"
③ 应该看到 "Espressif IDF" 已安装 (有蓝色勾mark)

   ✅ 已安装 → 进入步骤 1.2
   ❌ 未安装 → 点击蓝色 "Install" 按钮，等待 1-2 分钟
```

### 1.2 检查 Python

打开 PowerShell (Win+R，输入 powershell):

```
输入: python --version
期望输出: Python 3.8.x 或更高版本
```

---

## 🔌 步骤 2: 硬件连接 (2 分钟)

### 2.1 连接开发板

用 USB-C 数据线连接 ESP32-S3 到电脑：

```
                    ┌─────────────┐
                    │  ESP32-S3   │
┌──────────────────→│ USB-C ◇◇◇◇  │←──────────────────┐
│                   └─────────────┘                    │
│                                                       │
电脑 USB                                         数据传输
(5V 供电)
```

**等待灯亮** (通常是蓝色 LED)

### 2.2 检查串口

打开 Windows 设备管理器 (Win+X，选择"设备管理器"):

```
① 展开 "COM 和并行端口"
② 应该看到 "USB Serial Device (COM3)" 或类似
   保存此 COM 号码 (后面要用)

   ✅ 看到了 → 进入步骤 3
   ❌ 看不到 → 更换 USB 线或检查接触
```

### 2.3 连接外围设备 (可选)

如果要测试传感器，按下图连接 IMU:

```
ESP32-S3          MPU6050 (IMU)
─────────────────────────────────
GPIO21 (SDA) ───→ SDA
GPIO22 (SCL) ───→ SCL  
GND         ───→ GND
3V3         ───→ VCC
```

---

## 🔨 步骤 3: 编译烧录 (1 分钟)

### 3.1 打开项目

在 VS Code 中:

```
File → Open Folder → 选择 d:\SteadySail--1\maincode
```

**等待 VS Code 索引项目** (右下角会提示)

### 3.2 编译

按下快键组合 **Ctrl+Shift+P**:

```
① 会弹出命令搜索框
② 输入: "build" (不要输入完整)
③ 看到 "ESP-IDF: Build your Project" 
④ 按 Enter

等待编译... (第一次需要 1-2 分钟)

成功标志:
  [100%] Built target steadysail_esp32s3
```

### 3.3 烧录

再次按 **Ctrl+Shift+P**:

```
① 输入: "flash"
② 选择 "ESP-IDF: Flash your Project"
③ 出现对话框要求选择串口
④ 选择步骤 2.2 中记录的 COM 号 (如 COM3)
⑤ 按 Enter
⑥ 烧录开始...

成功标志:
  Wrote XXX bytes
  Hash of data verified
```

---

## 🔍 步骤 4: 验证运行 (1 分钟)

### 4.1 打开监控

烧录完成后，再次按 **Ctrl+Shift+P**:

```
① 输入: "monitor"
② 选择 "ESP-IDF: Monitor your Device"
③ 会打开新的监控窗口
④ 等待日志输出...

大约 2-3 秒后应该看到:
```

### 4.2 正常输出示例

```
I (xx) MAIN: ============================================
I (xx) MAIN: SteadySail - Boat Stabilization System
I (xx) MAIN: ESP32-S3 Firmware v1.0
I (xx) MAIN: ============================================
I (xx) MAIN: === SteadySail ESP32-S3 System Initialization ===

I (xxx) I2C_DRIVER: I2C initialized: port=0, SDA=21, SCL=22, freq=400000 Hz
I (xxx) I2C_DRIVER: I2C scan found device at: 0x40    ← PCA9685
I (xxx) I2C_DRIVER: I2C scan...

I (xxx) MPU6050: MPU6050 @ 0x68 initialized
I (xxx) MPU6050: MPU6050 @ 0x69 initialized

I (xxx) CONTROL_LOOP: Control loop task created
I (xxx) MAIN: === System initialization complete ===
```

**✅ 看到此输出系统正常运行！**

---

## 🎯 常见问题快速解决

### Q1: 烧录时提示 "Device not found"

**解决** (10 秒):
```
1. 检查 USB 线是否正确插入
2. 在设备管理器中确认 COM 端口存在
3. 重新烧录时选择正确的 COM 端口
```

### Q2: 没有看到任何日志

**解决** (30 秒):
```
1. 按 ESP32 上的 RST 按钮 (小黑色按钮)
2. 重新启动监控 (Ctrl+Shift+P → monitor)
3. 等待 2-3 秒
```

### Q3: 编译失败，提示 "error"

**解决** (1 分钟):
```
1. 按 Ctrl+Shift+P
2. 输入 "clean" → "Clean Build"
3. 删除 build/ 文件夹
4. 重新编译
```

---

## 📚 下一步

### 了解系统

```
□ 打开 README.md，深入了解系统架构
□ 打开 config.h，查看可调参数
□ 运行监控，观察系统的运行状态
```

### 调试和优化

```
□ 查看 QUICK_DEBUG.md，学习故障排查
□ 查看 PARAMETER_REFERENCE.md，学习参数调整
□ 按照调参指南微调系统性能
```

---

## 🛑 停止运行

### 停止监控

```
在监控窗口中按 Ctrl+C
或点击窗口右上角的 × 按钮
```

### 重置开发板

```
按 ESP32 上的 RST 按钮可重启系统
```

---

## 📊 系统状态查看

### 每 5 秒的系统状态

```
I (xxxx) CONTROL_LOOP: Loops=500 | Attitude: roll=-0.5° | PID: Δ=5.2
```

**解释**:
- `Loops=500`: 系统已运行 500 个控制周期 (约 5 秒)
- `roll=-0.5°`: 当前翻滚角为 -0.5 度
- `PID: Δ=5.2`: PID 输出为 5.2

---

## ✨ 首次启动检查清单

```
□ 能看到初始化日志
□ 能看到 "3 devices at: 0x68, 0x69, 0x40"
   (如果接了传感器的话)
□ 看到 "Control loop task created"
□ 每 5 秒看到状态更新日志

全部 ✅ → 系统正常！
```

---

## 🎓 学习路径

### 第 1 天: 了解系统

```
1. 跑完这份快速指南 (15 分钟)
2. 阅读 README.md (20 分钟)
3. 理解系统架构图 (15 分钟)
```

### 第 2 天: 基础调试

```
1. 阅读 QUICK_DEBUG.md (30 分钟)
2. 观察系统日志，理解数据流 (30 分钟)
3. 尝试修改一个参数 (20 分钟)
```

### 第 3 天: 深入学习

```
1. 阅读源代码注释 (60 分钟)
2. 理解 PID 算法 (30 分钟)
3. 理解 IMU 融合算法 (30 分钟)
```

---

## 🆘 获得帮助

### 快速问题

| 问题 | 答案 |
|------|------|
| 系统怎么启动? | 按照此指南走 5 步 |
| 如何调参? | 查看 PARAMETER_REFERENCE.md |
| 系统出错了? | 查看 QUICK_DEBUG.md |
| 想改代码? | 主要改 config.h 或 imu_fusion.c |

### 查找信息

```
快速查找参数 → config.h (Ctrl+F 搜索)
快速查找问题 → QUICK_DEBUG.md 的症状诊断表
快速查找调参 → PARAMETER_REFERENCE.md
```

---

## 🎉 恭喜！

**你已经成功启动了 SteadySail 系统！**

现在可以：
- ✅ 观察系统运行
- ✅ 调整参数优化性能
- ✅ 进行深入学习

享受调试过程！🚀

---

**下一步建议**: 现在打开 `README.md` 了解项目细节。

---

最后更新: 2026-04-14
