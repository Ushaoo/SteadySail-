# ⚡ 快速调试指南

**适用场景**: 遇到问题时的快速排查流程

---

## 🔴 症状诊断表

快速查找问题的根源。

### 症状 1: 没有任何日志输出

| 可能原因 | 检查项 | 解决方案 |
|---------|--------|--------|
| 串口选择错误 | 设备管理器中的 COM 端口 | 重新选择正确的 COM 端口 |
| 波特率不匹配 | VS Code 监控窗口波特率 | 设为 115200 |
| 硬件故障 | USB 连接状态 | 更换 USB 线，重启 ESP32 |

**快速修复**:
```
1. 按 ESP32 上的 RST 按钮
2. 重新启动监控工具
3. 如果仍无输出，检查 USB 驱动程序
```

---

### 症状 2: I2C 扫描找不到设备

| 可能原因 | 检查命令 | 解决方案 |
|---------|---------|---------|
| 硬件未连接 | 查看日志: "I2C scan..." | 检查接线 |
| I2C 地址冲突 | 两个 IMU 地址相同 | 检查 AD0 引脚 |
| 上拉电阻缺失 | 用万用表测 SDA/SCL | 焊接 4.7kΩ 上拉电阻 |

**快速排查步骤**:
```
步骤 1: 在 config.h 中临时打印地址
#define IMU1_ADDRESS        0x68
#define IMU2_ADDRESS        0x69
#define PCA9685_ADDRESS     0x40

步骤 2: 重新编译烧录

步骤 3: 查看日志输出
"I2C scan found device at: 0x68"  ← 应该看到这些
"I2C scan found device at: 0x69"
"I2C scan found device at: 0x40"

如果没有，逐一检查接线
```

---

### 症状 3: IMU 初始化失败

**日志示例**:
```
E (523) MPU6050: Failed to initialize MPU6050 at 0x68
```

| 可能原因 | 检查项 | 解决方案 |
|---------|--------|---------|
| I2C 通信失败 | 硬件连接 | 检查 SDA/SCL 接线 |
| 传感器损坏 | 尝试另一个 IMU | 更换传感器 |
| 地址错误 | config.h 中的地址 | 修改地址定义 |

---

### 症状 4: 船体持续振荡

**日志显示**:
```
Loops=500 | Attitude: roll=-5.2° | PID: Δ=120.5
Loops=1000 | Attitude: roll=4.8° | PID: Δ=-115.2
```

**快速修复** (从小到大尝试):
```
第 1 步 (最快): 减小 Kp
#define PID_KP              15.0f  // 原为 20.0

第 2 步: 增大死区
#define ANGLE_DEADZONE      2.0f   // 原为 1.0
#define ANGLE_DEADZONE_SOFT 4.0f   // 原为 3.0

第 3 步: 强化低通滤波
#define ALPHA_EMA           0.1f   // 原为 0.15
```

---

### 症状 5: 电机不响应

**日志示例** (无电机相关日志):
```
Motor controller initialized
Control loop task created
(但看不到电机做出反应)
```

| 可能原因 | 检查项 | 解决方案 |
|---------|--------|---------|
| PCA9685 未初始化 | 日志: "PCA9685 @ 0x40 initialized" | 检查 I2C 连接 |
| 脉宽超出范围 | config.h: MIN_PULSE, MAX_PULSE | 确保在 1000-2000 范围 |
| 电机未接好 | PCA9685 的通道 0, 1 | 检查电路连接 |

**快速诊断代码** (添加到 main.c):
```c
// 临时测试，烧录后观察电机是否抖动
static void test_motor_output(void) {
    pca9685_t pca;
    pca9685_init(&pca, PCA9685_ADDRESS, 50);
    
    // 推进器摇摆测试
    for (int i = 0; i < 5; i++) {
        pca9685_set_pulse(&pca, LEFT_THRUSTER, 1600);  // 向前
        pca9685_set_pulse(&pca, RIGHT_THRUSTER, 1400);
        vTaskDelay(pdMS_TO_TICKS(500));
        
        pca9685_set_pulse(&pca, LEFT_THRUSTER, 1400);  // 向后
        pca9685_set_pulse(&pca, RIGHT_THRUSTER, 1600);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    // 回到中立
    pca9685_set_pulse(&pca, LEFT_THRUSTER, 1500);
    pca9685_set_pulse(&pca, RIGHT_THRUSTER, 1500);
}
```

---

## 📋 分步故障排查清单

按照顺序逐一检查:

### ✅ 硬件层面

```
□ ESP32-S3 是否连接到电脑?
   → 检查 USB 线是否正确


□ I2C 接线是否正确?
   GPIO21 (SDA) ←→ 是否连接到传感器 SDA
   GPIO22 (SCL) ←→ 是否连接到传感器 SCL
   GND         ←→ 是否连接到传感器 GND
   3V3         ←→ 是否连接到传感器 VCC


□ 上拉电阻是否存在?
   → I2C 的 SDA 和 SCL 各需要一个 4.7kΩ 的上拉电阻到 3V3


□ IMU 的 AD0 引脚配置是否正确?
   IMU1: AD0 ←→ GND (地址 0x68)
   IMU2: AD0 ←→ VCC (地址 0x69)


□ PCA9685 是否正确供电?
   VCC ←→ 3V3, GND ←→ GND
```

### ✅ 驱动层面

```
□ ESP-IDF 是否正确安装?
   在 PowerShell 中运行: idf.py --version
   应显示: "esp-idf version v5.x"


□ VS Code 扩展是否正确配置?
   按 Ctrl+Shift+P → "ESP-IDF: Configure ESP-IDF Extension"
   选择已有的 IDF 路径或安装新的


□ Python 环境是否满足要求?
   在 PowerShell 中运行: python --version
   应为 3.8 或以上
```

### ✅ 编译层面

```
□ 编译是否通过没有错误?
   查看编译输出中是否出现 "error:" 字样
   
   如果有错误:
   → 检查 config.h 是否有语法错误
   → 清除编译缓存: Click "Clean Build" in VS Code


□ 是否成功生成 .bin 文件?
   检查 build/esp-idf... 文件夹中是否有 .bin 文件
```

### ✅ 烧录层面

```
□ 烧录是否完成而无错误?
   日志中应显示: "Wrote XXX bytes to address 0x00000000"
   
   如果烧录失败:
   → 更换 USB 线
   → 按 ESP32 上的 RST 按钮后重试
   → 重新选择 COM 端口


□ ESP32 是否正确复位?
   烧录完成后，应在日志中看到启动信息
```

### ✅ 运行时层面

```
□ 是否看到系统初始化日志?
   "=== SteadySail ESP32-S3 System Initialization ==="
   
   没有? → 重启监控工具或按 RST 按钮


□ I2C 设备是否被检测到?
   "I2C scan found device at: 0x68"
   "I2C scan found device at: 0x69"
   "I2C scan found device at: 0x40"
   
   少了某个设备? → 检查该设备的硬件接线


□ 校准是否成功?
   "MPU6050 @ 0x68 calibration complete!"
   "Gyro bias: x=0.05, y=0.02, z=-0.01"
   
   如果零偏过大 (>1.0)? → 重新校准或检查传感器


□ 控制循环是否启动?
   "Control loop task created (priority=20, stack=8192 bytes)"
   
   没有? → 检查 FreeRTOS 内存是否充足
```

---

## 🔧 常见快速修复

### 问题 1: "Device not found"

```bash
# 快速修复步骤
1. 断开 USB 线
2. 等待 2 秒
3. 重新连接 USB 线
4. 按 VS Code 中的 Flash 按钮
5. 如果还是不行，更换 USB 线
```

### 问题 2: 编译失败 "IDF_PATH not set"

```
快速修复:
1. 按 Ctrl+Shift+P
2. 输入 "Delete Profile"
3. 删除当前配置
4. 重启 VS Code
5. 重新编译 (会自动配置)
```

### 问题 3: "Build failed"

```c
// 快速修复清单:
1. √ 检查 config.h 有无语法错误
2. √ 检查 CMakeLists.txt 有无修改错误
3. √ 尝试 Clean Build (VS Code 中 Clean)
4. √ 删除 build/ 文件夹，重新编译
```

### 问题 4: 监控日志乱码

```
快速修复:
1. 重新选择波特率 (通常 115200)
2. 重新打开监控窗口
3. 按 ESP32 的 RST 按钮
4. 如果仍乱码，更换 USB 线或检查驱动
```

### 问题 5: 船体抖动或不稳定

```c
// 快速参数调整 (config.h)

// 第 1 步: 减小响应灵敏度
#define PID_KP              17.0f  // 原: 20.0

// 第 2 步: 增大死区范围
#define ANGLE_DEADZONE      1.5f   // 原: 1.0

// 第 3 步: 加强低通滤波
#define ALPHA_EMA           0.1f   // 原: 0.15

// 重新编译烧录后观察效果
```

---

## 🎯 性能监控

### 查看系统状态

每 5 秒查看一次状态日志:
```
I (xx) CONTROL_LOOP: Loops=500 | Attitude: roll=-0.5° | PID: Δ=5.2
```

**字段解释**:
- `Loops=500`: 已运行 500 个控制周期 (5 秒)
- `roll=-0.5°`: 当前翻滚角为 -0.5 度
- `PID: Δ=5.2`: PID 输出为 5.2

---

## 📊 实时数据监控

添加临时日志代码查看更多信息:

```c
// 在 control_loop.c 的 control_loop_task() 中添加
// 放在 Step 6 之后

if ((loop->loop_count % 100) == 0) {
    ESP_LOGI(TAG, "[STATUS] "
             "Roll=%.2f° | "
             "Gyro=%.2f°/s | "
             "PID=%.2f | "
             "L=%u, R=%u | "
             "Errors=%lu",
             euler.roll,
             gyro1.x,
             pid_output,
             pulse_left,
             pulse_right,
             loop->last_imu_error);
}
```

编译烧录后，每 1 秒会打印详细信息。

---

## 🚨 紧急处理

### 系统崩溃 (无日志输出)

```
1. 按 ESP32 上的 RST 按钮
2. 如果还是无响应，按住 BOOT 按钮 3 秒
3. 重新烧录固件
```

### 进入无限重启循环

```
日志示例:
ets Jun  8 2016 00:22:57
rst:0x7 (TG0WDT_SYS_RESET),...
ets Jun  8 2016 00:22:57
rst:0x7 (TG0WDT_SYS_RESET),...

原因: 代码有 bug 导致看门狗超时

解决:
1. 检查最近改过的代码
2. 回滚到上一个可用版本
3. 或者检查堆栈大小是否太小 (control_loop.h)
```

### I2C 总线锁定

```
日志示例 (所有 I2C 操作都失败):
W (xxx) I2C_DRIVER: I2C read failed from addr=0x68 reg=0x3B

原因: I2C 总线被某个从设备锁定

解决:
1. 断开 ESP32 的电源 5 秒
2. 检查是否有传感器接触不良
3. 尝试热插拔 USB 线
4. 更换 USB 线或传感器
```

---

## ✅ 调试完成检查表

```
系统正常运行的标志:
□ 能看到初始化日志
□ 能检测到 3 个 I2C 设备 (0x68, 0x69, 0x40)
□ 校准完成，零偏合理 (< 0.1)
□ 能看到每 500 个周期的状态日志
□ Roll 角度信号稳定，无异常跳跃
□ PID 输出在合理范围 (-200 ~ 200)
□ 电机会根据船体倾角做出响应

如果以上都满足 → 系统正常！✅
```

---

## 📞 更多帮助

- 📖 详细文档: 查看 `README.md`
- 💻 源代码: 所有 .c/.h 文件都有详细中文注释
- 🔍 参数说明: 查看 `config.h` 中的参数注释

---

**最后更新**: 2026-04-14
