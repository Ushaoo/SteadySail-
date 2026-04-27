# SteadySail (ESP32-S3) 当前工程说明

本 README 对应当前仓库中的 ESP32-S3 主控代码实现，重点覆盖以下目录：

- `main/`
- `CMakeLists.txt`
- `main/CMakeLists.txt`
- `sdkconfig` / `config/`

说明：`raspberry_pi_backup/` 是历史备份，不属于当前控制链路，本文不展开。

---

## 1. 项目概览

SteadySail 是一个基于 ESP32-S3 的双推进器姿态稳定控制系统。当前实现采用：

- 主循环 100Hz（10ms 固定周期）
- MPU6050 IMU（默认单 IMU，可切换双 IMU）
- MT6826S PWM 编码器采样转向角
- LEDC 50Hz PWM 驱动转向电机与推进电调
- 前馈 + 2DOF PID 的横滚平衡控制
- 推进与平衡矢量耦合输出

核心目标：在给定前进推力的同时维持横滚稳定。

---

## 2. 代码结构

### 顶层构建文件

- `CMakeLists.txt`：ESP-IDF 工程入口，项目名 `version_2`
- `main/CMakeLists.txt`：注册以下组件源文件
  - `main.c`
  - `imu_driver.c`
  - `balance_controller.c`
  - `motor_control.c`
  - `steering_control.c`

### 主功能模块（main/）

- `main.c`
  - 系统初始化
  - 按模式创建任务
  - 100Hz 控制核心任务
  - 串口命令解析
- `system_config.h`
  - 运行模式
  - 转向控制模式
  - GPIO 映射
  - 控制与安全参数
- `imu_driver.c/.h`
  - I2C 初始化
  - MPU6050 唤醒与读取
  - 单/双 IMU 数据输出统一接口
- `balance_controller.c/.h`
  - Mahony 四元数姿态估计
  - 陀螺仪零偏自动校准
  - 前馈 + PID 力矩计算
- `steering_control.c/.h`
  - 编码器中断采样
  - 占空比转角度
  - 转向 PID（或直接映射）
- `motor_control.c/.h`
  - LEDC 初始化与 PWM 输出
  - 推进器双向推力输出
  - 推进器校准任务
  - 紧急停推

---

## 3. 运行模式

在 `main/system_config.h` 中通过 `CURRENT_RUN_MODE` 选择：

- `MODE_TEST_SENSORS` (0)
  - 传感器/编码器测试模式
- `MODE_TEST_STEERING_ONLY` (1)
  - 仅转向闭环测试，大电机锁定中立
- `MODE_TEST_BALANCE_ONLY` (2)
  - 仅平衡链路测试
- `MODE_FULL_INTEGRATION` (3)
  - 全链路运行（默认推荐）
- `MODE_CALIBRATE_ESC` (4)
  - 大电机 ESC 校准模式
- `MODE_TEST_IMU_ONLY` (5)
  - IMU 原始 + 融合数据测试

当前默认：`MODE_FULL_INTEGRATION`。

---

## 4. 转向控制模式

在 `main/system_config.h` 中通过 `STEERING_CONTROL_MODE` 切换：

- `STEERING_MODE_PID` (0)
  - PID 闭环，精度更高
- `STEERING_MODE_DIRECT` (1)
  - 直接映射，结构更简单

当前默认：`STEERING_MODE_PID`。

---

## 5. 硬件接口与引脚

来自 `main/system_config.h` 的当前定义：

- IMU I2C0：
  - SDA: GPIO8
  - SCL: GPIO9
- IMU I2C1（双 IMU 预留）：
  - SDA: GPIO10
  - SCL: GPIO11
- 编码器输入：
  - 左：GPIO4
  - 右：GPIO5
- 转向 PWM：
  - 左：GPIO1
  - 右：GPIO2
- 推进 PWM：
  - 左：GPIO18
  - 右：GPIO19

PWM 配置（电机与转向共用 LEDC Timer0）：

- 频率：50Hz
- 分辨率：14-bit
- 中立脉宽：1500us

---

## 6. 主控制流程（FULL_INTEGRATION）

`control_core_task()` 以 100Hz 运行，主要步骤：

1. 读取当前转向实际角度（编码器）
2. 读取 IMU 数据
3. 执行 `balance_controller_update()` 得到姿态和 `tau_total`
4. 执行安全保护：
   - 若 `ENABLE_EMERGENCY_STOP=1` 且 `|roll| > 60 deg`，立即停推并将转向回到 180
5. 计算前进推力分量与平衡推力分量
6. 计算目标转向角并做低通平滑
7. 根据实际舵角进行推力补偿（含 cos 项防退化处理）
8. 通过 `motor_control_set_pwm_bidirectional()` 下发双向推力
9. 更新编码器故障状态与日志
10. `vTaskDelayUntil()` 保持 10ms 周期

串口监视输出频率：约 10Hz。

---

## 7. 姿态融合与平衡算法

实现位于 `main/balance_controller.c`。

### 7.1 启动零偏校准

- 启动后累计 200 个样本（100Hz 下约 2 秒）
- 求陀螺仪零偏均值
- 校准期间输出中立状态

### 7.2 Mahony 姿态估计

- 四元数状态：`q0 q1 q2 q3`
- 由加速度构造重力参考误差进行修正
- 单 IMU 模式：固定 `Kp_mahony = 30.0`
- 双 IMU 模式：根据误差自适应增益

### 7.3 控制律

- 前馈：重力扰动 + 惯量项 + 虚拟刚度
- 反馈：2DOF PID（含积分限幅）
- 输出：
  - `tau_total = FEEDFORWARD_PARAM * tau_ff - FEEDBACK_PARAM * tau_pid`
- 额外处理：
  - 角度死区与软过渡
  - 角速度死区与软过渡
  - 综合因子衰减最终力矩

当前关键参数（`main/system_config.h`）：

- `PID_KP = 20.0`
- `PID_KI = 1.0`
- `PID_KD = 0.0`
- `FEEDFORWARD_PARAM = 0.28`
- `FEEDBACK_PARAM = 0.5`
- `ANGLE_DEADZONE = 1.0`
- `ANGLE_DEADZONE_SOFT = 3.0`

---

## 8. 转向闭环与编码器

实现位于 `main/steering_control.c`。

- 采用 GPIO 双边沿中断捕获 PWM 高低电平时长
- 占空比映射：5% 到 95% 对应 0 到 360 度
- 启动时执行 `steering_control_calibrate_encoders()`
  - 将当前姿态标定为 180 度附近参考
- `steering_control_update()` 按当前模式输出转向 PWM

编码器健康状态：

- 通过 `valid` 状态判断
- 主循环定期打印故障/恢复日志

---

## 9. 推进输出与安全约束

实现位于 `main/motor_control.c`。

### 9.1 推进输出

- 支持双向推力：`motor_control_set_pwm_bidirectional(push_L, push_R, invert_L, invert_R)`
- 推力输入先限幅后低通滤波
- 方向通过 `invert` 标志控制（正反转）

### 9.2 实际 PWM 约束

- 当前推进 PWM 最终限制为 `1400~1600us`
- 紧急停止时回到 `1500us`

### 9.3 校准模式

`MODE_CALIBRATE_ESC` 下运行独立任务：

- 循环扫描 `1000 -> 2000 -> 1000us`
- 每步打印期望/实际占空比
- 用于确认 ESC 响应区间

---

## 10. 串口交互（FULL_INTEGRATION）

在 `main.c` 主循环中：

- 输入 `w` / `W`：前进推力 +10%
- 输入 `s` / `S`：前进推力 -10%
- 输入 `space` 或空格：前进推力归零
- 输入数字：直接设置前进推力百分比（自动限制到 -100 到 100）

调试输出包含：

- 目标前进推力
- 左右目标/实际转向角
- Roll
- 左右 PWM
- 力矩与平衡分量
- 反转状态
- 编码器健康状态

---

## 11. 构建与烧录

本工程为标准 ESP-IDF 项目。

在工程根目录执行：

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p <PORT> flash monitor
```

如果需要切换运行模式或参数：

1. 修改 `main/system_config.h`
2. 重新 `idf.py build flash`

---

## 12. 当前默认配置（代码事实）

- 默认运行模式：`MODE_FULL_INTEGRATION`
- 默认 IMU 配置：单 IMU（`USE_DUAL_IMU = 0`）
- 默认控制频率：100Hz（`CONTROL_DT = 0.01`）
- 倾覆保护：开启（`ENABLE_EMERGENCY_STOP = 1`）
- I2C 恢复：开启（`ENABLE_I2C_RECOVERY = 1`）

---

## 13. 调参与排障建议

### 13.1 先做的检查

- IMU 是否成功初始化
- 编码器是否持续有效（串口状态是否出现 `X`）
- 推进 PWM 是否在预期区间变化

### 13.2 常见现象

- 船体抖动：先降低 `PID_KP` 或 `FEEDBACK_PARAM`
- 响应偏慢：适度提高 `PID_KP` 或 `FEEDFORWARD_PARAM`
- 转向滞后：检查编码器占空比质量与中断连线
- 推进无响应：先进入 `MODE_CALIBRATE_ESC` 验证 ESC

### 13.3 双 IMU 切换

- 将 `USE_DUAL_IMU` 改为 `1`
- 接入第二路 I2C 的 IMU 硬件
- 重新编译烧录

---

## 14. 版本说明

此 README 以当前仓库代码为准，目标是和现有实现保持一致，不包含历史备份目录的旧方案描述。
