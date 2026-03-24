# CSV 文件名参数格式说明

## 文件名结构
```
feedforward_dual_imu_Kp{kp}_Ki{ki}_Kd{kd}_FF{ff}_FB{fb}_DZ{dz}_{dzs}_ODZ{odz}_M{m}_W{w}_TS{ts}[_L{li}R{ri}]_{timestamp}.csv
```

## 参数缩写说明

### PID 控制参数组
- **Kp**: 比例增益 (Proportional gain)，范围 0-100，步长 0.1
- **Ki**: 积分增益 (Integral gain)，范围 0-10，步长 0.1
- **Kd**: 微分增益 (Derivative gain)，范围 0-1，步长 0.01

### 前馈/反馈控制组
- **FF**: 前馈参数 (Feedforward param)，范围 0-1，步长 0.01
- **FB**: 反馈参数 (Feedback param)，范围 0-1，步长 0.01

### 死区参数组
- **DZ**: 角度死区 (Angle deadzone)，单位：度，范围 0-5，步长 0.1
- **DZ_soft**: 角度软死区 (Angle deadzone soft)，单位：度，范围 0-10，步长 0.1
- **ODZ**: 角速度死区 (Omega deadzone)，单位：度/秒，范围 0-20，步长 0.5

### 物理参数组
- **M**: 质量 (Mass)，单位：kg，范围 50-150，步长 1
- **W**: 宽度 (Width)，单位：m，范围 0.3-1.0，步长 0.05

### 电机参数组
- **TS**: 力矩缩放 (Thrust scale)，范围 0-2，步长 0.01
- **LxRx**: 电机反转标志 (Motor invert)，其中 x 为 0 或 1
  - **L0**: 左电机正常方向
  - **L1**: 左电机反转方向
  - **R0**: 右电机正常方向
  - **R1**: 右电机反转方向
  - 仅当有电机反转时才会显示

## 文件名示例

### 示例 1：未反转电机
```
feedforward_dual_imu_Kp20.0_Ki1.000_Kd0.0_FF0.28_FB0.50_DZ1.0_3.0_ODZ6.0_M80_W0.60_TS0.55_20251216_184127.csv
```

### 示例 2：左电机反转
```
feedforward_dual_imu_Kp25.5_Ki1.500_Kd0.1_FF0.30_FB0.45_DZ1.5_4.0_ODZ8.0_M75_W0.65_TS0.60_L1R0_20251216_184200.csv
```

### 示例 3：两个电机都反转
```
feedforward_dual_imu_Kp30.0_Ki2.000_Kd0.2_FF0.35_FB0.55_DZ2.0_5.0_ODZ10.0_M90_W0.70_TS0.65_L1R1_20251216_184300.csv
```

## 如何通过文件名识别配置

1. **快速识别 PID 参数**：查看 `Kp` 后面的值
2. **比较前馈反馈策略**：查看 `FF` 和 `FB` 的比例
3. **识别死区设置**：查看 `DZ` 和 `ODZ` 的值，用于判断系统的响应灵敏度
4. **识别电机配置**：查看末尾是否有 `L` 和 `R` 标志
5. **识别时间戳**：最后的 `_YYYYMMDD_HHMMSS` 是数据记录的日期和时间

## 用于参数搜索和对比

### 查找特定 Kp 值的所有测试
```bash
ls feedforward_dual_imu_Kp20.0_*.csv
```

### 查找特定前馈值的所有测试
```bash
ls feedforward_dual_imu_*_FF0.28_*.csv
```

### 查找有电机反转的所有测试
```bash
ls feedforward_dual_imu_*_L1*.csv
```

### 查找特定日期的所有测试
```bash
ls feedforward_dual_imu_*_20251216_*.csv
```

## 参数缩放精度

- **PID**: 0.1 / 0.001 / 0.01 精度
- **前馈/反馈**: 0.01 精度
- **死区**: 0.1 精度
- **物理参数**: 整数 / 0.01 精度
- **力矩缩放**: 0.01 精度

这些精度设置确保文件名长度合理，同时保持足够的参数区分度。
