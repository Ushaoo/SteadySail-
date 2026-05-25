#include "bno055_driver.h"
#include "system_config.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "nvs.h"


/*
串口输入 magstat 查实时校准等级（每项 0~3）。
设备做"8 字"晃动直到 Mag=3、Sys=3。
输入 mag → 22 字节 calib profile 写入 NVS（namespace bno055 / key cal22）。
重启后 bno055_init() 自动恢复，无需再校。
*/


static const char *TAG = "BNO055";

// NVS namespace / key（22 字节校准 blob + 航向零点偏置）
#define BNO_NVS_NS              "bno055"
#define BNO_NVS_KEY_CAL         "cal22"
#define BNO_NVS_KEY_HDG_OFF     "hdg_off"  // float，单位度，0~360
#define BNO055_CALIB_DATA_ADDR  0x55   // ACC/MAG/GYR offset + radius 共 22 字节
#define BNO055_CALIB_STAT_ADDR  0x35

// 航向零点偏置：真北 heading = raw_heading - s_hdg_offset_deg（结果归一化到 0~360）
// 在 bno055_calibrate_and_save() 时被设为"当前 raw 航向"，使船头方向被定义为新的 0°（真北）。
static volatile float s_hdg_offset_deg = 0.0f;

// ==================== 寄存器地址 ====================
#define BNO055_CHIP_ID_ADDR      0x00   // 固定值 0xA0，用于芯片识别
#define BNO055_OPR_MODE_ADDR     0x3D   // 操作模式寄存器
#define BNO055_PWR_MODE_ADDR     0x3E   // 电源模式寄存器
#define BNO055_SYS_TRIGGER_ADDR  0x3F   // 系统触发（复位等）
#define BNO055_UNIT_SEL_ADDR     0x3B   // 单位选择（角度 or 弧度）
#define BNO055_EUL_HEADING_LSB   0x1A   // 欧拉角 Heading 低字节（Yaw）
#define BNO055_EUL_HEADING_MSB   0x1B   // 欧拉角 Heading 高字节
#define BNO055_EUL_ROLL_LSB      0x1C   // 欧拉角 Roll 低字节（横滚，1/16°）
#define BNO055_EUL_ROLL_MSB      0x1D   // 欧拉角 Roll 高字节
#define BNO055_GYR_DATA_X_LSB    0x14   // 陀螺仪 X 轴低字节（1/16 °/s）
#define BNO055_GYR_DATA_X_MSB    0x15   // 陀螺仪 X 轴高字节

// ==================== 操作模式 ====================
#define BNO055_OPR_MODE_CONFIG   0x00   // 配置模式（切换模式时必须先进此模式）
#define BNO055_OPR_MODE_NDOF     0x0C   // NDOF 9 轴全融合（含磁力计，偏航不漂移）

// ==================== 期望芯片 ID ====================
#define BNO055_CHIP_ID           0xA0

// I2C 超时（ms）
#define I2C_TIMEOUT_MS           100

// ========================================================
// 底层 I2C 读写（使用 I2C_NUM_0，与 MPU6050 同总线引脚 GPIO 8/9）
// ========================================================
static esp_err_t bno055_write_byte(uint8_t reg, uint8_t val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BNO055_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t bno055_read_bytes(uint8_t reg, uint8_t *buf, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BNO055_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);  // repeated start
    i2c_master_write_byte(cmd, (BNO055_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, buf + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// ========================================================
// 公共接口
// ========================================================
esp_err_t bno055_init(void)
{
    // 1. 初始化 I2C0 总线（GPIO 8/9，复用原 MPU6050 接口，imu_driver_init 不再调用）
    i2c_config_t conf = {
        .mode            = I2C_MODE_MASTER,
        .sda_io_num      = PIN_I2C0_SDA,
        .scl_io_num      = PIN_I2C0_SCL,
        .sda_pullup_en   = GPIO_PULLUP_ENABLE,
        .scl_pullup_en   = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
    ESP_LOGI(TAG, "I2C0 初始化完成 (SDA=%d SCL=%d)", PIN_I2C0_SDA, PIN_I2C0_SCL);

    // 2. 等待 BNO055 上电复位完成（POR 时间 650ms，留余量到 800ms）
    vTaskDelay(pdMS_TO_TICKS(800));

    // 3. I2C 总线扫描：找出实际响应的设备地址
    ESP_LOGI(TAG, "扫描 I2C 总线...");
    uint8_t found_addr = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        i2c_cmd_handle_t scan_cmd = i2c_cmd_link_create();
        i2c_master_start(scan_cmd);
        i2c_master_write_byte(scan_cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(scan_cmd);
        esp_err_t scan_ret = i2c_master_cmd_begin(I2C_NUM_0, scan_cmd, pdMS_TO_TICKS(10));
        i2c_cmd_link_delete(scan_cmd);
        if (scan_ret == ESP_OK) {
            ESP_LOGI(TAG, "  发现设备: 0x%02X", addr);
            if (addr == 0x28 || addr == 0x29 || addr == 0x2A) found_addr = addr;
        }
    }
    if (found_addr == 0) {
        ESP_LOGE(TAG, "I2C 总线上未发现任何设备！请检查接线和供电");
        return ESP_ERR_NOT_FOUND;
    }
    if (found_addr != BNO055_I2C_ADDR) {
        ESP_LOGW(TAG, "BNO055 实际地址=0x%02X，代码配置=0x%02X，请更新 BNO055_I2C_ADDR",
                 found_addr, BNO055_I2C_ADDR);
    }

    // 4. 校验芯片 ID
    uint8_t chip_id = 0;
    esp_err_t err = bno055_read_bytes(BNO055_CHIP_ID_ADDR, &chip_id, 1);
    if (err != ESP_OK || chip_id != BNO055_CHIP_ID) {
        ESP_LOGE(TAG, "BNO055 未检测到！chip_id=0x%02X (期望 0xA0), err=%s",
                 chip_id, esp_err_to_name(err));
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "BNO055 检测到，chip_id=0x%02X", chip_id);

    // 4. 复位到 CONFIG 模式
    err = bno055_write_byte(BNO055_OPR_MODE_ADDR, BNO055_OPR_MODE_CONFIG);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(25));  // CONFIG 模式切换需要 ~19ms

    // 5. 软复位（清除上电残留状态）
    err = bno055_write_byte(BNO055_SYS_TRIGGER_ADDR, 0x20);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(650));  // 复位后芯片重启约 600ms

    // 6. 设置单位：角度（度），摄氏度，m/s²，dps
    //    UNIT_SEL = 0x00：欧拉角输出为度（1LSB = 1/16 度），加速度 m/s²，角速度 dps
    err = bno055_write_byte(BNO055_UNIT_SEL_ADDR, 0x00);
    if (err != ESP_OK) return err;

    // 7. 切换到 NDOF 全融合模式（加重试，确保模式切换成功）
    for (int retry = 0; retry < 3; retry++) {
        err = bno055_write_byte(BNO055_OPR_MODE_ADDR, BNO055_OPR_MODE_NDOF);
        if (err != ESP_OK) return err;
        vTaskDelay(pdMS_TO_TICKS(20));  // NDOF 模式切换需要 ~7ms

        // 回读验证：确认芯片已进入 NDOF 模式（0x0C）
        uint8_t opr_mode_rb = 0;
        err = bno055_read_bytes(BNO055_OPR_MODE_ADDR, &opr_mode_rb, 1);
        if (err == ESP_OK && opr_mode_rb == BNO055_OPR_MODE_NDOF) {
            ESP_LOGI(TAG, "✓ OPR_MODE 确认 = 0x%02X (NDOF)", opr_mode_rb);
            break;
        }
        ESP_LOGW(TAG, "NDOF 模式切换未成功，OPR_MODE=0x%02X，重试 %d/3...", opr_mode_rb, retry + 1);
        vTaskDelay(pdMS_TO_TICKS(50));
        if (retry == 2) {
            ESP_LOGE(TAG, "无法切换到 NDOF 模式！");
            return ESP_FAIL;
        }
    }

    // 8. 等待传感器融合收敛，并做诊断打印
    ESP_LOGI(TAG, "等待 NDOF 融合收敛 (1s)...");
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 诊断：读取 SYS_STATUS 和原始 Roll 值，确认传感器正常输出
    uint8_t sys_status = 0;
    bno055_read_bytes(0x39, &sys_status, 1);  // SYS_STATUS: 0x05 = 融合算法运行中
    uint8_t cal_stat = 0;
    bno055_read_bytes(0x35, &cal_stat, 1);   // CALIB_STAT: [7:6]=Sys [5:4]=Gyro [3:2]=Accel [1:0]=Mag
    uint8_t roll_buf[2] = {0};
    bno055_read_bytes(BNO055_EUL_ROLL_LSB, roll_buf, 2);
    int16_t roll_raw = (int16_t)((uint16_t)roll_buf[1] << 8 | roll_buf[0]);
    uint8_t hdg_buf[2] = {0};
    bno055_read_bytes(BNO055_EUL_HEADING_LSB, hdg_buf, 2);
    int16_t hdg_raw = (int16_t)((uint16_t)hdg_buf[1] << 8 | hdg_buf[0]);
    ESP_LOGI(TAG, "诊断: SYS_STATUS=0x%02X (期望0x05), CALIB=0x%02X, Roll_raw=%d (%.2f°), Hdg_raw=%d (%.2f°)",
             sys_status, cal_stat,
             roll_raw, roll_raw / 16.0f,
             hdg_raw, hdg_raw / 16.0f);

    // 9. 尝试从 NVS 加载历史校准 profile（自动恢复磁力计/加速度计/陀螺仪偏置）
    if (bno055_load_calib_from_nvs() == ESP_OK) {
        ESP_LOGI(TAG, "✓ BNO055 校准 profile 已从 NVS 恢复");
    } else {
        ESP_LOGW(TAG, "⚠ 未发现 BNO055 校准数据。请做 8 字晃动直到 Mag=3，再串口输入 'mag' 保存。");
    }

    ESP_LOGI(TAG, "✓ BNO055 初始化完成，NDOF 模式运行中");
    return ESP_OK;
}

// 内部：读取 raw 航向（未应用 s_hdg_offset_deg），单位度，归一化到 0~360
static esp_err_t bno055_read_raw_heading(float *raw_deg)
{
    uint8_t buf[2];
    esp_err_t err = bno055_read_bytes(BNO055_EUL_HEADING_LSB, buf, 2);
    if (err != ESP_OK) return err;
    int16_t raw = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
    float h = (float)raw / 16.0f;
    while (h < 0.0f)    h += 360.0f;
    while (h >= 360.0f) h -= 360.0f;
    *raw_deg = h;
    return ESP_OK;
}

esp_err_t bno055_get_heading(float *heading_deg)
{
    float raw;
    esp_err_t err = bno055_read_raw_heading(&raw);
    if (err != ESP_OK) return err;

    // 应用 mag 校准时记下的零点偏置，使"当时的朝向"被视为真北 0°
    float h = raw - s_hdg_offset_deg;
    while (h < 0.0f)    h += 360.0f;
    while (h >= 360.0f) h -= 360.0f;

    *heading_deg = h;
    return ESP_OK;
}

esp_err_t bno055_get_roll(float *roll_deg)
{
    uint8_t buf[2];
    esp_err_t err = bno055_read_bytes(BNO055_EUL_ROLL_LSB, buf, 2);
    if (err != ESP_OK) return err;

    // 有符号 16 位，单位 1/16 度；NDOF 模式 Roll 范围 -180~+180°
    int16_t raw = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
    *roll_deg = (float)raw / 16.0f;
    return ESP_OK;
}

esp_err_t bno055_get_gyro_x(float *gyrox_dps)
{
    uint8_t buf[2];
    esp_err_t err = bno055_read_bytes(BNO055_GYR_DATA_X_LSB, buf, 2);
    if (err != ESP_OK) return err;

    // 有符号 16 位，单位 1/16 °/s（UNIT_SEL bit1=0 时）
    int16_t raw = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
    *gyrox_dps = (float)raw / 16.0f;
    return ESP_OK;
}

esp_err_t bno055_get_linear_accel(float *ax, float *ay, float *az)
{
    // LIA_Data 寄存器：0x28~0x2D（X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB）
    // NDOF 模式已去除重力；UNIT_SEL bit0=0 → 1 LSB = 1/100 m/s²
    uint8_t buf[6];
    esp_err_t err = bno055_read_bytes(0x28, buf, 6);
    if (err != ESP_OK) return err;

    int16_t rx = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
    int16_t ry = (int16_t)((uint16_t)buf[3] << 8 | buf[2]);
    int16_t rz = (int16_t)((uint16_t)buf[5] << 8 | buf[4]);
    *ax = (float)rx / 100.0f;
    *ay = (float)ry / 100.0f;
    *az = (float)rz / 100.0f;
    return ESP_OK;
}

 

// ========================================================
// 校准 profile 持久化（与 steering encoder NVS 范式一致）
// ========================================================

esp_err_t bno055_get_calib_status(uint8_t *sys, uint8_t *gyr, uint8_t *acc, uint8_t *mag)
{
    uint8_t s = 0;
    esp_err_t err = bno055_read_bytes(BNO055_CALIB_STAT_ADDR, &s, 1);
    if (err != ESP_OK) return err;
    if (sys) *sys = (s >> 6) & 0x03;
    if (gyr) *gyr = (s >> 4) & 0x03;
    if (acc) *acc = (s >> 2) & 0x03;
    if (mag) *mag = (s     ) & 0x03;
    return ESP_OK;
}

esp_err_t bno055_read_calib_profile(uint8_t buf[22])
{
    return bno055_read_bytes(BNO055_CALIB_DATA_ADDR, buf, 22);
}

esp_err_t bno055_write_calib_profile(const uint8_t buf[22])
{
    // 数据手册要求逐字节写入（无 burst write 支持的明确说明，稳妥起见单字节）
    for (int i = 0; i < 22; i++) {
        esp_err_t e = bno055_write_byte(BNO055_CALIB_DATA_ADDR + i, buf[i]);
        if (e != ESP_OK) return e;
    }
    return ESP_OK;
}

// 内部辅助：切换 OPR_MODE 并等待对应延时
static esp_err_t bno_set_mode(uint8_t mode, uint32_t delay_ms)
{
    esp_err_t err = bno055_write_byte(BNO055_OPR_MODE_ADDR, mode);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
    return ESP_OK;
}

esp_err_t bno055_calibrate_and_save(void)
{
    // 1. 打印当前校准等级供用户参考
    uint8_t sys = 0, gyr = 0, acc = 0, mag = 0;
    if (bno055_get_calib_status(&sys, &gyr, &acc, &mag) != ESP_OK) {
        ESP_LOGE(TAG, "读取 CALIB_STAT 失败");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "当前校准等级: Sys=%d Gyr=%d Acc=%d Mag=%d (0~3, 3=已校准)",
             sys, gyr, acc, mag);
    if (mag < 3) {
        ESP_LOGW(TAG, "⚠ 磁力计未完全校准 (Mag=%d/3)；仍将保存当前进度。建议做 8 字晃动后再保存。", mag);
    }

    // 2. 切到 CONFIG 模式才能读出有效的 calib profile
    if (bno_set_mode(BNO055_OPR_MODE_CONFIG, 25) != ESP_OK) {
        ESP_LOGE(TAG, "切换 CONFIG 失败");
        return ESP_FAIL;
    }

    uint8_t profile[22] = {0};
    esp_err_t err = bno055_read_calib_profile(profile);

    // 3. 无论成功失败都切回 NDOF
    bno_set_mode(BNO055_OPR_MODE_NDOF, 20);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "读 calib profile 失败: %s", esp_err_to_name(err));
        return err;
    }

    // 4. 捕获当前 raw 航向作为新的"真北"零点偏置
    //    切回 NDOF 后等一会儿让融合稳定再读，避免读到模式切换中的脏值。
    vTaskDelay(pdMS_TO_TICKS(150));
    float raw_hdg = 0.0f;
    if (bno055_read_raw_heading(&raw_hdg) == ESP_OK) {
        s_hdg_offset_deg = raw_hdg;
        ESP_LOGI(TAG, "✓ 已将当前朝向 (raw=%.2f°) 设为真北 0°", raw_hdg);
    } else {
        ESP_LOGW(TAG, "读取 raw 航向失败，保留原偏置 %.2f°", s_hdg_offset_deg);
    }

    // 5. 写入 NVS：22 字节 calib profile + 航向零点偏置
    nvs_handle_t h;
    err = nvs_open(BNO_NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open 失败: %s", esp_err_to_name(err));
        return err;
    }
    err = nvs_set_blob(h, BNO_NVS_KEY_CAL, profile, 22);
    if (err == ESP_OK) {
        float off = s_hdg_offset_deg;
        err = nvs_set_blob(h, BNO_NVS_KEY_HDG_OFF, &off, sizeof(off));
    }
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "✓ BNO055 校准 22 字节 + 航向偏置 %.2f° 已写入 NVS（重启自动加载）", s_hdg_offset_deg);
    } else {
        ESP_LOGE(TAG, "✗ 写 NVS 失败: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t bno055_load_calib_from_nvs(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(BNO_NVS_NS, NVS_READONLY, &h);
    if (err != ESP_OK) return ESP_ERR_NOT_FOUND;

    uint8_t profile[22] = {0};
    size_t len = sizeof(profile);
    err = nvs_get_blob(h, BNO_NVS_KEY_CAL, profile, &len);

    // 同时读取航向零点偏置（缺失时保持 0，向后兼容旧版本数据）
    float off = 0.0f;
    size_t off_len = sizeof(off);
    if (nvs_get_blob(h, BNO_NVS_KEY_HDG_OFF, &off, &off_len) == ESP_OK && off_len == sizeof(off)) {
        s_hdg_offset_deg = off;
        ESP_LOGI(TAG, "✓ 已加载航向零点偏置 %.2f° (真北方向)", off);
    } else {
        ESP_LOGW(TAG, "未发现航向零点偏置，使用 0°（heading 仍为磁北）");
    }
    nvs_close(h);
    if (err != ESP_OK || len != 22) return ESP_ERR_NOT_FOUND;

    // 写回 22 字节必须在 CONFIG 模式
    if (bno_set_mode(BNO055_OPR_MODE_CONFIG, 25) != ESP_OK) return ESP_FAIL;
    err = bno055_write_calib_profile(profile);
    bno_set_mode(BNO055_OPR_MODE_NDOF, 20);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "写回 calib profile 失败: %s", esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}
