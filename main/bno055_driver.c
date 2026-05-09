#include "bno055_driver.h"
#include "system_config.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "BNO055";

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
            if (addr == 0x28 || addr == 0x29) found_addr = addr;
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

    ESP_LOGI(TAG, "✓ BNO055 初始化完成，NDOF 模式运行中");
    return ESP_OK;
}

esp_err_t bno055_get_heading(float *heading_deg)
{
    uint8_t buf[2];
    esp_err_t err = bno055_read_bytes(BNO055_EUL_HEADING_LSB, buf, 2);
    if (err != ESP_OK) return err;

    // 合并 LSB + MSB，值为有符号 16 位，单位 1/16 度
    int16_t raw = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
    float heading = (float)raw / 16.0f;  // 转换为度

    // BNO055 NDOF 模式：Heading 范围 0~360°（顺时针为正）
    // 对超出范围的值做归一化保护
    while (heading < 0.0f)    heading += 360.0f;
    while (heading >= 360.0f) heading -= 360.0f;

    *heading_deg = heading;
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
