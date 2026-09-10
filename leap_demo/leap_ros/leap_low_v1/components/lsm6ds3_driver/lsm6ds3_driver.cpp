#include "lsm6ds3_driver.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus_lock.h"
#include <cmath>

static const char* TAG = "LSM6DS3_IMU";

// 寄存器地址常量定义
constexpr uint8_t kLsm6ds3Addr = 0x6A;
constexpr uint8_t kRegWhoAmI   = 0x0F;
constexpr uint8_t kValWhoAmI   = 0x6A;
constexpr uint8_t kRegCtrl1Xl  = 0x10;
constexpr uint8_t kRegCtrl2G   = 0x11;
constexpr uint8_t kRegCtrl3C   = 0x12;
constexpr uint8_t kRegOutxLG   = 0x22;

// ================= P0 时间同步：量程/灵敏度配置 =================
// 口径冻结：104Hz ODR、加速度 ±4g、陀螺 ±500dps（Camera-IMU 标定与
// ORB-SLAM3 全程保持一致，标定结束后不允许切回 ±2g/±250dps）。
// CTRL1_XL = 0x48 : ODR_XL=0100(104Hz), FS_XL=10(±4g)
// CTRL2_G  = 0x44 : ODR_G =0100(104Hz), FS_G =01(±500dps)
constexpr uint8_t kCtrl1XlValue = 0x48;
constexpr uint8_t kCtrl2GValue  = 0x44;
// LSM6DS3 数据手册灵敏度：
//   ±4g    -> 0.122 mg/LSB
//   ±500dps-> 17.5  mdps/LSB
constexpr float kGyroSensitivityDpsPerLsb = 17.5f / 1000.0f;
constexpr float kAccSensitivityGPerLsb    = 0.122f / 1000.0f;
// ================================================================

Lsm6ds3Imu::Lsm6ds3Imu(i2c_port_t port, gpio_num_t sda_pin, gpio_num_t scl_pin)
    : port_(port), sda_pin_(sda_pin), scl_pin_(scl_pin),
      acc_x_(0.0f), acc_y_(0.0f), acc_z_(0.0f),
      gyro_x_(0.0f), gyro_y_(0.0f), gyro_z_(0.0f),
      roll_(0.0f), pitch_(0.0f), yaw_(0.0f),
      q_w_(1.0f), q_x_(0.0f), q_y_(0.0f), q_z_(0.0f),
      last_us_(0), sample_us_(0),
      gyro_bias_x_(0.0f), gyro_bias_y_(0.0f), gyro_bias_z_(0.0f) { // 初始化零偏
}

// ---------------- 新增：陀螺仪静态校准函数 ----------------
void Lsm6ds3Imu::CalibrateGyro() {
  ESP_LOGI(TAG, "Starting Gyro Calibration. PLEASE KEEP SENSOR STILL...");
  float sum_gx = 0, sum_gy = 0, sum_gz = 0;
  uint8_t buffer[6];
  const uint16_t kSamples = 300; // 采样300次，每次10ms，总计约3秒

  for (uint16_t i = 0; i < kSamples; ++i) {
    if (ReadRegisters(kRegOutxLG, buffer, 6) == ESP_OK) {
      int16_t gx = (buffer[1] << 8) | buffer[0];
      int16_t gy = (buffer[3] << 8) | buffer[2];
      int16_t gz = (buffer[5] << 8) | buffer[4];

      // ±500 dps -> 17.5 mdps/LSB
      sum_gx += gx * kGyroSensitivityDpsPerLsb;
      sum_gy += gy * kGyroSensitivityDpsPerLsb;
      sum_gz += gz * kGyroSensitivityDpsPerLsb;
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // 匹配 104Hz 的数据更新率
  }

  gyro_bias_x_ = sum_gx / kSamples;
  gyro_bias_y_ = sum_gy / kSamples;
  gyro_bias_z_ = sum_gz / kSamples;

  ESP_LOGI(TAG, "Calibration Done! Bias: X=%.3f, Y=%.3f, Z=%.3f", 
           gyro_bias_x_, gyro_bias_y_, gyro_bias_z_);
}
// ---------------------------------------------------------

bool Lsm6ds3Imu::Init() {
  // 配置 I2C 主机模式
  i2c_config_t conf = {};
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = sda_pin_;
  conf.scl_io_num = scl_pin_;
  conf.sda_pullup_en = true;
  conf.scl_pullup_en = true;
  conf.master.clk_speed = 400000;
  
  ESP_ERROR_CHECK(i2c_param_config(port_, &conf));
  
  // 忽略 ESP_ERR_INVALID_STATE 防止其他外设已经注册过该 I2C 端口导致崩溃
  esp_err_t err = i2c_driver_install(port_, conf.mode, 0, 0, 0);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
      ESP_LOGE(TAG, "I2C driver install failed");
      return false;
  }

  // 验证 WHO_AM_I
  uint8_t who = 0;
  if (ReadRegisters(kRegWhoAmI, &who, 1) != ESP_OK || who != kValWhoAmI) {
    ESP_LOGE(TAG, "WHO_AM_I check failed. Expected: 0x%02X, Got: 0x%02X", kValWhoAmI, who);
    return false;
  }

  // 初始化陀螺仪和加速度计配置（P0 冻结口径：104Hz / ±4g / ±500dps）
  WriteRegister8(kRegCtrl1Xl, kCtrl1XlValue); // 加速度计：104Hz, ±4g
  WriteRegister8(kRegCtrl2G, kCtrl2GValue);   // 陀螺仪：  104Hz, ±500dps
  WriteRegister8(kRegCtrl3C, 0x04);  // BDU=1, 地址自动递增

  // 执行上电零偏校准 (耗时约 3 秒)
  CalibrateGyro();

  // 状态复位
  roll_ = 0.0f;
  pitch_ = 0.0f;
  yaw_ = 0.0f;
  last_us_ = esp_timer_get_time();
  
  ESP_LOGI(TAG, "LSM6DS3 Initialized Successfully");
  return true;
}

esp_err_t Lsm6ds3Imu::Update() {
  uint8_t buffer[12];
  if (ReadRegisters(kRegOutxLG, buffer, 12) != ESP_OK) {
    return ESP_FAIL;
  }

  // P0 时间同步：原始寄存器批量读取成功返回的瞬间立即记录 sample_us_，
  // 代表“这一组 12 字节数据真正被读出来的时刻”，随后做的一切换算/滤波
  // 都不影响这个时间戳。互补滤波的 dt 也统一以它为准。
  sample_us_ = esp_timer_get_time();

  // 数据拼接 (LSM6DS3 是小端模式)
  int16_t gx = (buffer[1] << 8) | buffer[0];
  int16_t gy = (buffer[3] << 8) | buffer[2];
  int16_t gz = (buffer[5] << 8) | buffer[4];
  int16_t ax = (buffer[7] << 8) | buffer[6];
  int16_t ay = (buffer[9] << 8) | buffer[8];
  int16_t az = (buffer[11] << 8) | buffer[10];

  // 转换为物理单位 (±500dps -> 17.5mdps/LSB) 并减去开机零偏。
  // 注意：P0 口径下不得再引入陀螺仪死区/阈值钳位 —— ORB-SLAM3 / Kalibr
  // 需要真实的微小角速度变化，死区对 VIO 是有害的。
  gyro_x_ = (gx * kGyroSensitivityDpsPerLsb) - gyro_bias_x_;
  gyro_y_ = (gy * kGyroSensitivityDpsPerLsb) - gyro_bias_y_;
  gyro_z_ = (gz * kGyroSensitivityDpsPerLsb) - gyro_bias_z_;

  acc_x_  = ax * kAccSensitivityGPerLsb; // ±4g -> 0.122 mg/LSB
  acc_y_  = ay * kAccSensitivityGPerLsb;
  acc_z_  = az * kAccSensitivityGPerLsb;

  // 获取积分时间 dt（以本次寄存器读取时间为准）
  float dt = (sample_us_ - last_us_) / 1000000.0f;
  last_us_ = sample_us_;

  // 通过加速度计计算静态倾角 (弧度转角度)
  float roll_acc  = std::atan2(acc_y_, std::sqrt(acc_x_ * acc_x_ + acc_z_ * acc_z_)) * 57.29578f;
  float pitch_acc = std::atan2(-acc_x_, std::sqrt(acc_y_ * acc_y_ + acc_z_ * acc_z_)) * 57.29578f;

  // -------- 互补滤波算法 --------
  constexpr float kAlpha = 0.95f; // 信任陀螺仪的比例
  
  roll_  = kAlpha * (roll_  + gyro_x_ * dt) + (1.0f - kAlpha) * roll_acc;
  pitch_ = kAlpha * (pitch_ + gyro_y_ * dt) + (1.0f - kAlpha) * pitch_acc;
  yaw_  += gyro_z_ * dt;

  // 限制 Yaw 角在 [-180, 180] 之间
  if (yaw_ > 180.0f) yaw_ -= 360.0f;
  if (yaw_ < -180.0f) yaw_ += 360.0f;
  
  // 1. 将角度转换为弧度，并除以 2 
  constexpr float kDegToRadHalf = 0.01745329252f * 0.5f; 
  float cy = std::cos(yaw_ * kDegToRadHalf);
  float sy = std::sin(yaw_ * kDegToRadHalf);
  float cp = std::cos(pitch_ * kDegToRadHalf);
  float sp = std::sin(pitch_ * kDegToRadHalf);
  float cr = std::cos(roll_ * kDegToRadHalf);
  float sr = std::sin(roll_ * kDegToRadHalf);

  // 2. 根据 Z(Yaw) - Y(Pitch) - X(Roll) 的标准航空旋转顺序计算四元数
  q_w_ = cr * cp * cy + sr * sp * sy;
  q_x_ = sr * cp * cy - cr * sp * sy;
  q_y_ = cr * sp * cy + sr * cp * sy;
  q_z_ = cr * cp * sy - sr * sp * cy;

  return ESP_OK;
}

// ---------------- 内部 I2C 驱动封装 ----------------

esp_err_t Lsm6ds3Imu::WriteRegister8(uint8_t reg, uint8_t val) {
  if (!shared_i2c_bus_lock_take(pdMS_TO_TICKS(100))) {
    return ESP_ERR_TIMEOUT;
  }

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (kLsm6ds3Addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_write_byte(cmd, val, true);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(port_, cmd, pdMS_TO_TICKS(50));
  i2c_cmd_link_delete(cmd);
  shared_i2c_bus_lock_give();
  return ret;
}

esp_err_t Lsm6ds3Imu::ReadRegisters(uint8_t reg, uint8_t *data, size_t len) {
  if (!shared_i2c_bus_lock_take(pdMS_TO_TICKS(100))) {
    return ESP_ERR_TIMEOUT;
  }

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (kLsm6ds3Addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (kLsm6ds3Addr << 1) | I2C_MASTER_READ, true);
  if (len > 1) {
    i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
  }
  i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  
  esp_err_t ret = i2c_master_cmd_begin(port_, cmd, pdMS_TO_TICKS(50));
  i2c_cmd_link_delete(cmd);
  shared_i2c_bus_lock_give();
  return ret;
}
