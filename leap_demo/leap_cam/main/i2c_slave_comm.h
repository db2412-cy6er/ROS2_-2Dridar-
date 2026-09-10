#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define I2C_SLAVE_COMM_ADDR 0x42
#define I2C_SLAVE_COMM_SCL_GPIO 47
#define I2C_SLAVE_COMM_SDA_GPIO 48

esp_err_t i2c_slave_comm_start(void);

// ================= P0 时间同步 =================
// 由主板(I2C master)周期发起：
//   SYNC    -> 摄像头回 {"ok":true,"status":"sync","ts":<esp_timer us>}
//   ROSOFF <ns> -> 保存 camera_local_us*1000 + rosoff_ns ≈ ROS epoch 的映射。
// hold 期间忽略新的 ROSOFF（录制标定 bag 前由 PC bridge 置 hold，保证映射不变）。
void camera_time_sync_set_hold(bool hold);
bool camera_time_sync_is_held(void);
// 返回当前生效的 rosoff_ns（camera_local_us*1000 + rosoff_ns ≈ ROS epoch）。
// 返回 true 表示映射已就绪。
bool camera_time_sync_get_rosoff(int64_t *rosoff_ns);
// ==============================================
