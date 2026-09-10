#pragma once

#include <stdint.h>

struct ImuMsg {
    float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
    float acc_x = 0.0f, acc_y = 0.0f, acc_z = 0.0f;
    float gyro_x = 0.0f, gyro_y = 0.0f, gyro_z = 0.0f;
    // P0 时间同步：sample_us 是 LSM6DS3 寄存器读取完成瞬间的 esp_timer 时间(us)。
    // sample_seq 每采样一次自增一次，供 micro-ROS 发布端做“同一份样本只发一次”去重。
    uint64_t sample_us = 0;
    uint32_t sample_seq = 0;
};