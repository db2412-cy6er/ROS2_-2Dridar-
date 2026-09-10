#include "i2c_slave_comm.h"

#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/i2c_slave.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_netif_ip_addr.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "wifi_manager.h"

#define I2C_SLAVE_COMM_PORT I2C_NUM_0
#define I2C_SLAVE_COMM_RX_SIZE 128
#define I2C_SLAVE_COMM_TX_SIZE 256
#define I2C_SLAVE_COMM_QUEUE_LEN 4
#define I2C_SLAVE_COMM_TASK_STACK 4096

static const char *TAG = "I2C_SLAVE_COMM";

typedef struct {
    bool status_request;
    size_t len;
    uint8_t data[I2C_SLAVE_COMM_RX_SIZE];
} i2c_slave_command_t;

static i2c_slave_dev_handle_t s_i2c_slave;
static QueueHandle_t s_command_queue;
static TaskHandle_t s_task_handle;
static uint8_t s_tx_buffer[I2C_SLAVE_COMM_TX_SIZE];

// ================= P0 时间同步状态 =================
// 访问用 spinlock 保护，供 i2c 从机任务 / httpd(web_mjpeg) / web_config 任务间安全读写。
static portMUX_TYPE s_time_sync_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool s_sync_hold = false;      // hold: 忽略新 ROSOFF（录制标定 bag 用）
static volatile bool s_rosoff_valid = false;   // 映射是否已就绪
static volatile int64_t s_rosoff_ns = 0;       // camera_local_us*1000 + rosoff_ns ≈ ROS epoch
static volatile uint64_t s_last_rx_us = 0;     // 最近一次 I2C 写事务完成瞬间的 esp_timer(us)

void camera_time_sync_set_hold(bool hold)
{
    portENTER_CRITICAL(&s_time_sync_mux);
    s_sync_hold = hold;
    portEXIT_CRITICAL(&s_time_sync_mux);
    ESP_LOGI(TAG, "Time sync mapping %s", hold ? "HELD (frozen)" : "released");
}

bool camera_time_sync_is_held(void)
{
    bool held;
    portENTER_CRITICAL(&s_time_sync_mux);
    held = s_sync_hold;
    portEXIT_CRITICAL(&s_time_sync_mux);
    return held;
}

bool camera_time_sync_get_rosoff(int64_t *rosoff_ns)
{
    if (rosoff_ns == NULL) {
        return false;
    }
    bool valid;
    portENTER_CRITICAL(&s_time_sync_mux);
    valid = s_rosoff_valid;
    if (valid) {
        *rosoff_ns = s_rosoff_ns;
    }
    portEXIT_CRITICAL(&s_time_sync_mux);
    return valid;
}

static uint64_t time_sync_get_last_rx_us(void)
{
    uint64_t us;
    portENTER_CRITICAL(&s_time_sync_mux);
    us = s_last_rx_us;
    portEXIT_CRITICAL(&s_time_sync_mux);
    return us;
}

static void time_sync_set_last_rx_us_from_isr(void)
{
    portENTER_CRITICAL_ISR(&s_time_sync_mux);
    s_last_rx_us = (uint64_t)esp_timer_get_time();
    portEXIT_CRITICAL_ISR(&s_time_sync_mux);
}

static void time_sync_set_rosoff_from_task(int64_t rosoff_ns)
{
    portENTER_CRITICAL(&s_time_sync_mux);
    s_rosoff_ns = rosoff_ns;
    s_rosoff_valid = true;
    portEXIT_CRITICAL(&s_time_sync_mux);
}

// ================================================

static void trim_ascii(char *text)
{
    char *start = text;
    while (*start && isspace((unsigned char)*start)) {
        start++;
    }

    if (start != text) {
        memmove(text, start, strlen(start) + 1);
    }

    size_t len = strlen(text);
    while (len > 0 && isspace((unsigned char)text[len - 1])) {
        text[--len] = '\0';
    }
}

static bool copy_json_string_value(
    const char *json,
    const char *key,
    char *out,
    size_t out_size
)
{
    char pattern[32];
    int pattern_len = snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    if (pattern_len <= 0 || pattern_len >= (int)sizeof(pattern)) {
        return false;
    }

    const char *p = strstr(json, pattern);
    if (!p) {
        return false;
    }

    p += pattern_len;
    while (*p && isspace((unsigned char)*p)) {
        p++;
    }
    if (*p != ':') {
        return false;
    }
    p++;
    while (*p && isspace((unsigned char)*p)) {
        p++;
    }
    if (*p != '"') {
        return false;
    }
    p++;

    size_t out_len = 0;
    while (*p && *p != '"' && out_len + 1 < out_size) {
        if (*p == '\\' && p[1]) {
            p++;
        }
        out[out_len++] = *p++;
    }
    out[out_len] = '\0';
    return out_len > 0 || strcmp(key, "password") == 0;
}

static bool parse_wifi_command(char *cmd, char *ssid, size_t ssid_size, char *password, size_t password_size)
{
    trim_ascii(cmd);

    if (cmd[0] == '{') {
        if (!copy_json_string_value(cmd, "ssid", ssid, ssid_size)) {
            return false;
        }
        if (!copy_json_string_value(cmd, "password", password, password_size)) {
            password[0] = '\0';
        }
        trim_ascii(ssid);
        trim_ascii(password);
        return ssid[0] != '\0';
    }

    if (strncasecmp(cmd, "WIFI:", 5) == 0) {
        cmd += 5;
    } else if (strncasecmp(cmd, "SET_WIFI:", 9) == 0) {
        cmd += 9;
    } else if (strncasecmp(cmd, "PROV:", 5) == 0) {
        cmd += 5;
    } else {
        return false;
    }

    char *separator = strchr(cmd, ',');
    if (!separator) {
        separator = strchr(cmd, '|');
    }
    if (!separator) {
        return false;
    }

    *separator = '\0';
    strlcpy(ssid, cmd, ssid_size);
    strlcpy(password, separator + 1, password_size);
    trim_ascii(ssid);
    trim_ascii(password);
    return ssid[0] != '\0';
}

static void restart_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
}

static void update_tx_payload(const char *status, esp_err_t last_err)
{
    esp_ip4_addr_t ip = wifi_manager_get_ip();
    const char *mode = wifi_manager_get_mode_name();
    const char *name = wifi_manager_get_device_name();

    int len = snprintf(
        (char *)s_tx_buffer,
        sizeof(s_tx_buffer),
        "{\"ok\":%s,\"status\":\"%s\",\"ip\":\"" IPSTR "\",\"mode\":\"%s\","
        "\"name\":\"%s\",\"config_url\":\"http://" IPSTR "/\","
        "\"stream_url\":\"http://" IPSTR ":81/\",\"last_err\":%d}\n",
        last_err == ESP_OK ? "true" : "false",
        status,
        IP2STR(&ip),
        mode,
        name,
        IP2STR(&ip),
        IP2STR(&ip),
        (int)last_err
    );

    if (len <= 0 || len >= (int)sizeof(s_tx_buffer)) {
        strlcpy((char *)s_tx_buffer, "{\"ok\":false,\"status\":\"tx_overflow\"}\n", sizeof(s_tx_buffer));
        len = strlen((char *)s_tx_buffer);
    }

    uint32_t written = 0;
    esp_err_t err = i2c_slave_write(s_i2c_slave, s_tx_buffer, len, &written, 0);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "I2C TX preload failed: %s", esp_err_to_name(err));
    }
}

// P0 时间同步：SYNC 响应，携带"最近一次 I2C 写事务完成瞬间"的摄像头 esp_timer 时间(us)。
static void update_tx_payload_sync(void)
{
    const uint64_t rx_us = time_sync_get_last_rx_us();
    int len = snprintf(
        (char *)s_tx_buffer,
        sizeof(s_tx_buffer),
        "{\"ok\":true,\"status\":\"sync\",\"ts\":%llu}\n",
        (unsigned long long)rx_us
    );

    if (len <= 0 || len >= (int)sizeof(s_tx_buffer)) {
        strlcpy((char *)s_tx_buffer, "{\"ok\":false,\"status\":\"tx_overflow\"}\n", sizeof(s_tx_buffer));
        len = strlen((char *)s_tx_buffer);
    }

    uint32_t written = 0;
    esp_err_t err = i2c_slave_write(s_i2c_slave, s_tx_buffer, len, &written, 0);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "I2C TX preload (sync) failed: %s", esp_err_to_name(err));
    }
}

// P0 时间同步：ROSOFF <ns>。摄像头处于 hold 时忽略，保证录制期间映射不变。
static void handle_rosoff_command(const char *arg)
{
    char *end = NULL;
    const long long rosoff_ns = strtoll(arg, &end, 10);
    if (end == arg || *end != '\0') {
        update_tx_payload("bad_command", ESP_ERR_INVALID_ARG);
        return;
    }

    if (camera_time_sync_is_held()) {
        ESP_LOGI(TAG, "ROSOFF ignored while mapping is HELD");
        update_tx_payload("rosoff_held", ESP_OK);
        return;
    }

    time_sync_set_rosoff_from_task((int64_t)rosoff_ns);
    ESP_LOGI(TAG, "ROSOFF updated to %lld ns", rosoff_ns);
    update_tx_payload("rosoff", ESP_OK);
}

static void handle_command(const i2c_slave_command_t *command)
{
    char cmd[I2C_SLAVE_COMM_RX_SIZE + 1];

    if (command->status_request) {
        update_tx_payload("status", ESP_OK);
        return;
    }

    size_t len = command->len;
    if (len > I2C_SLAVE_COMM_RX_SIZE) {
        len = I2C_SLAVE_COMM_RX_SIZE;
    }

    memcpy(cmd, command->data, len);
    cmd[len] = '\0';
    trim_ascii(cmd);

    if (strcasecmp(cmd, "STATUS") == 0 ||
        strcasecmp(cmd, "GET_IP") == 0 ||
        strcasecmp(cmd, "IP?") == 0) {
        update_tx_payload("status", ESP_OK);
        return;
    }

    // P0 时间同步命令（由主板主动发起）
    if (strcasecmp(cmd, "SYNC") == 0 || strcasecmp(cmd, "TSYNC") == 0) {
        update_tx_payload_sync();
        return;
    }
    if (strncasecmp(cmd, "ROSOFF ", 7) == 0) {
        handle_rosoff_command(cmd + 7);
        return;
    }

    char ssid[33] = {0};
    char password[65] = {0};
    if (!parse_wifi_command(cmd, ssid, sizeof(ssid), password, sizeof(password))) {
        ESP_LOGW(TAG, "Unsupported I2C command: %.*s", (int)len, (const char *)command->data);
        update_tx_payload("bad_command", ESP_ERR_INVALID_ARG);
        return;
    }

    esp_err_t err = wifi_manager_save_sta(ssid, password);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Save Wi-Fi from I2C failed: %s", esp_err_to_name(err));
        update_tx_payload("save_failed", err);
        return;
    }

    ESP_LOGI(TAG, "Wi-Fi config saved from I2C, ssid=%s, restarting", ssid);
    update_tx_payload("wifi_saved_restarting", ESP_OK);
    xTaskCreate(restart_task, "i2c_wifi_restart", 2048, NULL, 5, NULL);
}

static bool i2c_slave_recv_done_cb(
    i2c_slave_dev_handle_t i2c_slave,
    const i2c_slave_rx_done_event_data_t *evt_data,
    void *arg
)
{
    QueueHandle_t queue = (QueueHandle_t)arg;
    BaseType_t high_task_wakeup = pdFALSE;
    i2c_slave_command_t command = {0};

    // P0 时间同步：记录本次 I2C 写事务完成瞬间的摄像头本地时间（esp_timer us）。
    time_sync_set_last_rx_us_from_isr();

    size_t len = evt_data->length;
    if (len > I2C_SLAVE_COMM_RX_SIZE) {
        len = I2C_SLAVE_COMM_RX_SIZE;
    }

    command.len = len;
    memcpy(command.data, evt_data->buffer, len);
    (void)xQueueSendFromISR(queue, &command, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

static bool i2c_slave_request_cb(
    i2c_slave_dev_handle_t i2c_slave,
    const i2c_slave_request_event_data_t *evt_data,
    void *arg
)
{
    QueueHandle_t queue = (QueueHandle_t)arg;
    BaseType_t high_task_wakeup = pdFALSE;
    i2c_slave_command_t command = {
        .status_request = true,
    };

    (void)xQueueSendFromISR(queue, &command, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

static void i2c_slave_comm_task(void *arg)
{
    i2c_slave_command_t command;
    update_tx_payload("ready", ESP_OK);

    while (true) {
        if (xQueueReceive(s_command_queue, &command, pdMS_TO_TICKS(1000)) == pdTRUE) {
            handle_command(&command);
        }
    }
}

esp_err_t i2c_slave_comm_start(void)
{
    if (s_task_handle) {
        return ESP_OK;
    }

    s_command_queue = xQueueCreate(I2C_SLAVE_COMM_QUEUE_LEN, sizeof(i2c_slave_command_t));
    if (!s_command_queue) {
        return ESP_ERR_NO_MEM;
    }

    i2c_slave_config_t config = {
        .i2c_port = I2C_SLAVE_COMM_PORT,
        .sda_io_num = I2C_SLAVE_COMM_SDA_GPIO,
        .scl_io_num = I2C_SLAVE_COMM_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .send_buf_depth = I2C_SLAVE_COMM_TX_SIZE,
        .receive_buf_depth = I2C_SLAVE_COMM_RX_SIZE,
        .slave_addr = I2C_SLAVE_COMM_ADDR,
        .addr_bit_len = I2C_ADDR_BIT_LEN_7,
        .intr_priority = 0,
        .flags.enable_internal_pullup = 1,
    };

    ESP_RETURN_ON_ERROR(i2c_new_slave_device(&config, &s_i2c_slave), TAG, "Create I2C slave failed");

    i2c_slave_event_callbacks_t callbacks = {
        .on_receive = i2c_slave_recv_done_cb,
        .on_request = i2c_slave_request_cb,
    };
    ESP_RETURN_ON_ERROR(
        i2c_slave_register_event_callbacks(s_i2c_slave, &callbacks, s_command_queue),
        TAG,
        "Register I2C callbacks failed"
    );
    BaseType_t ok = xTaskCreate(
        i2c_slave_comm_task,
        "i2c_slave_comm",
        I2C_SLAVE_COMM_TASK_STACK,
        NULL,
        4,
        &s_task_handle
    );
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG,
             "I2C slave started addr=0x%02X SCL=%d SDA=%d",
             I2C_SLAVE_COMM_ADDR,
             I2C_SLAVE_COMM_SCL_GPIO,
             I2C_SLAVE_COMM_SDA_GPIO);
    return ESP_OK;
}
