#include "web_mjpeg.h"

#include <stdio.h>
#include <string.h>

#include "esp_camera.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "i2c_slave_comm.h"

static const char *TAG = "WEB_MJPEG";

#define STREAM_BOUNDARY "frame"

static const char *STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" STREAM_BOUNDARY;

static const char *STREAM_BOUNDARY_LINE =
    "\r\n--" STREAM_BOUNDARY "\r\n";

// P0 时间同步：每个 part 除 JPEG 长度外，附带：
//   X-Frame-Id               连接内递增帧号
//   X-Capture-Timestamp-Us   摄像头原始采集时间（fb->timestamp，esp_timer us，
//                            未就绪时回退为 esp_camera_fb_get 返回后立即打点）
//   X-Capture-Ros-Time-Ns    camera_local_us*1000 + ROSOFF（同步未就绪为 0）
static const char *STREAM_PART_HEADER =
    "Content-Type: image/jpeg\r\n"
    "Content-Length: %u\r\n"
    "X-Frame-Id: %u\r\n"
    "X-Capture-Timestamp-Us: %lld\r\n"
    "X-Capture-Ros-Time-Ns: %lld\r\n"
    "\r\n";

static esp_err_t stream_handler(httpd_req_t *req)
{
    esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    if (res != ESP_OK) {
        return res;
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    char header_buf[256];
    uint32_t frame_id = 0; // P0：连接内递增帧号
    ESP_LOGI(TAG, "MJPEG client connected");

    while (true) {
        camera_fb_t *fb = esp_camera_fb_get();

        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed during stream");
            return ESP_FAIL;
        }

        if (fb->format != PIXFORMAT_JPEG) {
            ESP_LOGE(TAG, "Streaming frame is not JPEG");
            esp_camera_fb_return(fb);
            return ESP_FAIL;
        }

        // P0 时间同步：优先用驱动在采集起始写入 fb->timestamp（esp_timer us），
        // 它代表“这一帧第一个 DMA buffer / 图像采集开始附近”的时间，而非发送时间。
        // 若驱动未填充(全 0)，回退为拿到 fb 后立即打点。
        int64_t capture_us =
            (fb->timestamp.tv_sec == 0 && fb->timestamp.tv_usec == 0)
                ? (int64_t)esp_timer_get_time()
                : (int64_t)fb->timestamp.tv_sec * 1000000LL + fb->timestamp.tv_usec;

        int64_t rosoff_ns = 0;
        if (camera_time_sync_get_rosoff(&rosoff_ns)) {
            rosoff_ns = capture_us * 1000LL + rosoff_ns;
        } else {
            rosoff_ns = 0; // 尚未同步：bridge 收到 0 会打 WARN，不回退成接收时间
        }

        res = httpd_resp_send_chunk(req, STREAM_BOUNDARY_LINE, strlen(STREAM_BOUNDARY_LINE));
        if (res != ESP_OK) {
            esp_camera_fb_return(fb);
            break;
        }

        int header_len = snprintf(
            header_buf,
            sizeof(header_buf),
            STREAM_PART_HEADER,
            (unsigned int)fb->len,
            (unsigned int)++frame_id,
            (long long)capture_us,
            (long long)rosoff_ns
        );

        if (header_len <= 0 || header_len >= sizeof(header_buf)) {
            ESP_LOGE(TAG, "MJPEG header build failed");
            esp_camera_fb_return(fb);
            return ESP_FAIL;
        }

        res = httpd_resp_send_chunk(req, header_buf, header_len);
        if (res != ESP_OK) {
            esp_camera_fb_return(fb);
            break;
        }

        res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);

        esp_camera_fb_return(fb);

        if (res != ESP_OK) {
            break;
        }
    }

    ESP_LOGW(TAG, "MJPEG client disconnected");
    return res;
}

httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 81;
    config.ctrl_port = 32769;
    config.max_uri_handlers = 4;
    config.stack_size = 8192;

    esp_err_t ret = httpd_start(&server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: 0x%x", ret);
        return NULL;
    }

    httpd_uri_t root_stream_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = stream_handler,
        .user_ctx = NULL
    };

    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &root_stream_uri));

    ESP_LOGI(TAG, "HTTP MJPEG stream started: http://<ESP32_IP>:%d/", config.server_port);
    return server;
}
