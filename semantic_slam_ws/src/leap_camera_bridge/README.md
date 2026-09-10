# leap_camera_bridge

HTTP MJPEG -> ROS2 `sensor_msgs/Image` bridge for the leap camera board（P0 时间同步版）。

## 功能
- 拉取 `http://<cam_ip>:81/` 的 multipart/x-mixed-replace 流；
- 解析每 part 头 `X-Frame-Id` / `X-Capture-Timestamp-Us` / `X-Capture-Ros-Time-Ns`；
- 发布：
  - `/camera/image_raw`：Sensor Data QoS（best-effort），`frame_id=camera_optical_frame`
    - `header.stamp` **只**来自 `X-Capture-Ros-Time-Ns`；
    - 若该值为 0/缺失（micro-ROS 未同步或 hold 中）→ stamp=0 + 每秒 WARN，**绝不回退为 PC 接收时间**。
  - `/camera/camera_info`：transient-local；内参未标定前默认 800×600（K/D 空），标定后由 `camera_info_yaml` 参数加载。
- Service `~/sync_hold`（`std_srvs/SetBool`）：通过摄像头 80 端口 `/api/sync/hold|release` 冻结/恢复 Camera→ROS 映射（录标定 bag 前用）。

## 运行
```bash
colcon build --packages-select leap_camera_bridge
source install/setup.bash

ros2 run leap_camera_bridge camera_http_bridge --ros-args \
  -p stream_url:=http://192.168.31.100:81/
```
或 launch：
```bash
ros2 launch leap_camera_bridge camera_http_bridge.launch.py \
  stream_url:=http://192.168.31.100:81/
```

## 参数
| 参数 | 默认 | 说明 |
|---|---|---|
| `stream_url` | `http://192.168.31.100:81/` | MJPEG 地址 |
| `config_url` | 由 stream_url 主机推导（:80） | 摄像头配置/时间同步端点 |
| `frame_id` | `camera_optical_frame` | 图像 frame_id |
| `image_topic` | `camera/image_raw` | 图像话题 |
| `camera_info_topic` | `camera/camera_info` | camera_info 话题 |
| `camera_info_yaml` | 空 | 标定后的 camera_info yaml（可选） |

### camera_info_yaml 支持字段
`width`、`height`、`distortion_model`、`k`/`camera_matrix`(9)、`d`/`distortion_coefficients`、
`r`/`rectification_matrix`(9)、`p`/`projection_matrix`(12)。

## 质量校验工具
```bash
# 离线校验 rosbag2
check_p0_time ~/bags/p0_timecheck
# 或在线 10s
check_p0_time --live 10
```
输出：IMU rate、dt(min/mean/max)、非单调计数、Cam-IMU 最近邻时差统计（min/mean/std/max、>10ms 跳变数）。Exit 0 = PASS。
