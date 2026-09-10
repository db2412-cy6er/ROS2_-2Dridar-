# ROS2 2D-Lidar 语义导航小车（Leap 系列）

本项目基于出云科技的ROS2小车进行开发
项目地址https://github.com/czu963889306-dev/leap_ros_ws
本仓库是一个 **monorepo**，把整车相关的四棵目录树放在一起：

| 目录 | 内容 |
|---|---|
| `xuegeros_ws/` | **整车 ROS 2 (Humble) 工作区**：17 个包（底盘/相机/雷达/检测/定位/语义地图/自然语言导航/建图/上位机），另含 `maps/`（建图产物）与 `models/`（YOLO 26n权重） |
| `leap_demo/` | **固件与文档**：`leap_cam/`（ESP32-S3 OV3660 图传固件）、`leap_ros/leap_low_v1/`（ESP32-S3 micro-ROS 底盘驱动固件）、`report/`（阶段性技术报告） |
| `semantic_slam_ws/` | 早期 ROS 2 工作区（`src/leap_camera_bridge`，相机桥接包） |
| `YDLidar-SDK/` | YDLidar 官方 SDK（C++ / Python，含 examples 与 doc） |

---

## 1. 系统功能流水线

```
P1 底盘与定位  xuegecar_bringup(/odom -> TF odom->base_footprint) + xuegecar_description(URDF)
P2 语义识别    xuegecar_camera(OV3660 MJPEG) -> xuegecar_yolo(YOLO 检测, /semantic/detections)
P3 空间定位    xuegecar_localizer(单目地面求交 -> map 坐标, /semantic/objects_localized)
P4 动态语义地图 xuegecar_dynamic_semantic_mapper(关联/EMA/确认 -> /semantic/dynamic_map)
P5 自然语言导航 xuegecar_llm_navigation(DeepSeek 意图 -> 确定性查询 -> Nav2 /navigate_to_pose)
               xuegecar_navigation2(Nav2 参数/launch/maps/rviz)
建图          xuegecar_cartographer(2D SLAM 配置) + maps/room_baseline.*
```

技术细节见 `leap_demo/report/`：

- `P0时间同步.md`
- `P2语义识别YOLO节点.md`
- `P3空间定位层.md`
- `P4动态语义地图管理器.md`
- `P5自然语言语义导航.md`
- `磁盘清理报告.md`

---

## 2. 编译与运行

### 2.1 整车工作区（ROS 2 Humble / Ubuntu 22.04）

```bash
cd xuegeros_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

> ⚠️ **配置里有绝对路径**（`/home/chuyun/xuegeros_ws/...`）：`xuegecar_localizer` 的 `camera_info_file`、`xuegecar_dynamic_semantic_mapper` 的 `snapshot_file`、launch 里的默认地图 `maps/room_baseline.yaml`。换机器请全局替换为自己的路径。

### 2.2 固件（ESP-IDF，目标芯片 esp32s3）

```bash
# 图传固件（工程名 ov3660_http_stream）
cd leap_demo/leap_cam && idf.py set-target esp32s3 && idf.py build flash monitor

# 底盘 micro-ROS 固件（工程名 leap_low_v1）
cd leap_demo/leap_ros/leap_low_v1 && idf.py set-target esp32s3 && idf.py build flash monitor
```

> `managed_components/`（约 400MB）未入库，`idf.py reconfigure` 会自动拉取依赖。

### 2.3 YDLidar SDK

```bash
cd YDLidar-SDK && mkdir -p build && cd build
cmake .. && make -j$(nproc) && sudo make install && sudo ldconfig
```

### 2.4 早期相机桥工作区

```bash
cd semantic_slam_ws && colcon build --symlink-install && source install/setup.bash
```

---

## 3. 未入库的内容（有意排除）

| 路径 | 大小 | 原因 |
|---|---|---|
| `xuegeros_ws/bags/` | 920 MB | 含 **919MB 单个 rosbag**，超过 GitHub 单文件 100MB 硬上限；如需版本化请改用 Git LFS |
| `leap_demo/leap_ros.tar` | 82.8 MB | 与 `leap_ros/` 内容重复的打包产物 |
| 各 `build/` `install/` `log/` | ~450 MB | 编译产物，可重新生成 |
| 各 `managed_components/` | ~400 MB | ESP-IDF 依赖缓存，可重新拉取 |
| `xuegeros_ws/maps/*.pbstream` | ~13 MB | cartographer 中间件；导出的 `pgm/yaml` 已入库 |
| `xuegeros_ws/src/**/.git` | — | 嵌套仓库（会让外层仓库变成空指针） |

---

## 4. 本仓库与本地工作区的关系

本仓库目录是**四棵源目录的镜像拷贝**（原工作区仍在 `~` 下按原路径使用，绝对路径不受影响）。
需要把最新改动同步进仓库时：

```bash
./sync_to_repo.sh          # 见仓库根目录脚本，rsync 增量同步后自行 git commit/push
```

---

## 5. 硬件与环境

- 上位机：Ubuntu 22.04 + ROS 2 Humble
- 底盘：ESP32-S3 + micro-ROS 固件（`leap_low_v1`）
- 相机：OV3660（ESP32-S3 图传固件 `leap_cam`，MJPEG HTTP 流）
- 雷达：2D LiDAR（`YDLidar-SDK` / `ydlidar_ros2_driver` / `ldlidar_stl_ros2` / `camsense_lidar`）
- 大模型：DeepSeek API（P5 意图解析，密钥通过环境变量 `DEEPSEEK_API_KEY` 提供，**不入库**）
