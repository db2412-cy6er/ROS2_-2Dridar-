# P2 YOLO 语义识别节点（xuegecar_yolo）报告

> 项目：Leap 实物小车（leap_demo）· P2：在 PC 端部署 YOLO 语义识别模块，为 P3 地面投影提供检测输出
> 日期：2026-09-08
> 状态：代码落地完成；colcon 编译通过；无真机端到端冒烟测试通过（yolo26n / 800×600 样例帧检出 4 目标）；待真机复测。

---

## 1. 目标与范围

| 项 | 值 | 说明 |
|---|---|---|
| 定位 | `xuegecar_yolo` 作为 leap_demo 源码对应实物小车的 YOLO 识别模块 | 包位于 `xuegeros_ws/src/xuegecar_yolo` |
| 图像输入 | `/camera/image_raw`（bgr8，800×600，best-effort） | 由 `semantic_slam_ws/src/leap_camera_bridge` 发布，`frame_id=camera_optical_frame` |
| 检测输出 | `/semantic/detections`（std_msgs/String，JSON，reliable depth 10） | **P3 地面投影接口，本阶段已按 P3 需求设计好字段** |
| 可视化输出 | `/semantic/image_annotated`（bgr8，best-effort） | 仅供 rqt 查看，丢帧可接受 |
| 模型 | `~/xuegeros_ws/models/yolo26n.pt` | ultralytics 8.4.143，task=detect，80 类 COCO（含 bottle） |
| 默认目标类 | `bottle`（COCO 类 39） | 空列表 = 全部 80 类 |
| 默认参数 | conf 0.30 / iou 0.45 / imgsz 640 / device cpu / infer_every_n_frames 1 | 见 `config/yolo26n.yaml` |

### 框底中心点（P3 需要）

本节点对每个目标额外输出框底中点，供 P3 做地面投影：

```
bottom_x = (x_min + x_max) / 2   （即 center_x）
bottom_y = y_max                 （检测框底边 y）
```

同时保留 `center_x / center_y` 与完整 `x_min/y_min/x_max/y_max`。

---

## 2. 可行性分析结论

| # | 检查项 | 结论 | 依据 |
|---|---|---|---|
| 1 | 模型文件与可加载性 | ✅ | `yolo26n.pt`（5.5 MB）实测加载：task=detect、80 类、`bottle` 在 names 中 |
| 2 | CPU 推理性能 | ✅ | 独立实测 ~39 ms/帧（imgsz 640），节点内稳态 ~38–58 ms/帧；相机 20 fps 可 1:1 处理，余量可调 `infer_every_n_frames` |
| 3 | 图像上游匹配 | ✅ | `leap_camera_bridge` 发布 `/camera/image_raw`（bgr8、best-effort、`camera_optical_frame`），与订阅 QoS/编码一致 |
| 4 | 分辨率口径 | ✅ | `leap_cam` 固件 `FRAMESIZE_SVGA` = 800×600，与 P0 口径一致 |
| 5 | cv_bridge 可用性 | ❌（已规避） | 本机 `~/.local` 为 numpy 2.2.6 + pip opencv，与 apt `ros-humble-cv-bridge`（numpy 1.x ABI）冲突 → `AttributeError: _ARRAY_API`；**节点改用 numpy 直解 bgr8，去掉 cv_bridge**（与 leap_camera_bridge 同风格） |
| 6 | 编译环境 | ⚠️（已解决） | `~/.local` setuptools 84.0.0 与 colcon 不兼容 → 编译时 `PYTHONNOUSERSITE=1` 走 apt setuptools 59.6.0 |

---

## 3. 与原始设计稿的差异修正（针对性改动）

原始计划代码按 `yolo11n.pt` 编写，本项目实际模型为 `yolo26n.pt`，逐点修正：

1. **模型名全部 `yolo11n → yolo26n`**：默认 `model_path`、config/launch 文件名、日志、JSON 的 `model` 字段；且 `model_label` 默认从 model_path 文件名派生（不写死）。
2. **移除 cv_bridge**：图像收发用 numpy `frombuffer/reshape/tobytes`（bgr8，兼容 rgb8），规避本机 ABI 冲突；`package.xml` 同步去掉 `<depend>cv_bridge</depend>`。
3. **补全 setup.py**：原为空壳，现含 `console_scripts: yolo_detector`、`launch/*.launch.py` 与 `config/*.yaml` 的 data_files、版本/描述/maintainer。
4. **package.xml 同步**：版本 0.0.1、描述、license（Apache-2.0）、maintainer 统一。
5. **QoS 设计**：`/semantic/image_annotated` 采用 best-effort（图像流惯例，避免反压）；`/semantic/detections` 采用 reliable depth 10（保证 P3 消费者按序收全）。
6. **其它健壮性**：解码异常、推理异常、模型缺失均记录日志不崩溃；box 坐标裁剪到图像范围。

---

## 4. 具体改动清单

| 文件 | 改动 |
|---|---|
| `xuegeros_ws/src/xuegecar_yolo/xuegecar_yolo/yolo_detector.py` | 新建：P2 检测节点（见 §3 全部修正点） |
| `xuegeros_ws/src/xuegecar_yolo/setup.py` | 重写：入口/data_files/版本补齐 |
| `xuegeros_ws/src/xuegecar_yolo/package.xml` | 更新：版本/描述/license，去 cv_bridge |
| `xuegeros_ws/src/xuegecar_yolo/config/yolo26n.yaml` | 新建：节点参数（bottle、话题、阈值、device） |
| `xuegeros_ws/src/xuegecar_yolo/launch/yolo26n.launch.py` | 新建：加载上述参数启动节点 |
| `leap_demo/report/P2语义识别YOLO节点.md` | 本文档 |

### 参数表（config/yolo26n.yaml）

| 参数 | 值 | 说明 |
|---|---|---|
| `model_path` | `/home/chuyun/xuegeros_ws/models/yolo26n.pt` | 模型文件 |
| `model_label` | `yolo26n` | JSON/日志中的模型标识 |
| `image_topic` | `/camera/image_raw` | 订阅图像 |
| `detections_topic` | `/semantic/detections` | 检测 JSON 输出 |
| `annotated_topic` | `/semantic/image_annotated` | 可视化输出 |
| `confidence_threshold` | 0.30 | conf 阈值 |
| `iou_threshold` | 0.45 | NMS IoU |
| `image_size` | 640 | 推理边长 |
| `device` | `cpu` | 推理设备 |
| `infer_every_n_frames` | 1 | 每 N 帧推理一次 |
| `target_classes` | `["bottle"]` | 空数组 = 全部 80 类 |

---

## 5. 编译与测试记录

### 5.1 编译

```bash
cd ~/xuegeros_ws
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build --symlink-install --packages-select xuegecar_yolo
```

结果：`Summary: 1 package finished`，EXIT=0。
安装产物确认：`install/xuegecar_yolo/lib/xuegecar_yolo/yolo_detector`（console script）、`share/xuegecar_yolo/{config,launch}/*`（symlink 至 build）、egg-link 正常。

> 注：必须用 `PYTHONNOUSERSITE=1`（或清理 `~/.local` 中的 setuptools 84.0.0），否则 colcon 报 `error: option --editable not recognized`。

### 5.2 端到端冒烟测试（无真机）

方法：`ROS_DOMAIN_ID=42` 隔离运行；启动 `yolo_detector`（测试时 `target_classes` 临时覆写为 `["person"]`）；用 800×600 的 ultralytics 样例帧以 bgr8 发布到 `/camera/image_raw`；订阅 `/semantic/detections` 与 `/semantic/image_annotated`。

节点启动日志（节选）：

```
[INFO] [yolo_detector]: Loading YOLO model: /home/chuyun/xuegeros_ws/models/yolo26n.pt
[INFO] [yolo_detector]: Model yolo26n loaded: task=detect, classes=80
[INFO] [yolo_detector]: YOLO detector started | image=/camera/image_raw |
        detections=/semantic/detections | annotated=/semantic/image_annotated |
        targets=person | conf=0.30 | device=cpu
```

稳态性能日志：`FPS: 1.0 | infer: 38.0–58.9 ms | detections: 4`（单帧独立推理 ~39 ms；首帧冷启动一次性 ~1.1–1.3 s，含 torch 线程初始化）。

实际收到的完整 DETJSON 样例：

```json
{
  "stamp": { "sec": 1788880548, "nanosec": 550598909 },
  "frame_id": "camera_optical_frame",
  "image_width": 800,
  "image_height": 600,
  "model": "yolo26n",
  "inference_time_ms": 1320.25,
  "num_detections": 4,
  "detections": [
    {
      "class_id": 0, "class_name": "person", "confidence": 0.8794,
      "x_min": 662, "y_min": 215, "x_max": 799, "y_max": 487,
      "center_x": 730, "center_y": 351,
      "bottom_x": 730, "bottom_y": 487
    },
    {
      "class_id": 0, "class_name": "person", "confidence": 0.8591,
      "x_min": 50, "y_min": 220, "x_max": 223, "y_max": 504,
      "center_x": 136, "center_y": 362,
      "bottom_x": 136, "bottom_y": 504
    },
    {
      "class_id": 0, "class_name": "person", "confidence": 0.8261,
      "x_min": 221, "y_min": 222, "x_max": 340, "y_max": 478,
      "center_x": 280, "center_y": 350,
      "bottom_x": 280, "bottom_y": 478
    },
    {
      "class_id": 0, "class_name": "person", "confidence": 0.7367,
      "x_min": 0, "y_min": 307, "x_max": 77, "y_max": 484,
      "center_x": 38, "center_y": 395,
      "bottom_x": 38, "bottom_y": 484
    }
  ]
}
```

话题流量（`ros2 topic hz`）：`/semantic/detections` ~1.1 Hz；`/semantic/image_annotated` 约 8 帧/35 s（best-effort 可视化流，丢帧符合预期）。模型单帧推理稳定 ~40 ms，说明观测吞吐上限来自测试场景本身（800×600 bgr 大图 + 双 python 进程 + 无真机节拍），非模型瓶颈。

### 5.3 launch 验证（默认 bottle 配置）

```bash
ros2 launch xuegecar_yolo yolo26n.launch.py
```

结果：节点正常启动，正确读取共享目录参数：
`Loading YOLO model: .../yolo26n.pt` → `Model yolo26n loaded: task=detect, classes=80` → `targets=bottle | conf=0.30 | device=cpu`（25 s 后由 timeout 正常结束，EXIT=124 为预期）。

### 5.4 真机运行方式（2026-09-09 实测版）

摄像头与 YOLO 都在 `xuegeros_ws` 下运行，分两个终端：

```bash
# 终端 1：摄像头发布器（保持运行）
source /opt/ros/humble/setup.bash
source ~/xuegeros_ws/install/setup.bash
ros2 launch xuegecar_camera http_video_publisher.launch.py
#   → 同时发布 /camera/image_raw/compressed（jpeg）与 /camera/image_raw（raw RGB8）

# 终端 2：YOLO26n 语义识别节点（首次加载模型稍慢属正常）
source /opt/ros/humble/setup.bash
source ~/xuegeros_ws/install/setup.bash
ros2 launch xuegecar_yolo yolo26n.launch.py
```

查看结果：

```bash
ros2 topic echo /semantic/detections        # 无目标时 num_detections=0
rqt_image_view /semantic/image_annotated    # 绿框=bbox，红点=bbox 底边中心（P3 投影点）
```

> 上游也可以换成 semantic_slam_ws 的 leap_camera_bridge（bgr8，/camera/image_raw），
> 但该相机 :81 视频端口为**单路客户端**，两个发布器不能同时运行（见 5.5）。

### 5.5 真机联调实测：YOLO 收不到图像的问题与修复（2026-09-09）

现象：rqt 里能看到 `/camera/image_raw/compressed` 有画面，但 `/semantic/detections`
完全没有消息、yolo 节点没有任何 FPS/推理日志。

根因（`ros2 topic info /camera/image_raw` 证实）：`xuegecar_camera` 的
`http_video_publisher` 默认 `publish_raw: false`，只发布压缩话题
`/camera/image_raw/compressed`；而 yolo 节点订阅的是**原始图像**话题
`/camera/image_raw`（排查时该话题 Publisher count=0），因此 YOLO 永远收不到帧。

修复（已落地并重编 xuegecar_camera）：
1. `xuegecar_camera/config/http_video_publisher.yaml`：`publish_raw: true`；
2. `xuegecar_camera/xuegecar_camera/http_video_publisher.py`：节点默认参数同步改为 `True`；
3. `xuegecar_camera/README.md`：补充默认发布 raw 的说明。

兼容性要点：http_video_publisher 在 `/camera/image_raw` 上发布 **RGB8** 编码原始图
（`frame_id=camera_link`）；本 yolo 节点的 numpy 解码器原生支持 rgb8/bgr8，无需改动，
annotated 输出统一为 bgr8。

实测结果（重启发布器后生效，yolo_detector 无需重启）：
- `/camera/image_raw`：Publisher=1 / Subscription=1，实测帧率约 3–9 Hz（与相机负载有关，符合预期 2–6+ Hz）；
- `/semantic/detections`：持续输出，实测约 6.5 Hz；
- `/semantic/image_annotated`：best-effort 持续输出（实测约 1.5–4.6 Hz，按图像 QoS 丢弃属正常）；
- 实时 JSON 样例（画面暂无目标时）：
```json
{"frame_id": "camera_link", "image_width": 800, "image_height": 600,
 "model": "yolo26n", "inference_time_ms": 503.61,
 "num_detections": 0, "detections": []}
```

注意事项：该 Maturo/hi3510 图传相机 :81 视频端口为单路客户端，同一时刻只能有一个
发布器占用；xuegecar_camera 与 leap_camera_bridge 二选一，切换前先 Ctrl-C 旧的。

---

## 6. P3 对接接口定义（本节点已按此输出）

**话题**：`/semantic/detections`，类型 `std_msgs/String`，QoS reliable depth 10。

JSON 顶层字段：`stamp{sec,nanosec}`、`frame_id`、`image_width`、`image_height`、`model`、`inference_time_ms`、`num_detections`、`detections[]`。

每个检测元素：

| 字段 | 类型 | 含义 / P3 用途 |
|---|---|---|
| `class_id` / `class_name` | int / str | 类别（默认 bottle，COCO 39） |
| `confidence` | float | 置信度 |
| `x_min / y_min / x_max / y_max` | int | 检测框（图像像素，800×600 坐标系） |
| `center_x / center_y` | int | 框中心 |
| `bottom_x / bottom_y` | int | **框底中点：P3 地面投影的关键输入**（目标底部与地面/平面接触点的投影基准） |

`stamp` 取自输入图像 header.stamp：若用 leap_camera_bridge 上游，为 X-Capture-Ros-Time-Ns 同步时间戳（P0 口径）；若用 xuegecar_camera 上游，为其发布时刻时间戳。P3 与 /odom、/imu 对齐时需注意该差异（建议真机用 leap_camera_bridge 口径或统一做时间同步）。

---

## 7. 已知注意点 / 待真机确认

1. **cv_bridge 不可用**为本机既有环境事实（numpy 2.2.6 vs apt cv_bridge）。本节点已用 numpy 直解规避；如未来其它节点必须用 cv_bridge，请单独准备 numpy<2 的环境，勿改本机 `~/.local`。
2. **colcon 编译需 `PYTHONNOUSERSITE=1`**（`~/.local` setuptools 84 覆盖 apt 59.6 导致 `--editable` 报错）；若清理用户级 setuptools 后可直接编译。
3. 冷启动首帧 ~1.1–1.3 s（torch 线程初始化），稳态单帧 ~40 ms；若 800×600 相机跑到 20 fps 且主机负载高，可把 `infer_every_n_frames` 调为 2–3。
4. `/semantic/image_annotated` 为 best-effort，仅用于可视化；**P3 必须订阅 `/semantic/detections`（reliable）**。
5. 真机测试需先确认 leap_camera_bridge 时间同步状态（P0：推理前 hold，保证 stamp 稳定），再用真实 bottle 目标测检出率；当前为 80 类 COCO 通用预训练，检出不佳时可换自训练模型（仅改 `model_path`，JSON 的 `model` 字段随之更新）。
6. 输出坐标为图像像素（800×600），与相机内参/外参一起交由 P1（内参标定）与 P3（投影模型）使用，本阶段不做坐标系变换。
7. （2026-09-09 真机实测）`xuegecar_camera` 必须 `publish_raw: true`（现已默认开启）才会在 `/camera/image_raw` 发布 raw（RGB8），否则 YOLO 收不到帧（只看到压缩流无效）。
8. 相机 :81 为单路视频通道，`xuegecar_camera` 与 `leap_camera_bridge` 只能二选一运行；切换发布器后 yolo 节点会自动重新匹配订阅，无需重启。
9. 当前原始图像 `frame_id=camera_link`、编码 RGB8；P3 若需用 TF（camera_optical_frame 口径）做地面投影，需在 P1/P3 阶段统一 frame_id 与相机外参。

---

## 8. 下一步（P3 前置）

1. P1：相机内参标定（Kalibr + Aprilgrid，800×600），生成 `camera_info`；
2. P3：新增消费节点订阅 `/semantic/detections`，用 `bottom_x/bottom_y` + 相机模型/地面平面假设把目标投影到地面，形成语义目标列表（话题/格式另定）；
3. 结合 /odom（P0 已 ~50 Hz）做时间对齐与运动滤波；
4. 每完成一轮在本目录追加对应报告（如 `P3地面投影与语义目标.md`）。




