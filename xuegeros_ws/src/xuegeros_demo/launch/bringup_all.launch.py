# bringup_all.launch.py — 终端1 / P1 一键启动(底盘 + 雷达 + SLAM)
#
# 本 launch 的定位(与 leap_demo/report 中 P4 §8 "终端1" 对齐):
#   1. micro-ROS Agent(UDP 8888):唯一 agent,让主控固件连上后自动发布
#      /odom(50Hz) /imu(~100Hz) /scan(10Hz, laser_frame) /battery_state,订阅 /cmd_vel;
#   2. xuegecar_bringup:URDF/robot_state_publisher + 把 /odom 转发为 TF odom->base_footprint;
#   3. (可选, slam:=true, 默认开启) xuegecar_cartographer:提供 map->odom,完成 P1 TF 树
#      map -> odom -> base_footprint -> base_link -> laser_frame。
#
# 注意:
#   * 本 launch 已内含 agent,请勿再另开终端手动 `sudo docker run ... micro-ros-agent ...`,
#     否则两个 agent 会同时抢 UDP 8888,导致 /odom 等话题不可用(端口检测会拦下)。
#   * 已知固件问题:小车若在 agent 消失后(如 Ctrl+C 结束 bringup_all)再启动,
#     micro-ROS 会话会卡在"假活跃"状态 —— 满速发数据但任何 agent 都建不上会话,
#     /odom 不可用。解决办法:重启小车主控(reboot_car:=true 会自动做并等待上线)。
#   * 雷达 /scan 单一来源 = 主控 micro-ROS /scan(雷达接在小车主板),因此不再需要
#     socat(UDP8889 -> /dev/lidar) 与 ydlidar_ros2_driver 这条 PC 侧链路。

import json
import os
import socket
import time
import urllib.parse
import urllib.request

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    Shutdown,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

AGENT_PORT = 8888
CAR_DEFAULT_IP = "192.168.31.63"


def _udp_port_in_use(port):
    """尝试绑定 0.0.0.0:port;能绑上说明端口空闲,绑不上说明已被其它进程占用。"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind(("0.0.0.0", port))
        return False
    except OSError:
        return True
    finally:
        s.close()


def _check_agent_port(context, *args, **kwargs):
    """运行时检查 UDP 8888:检测到其它 agent(如手动 docker run)就停止并提示。"""
    start_agent = str(
        context.launch_configurations.get("start_agent", "true")).lower()
    if start_agent in ("false", "0", "off", "no"):
        return [LogInfo(
            msg="[bringup_all] start_agent=false:使用外部已运行的 micro-ROS Agent,跳过内嵌 agent。")]
    if _udp_port_in_use(AGENT_PORT):
        return [
            LogInfo(msg=(
                f"[bringup_all] 错误:UDP {AGENT_PORT} 已被占用,检测到另一个 micro-ROS Agent 正在运行\n"
                "  原因通常是:另开终端手动 `sudo docker run ... micro-ros-agent udp4 --port 8888`,\n"
                "  或上一次 launch 还没退出。两个 agent 抢同一端口会导致 /odom 无消息。\n"
                "  处理:停止那个 agent 后重试(在其终端 Ctrl+C;docker 可用 `sudo docker ps` 查看),\n"
                "  或确认要使用外部 agent 时,以 `start_agent:=false` 启动本 launch。")),
            Shutdown(reason=f"UDP port {AGENT_PORT} already in use by another micro-ROS Agent"),
        ]
    return []


def _local_ip_for(remote_ip):
    """探测去往小车的本机出口 IP(用于告诉小车 agent 地址)。"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((remote_ip, 80))
        return s.getsockname()[0]
    except OSError:
        return None
    finally:
        s.close()


def _post_form(url, fields):
    body = urllib.parse.urlencode(fields).encode()
    req = urllib.request.Request(url, data=body, method="POST")
    with urllib.request.urlopen(req, timeout=5) as resp:
        return resp.status


def _maybe_reboot_car(context, *args, **kwargs):
    """reboot_car=true 时:软重启小车主控并等待其上线(解决固件 micro-ROS 假活跃问题)。"""
    enabled = str(context.launch_configurations.get("reboot_car", "false")).lower()
    if enabled not in ("true", "1", "yes", "on"):
        return []
    car_ip = str(context.launch_configurations.get("car_ip", CAR_DEFAULT_IP))
    local_ip = _local_ip_for(car_ip)
    if not local_ip:
        print("[bringup_all][reboot_car] 无法探测本机 IP,跳过自动重启。", flush=True)
        return []

    status_url = f"http://{car_ip}/api/status"
    print(f"[bringup_all][reboot_car] 准备软重启小车主控 {car_ip}(agent 地址 {local_ip}:{AGENT_PORT})...", flush=True)
    time.sleep(2.0)  # 等内嵌 agent 先绑定好 8888

    try:
        _post_form(f"http://{car_ip}/api/runtime-config", {
            "comm_mode": "micro_ros",
            "microros_agent_ip": local_ip,
            "microros_agent_port": str(AGENT_PORT),
        })
        print("[bringup_all][reboot_car] 重启指令已下发,等待小车重启(HTTP 掉线->恢复)...", flush=True)
    except Exception as exc:  # noqa: BLE001
        print(f"[bringup_all][reboot_car] 下发重启指令失败(小车可能不在线): {exc}", flush=True)

    def _fetch_status():
        try:
            with urllib.request.urlopen(status_url, timeout=3) as resp:
                return json.load(resp)
        except Exception:  # noqa: BLE001
            return None

    # 阶段1:等 HTTP 掉线(说明重启真正开始),最多 ~20s(未掉线也继续往下走)
    deadline = time.time() + 20.0
    went_down = False
    while time.time() < deadline:
        if _fetch_status() is None:
            went_down = True
            break
        time.sleep(1.0)
    if not went_down:
        print("[bringup_all][reboot_car] 未观察到小车 HTTP 掉线(可能未重启或重启太快),继续等待其上线...", flush=True)

    # 阶段2:等小车重新上线且是新启动(uptime 较小),最多 ~50s
    deadline = time.time() + 50.0
    while time.time() < deadline:
        data = _fetch_status()
        if isinstance(data, dict) and data.get("wifi", {}).get("sta_connected"):
            uptime = int(data.get("uptime_ms", 0) or 0)
            if uptime > 0 and uptime < 60000:  # 新一次开机才认为重启完成
                print("[bringup_all][reboot_car] 小车已重启上线"
                      f"(uptime={uptime} ms),等待其连上 agent ...", flush=True)
                return []
        time.sleep(2.0)

    print("[bringup_all][reboot_car] 等待小车重启上线超时,继续启动(可稍后手动确认)。", flush=True)
    return []


def generate_launch_description():
    pkg_xuegecar_bringup = get_package_share_directory("xuegecar_bringup")
    pkg_cartographer = get_package_share_directory("xuegecar_cartographer")

    xuegecar_launch_file = os.path.join(
        pkg_xuegecar_bringup, "launch", "xuegecar_bringup.launch.py")
    cartographer_launch_file = os.path.join(
        pkg_cartographer, "launch", "cartographer.launch.py")

    # ---------------------------------------------------------------- 参数
    declare_start_agent = DeclareLaunchArgument(
        "start_agent", default_value="true",
        description="是否由本 launch 启动内嵌 micro-ROS Agent(外部已有 agent 时可设 false)")
    declare_slam = DeclareLaunchArgument(
        "slam", default_value="true",
        description="是否内嵌启动 cartographer 提供 map->odom TF(true=终端1完整 P1)")
    declare_slam_rviz = DeclareLaunchArgument(
        "slam_rviz", default_value="false",
        description="内嵌 cartographer 时是否同时打开 rviz2")
    declare_reboot_car = DeclareLaunchArgument(
        "reboot_car", default_value="false",
        description="启动前软重启小车主控并等待上线(解决固件 micro-ROS 假活跃导致 /odom 不可用)")
    declare_car_ip = DeclareLaunchArgument(
        "car_ip", default_value=CAR_DEFAULT_IP,
        description="小车主控 IP(reboot_car=true 时用于下发重启/状态探测)")

    # ---------------------------------------------------------------- 1. Micro-ROS Agent
    # 不开 -v6:避免每个 UDP 包的 hex dump 刷屏,保留默认 info(会话建立/断开等关键信息)。
    micro_ros_agent = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent",
        arguments=["udp4", "--port", str(AGENT_PORT)],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_agent")),
    )

    # ---------------------------------------------------------------- 1b. (可选)软重启小车主控
    # 放在 agent 之后:先让 agent 绑定 8888,重启的小车一上线就能直接建会话。
    # 此步骤仅在 reboot_car:=true 时阻塞等待(~最多45s);默认 false 时立即返回。
    reboot_car_action = OpaqueFunction(function=_maybe_reboot_car)

    # ---------------------------------------------------------------- 2. 底盘 TF(xuegecar_bringup)
    # 稍作延迟,等 agent 起来、/odom 到达后再加载 TF 更稳。
    xuegecar_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(xuegecar_launch_file))
        ],
    )

    # ---------------------------------------------------------------- 3. SLAM(cartographer,可选)
    cartographer_launch = TimerAction(
        period=8.0,  # 等 /scan、/odom 已在 ~1s 内到达,8s 余量充足
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(cartographer_launch_file),
                # 注意:launch_arguments 必须传 2 元组列表(Humble 里传 dict 会崩)
                launch_arguments=[
                    ("use_rviz", LaunchConfiguration("slam_rviz")),
                ],
            )
        ],
        condition=IfCondition(LaunchConfiguration("slam")),
    )

    slam_hint = LogInfo(
        msg="[bringup_all] slam=true:cartographer 已内嵌启动(提供 map->odom)。"
            "请勿再单独 ros2 launch xuegecar_cartographer,以免重复启动两个 cartographer。",
        condition=IfCondition(LaunchConfiguration("slam")),
    )

    return LaunchDescription([
        declare_start_agent,
        declare_slam,
        declare_slam_rviz,
        declare_reboot_car,
        declare_car_ip,
        OpaqueFunction(function=_check_agent_port),
        micro_ros_agent,
        reboot_car_action,
        xuegecar_launch,
        cartographer_launch,
        slam_hint,
    ])
