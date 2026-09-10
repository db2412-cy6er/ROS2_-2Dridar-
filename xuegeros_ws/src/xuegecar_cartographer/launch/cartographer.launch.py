import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 定位到功能包的地址
    pkg_share = FindPackageShare(package='xuegecar_cartographer').find('xuegecar_cartographer')

    # =====================运行节点需要的配置=======================================================================
    # 是否使用仿真时间，我们用gazebo，这里设置成true
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # 地图的分辨率
    resolution = LaunchConfiguration('resolution', default='0.05')
    # 地图的发布周期
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    # 配置文件夹路径
    configuration_directory = LaunchConfiguration('configuration_directory', default=os.path.join(pkg_share, 'config'))
    # 配置文件
    # xuegecar_2d.lua:标准 TF 树 map->odom(cartographer)+odom->base_footprint(轮式里程计),
    #                  tracking=base_footprint,把地图平面锚在路面(z=0)。
    configuration_basename = LaunchConfiguration('configuration_basename', default='xuegecar_2d.lua')
    # 是否同时打开 rviz2(被 bringup_all 内嵌时一般设 false)
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='是否同时打开 rviz2(内嵌到 bringup_all 时可设 false)')

    # =====================声明三个节点，cartographer/occupancy_grid_node/rviz_node=================================
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename],
        # odom 里程计输入用 xuegecar_bringup 重发布的 child=base_footprint 版本
        # (固件原始 /odom 的 child=base_link,若直接喂 cartographer 会把地图锚在底盘高度)
        remappings=[('odom', '/odom/base_footprint')])

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz))

    # ===============================================定义启动文件========================================================
    ld = LaunchDescription()
    ld.add_action(declare_use_rviz)
    ld.add_action(cartographer_node)
    ld.add_action(cartographer_occupancy_grid_node)
    ld.add_action(rviz_node)

    return ld