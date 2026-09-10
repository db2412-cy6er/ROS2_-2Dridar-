import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    xuegecar_navigation2_dir = get_package_share_directory('xuegecar_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_path = LaunchConfiguration('map',default=os.path.join(xuegecar_navigation2_dir,'maps','room.yaml'))
    nav2_param_path = LaunchConfiguration('params_file',default=os.path.join(xuegecar_navigation2_dir,'param','xuegebot.yaml'))

    # 默认仍是 nav2 自带视图（向后兼容）；P5 建议用本包的 p5_view.rviz：
    # 那个视图只显示一张地图（静态地图），关掉了 global/local costmap 叠加，
    # 否则 rolling 的 costmap 会在静态地图上滑动，看起来像"地图重叠、越来越乱"。
    default_rviz_config = os.path.join(nav2_bringup_dir,'rviz','nav2_default_view.rviz')
    rviz_config = LaunchConfiguration('rviz_config', default=default_rviz_config)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',default_value=use_sim_time,description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('map',default_value=map_yaml_path,description='Full path to map file to load'),
        DeclareLaunchArgument('params_file',default_value=nav2_param_path,description='Full path to param file to load'),
        DeclareLaunchArgument('rviz_config',default_value=default_rviz_config,
                              description='RViz config file; P5: <xuegecar_navigation2>/rviz/p5_view.rviz'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_bringup_dir,'/launch','/bringup_launch.py']),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
