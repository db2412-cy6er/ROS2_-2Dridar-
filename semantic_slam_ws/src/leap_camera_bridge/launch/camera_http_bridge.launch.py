import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'stream_url', default_value='http://192.168.31.100:81/',
            description='Camera MJPEG HTTP stream URL'),
        DeclareLaunchArgument(
            'frame_id', default_value='camera_optical_frame',
            description='frame_id for /camera/image_raw'),
        DeclareLaunchArgument(
            'camera_info_yaml', default_value='',
            description='Path to calibrated camera_info YAML (optional, before calibration leave empty)'),
        Node(
            package='leap_camera_bridge',
            executable='camera_http_bridge',
            name='camera_http_bridge',
            output='screen',
            parameters=[{
                'stream_url': LaunchConfiguration('stream_url'),
                'frame_id': LaunchConfiguration('frame_id'),
                'camera_info_yaml': LaunchConfiguration('camera_info_yaml'),
            }],
        ),
    ])
