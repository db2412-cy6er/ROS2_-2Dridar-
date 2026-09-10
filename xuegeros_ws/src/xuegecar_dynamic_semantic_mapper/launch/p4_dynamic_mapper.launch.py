import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_dir = get_package_share_directory('xuegecar_dynamic_semantic_mapper')

    config_file = os.path.join(pkg_dir, 'config', 'p4_dynamic_mapper.yaml')

    return LaunchDescription([
        Node(
            package='xuegecar_dynamic_semantic_mapper',
            executable='dynamic_semantic_mapper',
            name='dynamic_semantic_mapper',
            output='screen',
            parameters=[config_file],
        ),
    ])
