import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():

    pkg_dir = get_package_share_directory('xuegecar_yolo')

    config_file = os.path.join(
        pkg_dir,
        'config',
        'yolo26n.yaml',
    )

    return LaunchDescription([

        Node(
            package='xuegecar_yolo',
            executable='yolo_detector',
            name='yolo_detector',
            output='screen',
            parameters=[
                config_file,
            ],
        ),

    ])
