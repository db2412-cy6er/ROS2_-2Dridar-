import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():

    pkg_dir = get_package_share_directory('xuegecar_localizer')

    config_file = os.path.join(
        pkg_dir,
        'config',
        'semantic_localizer.yaml',
    )

    return LaunchDescription([

        Node(
            package='xuegecar_localizer',
            executable='semantic_localizer',
            name='semantic_localizer',
            output='screen',
            parameters=[
                config_file,
            ],
        ),

    ])
