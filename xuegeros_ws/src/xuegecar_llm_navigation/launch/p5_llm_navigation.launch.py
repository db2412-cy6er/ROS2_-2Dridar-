"""Launch file for the P5 LLM navigation node.

    ros2 launch xuegecar_llm_navigation p5_llm_navigation.launch.py

The DeepSeek API key is read from the environment (DEEPSEEK_API_KEY) and
is deliberately NOT a launch argument: launch arguments and ROS
parameters are visible to `ros2 param dump` and to anyone on the graph.

`api_model` *is* a launch argument so the model can be switched without
editing the yaml, e.g.::

    ros2 launch xuegecar_llm_navigation p5_llm_navigation.launch.py \
        api_model:=deepseek-flash

The node also verifies the name against ``GET /models`` at startup and
falls back to ``api_models_fallback`` when the account does not offer it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    package_dir = get_package_share_directory('xuegecar_llm_navigation')
    default_config = os.path.join(package_dir, 'config',
                                  'p5_llm_navigation.yaml')

    config_file = LaunchConfiguration('config_file')
    auto_execute = LaunchConfiguration('auto_execute_navigation')
    api_model = LaunchConfiguration('api_model')

    declare_config = DeclareLaunchArgument(
        'config_file', default_value=default_config,
        description='P5 parameter file')

    declare_auto_execute = DeclareLaunchArgument(
        'auto_execute_navigation', default_value='true',
        description='false = answer only, never command Nav2 (dry run)')

    declare_api_model = DeclareLaunchArgument(
        'api_model', default_value='deepseek-v4-pro',
        description='DeepSeek model id; overrides the yaml value and is '
                    'verified against GET /models at startup')

    node = Node(
        package='xuegecar_llm_navigation',
        executable='llm_navigation_node',
        name='llm_navigation_node',
        output='screen',
        parameters=[
            config_file,
            {'auto_execute_navigation': auto_execute,
             'api_model': api_model},
        ],
    )

    return LaunchDescription([
        declare_config,
        declare_auto_execute,
        declare_api_model,
        node,
    ])
