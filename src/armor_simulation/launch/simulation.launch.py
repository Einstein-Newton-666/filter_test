import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('armor_simulation'), 'config')
    config_file = os.path.join(config_dir, 'simulation_config.yaml')

    armor_simulation_node = Node(
        package='armor_simulation',
        executable='armor_simulation_node',
        name='armor_simulation_node',
        output='screen',
        parameters=[config_file],
    )

    rune_simulation_node = Node(
        package='armor_simulation',
        executable='rune_simulation_node',
        name='rune_simulation_node',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([
        armor_simulation_node,
        # rune_simulation_node,
    ])
