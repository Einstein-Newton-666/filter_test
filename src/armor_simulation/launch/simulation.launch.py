import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('armor_simulation'), 'config')
    config_file = os.path.join(config_dir, 'simulation_config.yaml')

    # 3D仿真器节点
    armor_simulation_node = Node(
        package='armor_simulation',
        executable='armor_simulation_node',
        name='armor_simulation_node',
        output='screen',
        parameters=[config_file],
    )

    # 2D相机仿真器节点
    camera_simulator_node = Node(
        package='armor_simulation',
        executable='camera_simulator',
        name='camera_simulator',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([
        armor_simulation_node,
        camera_simulator_node,
    ])
