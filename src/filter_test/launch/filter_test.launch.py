import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable
from launch.actions import TimerAction

def generate_launch_description():
    node_params = os.path.join(
        get_package_share_directory('filter_test'), 'config', 'config.yaml')

    filter_test_node = Node(
        package='filter_test',
        executable='filter',
        name='filter',
        output='screen',
        parameters=[node_params],
    )

    armor_simulation_node = Node(
        package='filter_test',
        executable='armor_simulation',
        name='armor_simulation',
        output='screen',
        parameters=[node_params],
    )

    return LaunchDescription([
        filter_test_node,
        armor_simulation_node
        ])