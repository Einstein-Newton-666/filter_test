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


    armor_simulation_node = Node(
        package='filter_test',
        executable='armor_simulation',
        name='armor_simulation',
        output='screen',
        # parameters=[params_file],
    )

    tracker_node = Node(
    package='armor_tracker',
    executable='armor_tracker_node',
    output='both',
    emulate_tty=True,
    parameters=[node_params],
    # arguments=['--ros-args', '--log-level', 'armor_tracker:='+launch_params['tracker_log_level']],
)

    return LaunchDescription([
        # tracker_node,
        armor_simulation_node
        ])