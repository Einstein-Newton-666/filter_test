import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # 配置文件路径
    filter_test_config = os.path.join(
        get_package_share_directory('filter_test'), 'config', 'config.yaml')
    simulation_config = os.path.join(
        get_package_share_directory('armor_simulation'), 'config', 'simulation_config.yaml')

    # 启动参数
    use_graph_optimizer = DeclareLaunchArgument(
        'use_graph_optimizer',
        default_value='true',
        description='Whether to use graph optimizer instead of EKF/UKF')

    # 仿真器 (运动 + 相机投影 + 噪声 + PnP, 相机外参从 TF 获取)
    armor_simulation_node = Node(
        package='armor_simulation',
        executable='armor_simulation_node',
        name='armor_simulation_node',
        output='screen',
        parameters=[simulation_config],
    )

    # 云台仿真 (S-curve 动力学 + TF 广播 + 反馈)
    gimbal_simulation_node = Node(
        package='armor_simulation',
        executable='gimbal_simulation',
        name='gimbal_simulation',
        output='screen',
        parameters=[filter_test_config],
    )

    # 角度解算器 placeholder (简单几何, 未来扩展 MPC)
    angle_solver_node = Node(
        package='armor_simulation',
        executable='angle_solver',
        name='angle_solver',
        output='screen',
    )

    # 传统滤波器
    filter_node = Node(
        package='filter_test',
        executable='filter',
        name='filter',
        output='screen',
        parameters=[filter_test_config],
    )

    # 图优化器 (发布 /tracker/target 给 angle_solver)
    graph_optimizer_node = Node(
        package='filter_test',
        executable='graph_optimizer_test',
        name='graph_optimizer_test',
        output='screen',
        parameters=[filter_test_config],
    )

    # jlu tracker (移植自 jlu_vision_26)
    jlu_tracker_node = Node(
        package='filter_test',
        executable='jlu_tracker',
        name='jlu_tracker',
        output='screen',
        parameters=[filter_test_config],
    )

    return LaunchDescription([
        use_graph_optimizer,
        armor_simulation_node,
        gimbal_simulation_node,
        # angle_solver_node,
        # filter_node,
        graph_optimizer_node,
        # jlu_tracker_node,
    ])
