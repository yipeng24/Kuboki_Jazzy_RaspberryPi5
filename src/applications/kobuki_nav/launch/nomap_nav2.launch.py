#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import TimerAction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('kobuki_nav')
    nav2_params = os.path.join(pkg_share, 'config', 'nomap_nav2_params.yaml')

    basebringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('kobuki_node'),
                'launch',
                'kobuki_node.launch.py'
            ])
        )
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('sllidar_ros2'),
                'launch',
                'sllidar_c1_launch.py'
            ])
        )
    )
    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_basefootprint_to_laser',
        # x y z yaw pitch roll parent child
        # 把 0.20 0.00 0.15 改成你激光雷达相对底盘中心的安装位置
        arguments=['0.0', '0.0', '0.15', '3.14', '0', '0', 'base_footprint', 'laser']
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params],
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params],
        remappings=[
            ('/cmd_vel', '/cmd_vel_nav'),
        ],
    )

    # 如果你要启用 smoother，建议也纳入 lifecycle_manager 的 node_names
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params],
        remappings=[
            ('/cmd_vel', '/cmd_vel_nav'),
            ('/cmd_vel_smoothed', '/cmd_vel'),
        ],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params],
    )

    lifecycle_nav2 = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': [
                'planner_server',
                'controller_server',
                # 'velocity_smoother',  
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
            ]
        }]
    )

    nav2_group = GroupAction([
        tf_node,
        planner_server,
        controller_server,
        # velocity_smoother,     
        behavior_server,
        bt_navigator,
        waypoint_follower,
        lifecycle_nav2,
    ])

    delayed_nav2 = TimerAction(
        period=5.0,
        actions=[nav2_group]
    )

    return LaunchDescription([
        basebringup_launch,
        lidar_launch,
        delayed_nav2,
    ])
