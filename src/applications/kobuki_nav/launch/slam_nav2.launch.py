#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('kobuki_nav')

    slam_params = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    # 1) SLAM Toolbox
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params]
    )

    # 2) 静态 TF：base_footprint -> laser
    # 注意：你现在 arguments 的 rpy/ypr 顺序要跟你系统实际一致
    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_basefootprint_to_laser',
        output='screen',
        # x y z yaw pitch roll parent child
        arguments=['0.0', '0.0', '0.15', '3.14', '0', '0', 'base_footprint', 'laser']
    )

    # 3) SLAM lifecycle
    lifecycle_slam = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ["slam_toolbox"]
        }]
    )

    # -----------------------
    # 4) Nav2 core nodes (NO map_server / NO AMCL)
    # -----------------------
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params],
    )

    # controller_server：把 /cmd_vel 改到 /cmd_vel_nav
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params],
        # remappings=[
        #     ('/cmd_vel', '/cmd_vel_nav'),
        # ],
    )

    # velocity_smoother：订阅 /cmd_vel_nav，输出 /cmd_vel
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


    # 6) Nav2 lifecycle
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
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                # 'global_costmap',
                # 'local_costmap',
                #'velocity_smoother',
            ]
        }]
    )

    return LaunchDescription([
        # slam_node,
        # tf_node,
        # lifecycle_slam,

        planner_server,
        controller_server,
        # velocity_smoother,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        # global_costmap,
        # local_costmap,
        lifecycle_nav2,
    ])
