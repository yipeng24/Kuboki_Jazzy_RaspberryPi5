#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1) 静态 TF：base_footprint -> laser（把位姿改成你实际安装位置）
    tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_basefootprint_to_laser',
        # x y z yaw pitch roll parent child
        arguments=['0.20', '0.0', '0.15', '0', '0', '0', 'base_footprint', 'laser']
    )

    # 2) slam_toolbox：直接用 dict 给参数（不会出现 “Parameter not set” 的那种没加载问题）
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': False,

            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_footprint',
            'scan_topic': '/scan',

            'mode': 'mapping',

            # async 模式关键
            'scan_queue_size': 1,
            'transform_publish_period': 0.02,

            'map_update_interval': 2.0,
            'resolution': 0.05,
            'max_laser_range': 12.0,

            'minimum_travel_distance': 0.05,
            'minimum_travel_heading': 0.05,
        }],
    )

    return LaunchDescription([
        tf_base_to_laser,
        slam_node
    ])
