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

    camera_pkg_dir = get_package_share_directory('realsense_d435i_ros2')
    config_file = os.path.join(camera_pkg_dir, 'config', 'conf_realsense_640x480x15.yaml')

    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera',
        namespace='',
        output='screen',
        parameters=[config_file]
    )

    throttle_compress_node = Node(
        package='realsense_d435i_ros2',
        executable='throttle_compress_node',
        name='throttle_compress',
        namespace='',
        output='screen'
    )

    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('kobuki_nav'),
                'launch',
                'kobuki_slam.launch.py'
            ])
        )
    )

    nav2_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('kobuki_nav'),
                'launch',
                'nomap_nav2.launch.py'
            ])
        )
    )


    return LaunchDescription([
        basebringup_launch,
        TimerAction(period=5.0, actions=[lidar_launch]),
        TimerAction(period=10.0, actions=[realsense_node]),
        TimerAction(period=15.0, actions=[throttle_compress_node]),

    ])
