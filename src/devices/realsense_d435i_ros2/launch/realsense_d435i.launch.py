from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_dir = get_package_share_directory('realsense_d435i_ros2')
    config_file = os.path.join(pkg_dir, 'config', 'conf_realsense_640x480x15.yaml')

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
    delayed_compress = TimerAction(
        period=2.0,
        actions=[throttle_compress_node]
    )
    return LaunchDescription([
        realsense_node,
        delayed_compress
    ])
