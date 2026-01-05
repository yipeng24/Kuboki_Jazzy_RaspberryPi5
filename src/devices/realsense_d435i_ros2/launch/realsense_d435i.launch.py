from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_dir = get_package_share_directory('realsense_d435i_ros2')
    config_file = os.path.join(pkg_dir, 'config', 'conf_realsense.yaml')

    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera',
        namespace='',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        realsense_node
    ])
