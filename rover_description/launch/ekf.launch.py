import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_rover_description = get_package_share_directory('rover_description')
    ekf_params_file = os.path.join(pkg_rover_description, 'config', 'ekf.yaml')

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file],
    )

    return LaunchDescription([
        ekf_node
    ])
