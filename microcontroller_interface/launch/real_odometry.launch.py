import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('microcontroller_interface')
    ekf_config_file = os.path.join(pkg_share, 'config', 'ekf.yaml')
    rviz_config_file= os.path.join(pkg_share, 'config', 'rviz_config.rviz')

    odom_imu_node = Node(
        package='microcontroller_interface',
        executable='odom_imu_node',
        name='odom_imu_node',
        output='screen'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file]
    )

    return LaunchDescription([
        odom_imu_node,
        rviz_node,
        robot_localization_node
    ])