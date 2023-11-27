import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    pkg_share = get_package_share_directory('microcontroller_interface')
    ekf_config_file = os.path.join(pkg_share, 'config', 'ekf.yaml')
    
    kp = LaunchConfiguration('kp')
    kd = LaunchConfiguration('kd')
    ki = LaunchConfiguration('ki')

    declare_kp = DeclareLaunchArgument(
        name='kp',
        default_value='0.01',
        description='Parameter for proportional controller'
    )
    declare_kd = DeclareLaunchArgument(
        name='kd',
        default_value='0.01',
        description='Parameter for derivative controller'
    )
    declare_ki = DeclareLaunchArgument(
        name='ki',
        default_value='0.01',
        description='Parameter for integral controller'
    )

    odom_imu_node = Node(
        package='microcontroller_interface',
        executable='odom_imu_node',
        name='odom_imu_node',
        output='screen'
    )
    pid_motor_control = Node(
        package='microcontroller_interface',
        executable='pid_motor_control',
        name='pid_motor_control',
        output='screen',
        parameters=[
            {'kp': kp},
            {'kd': kd},
            {'ki': ki}
        ]
    )
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file]
    )

    return LaunchDescription([
        declare_kp,
        declare_kd,
        declare_ki,

        odom_imu_node,
        pid_motor_control,
        robot_localization_node
    ])
