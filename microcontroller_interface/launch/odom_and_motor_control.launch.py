import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    pkg_share = get_package_share_directory('microcontroller_interface')
    ekf_config_file = os.path.join(pkg_share, 'config', 'ekf.yaml')

    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    kp = LaunchConfiguration('kp', default='0.1')
    kd = LaunchConfiguration('kd', default='0.0')
    ki = LaunchConfiguration('ki', default='0.0')
    
    declare_serial_port = DeclareLaunchArgument(
        name='serial_port',
        default_value=serial_port,
        description='Specifying usb port to connected Arduino'
    )
    declare_kp = DeclareLaunchArgument(
        name='kp',
        default_value=kp,
        description='Constant for proportion term in PID control'
    )
    declare_kd = DeclareLaunchArgument(
        name='kd',
        default_value=kd,
        description='Constant for derivative term in PID control'
    )
    declare_ki = DeclareLaunchArgument(
        name='ki',
        default_value=ki,
        description='Constant for integral term in PID control'
    )

    odom_imu_node = Node(
        package='microcontroller_interface',
        executable='odom_imu_node',
        name='odom_imu_node',
        output='screen',
        parameters=[
            {'serial_port': serial_port}
        ]
    )
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file]
    )
    motor_control_node = Node(
        package='microcontroller_interface',
        executable='pid_motor_control',
        name='motor_control_node',
        output='screen',
        parameters=[
            {'kp': kp},
            {'kd': kd},
            {'ki': ki}
        ]
    )

    return LaunchDescription([
        declare_serial_port,
        declare_kp,
        declare_kd,
        declare_ki,

        odom_imu_node,
        robot_localization_node,
        motor_control_node
    ])
