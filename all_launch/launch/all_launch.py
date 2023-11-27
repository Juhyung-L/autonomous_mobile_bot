import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    pkg_share = get_package_share_directory('all_launch')
    rplidar_pkg_share = get_package_share_directory('rplidar_ros')
    # microcont_pkg_share = get_package_share_directory('microcontroller_interface')

    nav2_params_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')

    delcare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_params_file = DeclareLaunchArgument(
        name='params_file',
        default_value=nav2_params_path,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )
    declare_autostart = DeclareLaunchArgument(
        name='autostart',
        default_value='false',
        description='Automatically startup the nav2 stack'
    )
    declare_serial_port = DeclareLaunchArgument(
        name='serial_port',
        default_value=serial_port,
        description='Specifying usb port to connected lidar'
    )

    # launch nav2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart
        }.items()
    )

    # launch rplidar
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rplidar_pkg_share, 'launch', 'rplidar_a1_launch.py')),
        launch_arguments={
            'serial_port': serial_port
        }.items()
    )

    # launch odometry and motor control
    # real_odom_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(microcont_pkg_share, 'launch', 'odom_and_motor_control.launch.py'))
    # )

    return LaunchDescription([
        delcare_use_sim_time,
        declare_params_file,
        declare_autostart,
        declare_serial_port,

        nav2_launch,
        rplidar_launch,
        # real_odom_launch
    ])