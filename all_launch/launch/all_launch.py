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
    microcont_pkg_share = get_package_share_directory('microcontroller_interface')

    nav2_params_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    robot_model_path = os.path.join(pkg_share, 'urdf', 'lidar.urdf')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    serial_port_lidar = LaunchConfiguration('serial_port_lidar', default='/dev/ttyUSB0')
    serial_port_arduino = LaunchConfiguration('serial_port_arduino', default='/dev/ttyUSB1') 
    robot_model_file = LaunchConfiguration('robot_model_file')
    
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
    declare_serial_port_lidar = DeclareLaunchArgument(
        name='serial_port_lidar',
        default_value=serial_port_lidar,
        description='Specifying usb port to connected lidar'
    )
    declare_serial_port_arduino = DeclareLaunchArgument(
        name='serial_port_arduino',
        default_value=serial_port_arduino,
        description='Specifying usb port to connected arduino'
    )
    declare_robot_model_file = DeclareLaunchArgument(
        name='robot_model_file',
        default_value=robot_model_path,
        description='Path to URDF for publishing static transform for LiDAR and IMU'
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
            'serial_port': serial_port_lidar
        }.items()
    )

    # launch odometry and motor control
    real_odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(microcont_pkg_share, 'launch', 'odom_and_motor_control.launch.py')),
        launch_arguments={
            'serial_port': serial_port_arduino
        }.items()
    )

    # launch robot state publisher
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'robot_model_file': robot_model_file
        }.items()
    )

    return LaunchDescription([
        delcare_use_sim_time,
        declare_params_file,
        declare_autostart,
        declare_serial_port_lidar,
        declare_serial_port_arduino,
        declare_robot_model_file,
        
        robot_state_publisher,
        real_odom_launch,
        rplidar_launch,
        nav2_launch
    ])
