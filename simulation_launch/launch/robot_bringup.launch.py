import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    
    pkg_share = get_package_share_directory('simulation_launch')
    
    rviz_config_file_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    ekf_config_file_path = os.path.join(pkg_share, 'config/ekf.yaml')

    pkg_nav2 = get_package_share_directory('nav2_bringup')
    nav2_params_path = os.path.join(pkg_nav2, 'params', 'nav2_params.yaml')

    # launch configuration variables
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    # declare launch arguments
    delcare_rviz_config_file = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=rviz_config_file_path,
        description='Path to RViz config file'
    )
    delcare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) block if true'
    )
    declare_autostart = DeclareLaunchArgument(
        name='autostart', 
        default_value='false',
        description='Automatically startup the nav2 stack'
    )
    declare_params_file = DeclareLaunchArgument(
        name='params_file',
        default_value=nav2_params_path,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    # declare nodes to launch
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )
    robot_localization_node = Node( # needs to use simulation time
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file_path, 
                    {'use_sim_time': use_sim_time}]
    )
    keyboard_teleop_node = Node(
        package='keyboard_teleop',
        executable='start_keyboard_teleop',
        name='keyboard_teleop_node',
        output='screen'
    )

    # launch nav2
    start_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                          'params_file': params_file,
                          'autostart': autostart}.items()
    )

    # launch other launch files
    robot_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'robot_world.launch.py')
        )
    )

    return LaunchDescription([
        delcare_rviz_config_file,
        delcare_use_sim_time,
        declare_autostart,
        declare_params_file,

        robot_world_launch,
        robot_localization_node,
        rviz_node,
        keyboard_teleop_node,

        start_nav2
    ])
