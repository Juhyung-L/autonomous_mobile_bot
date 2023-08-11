import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('mobile_bot')

    size_weight = LaunchConfiguration('size_weight')
    distance_weight = LaunchConfiguration('distance_weight')
    progress_timeout = LaunchConfiguration('progress_timeout')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_size_weight = DeclareLaunchArgument(
        name='size_weight',
        default_value='0.01',
        description='Importance of the number of frontier points when calculating the Frontiers cost'
    )
    delcare_distance_weight = DeclareLaunchArgument(
        name='distance_weight',
        default_value='1.0',
        description='Importance of distance to closest frontier point when calculating the Frontiers cost'
    )
    declare_progress_timeout = DeclareLaunchArgument(
        name='progress_timeout',
        default_value='10',
        description="Time required for a goal to be considered unreachable and be put into the black list"
    )
    delcare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) block if true'
    )

    autonomous_explorer_node = Node(
        package='frontier_explorer',
        executable='start_frontier_explorer',
        name='frontier_explorer_node',
        output='screen',
        parameters=[{
            'size_weight': size_weight,
            'distance_weight': distance_weight,
            'progress_timeout': progress_timeout,
            'use_sim_time': use_sim_time
        }]
    )

    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'robot_bringup.launch.py')
        )
    )

    return LaunchDescription([
        declare_size_weight,
        delcare_distance_weight,
        declare_progress_timeout,
        delcare_use_sim_time,

        autonomous_explorer_node,
        robot_bringup
    ])