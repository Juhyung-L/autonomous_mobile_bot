import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('mobile_bot')
    sdf_file = os.path.join(pkg_share, 'models', 'model.sdf')

    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    declare_x_position = DeclareLaunchArgument(
        name='x_pose', 
        default_value='0.0',
        description='Starting x position of the robot'
    )
    declare_y_position = DeclareLaunchArgument(
        name='y_pose', 
        default_value='0.0',
        description='Starting y position of the robot'
    )

    start_gazebo_ros_spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'mobile_bot',
            '-file', sdf_file,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'
        ],
        output='screen',
    )

    return LaunchDescription([
        declare_x_position,
        declare_y_position,
        start_gazebo_ros_spawner
    ])