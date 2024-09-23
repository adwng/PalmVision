import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    
    description_prefix = get_package_share_directory('description')

    gazebo_launch_file = os.path.join(description_prefix, 'launch', 'gazebo.launch.py')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file)
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont']
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad']
    )

    return LaunchDescription(
        [gazebo, diff_drive_spawner, joint_broad_spawner,]
    )