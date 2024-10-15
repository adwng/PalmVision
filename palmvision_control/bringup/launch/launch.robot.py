from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    prefix = get_package_share_directory('palmvision_control')

    joystick_launch_file = os.path.join(prefix, 'launch', 'joystick.launch.py')

    launch_joy_stick = IncludeLaunchDescription(joystick_launch_file)

    palmvision_robot_hardware_file = os.path.join(prefix, 'launch', 'palmvision_launch.py')

    launch_palmvision_robot = IncludeLaunchDescription(palmvision_robot_hardware_file)


    return LaunchDescription([
        launch_joy_stick,  
        launch_palmvision_robot,
    ])