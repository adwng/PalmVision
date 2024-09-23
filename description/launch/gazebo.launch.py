import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():

    urdf_prefix = get_package_share_directory('description')
    gazebo_prefix = get_package_share_directory('gazebo_ros')

    urdf_file = os.path.join(urdf_prefix, 'launch', 'rsp.launch.py')
    world_file = os.path.join(urdf_prefix, 'worlds', "agriculture.world")

    gazebo_file = os.path.join(gazebo_prefix, 'launch', 'gazebo.launch.py')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(urdf_file)
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_file),
        launch_arguments={'world': world_file}.items()
    )
    
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description', 
            '-entity', 'palmvision',
            '-z', '0.1'
        ],
        output='screen'
    )   

    return LaunchDescription(
        [rsp, gazebo, spawn_entity]
    )