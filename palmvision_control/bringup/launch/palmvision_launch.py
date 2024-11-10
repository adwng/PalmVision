from launch import LaunchDescription
from launch.actions import RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("palmvision_control"), "urdf", "palmrobot_core.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("palmvision_control"),
            "config",
            "palmvision_controllers.yaml",
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("palmvision_control"), "config", "display.rviz"]
    )
 
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["palmvision_base_controller", "--controller-manager", "/controller_manager"],
    )

    servo_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["servo_base_controller", "--controller-manager", "/controller_manager"]
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    # Delay start of servo controller after "robot controller"
    delay_servo_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[servo_controller_spawner],
        )
    )

    joystick_launch_file = PathJoinSubstitution(
        [
            FindPackageShare("palmvision_control"),
            "launch",
            "joystick.launch.py",
        ]
    )

    twist_mux_config = PathJoinSubstitution(
        [
            FindPackageShare("palmvision_control"),
            "config",
            "twist_mux.yaml"
        ]
    )

    lidar_launch_file = PathJoinSubstitution(
        [
            FindPackageShare("ydlidar_ros2_driver"),
            "launch",
            "ydlidar_launch.py",
        ]
    )

    camera_launch_file = PathJoinSubstitution(
        [
            FindPackageShare("palmvision_control"),
            "launch",
            "camera.launch.py"
        ]
    )

    launch_joystick = IncludeLaunchDescription(joystick_launch_file)

    launch_lidar = IncludeLaunchDescription(lidar_launch_file)

    launch_camera = IncludeLaunchDescription(camera_launch_file)

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_config, {'use_sim_time' : True}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    nodes = [
        launch_joystick,
        twist_mux,
        launch_lidar,
        launch_camera,
        robot_state_pub_node,  # Publish robot description first
        control_node,  # Then start controller manager
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_servo_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(nodes)
