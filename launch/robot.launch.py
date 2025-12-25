from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "debug",
            default_value="false",
            description="debug mode"
        )
    )
    # Get file paths
    robot_description_path = PathJoinSubstitution([FindPackageShare("odrive_ros2_control_example"), "urdf", "robot.urdf.xacro"])
    controller_config_path = PathJoinSubstitution([FindPackageShare("odrive_ros2_control_example"), "config", "robot_controllers.yaml"])

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            robot_description_path
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Define Nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        arguments=["--ros-args", "--log-level", "error"]
    )
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config_path],
        output="both",
        arguments=["--ros-args", "--log-level", "info"]
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="both",
        arguments=["joint_state_broadcaster", "--ros-args", "--log-level", "error"]
    )
    odrive_forward_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="both",
        arguments=["odrive_forward_controller", "--param-file", controller_config_path, "--ros-args", "--log-level", "error"]
    )
    nodes = [
        robot_state_publisher_node,
        controller_manager,
        joint_state_broadcaster_spawner,
        odrive_forward_controller_spawner
    ]

    return LaunchDescription(declared_arguments + nodes)
