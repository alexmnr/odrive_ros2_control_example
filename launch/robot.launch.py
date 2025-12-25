from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathSubstitution, PathJoinSubstitution, FindExecutable

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
    # Get URDF via xacro
    description_package = "odrive_ros2_control_example"
    description_file = "robot.urdf.xacro"
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            PathSubstitution(FindPackageShare("odrive_ros2_control_example"))
            / "config"
            / "robot_controllers.yaml"
        ],
        output="both",
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    odrive_forward_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "odrive_forward_controller",
            "--param-file",
            PathSubstitution(FindPackageShare("odrive_ros2_control_example"))
            / "config"
            / "robot_controllers.yaml",
        ],
    )
    nodes = [
        robot_state_publisher_node,
        controller_manager,
        # joint_state_broadcaster_spawner,
        # odrive_forward_controller_spawner
    ]

    return LaunchDescription(declared_arguments + nodes)
