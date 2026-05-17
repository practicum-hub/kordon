from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    description_pkg = FindPackageShare("kordon_description")
    control_pkg = FindPackageShare("kordon_control")

    robot_description = Command([
        "xacro",
        " ",
        PathJoinSubstitution([
            description_pkg, "urdf", "kordon.xacro"
        ]),
        " ",
        "use_gz:=false"
    ])

    rviz2_config = PathJoinSubstitution([
        description_pkg,
        "rviz2",
        "config.rviz"
    ])

    controllers_config = PathJoinSubstitution([
        control_pkg,
        "config",
        "controllers.yaml"
    ])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d", rviz2_config
        ]
    )

    # ros2_control
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            controllers_config
        ]
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"]
    )

    diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"]
    )

    return LaunchDescription([
        robot_state_publisher,
        rviz2,
        controller_manager,
        joint_state_broadcaster,
        diff_drive_controller
    ])
