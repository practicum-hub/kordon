from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def nav2_for_robot(robot_name: str):
    bringup_pkg = FindPackageShare("kordon_bringup")
    navigation_pkg = FindPackageShare("kordon_navigation")

    params_file = PathJoinSubstitution(
        [
            navigation_pkg,
            "config",
            "nav2_params.yaml",
        ]
    )

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    bringup_pkg,
                    "launch",
                    "navigation_minimal.launch.py",
                ]
            )
        ),
        launch_arguments={
            "namespace": robot_name,
            "params_file": params_file,
            "use_sim_time": "true",
            "autostart": "true",
        }.items(),
    )


def generate_launch_description():
    return LaunchDescription(
        [
            nav2_for_robot("kordon001"),
            nav2_for_robot("kordon002"),
        ]
    )
