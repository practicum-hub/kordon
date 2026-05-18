from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    TimerAction,
)
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    namespaced_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites={},
        convert_types=True,
    )
    replaced_params = ReplaceString(
        source_file=namespaced_params,
        replacements={
            "<robot_namespace>": namespace,
        },
    )
    configured_params = ParameterFile(
        replaced_params,
        allow_substs=True,
    )

    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    geo_point_controller = LifecycleNode(
        package="kordon_navigation",
        executable="geo_point_controller",
        name="geo_point_controller",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_id": namespace,
                "goal_topic": [
                    "/",
                    namespace,
                    "/navigation/go_to_geo_point",
                ],
                "gps_topic": ["/", namespace, "/gps/fix"],
                "odom_topic": [
                    "/",
                    namespace,
                    "/diff_drive_controller/odom",
                ],
                "cmd_vel_topic": [
                    "/",
                    namespace,
                    "/diff_drive_controller/cmd_vel",
                ],
                "nav2_action_name": [
                    "/",
                    namespace,
                    "/navigate_to_pose",
                ],
                "nav2_goal_frame": [namespace, "/odom"],
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace"),
            DeclareLaunchArgument("params_file"),
            DeclareLaunchArgument(
                "use_sim_time", default_value="true"
            ),
            DeclareLaunchArgument("autostart", default_value="true"),
            Node(
                package="nav2_controller",
                executable="controller_server",
                namespace=namespace,
                output="screen",
                parameters=[
                    configured_params,
                    {"use_sim_time": use_sim_time},
                ],
                remappings=[
                    ("cmd_vel", "cmd_vel_nav"),
                ],
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                namespace=namespace,
                output="screen",
                parameters=[
                    configured_params,
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                namespace=namespace,
                output="screen",
                parameters=[
                    configured_params,
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                namespace=namespace,
                output="screen",
                parameters=[
                    configured_params,
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                namespace=namespace,
                output="screen",
                parameters=[
                    configured_params,
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                namespace=namespace,
                output="screen",
                parameters=[
                    configured_params,
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                namespace=namespace,
                output="screen",
                parameters=[
                    configured_params,
                    {"use_sim_time": use_sim_time},
                ],
                remappings=[
                    ("cmd_vel", "cmd_vel_nav"),
                    (
                        "cmd_vel_smoothed",
                        "diff_drive_controller/cmd_vel",
                    ),
                ],
            ),
            geo_point_controller,
            *lifecycle_autostart(geo_point_controller),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                namespace=namespace,
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": lifecycle_nodes,
                    }
                ],
            ),
        ]
    )


def lifecycle_autostart(node):
    configure = TimerAction(
        period=2.0,
        actions=[
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=matches_action(node),
                    transition_id=Transition.TRANSITION_CONFIGURE,
                )
            )
        ],
    )

    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    return [configure, activate]
