from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import LifecycleNode, Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import matches_action

from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition

from lifecycle_msgs.msg import Transition


def generate_launch_description():
    enable_video_streaming = LaunchConfiguration("enable_video_streaming")
    mediamtx_host = LaunchConfiguration("mediamtx_host")

    bringup_pkg = FindPackageShare("kordon_bringup")
    control_pkg = FindPackageShare("kordon_control")
    description_pkg = FindPackageShare("kordon_description")
    ros_gz_sim_pkg = FindPackageShare("ros_gz_sim")

    controllers001_config = PathJoinSubstitution(
        [control_pkg, "config", "controllers001.yaml"]
    )
    controllers002_config = PathJoinSubstitution(
        [control_pkg, "config", "controllers002.yaml"]
    )

    bridge_config = PathJoinSubstitution(
        [bringup_pkg, "config", "ros_gz_bridge.yaml"]
    )
    gz_world = PathJoinSubstitution(
        [bringup_pkg, "worlds", "world1.sdf"]
    )

    robot_description001 = Command(
        [
            "xacro ",
            PathJoinSubstitution(
                [description_pkg, "urdf", "kordon.xacro"]
            ),
            " use_gz:=true",
            " controllers_file:=",
            controllers001_config,
            " robot_name:=kordon001",
            " prefix:=kordon001/",
            " odom_topic:=/kordon001/ground_truth/odom",
            " scan_topic:=/kordon001/scan",
            " camera_topic:=/kordon001/camera/image_raw",
            " namespace:=/kordon001",
            " gps_topic:=/kordon001/gps/fix",
        ]
    )
    robot_description002 = Command(
        [
            "xacro ",
            PathJoinSubstitution(
                [description_pkg, "urdf", "kordon.xacro"]
            ),
            " use_gz:=true",
            " controllers_file:=",
            controllers002_config,
            " robot_name:=kordon002",
            " prefix:=kordon002/",
            " odom_topic:=/kordon002/ground_truth/odom",
            " scan_topic:=/kordon002/scan",
            " camera_topic:=/kordon002/camera/image_raw",
            " namespace:=/kordon002",
            " gps_topic:=/kordon002/gps/fix",
        ]
    )

    map_to_kordon001_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "map",
            "kordon001/odom",
        ],
    )
    map_to_kordon002_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "1.0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "map",
            "kordon002/odom",
        ],
    )

    rviz2_config = PathJoinSubstitution(
        [description_pkg, "rviz2", "config.rviz"]
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [ros_gz_sim_pkg, "launch", "gz_sim.launch.py"]
                )
            ]
        ),
        launch_arguments={
            "gz_args": ["-r ", gz_world],
            "on_exit_shutdown": "true",
        }.items(),
    )

    robot_state_publisher001 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="kordon001",
        parameters=[
            {"robot_description": robot_description001},
            {"use_sim_time": True},
        ],
    )
    robot_state_publisher002 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="kordon002",
        parameters=[
            {"robot_description": robot_description002},
            {"use_sim_time": True},
        ],
    )

    spawn_kordon001 = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        parameters=[
            {
                "topic": "/kordon001/robot_description",
                "name": "kordon001",
                "allow_renaming": True,
                "z": 0.15,
            }
        ],
    )
    spawn_kordon002 = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        parameters=[
            {
                "topic": "/kordon002/robot_description",
                "name": "kordon002",
                "allow_renaming": True,
                "x": 1.0,
                "z": 0.15,
            }
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="bridge_node",
        name="ros_gz_bridge",
        output="screen",
        parameters=[{"config_file": bridge_config}],
    )

    # kordon001
    joint_state_broadcaster001 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/kordon001/controller_manager",
        ],
    )
    diff_drive_controller001 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--controller-manager",
            "/kordon001/controller_manager",
        ],
    )

    # kordon002
    joint_state_broadcaster002 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/kordon002/controller_manager",
        ],
    )
    diff_drive_controller002 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--controller-manager",
            "/kordon002/controller_manager",
        ],
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz2_config],
        parameters=[{"use_sim_time": True}],
    )

    kordon001_c2_agent = LifecycleNode(
        package="kordon_c2_agent",
        executable="kordon_c2_agent",
        name="kordon001_c2_agent",
        namespace="",
        parameters=[
            {
                "robot_id": "kordon001",
                "odom_topic": "/kordon001/ground_truth/odom",
                "grpc_address": "localhost:50051",
                "telemetry_rate_hz": 10.0,
                "gps_topic": "/kordon001/gps/fix",
                "scan_topic": "/kordon001/scan",
                "geo_goal_topic": "/kordon001/navigation/go_to_geo_point",
            }
        ],
    )

    kordon002_c2_agent = LifecycleNode(
        package="kordon_c2_agent",
        executable="kordon_c2_agent",
        name="kordon002_c2_agent",
        namespace="",
        parameters=[
            {
                "robot_id": "kordon002",
                "odom_topic": "/kordon002/ground_truth/odom",
                "grpc_address": "localhost:50051",
                "telemetry_rate_hz": 10.0,
                "gps_topic": "/kordon002/gps/fix",
                "scan_topic": "/kordon002/scan",
                "geo_goal_topic": "/kordon002/navigation/go_to_geo_point",
            }
        ],
    )

    c2_lifecycle = [
        *lifecycle_autostart(kordon001_c2_agent),
        *lifecycle_autostart(kordon002_c2_agent),
    ]

    camera_stream_kordon001 = Node(
        package="kordon_video_stream",
        executable="camera_rtsp_publisher_node",
        name="kordon001_camera_rtsp_publisher",
        output="screen",
        condition=IfCondition(enable_video_streaming),
        parameters=[
            {
                "input_topic": "/kordon001/camera/image_raw",
                "rtsp_url": ["rtsp://", mediamtx_host, ":8554/kordon001"],
                "bitrate_kbps": 2500,
                "keyint": 30,
                "fps": 30.0,
            }
        ],
    )

    camera_stream_kordon002 = Node(
        package="kordon_video_stream",
        executable="camera_rtsp_publisher_node",
        name="kordon002_camera_rtsp_publisher",
        output="screen",
        condition=IfCondition(enable_video_streaming),
        parameters=[
            {
                "input_topic": "/kordon002/camera/image_raw",
                "rtsp_url": ["rtsp://", mediamtx_host, ":8554/kordon002"],
                "bitrate_kbps": 2500,
                "keyint": 30,
                "fps": 30.0,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_video_streaming",
                default_value="true",
                description="Start camera->H264->RTSP publishers for both robots",
            ),
            DeclareLaunchArgument(
                "mediamtx_host",
                default_value="127.0.0.1",
                description="MediaMTX host/IP for RTSP ingest",
            ),
            gz_sim,
            robot_state_publisher001,
            robot_state_publisher002,
            spawn_kordon001,
            spawn_kordon002,
            map_to_kordon001_odom,
            map_to_kordon002_odom,
            bridge,
            joint_state_broadcaster001,
            joint_state_broadcaster002,
            diff_drive_controller001,
            diff_drive_controller002,
            rviz2,
            kordon001_c2_agent,
            kordon002_c2_agent,
            camera_stream_kordon001,
            camera_stream_kordon002,
            *c2_lifecycle,
        ]
    )


def lifecycle_autostart(node):
    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
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
