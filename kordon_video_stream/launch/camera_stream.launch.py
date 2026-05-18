from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_topic_arg = DeclareLaunchArgument(
        "input_topic", default_value="/kordon001/camera/image_raw"
    )
    rtsp_url_arg = DeclareLaunchArgument(
        "rtsp_url", default_value="rtsp://127.0.0.1:8554/kordon001"
    )
    bitrate_arg = DeclareLaunchArgument("bitrate_kbps", default_value="2500")
    keyint_arg = DeclareLaunchArgument("keyint", default_value="30")
    fps_arg = DeclareLaunchArgument("fps", default_value="30.0")

    node = Node(
        package="kordon_video_stream",
        executable="camera_rtsp_publisher_node",
        name="camera_rtsp_publisher",
        output="screen",
        parameters=[
            {
                "input_topic": LaunchConfiguration("input_topic"),
                "rtsp_url": LaunchConfiguration("rtsp_url"),
                "bitrate_kbps": LaunchConfiguration("bitrate_kbps"),
                "keyint": LaunchConfiguration("keyint"),
                "fps": LaunchConfiguration("fps"),
            }
        ],
    )

    return LaunchDescription([
        input_topic_arg,
        rtsp_url_arg,
        bitrate_arg,
        keyint_arg,
        fps_arg,
        node,
    ])
