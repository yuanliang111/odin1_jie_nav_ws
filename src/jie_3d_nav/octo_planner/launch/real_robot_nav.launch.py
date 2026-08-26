import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    octo_planner_share = get_package_share_directory("octo_planner")
    genisom_bridge_share = get_package_share_directory("genisom_bridge")

    actuation_capable_arg = DeclareLaunchArgument(
        "actuation_capable",
        default_value="false",
        description="Allow explicit SDK-actuation enable requests after startup",
    )
    web_http_port_arg = DeclareLaunchArgument(
        "web_http_port",
        default_value="8088",
        description="HTTP port for the Web navigation interface",
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(octo_planner_share, "launch", "nav.launch.py")
        ),
        launch_arguments={
            "launch_rviz": "false",
            "launch_map_gui": "false",
            "launch_d1_core": "false",
            "launch_controller": "false",
            "publish_d1_odom": "false",
            "use_static_odom_to_base": "false",
            "launch_planner": "true",
            "launch_web": "true",
            "launch_rosbridge": "false",
            "web_http_port": LaunchConfiguration("web_http_port"),
        }.items(),
    )
    runtime_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(genisom_bridge_share, "launch", "real_nav_runtime.launch.py")
        ),
        launch_arguments={
            "actuation_capable": LaunchConfiguration("actuation_capable"),
        }.items(),
    )

    return LaunchDescription(
        [
            actuation_capable_arg,
            web_http_port_arg,
            nav_launch,
            runtime_launch,
        ]
    )
