import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    runtime_params = os.path.join(
        get_package_share_directory("genisom_bridge"),
        "config",
        "real_nav_runtime.yaml",
    )
    actuation_capable_arg = DeclareLaunchArgument(
        "actuation_capable",
        default_value="false",
        description="Allow explicit SDK-actuation enable requests after startup",
    )
    max_forward_normalized_arg = DeclareLaunchArgument(
        "max_forward_normalized",
        default_value="0.05",
        description="Maximum normalized GENISOM forward joystick command",
    )
    max_yaw_normalized_arg = DeclareLaunchArgument(
        "max_yaw_normalized",
        default_value="0.50",
        description="Maximum normalized GENISOM yaw joystick command",
    )

    genisom_state_node = Node(
        package="genisom_bridge",
        executable="genisom_state_node",
        name="genisom_state_node",
        output="screen",
        parameters=[
            runtime_params,
            {
                "actuation_capable": ParameterValue(
                    LaunchConfiguration("actuation_capable"), value_type=bool
                ),
                "max_forward_normalized": ParameterValue(
                    LaunchConfiguration("max_forward_normalized"), value_type=float
                ),
                "max_yaw_normalized": ParameterValue(
                    LaunchConfiguration("max_yaw_normalized"), value_type=float
                ),
            },
        ],
    )
    jie_dog_controller_node = Node(
        package="jie_dog_controller",
        executable="jie_dog_controller_node",
        name="jie_dog_controller",
        output="screen",
        parameters=[runtime_params],
    )

    return LaunchDescription(
        [
            actuation_capable_arg,
            max_forward_normalized_arg,
            max_yaw_normalized_arg,
            genisom_state_node,
            jie_dog_controller_node,
        ]
    )
