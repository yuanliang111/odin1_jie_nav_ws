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
                )
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
            genisom_state_node,
            jie_dog_controller_node,
        ]
    )
