import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 获取包路径
    pkg_name = 'd1_bringup'
    pkg_share = get_package_share_directory(pkg_name)

    # 2. 定义包含 d1_bringup.launch.py 的动作
    # 这会启动你的 d1_core 驱动节点
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'd1_bringup.launch.py')
        )
    )

    # 3. 定义启动 rqt_robot_steering 的动作
    # 这是一个通用的 ROS 2 GUI 工具，用于发布 cmd_vel
    rqt_node = Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering',
        name='rqt_robot_steering',
        output='screen',
        parameters=[{
            'default_topic': '/cmd_vel',   # 预设话题名称
            'default_vx_max': 0.5,         # 预设最大线速度 (m/s)
            'default_vx_min': -0.5,        # 预设最小线速度
            'default_vw_max': 1.0,         # 预设最大角速度 (rad/s)
            'default_vw_min': -1.0,        # 预设最小角速度
        }]
    )

    return LaunchDescription([
        driver_launch,
        rqt_node
    ])