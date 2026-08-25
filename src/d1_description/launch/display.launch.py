import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_description = 'd1_description'
    pkg_bringup = 'd1_bringup'

    # 1. 获取 URDF 文件路径
    urdf_file = os.path.join(
        get_package_share_directory(pkg_description),
        'urdf',
        'd1.urdf'
    )

    # 读取 URDF 内容
    with open(urdf_file, 'r') as inf:
        robot_desc = inf.read()

    # 2. 配置 Robot State Publisher 节点
    # 这个节点负责把 URDF 里的模型信息发布到 /robot_description 话题
    # 并且处理静态 TF (虽然这里只有一个 link，但也必须要有)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 3. 包含之前的 Teleop Launch 文件
    # 这会启动: d1_core (驱动) 和 rqt_robot_steering (控制工具)
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg_bringup), 'launch', 'teleop.launch.py')
        )
    )

    # 4. 启动 RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', [os.path.join(get_package_share_directory(pkg_description), 'rviz', 'display.rviz')]]
    )

    return LaunchDescription([
        rsp_node,
        # teleop_launch,
        rviz_node
    ])