import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_description = 'd1_description'
    pkg_bringup = 'd1_bringup'

    # 1. 获取资源路径
    urdf_file = os.path.join(
        get_package_share_directory(pkg_description), 'urdf', 'd1.urdf'
    )
    rviz_config_path = os.path.join(
        get_package_share_directory(pkg_bringup), 'rviz', 'odom.rviz'
    )

    # 读取 URDF
    with open(urdf_file, 'r') as inf:
        robot_desc = inf.read()

    # 2. 状态发布节点 (发布 TF: base_link -> visual)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 3. 启动底层驱动
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg_bringup), 'launch', 'd1_bringup.launch.py')
        )
    )

    # 4. 启动简易导航节点
    navigator_node = Node(
        package='d1_bringup',
        executable='simple_navigator',
        name='simple_navigator',
        output='screen'
    )

    # 5. RViz2 节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        rsp_node,
        driver_launch,
        navigator_node, 
        rviz_node
    ])