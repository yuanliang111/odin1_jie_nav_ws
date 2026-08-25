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
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'd1_bringup.launch.py')
        )
    )

    # 3. 定义手柄驱动节点
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',  # 添加此行以便在终端看到日志
        parameters=[{
            'device_id': 0,        # ROS2通常使用ID而不是路径，js0对应0
            'deadzone': 0.12,      # 设置死区
            'autorepeat_rate': 20.0 # 设置按键重复频率
        }]
    )

    # 4. 定义手柄控制转换节点
    teleop_node = Node(
        package='d1_bringup',
        executable='teleop_joy',
        name='teleop_joy',
        output='screen'   
    )

    return LaunchDescription([
        driver_launch,
        joy_node,
        teleop_node
    ])