#!/usr/bin/env python3
import os
import sys
import platform
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

# ROS 消息类型
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from std_msgs.msg import String
from nav_msgs.msg import Odometry

# TF 变换广播器
from tf2_ros import TransformBroadcaster

# 资源查找
from ament_index_python.packages import get_package_share_directory

class D1DriverNode(Node):
    def __init__(self):
        super().__init__('d1_core')
        
        # ---------------------------------------------------------
        # 1. 加载 SDK 库路径
        # ---------------------------------------------------------
        self.load_sdk_library()
        
        # 延迟导入 SDK (必须在 sys.path 设置之后)
        try:
            import mc_sdk_zsl_1_py
            self.sdk_module = mc_sdk_zsl_1_py
            self.get_logger().info('SDK 模块导入成功')
        except ImportError as e:
            self.get_logger().fatal(f'无法导入 mc_sdk_zsl_1_py: {e}')
            self.get_logger().fatal('请检查 setup.py 是否正确安装了 libraries 目录')
            sys.exit(1)

        # ---------------------------------------------------------
        # 2. 声明并获取参数 (从 YAML/Launch 加载)
        # ---------------------------------------------------------
        self.declare_parameter('client_ip', '127.0.0.1')
        self.declare_parameter('communication_port', 43988)
        self.declare_parameter('robot_ip', '127.0.0.1')

        self.client_ip = self.get_parameter('client_ip').value
        self.port = self.get_parameter('communication_port').value
        self.robot_ip = self.get_parameter('robot_ip').value

        self.get_logger().warn(f'配置参数: Client={self.client_ip}, Port={self.port}, Robot={self.robot_ip}')

        # ---------------------------------------------------------
        # 3. 初始化机器人连接
        # ---------------------------------------------------------
        self.app = self.sdk_module.HighLevel()
        self.connect_robot()

        # 等待连接稳定
        time.sleep(3)
            
        self.get_logger().info('硬件连接确认成功！')

        # --- 起立并等待 ---
        self.get_logger().info('发送起立指令 (StandUp)...')
        try:
            self.app.standUp()
        except Exception as e:
            self.get_logger().error(f'发送起立指令失败: {e}')
        
        self.get_logger().info('等待姿态稳定 (3秒)...')
        time.sleep(3)
        
        self.get_logger().info('机器狗已就绪，正在启动 ROS 通信接口...')

        # ---------------------------------------------------------
        # 看门狗相关变量
        # ---------------------------------------------------------
        # 记录最后一次收到 cmd_vel 的时间戳
        self.last_cmd_time = self.get_clock().now()
        # 标记是否处于运动控制活跃状态（防止超时后重复发送停止指令刷屏）
        self.cmd_active = False
        # 超时阈值（秒）
        self.cmd_timeout_sec = 1.0

        # ---------------------------------------------------------
        # 4. 创建发布者、订阅者、定时器
        # ---------------------------------------------------------
        
        # [订阅] 速度控制 cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist, 
            'cmd_vel', 
            self.cmd_vel_callback, 
            10
        )

        # [订阅] 字符串指令 d1_cmd
        self.d1_cmd_sub = self.create_subscription(
            String,
            'd1_cmd',
            self.d1_cmd_callback,
            10
        )

        # [发布] 里程计 odom
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # [广播] TF 变换 (odom -> base_link)
        self.tf_broadcaster = TransformBroadcaster(self)

        # [定时器] 20Hz (0.05s) 读取状态并发布里程计
        self.create_timer(0.05, self.timer_callback)

        # [定时器] 10Hz (0.1s) 看门狗检测
        self.create_timer(0.1, self.watchdog_timer_callback)

    def load_sdk_library(self):
        """动态计算库路径并加入 sys.path"""
        arch = platform.machine().replace('amd64', 'x86_64').replace('arm64', 'aarch64')
        self.get_logger().info(f"检测到系统架构: {arch}")

        try:
            package_share_dir = get_package_share_directory('d1_bringup')
            # 假设库文件被安装在 share/d1_bringup/libraries/zsl-1/{arch}
            lib_path = os.path.join(package_share_dir, 'libraries', 'zsl-1', arch)
            
            if not os.path.exists(lib_path):
                self.get_logger().error(f"库路径不存在: {lib_path}")
            else:
                self.get_logger().info(f"添加库路径到 sys.path: {lib_path}")
                sys.path.insert(0, lib_path)
                
        except Exception as e:
            self.get_logger().error(f"获取包路径失败: {e}")

    def connect_robot(self):
        try:
            self.app.initRobot(self.client_ip, self.port, self.robot_ip)
            self.get_logger().info("机器人连接初始化成功")
        except Exception as e:
            self.get_logger().error(f"机器人连接失败: {e}")

    # ---------------------------------------------------------
    # 业务回调函数
    # ---------------------------------------------------------

    def cmd_vel_callback(self, msg: Twist):
        """
        处理 cmd_vel 速度消息
        """
        # 收到指令，刷新看门狗时间
        self.last_cmd_time = self.get_clock().now()
        self.cmd_active = True

        vx = msg.linear.x
        vy = msg.linear.y
        vyaw = msg.angular.z
        
        try:
            self.app.move(vx, vy, vyaw)
        except Exception as e:
            self.get_logger().warn(f"发送运动指令失败: {e}")

    def watchdog_timer_callback(self):
        """
        看门狗定时器：检测是否超时
        """
        # 如果没有激活（还没收到过指令或已经超时处理过），则直接返回
        if not self.cmd_active:
            return

        now = self.get_clock().now()
        # 计算时间差 (纳秒转秒)
        dt = (now - self.last_cmd_time).nanoseconds / 1e9

        if dt > self.cmd_timeout_sec:
            self.get_logger().warn(f"看门狗超时 ({dt:.2f}s > {self.cmd_timeout_sec}s): cmd_vel 数据中断，紧急停车！")
            try:
                # 发送停止指令
                self.app.move(0.0, 0.0, 0.0)
            except Exception as e:
                self.get_logger().error(f"看门狗停车失败: {e}")
            finally:
                # 重置标志位，避免重复发送停止指令和刷屏日志
                self.cmd_active = False

    def d1_cmd_callback(self, msg: String):
        """
        处理 d1_cmd 字符串指令
        """
        cmd = msg.data.lower()
        self.get_logger().info(f"收到指令: {cmd}")
        
        try:
            if cmd == "standup":
                self.app.standUp()
            elif cmd == "liedown":
                self.app.lieDown()
            else:
                self.get_logger().warn(f"未知指令: {cmd}")
        except Exception as e:
            self.get_logger().error(f"执行指令 {cmd} 失败: {e}")

    def timer_callback(self):
        """
        定时读取传感器并发布里程计
        """
        try:
            # 1. 获取位置 (x, y, z)
            pos = self.app.getPosition() 
            if not pos: return # 保护检查
            x, y, z = pos[0], pos[1], pos[2]

            # 2. 获取姿态 (getRPY - Roll, Pitch, Yaw)
            rpy = self.app.getRPY()
            if not rpy: return # 保护检查
            pitch, roll, yaw = rpy[0], rpy[1], rpy[2]

            # 3. 构建当前时间戳
            current_time = self.get_clock().now().to_msg()

            # 4. 计算四元数 (从欧拉角)
            q = self.euler_to_quaternion(roll, pitch, yaw)

            # 5. 发布 TF (odom -> base_link)
            t = TransformStamped()
            t.header.stamp = current_time
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'

            t.transform.translation.x = float(x)
            t.transform.translation.y = float(y)
            t.transform.translation.z = float(z)
            t.transform.rotation = q

            self.tf_broadcaster.sendTransform(t)

            # 6. 发布 Odometry 消息
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'

            # 设置位置
            odom.pose.pose.position.x = float(x)
            odom.pose.pose.position.y = float(y)
            odom.pose.pose.position.z = float(z)
            odom.pose.pose.orientation = q

            # 设置速度
            odom.twist.twist.linear.x = 0.0
            odom.twist.twist.angular.z = 0.0

            self.odom_pub.publish(odom)

        except Exception as e:
            # 降低日志级别，防止刷屏
            pass 

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        欧拉角转四元数 helper 函数
        """
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        
        q = Quaternion()
        q.x = qx
        q.y = qy
        q.z = qz
        q.w = qw
        return q

def main(args=None):
    rclpy.init(args=args)
    node = D1DriverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 按下 Ctrl+C 时触发
        pass
    except Exception as e:
        # 其他未捕获异常
        node.get_logger().error(f"运行时发生未捕获异常: {e}")
    finally:
        node.get_logger().info('正在关闭 D1 驱动节点...')
        
        # 尝试发送复位指令
        try:
            node.app.lieDown()
        except Exception as e:
            node.get_logger().warn(f'发送离线指令异常 (这通常可以忽略): {e}')

        # 销毁节点
        try:
            node.destroy_node()
        except Exception:
            pass

        # 关闭 ROS 客户端库
        if rclpy.ok():
            rclpy.shutdown()
        
        # 强制退出 Python 进程
        sys.exit(0)

if __name__ == '__main__':
    main()