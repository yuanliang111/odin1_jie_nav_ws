#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import math
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Twist, PoseStamped
from tf_transformations import euler_from_quaternion # 如果没有这个库，下面有替代函数

# 定义状态枚举
class State:
    IDLE = 0
    TURN_TO_POINT = 1
    GO_TO_POINT = 2
    FINAL_ROTATION = 3

class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('simple_navigator')

        # --- 参数设置 ---
        self.kp_linear = 0.5    # 线速度比例系数
        self.kp_angular = 2.0   # 角速度比例系数
        self.max_v = 0.3        # 最大线速度 m/s
        self.max_w = 1.0        # 最大角速度 rad/s
        self.dist_tol = 0.1    # 位置容差 (米)
        self.angle_tol = 0.1   # 角度容差 (弧度)

        # --- 变量初始化 ---
        self.state = State.IDLE
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_yaw = 0.0
        
        # --- 通信接口 ---
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.goal_sub = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_callback,
            10
        )

        # --- TF 监听器 ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- 控制循环定时器 (20Hz) ---
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('简易导航节点已启动，等待 /goal_pose ...')

    def goal_callback(self, msg: PoseStamped):
        """
        接收导航目标点
        """
        try:
            # 1. 坐标系转换
            # Rviz 发出的 goal_pose 通常是 map 坐标系，我们需要将其转换为 odom 坐标系
            # 这样才能和 lookup_transform('odom', 'base_link') 统一
            if msg.header.frame_id != 'odom':
                transform = self.tf_buffer.lookup_transform(
                    'odom',
                    msg.header.frame_id,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=1.0)
                )
                target_pose = tf2_geometry_msgs.do_transform_pose(msg.pose, transform)
            else:
                target_pose = msg.pose

            # 2. 提取目标信息 (X, Y, Yaw)
            self.goal_x = target_pose.position.x
            self.goal_y = target_pose.position.y
            
            # 四元数转欧拉角
            q = target_pose.orientation
            _, _, self.goal_yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)

            self.get_logger().info(f"收到新目标: x={self.goal_x:.2f}, y={self.goal_y:.2f}, yaw={self.goal_yaw:.2f}")
            
            # 3. 重置状态机，开始执行
            self.state = State.TURN_TO_POINT

        except Exception as e:
            self.get_logger().error(f"处理 Goal 失败: {e}")

    def control_loop(self):
        """
        主控制循环
        """
        if self.state == State.IDLE:
            return

        # 1. 获取当前机器人在 odom 下的位姿
        try:
            trans = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
        except Exception as e:
            # 刚启动时可能TF还没准备好
            return

        # 提取当前位姿
        cur_x = trans.transform.translation.x
        cur_y = trans.transform.translation.y
        q = trans.transform.rotation
        _, _, cur_yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)

        # 初始化速度命令
        cmd = Twist()

        # ---------------------------------------------------------
        # 状态机逻辑
        # ---------------------------------------------------------
        
        # 计算相对于目标的距离和角度
        dx = self.goal_x - cur_x
        dy = self.goal_y - cur_y
        dist = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)

        # === 状态 1: 原地旋转朝向目标点 ===
        if self.state == State.TURN_TO_POINT:
            angle_err = self.normalize_angle(angle_to_goal - cur_yaw)
            
            if abs(angle_err) > self.angle_tol:
                # P控制旋转
                cmd.angular.z = self.limit_speed(angle_err * self.kp_angular, self.max_w)
                self.get_logger().debug(f"调整朝向: 误差 {angle_err:.2f}")
            else:
                # 角度对准了，进入下一阶段
                cmd.angular.z = 0.0
                self.state = State.GO_TO_POINT
                self.get_logger().info("朝向对准，开始移动...")

        # === 状态 2: 直线移动到目标点 ===
        elif self.state == State.GO_TO_POINT:
            # 持续更新朝向误差，一边走一边微调方向
            angle_err = self.normalize_angle(angle_to_goal - cur_yaw)
            
            if dist > self.dist_tol:
                # P控制线速度
                cmd.linear.x = self.limit_speed(dist * self.kp_linear, self.max_v)
                # P控制角速度 (修正偏差)
                cmd.angular.z = self.limit_speed(angle_err * self.kp_angular, self.max_w)
                self.get_logger().debug(f"移动中: 距离 {dist:.2f}")
            else:
                # 到达位置
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.state = State.FINAL_ROTATION
                self.get_logger().info("到达位置，开始最终旋转...")

        # === 状态 3: 原地旋转到目标姿态 ===
        elif self.state == State.FINAL_ROTATION:
            angle_err = self.normalize_angle(self.goal_yaw - cur_yaw)
            
            if abs(angle_err) > self.angle_tol:
                cmd.angular.z = self.limit_speed(angle_err * self.kp_angular, self.max_w)
                self.get_logger().debug(f"最终旋转: 误差 {angle_err:.2f}")
            else:
                # 任务全部完成
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.state = State.IDLE
                self.get_logger().info("导航完成！")

        # 发布速度指令
        self.cmd_pub.publish(cmd)

    # --- 辅助函数 ---

    def normalize_angle(self, angle):
        """将角度限制在 [-pi, pi] 之间"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def limit_speed(self, val, max_val):
        """限制最大速度"""
        if val > max_val:
            return max_val
        elif val < -max_val:
            return -max_val
        return val

    def euler_from_quaternion(self, x, y, z, w):
        """
        手动实现四元数转欧拉角 (RPY)，避免依赖 transforms3d
        """
        # Roll (x-axis rotation)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        
        # Pitch (y-axis rotation)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        
        # Yaw (z-axis rotation)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        
        return roll_x, pitch_y, yaw_z

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 退出时停车
        stop_msg = Twist()
        node.cmd_pub.publish(stop_msg)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()