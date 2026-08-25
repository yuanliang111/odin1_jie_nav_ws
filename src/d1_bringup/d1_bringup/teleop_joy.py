#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class TeleopJoy(Node):
    def __init__(self):
        super().__init__('teleop_joy_node')
        
        # 初始化变量
        self.b_start = False
        self.lx = 0.0
        self.ly = 0.0
        self.ry = 0.0

        # 创建发布者 /cmd_vel
        self.velcmd_pub_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 创建订阅者 joy
        self.sub_ = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        # 在Python中，我们使用定时器来实现相同的逻辑
        timer_period = 1.0 / 30.0  # 30Hz
        self.timer = self.create_timer(timer_period, self.send_vel_cmd)

        self.get_logger().info("TeleopJoy Node Started")

    def joy_callback(self, joy_msg):
        # 注意：这里假设手柄至少有4个轴，为了安全通常可以加 try/except 或者长度判断
        if len(joy_msg.axes) > 3:
            self.lx = joy_msg.axes[1]  # forward & backward
            self.ly = joy_msg.axes[0]  # shift (Left/Right strafe)
            self.ry = joy_msg.axes[3]  # rotation
            
            self.b_start = True

    def send_vel_cmd(self):
        if not self.b_start:
            return

        vel_cmd = Twist()
        vel_cmd.linear.x = self.lx * 0.2
        vel_cmd.linear.y = self.ly * 0.2
        vel_cmd.angular.z = self.ry * 0.5

        self.velcmd_pub_.publish(vel_cmd)

def main(args=None):
    rclpy.init(args=args)
    
    node = TeleopJoy()
    
    try:
        # spin 会让节点保持运行，处理回调（Joy）和定时器（send_vel_cmd）
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()