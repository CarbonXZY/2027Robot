#!/usr/bin/env python3
"""
演示底盘里程计自结算：订阅 mecanum_drive_controller 自动发布的里程计话题，
打印位姿（x/y/yaw）和线速度。不用发任何速度指令，只要控制器 active 就会持续出数。

话题：/mecanum_drive_controller/odometry  （nav_msgs/msg/Odometry）
默认以 10Hz（ros2_control_node update_rate）发布。

用法（先 source 工作空间，且底盘控制器已跑）：
    source install/setup.bash
    python3 test/odom_demo.py [话题名]

验证方式：
  - 车原地不动 → x/y/yaw 全 0（或一直不变）；
  - 给 /mecanum_drive_controller/reference_unstamped 发指令让车走一段 → x/y 跟着累计，yaw 随旋转变化。
"""
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from nav_msgs.msg import Odometry


class OdomDemo(Node):
    def __init__(self, topic: str):
        super().__init__("odom_demo")
        # 官方 controller 的 odom 用默认 reliable QoS，这里也用默认即可
        self.sub = self.create_subscription(Odometry, topic, self.on_odom, 10)
        self.get_logger().info(f"订阅里程计 {topic}，等待数据……")

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        # 四元数转 yaw
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = __import__("math").atan2(siny, cosy)

        v = msg.twist.twist
        self.get_logger().info(
            f"pos=({p.x:+.3f}, {p.y:+.3f}, yaw={yaw:+.3f} rad)  "
            f"vel=(x {v.linear.x:+.3f}, y {v.linear.y:+.3f}, w {v.angular.z:+.3f})")


def main():
    rclpy.init()
    topic = sys.argv[1] if len(sys.argv) > 1 else "/mecanum_drive_controller/odometry"
    node = OdomDemo(topic)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
