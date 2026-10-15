#!/usr/bin/env python3
"""
假装上位机：让机器人沿一个正方形走位（前→右→后→左），每个方向 1s、0.3 m/s，
走回原点后循环。

预期效果（对应 robot 节点的仲裁规则）：
  - 遥控不在（无帧 / failsafe）或 SA = HIGH → 上位机模式，本程序生效，车按正方形走；
  - 遥控健康且 SA = LOW → 遥控器模式，本程序被忽略，改听遥控器摇杆；
  - Ctrl+C 停掉本程序后，/cmd_vel 停发，超过 pc_alive_timeout（默认 0.5s）上位机模式
    判为失联，仲裁发零，车停（不会自动切回遥控器）。

方向约定（REP103：x 前 / y 左）：
  - linear.x 正 = 前，负 = 后
  - linear.y 正 = 左横移，负 = 右横移
  如果你的车左右横移方向反了，把 SEGMENTS 里第二、第四段的 y 符号对调即可。

注意：robot 节点的 pc_subscription 是 QoS(10).best_effort()，这里也必须 best_effort。
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data  # best_effort + keep_last(5)

from geometry_msgs.msg import Twist

SEGMENT_DURATION = 1.0   # 每段时长 s
SPEED = 0.3              # 每段速度 m/s

# (linear.x, linear.y)，依次：前、右、后、左
SEGMENTS = [
    (SPEED, 0.0),     # 前进
    (0.0, -SPEED),    # 右横移（y 负 = 右）
    (-SPEED, 0.0),    # 后退
    (0.0, SPEED),     # 左横移（y 正 = 左）
]
SEGMENT_NAMES = ["前进", "右横移", "后退", "左横移"]


class FakePc(Node):
    def __init__(self):
        super().__init__("fake_pc")
        # best_effort，和 robot 的 pc_subscription 匹配
        self.pub = self.create_publisher(Twist, "/cmd_vel", qos_profile_sensor_data)
        self.timer = self.create_timer(0.05, self.publish)  # 20Hz，远小于 0.5s 超时
        self.start_time = self.get_clock().now()
        self.last_seg = -1
        self.get_logger().info(
            "fake_pc 已启动：正方形走位（前→右→后→左，各 1s @0.3m/s）。"
            "遥控不在或 SA=HIGH 时生效，SA=LOW 时听遥控器；Ctrl+C 停发后车停。")

    def publish(self):
        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        seg = int(t // SEGMENT_DURATION) % len(SEGMENTS)
        vx, vy = SEGMENTS[seg]

        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        self.pub.publish(msg)

        if seg != self.last_seg:
            self.last_seg = seg
            self.get_logger().info(f"段 {seg}: {SEGMENT_NAMES[seg]} (x={vx:.1f}, y={vy:.1f})")


def main():
    rclpy.init()
    node = FakePc()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
