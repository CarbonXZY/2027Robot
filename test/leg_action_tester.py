#!/usr/bin/env python3
"""
测试腿的 Lift action：每隔 5 秒在 kInit（收回）和 kLift_All（全抬）之间来回切。

流程：等 /leg/lift 的 action server 起来 → 发「全抬」→ 等到 result → 隔 5s →
      发「收回」→ 等到 result → 隔 5s → 循环。

用法（先 source 工作空间，且 leg 控制器已在跑）：
    source install/setup.bash
    python3 test/leg_action_tester.py
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from leg_msgs.action import Lift

# 对应 leg_system.hpp 里的 LegStatus 枚举
INIT = 0
LIFT_FRONT = 1
LIFT_BACK = 2
LIFT_ALL = 3

INTERVAL = 5.0  # 两次动作之间的间隔（秒）


class LegActionTester(Node):
    def __init__(self):
        super().__init__("leg_action_tester")
        self.client = ActionClient(self, Lift, "/leg/lift")
        self.toggle = False            # False=下一条发 LIFT_ALL，True=下一条发 INIT
        self._delay_timer = None
        # 先等 action server 起来
        self._wait_timer = self.create_timer(1.0, self._wait_for_server)

    def _wait_for_server(self):
        if not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("等待 /leg/lift 的 action server ...")
            return
        self._wait_timer.cancel()
        self.get_logger().info(f"action server 就绪，开始循环：全抬 ↔ 收回，间隔 {INTERVAL}s")
        self.send_next()

    def send_next(self):
        status = INIT if self.toggle else LIFT_ALL
        goal = Lift.Goal()
        goal.status = status
        self.get_logger().info(f"发送 goal: status={status}")
        self.client.send_goal_async(goal).add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("goal 被拒绝（可能忙），稍后重试")
            self._schedule_next()
            return
        self.get_logger().info("goal 已接受，等待 result ...")
        goal_handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result().result
        self.get_logger().info(f"收到 result: success={result.success}, message='{result.message}'")
        self._schedule_next()

    def _schedule_next(self):
        self.toggle = not self.toggle
        self._delay_timer = self.create_timer(INTERVAL, self._on_delay)

    def _on_delay(self):
        self._delay_timer.cancel()
        self.send_next()


def main():
    rclpy.init()
    node = LegActionTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
