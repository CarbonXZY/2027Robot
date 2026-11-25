#!/usr/bin/env python3
"""
地块键鼠控制 demo：终端里按 1 发 Pickup，按 2 发 Place，按 q 退出。

用法（先 source 工作空间，并让 earth_block_controller 处于 active）：
    source install/setup.bash
    python3 test/earth_block_keyboard_demo.py

说明：
- 单键触发，不需要回车（Linux termios cbreak 模式）。
- PICKUP：status=1，走 kPick -> kUp。
- PLACE ：status=0，走 kPlace -> kInit。
- 上一条 action 未结束时，新按键会被 demo 忽略；服务端也会用 busy 拒绝。
"""
import sys
import termios
import threading
import tty

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from earth_block_msgs.action import Command

ACTION_NAME = "/earth_block/command"

PLACE = 0
PICKUP = 1
ACTION_NAMES = {PLACE: "PLACE", PICKUP: "PICKUP"}


class EarthBlockKeyboardDemo(Node):
    def __init__(self):
        super().__init__("earth_block_keyboard_demo")
        self.client = ActionClient(self, Command, ACTION_NAME)
        self._lock = threading.Lock()
        self._busy = False

    def wait_for_server(self):
        print(f"等待 action server {ACTION_NAME} ...")
        while rclpy.ok() and not self.client.wait_for_server(timeout_sec=0.5):
            pass
        if rclpy.ok():
            print("action server 就绪")

    def send(self, status):
        with self._lock:
            if self._busy:
                print(f"[跳过] 上一条 action 还没结束，不能发 {ACTION_NAMES[status]}")
                return
            self._busy = True

        goal = Command.Goal()
        goal.status = status
        print(f"发送 {ACTION_NAMES[status]} (status={status})")
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self._goal_response)

    def _goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("[拒绝] goal 被服务端拒绝（可能 busy，或当前状态不允许该动作）")
            with self._lock:
                self._busy = False
            return

        print("[接受] 等待 result ...")
        goal_handle.get_result_async().add_done_callback(self._result)

    def _result(self, future):
        result = future.result().result
        print(f"[结果] success={result.success}, message='{result.message}'")
        with self._lock:
            self._busy = False


def main():
    rclpy.init()
    node = EarthBlockKeyboardDemo()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    node.wait_for_server()

    if not rclpy.ok():
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        return

    print()
    print("地块控制 demo（单键，无需回车）：")
    print("  1 = PICKUP  (Pick + Up)")
    print("  2 = PLACE   (Place + Init)")
    print("  q = 退出")
    print()

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            key = sys.stdin.read(1)
            if key == "1":
                node.send(PICKUP)
            elif key == "2":
                node.send(PLACE)
            elif key in ("q", "Q"):
                print("退出")
                break
    except KeyboardInterrupt:
        print("\nCtrl+C，退出")
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
