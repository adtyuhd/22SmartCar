#!/usr/bin/env python3
"""
M3-3 阿克曼小车导航命令行工具。

示例：
python3 M3-3/scripts/goto.py --x 1.0 --y 2.0 --yaw 90

注意：
x、y 是 M3-2 地图中的坐标，不是 Gazebo world 坐标。
yaw 的单位是度。
不会硬编码任何导航目标。
"""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


def parse_args():
    parser = argparse.ArgumentParser(
        description="M3-3 Nav2 导航目标发送工具"
    )

    parser.add_argument(
        "--x", type=float, required=True,
        help="地图目标 X，单位米"
    )
    parser.add_argument(
        "--y", type=float, required=True,
        help="地图目标 Y，单位米"
    )
    parser.add_argument(
        "--yaw", type=float, required=True,
        help="目标朝向，单位度"
    )

    return parser.parse_args()


class GoToNode(Node):
    def __init__(self):
        super().__init__("m3_3_goto")

        # 与 Gazebo 和 Nav2 使用同一个仿真时钟
        self.set_parameters([
            Parameter("use_sim_time", value=True)
        ])

        self.client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose"
        )

        self.last_feedback_time = 0.0

    def on_feedback(self, msg):
        now = time.monotonic()

        # 每两秒显示一次，避免终端刷屏
        if now - self.last_feedback_time < 2.0:
            return

        self.last_feedback_time = now

        feedback = msg.feedback

        print(
            f"[导航中] 剩余路径距离："
            f"{feedback.distance_remaining:.2f} m，"
            f"恢复次数：{feedback.number_of_recoveries}",
            flush=True
        )

    def navigate(self, x, y, yaw_deg):
        print("正在等待 Nav2 导航服务器……")

        if not self.client.wait_for_server(timeout_sec=10.0):
            print("[失败] 找不到 /navigate_to_pose")
            return 1

        # 等待仿真时钟开始计时
        for _ in range(50):
            if self.get_clock().now().nanoseconds > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        else:
            print("[失败] 未收到有效的仿真时钟")
            return 1

        goal = NavigateToPose.Goal()
        pose = PoseStamped()

        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        yaw = math.radians(yaw_deg)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        goal.pose = pose

        print(
            f"发送目标：x={x:.2f} m，"
            f"y={y:.2f} m，yaw={yaw_deg:.1f}°"
        )

        goal_future = self.client.send_goal_async(
            goal,
            feedback_callback=self.on_feedback
        )

        rclpy.spin_until_future_complete(self, goal_future)
        handle = goal_future.result()

        if handle is None or not handle.accepted:
            print("[失败] Nav2 拒绝目标")
            return 1

        print("[已接受] Nav2 开始执行导航")
        print("按 Ctrl+C 可以取消导航")

        result_future = handle.get_result_async()

        try:
            rclpy.spin_until_future_complete(
                self, result_future
            )
        except KeyboardInterrupt:
            print("\n正在取消导航……")

            cancel_future = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(
                self, cancel_future,
                timeout_sec=5.0
            )

            print("[已请求取消] 请检查 Gazebo 中小车是否停止")
            return 130

        result = result_future.result()

        if result is None:
            print("[失败] 没有收到导航结果")
            return 1

        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            print("[Nav2 报告成功] 请核对 Gazebo 中的实际位置")
            return 0

        if status == GoalStatus.STATUS_CANCELED:
            print("[已取消] 导航任务结束")
            return 2

        if status == GoalStatus.STATUS_ABORTED:
            print("[失败] Nav2 中止导航")
            return 1

        print(f"[结束] 导航状态码：{status}")
        return 1


def main():
    args = parse_args()

    rclpy.init()
    node = GoToNode()

    try:
        return node.navigate(
            args.x,
            args.y,
            args.yaw
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())