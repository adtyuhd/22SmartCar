#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist


class SquareDriver(Node):

    def __init__(self):
        super().__init__('square_driver')

        # 使用 Gazebo 发布的 /clock 作为时间源。
        self.set_parameters([
            Parameter('use_sim_time', Parameter.Type.BOOL, True)
        ])

        # 创建 /cmd_vel 发布器。
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # -------------------------
        # 测试运动参数
        # -------------------------

        # 车辆线速度，单位 m/s。
        self.linear_speed = 0.2

        # 先直行 2 秒。
        # 理论距离 = 0.2 * 2.0 = 0.4 m。
        self.straight_duration = 2.0

        # 计划的转弯半径，单位 m。
        self.turn_radius = 0.5

        # 圆周运动关系：
        # angular_speed = linear_speed / turn_radius
        # 0.2 / 0.5 = 0.4 rad/s
        self.angular_speed = (
            self.linear_speed / self.turn_radius
        )

        # 90 度 = pi / 2 rad。
        self.turn_angle = math.pi / 2.0

        # 转弯时间：
        # time = angle / angular_speed
        # 大约 3.927 秒。
        self.turn_duration = (
            self.turn_angle / self.angular_speed
        )

        # -------------------------
        # 状态机
        # -------------------------

        # 刚启动时先不直接进入 STRAIGHT。
        # 因为 use_sim_time=True 后，
        # 在收到第一帧 /clock 之前 ROS 时间可能还是 0。
        self.state = 'WAIT_CLOCK'

        # 现在还没有真正开始任何运动状态，
        # 所以先不记录状态开始时间。
        self.state_start_time = None

        # 每 0.1 秒执行一次，也就是 10 Hz。
        self.timer = self.create_timer(
            0.1,
            self.timer_callback
        )

        self.get_logger().info('Square driver started.')
        self.get_logger().info(
            'Waiting for Gazebo /clock...'
        )

    def publish_cmd(self, linear_x, angular_z):
        """向 /cmd_vel 发布速度命令。"""

        msg = Twist()

        msg.linear.x = linear_x
        msg.angular.z = angular_z

        self.cmd_pub.publish(msg)

    def change_state(self, new_state):
        """切换状态，并重新记录该状态的开始时间。"""

        self.state = new_state
        self.state_start_time = self.get_clock().now()

        self.get_logger().info(
            f'State: {self.state}'
        )

    def timer_callback(self):
        """根据当前状态决定车辆应该做什么。"""

        now = self.get_clock().now()

        # -------------------------
        # 状态 0：等待 Gazebo 时钟
        # -------------------------
        if self.state == 'WAIT_CLOCK':

            # 仿真时间还没有开始时，不让车运动。
            self.publish_cmd(
                0.0,
                0.0
            )

            # nanoseconds > 0 表示已经收到有效的仿真时间。
            if now.nanoseconds > 0:
                self.change_state('STRAIGHT')

            return

        # 从这里开始，state_start_time 一定已经设置。
        elapsed = (
            now - self.state_start_time
        ).nanoseconds / 1e9

        # -------------------------
        # 状态 1：直行
        # -------------------------
        if self.state == 'STRAIGHT':

            self.publish_cmd(
                self.linear_speed,
                0.0
            )

            if elapsed >= self.straight_duration:
                self.change_state('TURN')

        # -------------------------
        # 状态 2：左转
        # -------------------------
        elif self.state == 'TURN':

            self.publish_cmd(
                self.linear_speed,
                self.angular_speed
            )

            if elapsed >= self.turn_duration:
                self.change_state('STOP')

        # -------------------------
        # 状态 3：停车
        # -------------------------
        elif self.state == 'STOP':

            self.publish_cmd(
                0.0,
                0.0
            )


def main(args=None):
    rclpy.init(args=args)

    node = SquareDriver()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        # 程序退出前再发送一次停车命令。
        node.publish_cmd(0.0, 0.0)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
