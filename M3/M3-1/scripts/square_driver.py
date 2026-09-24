#!/usr/bin/env python3

import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class SquareDriver(Node):

    def __init__(self, side_length, laps):
        super().__init__('square_driver')

        # 使用 Gazebo 发布的 /clock。
        self.set_parameters([
            Parameter(
                'use_sim_time',
                Parameter.Type.BOOL,
                True
            )
        ])

        # -------------------------
        # Publisher / Subscriber
        # -------------------------

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # 最近一次 /odom 的 yaw。
        self.current_theta = None

        # 当前一次转弯开始时的 yaw。
        self.turn_start_theta = None

        # -------------------------
        # 命令行参数
        # -------------------------

        self.side_length = side_length
        self.total_laps = laps

        # -------------------------
        # 车辆/路径参数
        # -------------------------

        # 线速度，单位 m/s。
        self.linear_speed = 0.2

        # 圆角半径，单位 m。
        self.turn_radius = 0.5

        # 2m 圆角方形的关系：
        #
        # 直线长度 = 边长 - 2 * 圆角半径
        #
        # side=2.0 时：
        # 1.0 = 2.0 - 2*0.5
        self.straight_length = (
            self.side_length
            - 2.0 * self.turn_radius
        )

        if self.straight_length <= 0.0:
            raise ValueError(
                'side length must be greater than '
                f'2 * turn radius = '
                f'{2.0 * self.turn_radius:.3f} m'
            )

        # 直线段理论时间。
        self.straight_duration = (
            self.straight_length
            / self.linear_speed
        )

        # 圆周运动：
        #
        # omega = v / R
        self.angular_speed = (
            self.linear_speed
            / self.turn_radius
        )

        # 每个角转 90 度。
        self.turn_angle = math.pi / 2.0

        # 理论转弯时间。
        self.theoretical_turn_duration = (
            self.turn_angle
            / self.angular_speed
        )

        # 根据前面实验校准的实际转弯时间。
        #
        # 理论约 3.927 s
        # 实验校准约 4.585 s
        self.turn_duration = 4.585

        # 每圈 4 个角。
        self.corners_per_lap = 4

        # -------------------------
        # 实验计数
        # -------------------------

        self.current_lap = 1
        self.completed_corners = 0

        # -------------------------
        # 状态机
        # -------------------------

        # WAIT_CLOCK
        #     ↓
        # STRAIGHT
        #     ↓
        # TURN
        #     ↓
        # STRAIGHT
        #     ↓
        # TURN
        # ...
        #     ↓
        # 第四次 TURN
        #     ↓
        # 如果还有下一圈 -> STRAIGHT
        # 如果全部完成 -> STOP

        self.state = 'WAIT_CLOCK'

        self.state_start_time = None

        # 10 Hz。
        self.timer = self.create_timer(
            0.1,
            self.timer_callback
        )

        self.get_logger().info(
            'Square driver started.'
        )

        self.get_logger().info(
            f'Side length: {self.side_length:.3f} m'
        )

        self.get_logger().info(
            f'Lap count: {self.total_laps}'
        )

        self.get_logger().info(
            f'Straight length: '
            f'{self.straight_length:.3f} m'
        )

        self.get_logger().info(
            f'Straight duration: '
            f'{self.straight_duration:.3f} s'
        )

        self.get_logger().info(
            f'Theoretical turn duration: '
            f'{self.theoretical_turn_duration:.3f} s'
        )

        self.get_logger().info(
            f'Calibrated turn duration: '
            f'{self.turn_duration:.3f} s'
        )

        self.get_logger().info(
            'Waiting for Gazebo /clock...'
        )

    def odom_callback(self, msg):
        """从 /odom 四元数计算当前 yaw。"""

        q = msg.pose.pose.orientation

        x = q.x
        y = q.y
        z = q.z
        w = q.w

        self.current_theta = math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z)
        )

    def publish_cmd(self, linear_x, angular_z):
        """向 /cmd_vel 发布速度命令。"""

        msg = Twist()

        msg.linear.x = linear_x
        msg.angular.z = angular_z

        self.cmd_pub.publish(msg)

    def normalize_angle(self, angle):
        """归一化到 (-pi, pi]。"""

        while angle <= -math.pi:
            angle += 2.0 * math.pi

        while angle > math.pi:
            angle -= 2.0 * math.pi

        return angle

    def change_state(self, new_state):
        """切换状态并重新开始该状态计时。"""

        self.state = new_state
        self.state_start_time = self.get_clock().now()

        self.get_logger().info(
            f'State: {self.state}'
        )

        if new_state == 'TURN':

            if self.current_theta is not None:
                self.turn_start_theta = self.current_theta

                self.get_logger().info(
                    'Turn start theta: '
                    f'{math.degrees(self.turn_start_theta):.2f} deg'
                )

    def finish_turn(self):
        """完成一次 90° 转弯。"""

        self.completed_corners += 1

        if (
            self.turn_start_theta is not None
            and self.current_theta is not None
        ):
            actual_turn = self.normalize_angle(
                self.current_theta
                - self.turn_start_theta
            )

            turn_error = self.normalize_angle(
                actual_turn
                - self.turn_angle
            )

            self.get_logger().info(
                f'Lap {self.current_lap}/{self.total_laps}, '
                f'corner {self.completed_corners}/'
                f'{self.corners_per_lap}'
            )

            self.get_logger().info(
                'Actual turn: '
                f'{math.degrees(actual_turn):.2f} deg'
            )

            self.get_logger().info(
                'Turn error: '
                f'{math.degrees(turn_error):+.2f} deg'
            )

        # 一圈的 4 个弯都完成。
        if self.completed_corners >= self.corners_per_lap:

            self.get_logger().info(
                f'Lap {self.current_lap}/'
                f'{self.total_laps} completed.'
            )

            # 是否还有下一圈？
            if self.current_lap < self.total_laps:

                self.current_lap += 1
                self.completed_corners = 0

                self.get_logger().info(
                    f'Starting lap '
                    f'{self.current_lap}/{self.total_laps}.'
                )

                self.change_state('STRAIGHT')

            else:

                self.change_state('STOP')

                self.get_logger().info(
                    'All requested laps completed.'
                )

        else:
            # 同一圈继续下一条直线。
            self.change_state('STRAIGHT')

    def timer_callback(self):
        """10 Hz 状态机控制。"""

        now = self.get_clock().now()

        # -------------------------
        # WAIT_CLOCK
        # -------------------------

        if self.state == 'WAIT_CLOCK':

            self.publish_cmd(
                0.0,
                0.0
            )

            if now.nanoseconds > 0:
                self.change_state('STRAIGHT')

            return

        elapsed = (
            now - self.state_start_time
        ).nanoseconds / 1e9

        # -------------------------
        # STRAIGHT
        # -------------------------

        if self.state == 'STRAIGHT':

            self.publish_cmd(
                self.linear_speed,
                0.0
            )

            if elapsed >= self.straight_duration:

                self.change_state('TURN')

        # -------------------------
        # TURN
        # -------------------------

        elif self.state == 'TURN':

            self.publish_cmd(
                self.linear_speed,
                self.angular_speed
            )

            if elapsed >= self.turn_duration:

                self.finish_turn()

        # -------------------------
        # STOP
        # -------------------------

        elif self.state == 'STOP':

            self.publish_cmd(
                0.0,
                0.0
            )


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            'Drive a rounded Ackermann square path.'
        )
    )

    parser.add_argument(
        '--side',
        type=float,
        required=True,
        help='Square side length in meters.'
    )

    parser.add_argument(
        '--laps',
        type=int,
        required=True,
        help='Number of complete laps.'
    )

    return parser.parse_args()


def main():
    args = parse_args()

    if args.side <= 0.0:
        raise SystemExit(
            '--side must be greater than 0.'
        )

    if args.laps <= 0:
        raise SystemExit(
            '--laps must be greater than 0.'
        )

    rclpy.init()

    try:
        node = SquareDriver(
            side_length=args.side,
            laps=args.laps
        )
    except ValueError as exc:
        rclpy.shutdown()
        raise SystemExit(str(exc))

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        # 退出程序前发送停车命令。
        node.publish_cmd(
            0.0,
            0.0
        )

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
