#!/usr/bin/env python3

import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class SquareDriver(Node):

    def __init__(self, side_length, laps):
        super().__init__('square_driver')

        self.set_parameters([
            Parameter(
                'use_sim_time',
                Parameter.Type.BOOL,
                True
            )
        ])

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.control_pub = self.create_publisher(
            String,
            '/experiment_control',
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.current_theta = None
        self.turn_start_theta = None

        self.side_length = side_length
        self.total_laps = laps

        self.linear_speed = 0.2

        # 使用可行的 Ackermann 圆角半径。
        #
        # wheelbase = 0.28 m
        # track = 0.18 m
        # steering limit = 30 deg
        #
        # R = 0.6 m 时，两个前轮理论转角约为：
        # 28.77 deg 和 22.09 deg，
        # 已通过 /joint_states 验证不会发生转向饱和。
        self.turn_radius = 0.6

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

        self.straight_duration = (
            self.straight_length
            / self.linear_speed
        )

        self.angular_speed = (
            self.linear_speed
            / self.turn_radius
        )

        self.turn_angle = math.pi / 2.0

        # 使用理论时间，不根据 odom 或 truth 反向校准。
        self.turn_duration = (
            self.turn_angle
            / self.angular_speed
        )

        self.corners_per_lap = 4

        self.current_lap = 1
        self.completed_corners = 0

        # 状态：
        #
        # WAIT_CLOCK
        #     等待 Gazebo /clock
        #
        # READY
        #     连续发布 START，让 recorder 稳定收到开始信号
        #
        # STRAIGHT
        #     直线
        #
        # TURN
        #     90° 圆弧
        #
        # STOP
        #     停车
        self.state = 'WAIT_CLOCK'
        self.state_start_time = None

        # READY 持续 1 秒。
        # 期间车辆保持静止并重复发布 START。
        self.ready_duration = 1.0

        self.stop_signal_sent = False

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
            f'Turn radius: {self.turn_radius:.3f} m'
        )
        self.get_logger().info(
            f'Straight length: '
            f'{self.straight_length:.3f} m'
        )
        self.get_logger().info(
            f'Linear speed: '
            f'{self.linear_speed:.3f} m/s'
        )
        self.get_logger().info(
            f'Angular speed: '
            f'{self.angular_speed:.6f} rad/s'
        )
        self.get_logger().info(
            f'Straight duration: '
            f'{self.straight_duration:.3f} s'
        )
        self.get_logger().info(
            f'Theoretical turn duration: '
            f'{self.turn_duration:.3f} s'
        )
        self.get_logger().info(
            'Waiting for Gazebo /clock...'
        )

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation

        self.current_theta = math.atan2(
            2.0 * (
                q.w * q.z
                + q.x * q.y
            ),
            1.0 - 2.0 * (
                q.y * q.y
                + q.z * q.z
            )
        )

    def publish_cmd(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_pub.publish(msg)

    def publish_control(self, command):
        msg = String()
        msg.data = command
        self.control_pub.publish(msg)

    def normalize_angle(self, angle):
        while angle <= -math.pi:
            angle += 2.0 * math.pi

        while angle > math.pi:
            angle -= 2.0 * math.pi

        return angle

    def change_state(self, new_state):
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
        self.completed_corners += 1

        # odom 只用于观察。
        # 不参与运动控制。
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
                'Odom turn: '
                f'{math.degrees(actual_turn):.2f} deg'
            )

            self.get_logger().info(
                'Odom turn error: '
                f'{math.degrees(turn_error):+.2f} deg'
            )

        if self.completed_corners >= self.corners_per_lap:

            self.get_logger().info(
                f'Lap {self.current_lap}/'
                f'{self.total_laps} completed.'
            )

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
            self.change_state('STRAIGHT')

    def timer_callback(self):
        now = self.get_clock().now()

        # =========================
        # 等待 Gazebo /clock
        # =========================

        if self.state == 'WAIT_CLOCK':
            self.publish_cmd(
                0.0,
                0.0
            )

            if now.nanoseconds > 0:
                self.change_state('READY')

            return

        elapsed = (
            now - self.state_start_time
        ).nanoseconds / 1e9

        # =========================
        # 实验开始同步
        # =========================

        if self.state == 'READY':

            self.publish_cmd(
                0.0,
                0.0
            )

            # READY 阶段重复发送 START，
            # 避免 recorder 漏掉单次 topic 消息。
            self.publish_control('START')

            if elapsed >= self.ready_duration:
                self.change_state('STRAIGHT')

        # =========================
        # 直线
        # =========================

        elif self.state == 'STRAIGHT':

            self.publish_cmd(
                self.linear_speed,
                0.0
            )

            if elapsed >= self.straight_duration:
                self.change_state('TURN')

        # =========================
        # 90° 圆弧
        # =========================

        elif self.state == 'TURN':

            self.publish_cmd(
                self.linear_speed,
                self.angular_speed
            )

            if elapsed >= self.turn_duration:
                self.finish_turn()

        # =========================
        # 停车 + 实验结束信号
        # =========================

        elif self.state == 'STOP':

            self.publish_cmd(
                0.0,
                0.0
            )

            if not self.stop_signal_sent:
                self.publish_control('STOP')
                self.stop_signal_sent = True

                self.get_logger().info(
                    'Experiment STOP signal sent.'
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
        node.publish_cmd(
            0.0,
            0.0
        )

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
