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

        # 使用 Gazebo 的仿真时间 /clock
        self.set_parameters([
            Parameter(
                'use_sim_time',
                Parameter.Type.BOOL,
                True
            )
        ])

        # 发布车辆速度命令
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # 订阅里程计，仅用于观察和记录转弯结果
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.current_theta = None
        self.turn_start_theta = None

        # =========================
        # 路径参数
        # =========================

        self.side_length = side_length
        self.total_laps = laps

        # 车辆前进速度
        self.linear_speed = 0.2

        # 圆角半径。
        #
        # 不能再使用 0.5 m。
        #
        # 对于：
        # wheelbase = 0.28 m
        # track     = 0.18 m
        # steering limit = 30 deg
        #
        # R = 0.5 m 会要求内侧前轮转到约 34.3 deg，
        # 超过车辆 30 deg 的转向限制。
        #
        # R = 0.6 m 时：
        # inner steering ≈ 28.77 deg
        # outer steering ≈ 22.09 deg
        #
        # 已通过 /joint_states 实际验证没有发生转向饱和。
        self.turn_radius = 0.6

        # 一个 2 m × 2 m 的圆角方形：
        #
        # straight_length = side - 2R
        #
        # 当 side = 2.0 m、R = 0.6 m：
        #
        # straight_length = 0.8 m
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

        # 直线运动理论时间
        #
        # 0.8 / 0.2 = 4.0 s
        self.straight_duration = (
            self.straight_length
            / self.linear_speed
        )

        # Ackermann 圆弧运动：
        #
        # omega = v / R
        #
        # 0.2 / 0.6 = 0.333333... rad/s
        self.angular_speed = (
            self.linear_speed
            / self.turn_radius
        )

        # 每个圆角转 90°
        self.turn_angle = math.pi / 2.0

        # 90° 圆弧的理论运动时间：
        #
        # t = angle / omega
        #
        # (pi / 2) / (0.2 / 0.6)
        # ≈ 4.712 s
        #
        # 这里直接使用理论值。
        # 不使用 Gazebo truth 或 /odom 反向校准这个时间。
        self.turn_duration = (
            self.turn_angle
            / self.angular_speed
        )

        self.corners_per_lap = 4

        self.current_lap = 1
        self.completed_corners = 0

        # WAIT_CLOCK:
        # 等待 Gazebo /clock 真正开始工作。
        self.state = 'WAIT_CLOCK'
        self.state_start_time = None

        # 10 Hz 状态机
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

        x = q.x
        y = q.y
        z = q.z
        w = q.w

        # quaternion -> yaw
        self.current_theta = math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z)
        )

    def publish_cmd(self, linear_x, angular_z):
        msg = Twist()

        msg.linear.x = linear_x
        msg.angular.z = angular_z

        self.cmd_pub.publish(msg)

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

        # /odom 这里只用于观察误差。
        #
        # 它不会参与控制，也不会改变下一次转弯时间。
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

        # 一圈四个圆角
        if self.completed_corners >= self.corners_per_lap:

            self.get_logger().info(
                f'Lap {self.current_lap}/'
                f'{self.total_laps} completed.'
            )

            # 如果还有下一圈
            if self.current_lap < self.total_laps:

                self.current_lap += 1
                self.completed_corners = 0

                self.get_logger().info(
                    f'Starting lap '
                    f'{self.current_lap}/{self.total_laps}.'
                )

                self.change_state('STRAIGHT')

            # 所有圈数完成
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
        # 等待 Gazebo 仿真时间
        # =========================

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

        # =========================
        # 直线
        # =========================

        if self.state == 'STRAIGHT':

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
        # 停车
        # =========================

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
        node.publish_cmd(
            0.0,
            0.0
        )

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
