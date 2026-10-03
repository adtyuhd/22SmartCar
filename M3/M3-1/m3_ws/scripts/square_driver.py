#!/usr/bin/env python3

import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist


class SquareDriver(Node):

    def __init__(self):
        super().__init__('square_driver')

        # 使用 Gazebo 仿真时间
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

        # 控制命令发布周期：0.05 s = 20 Hz
        self.publish_period = 0.05

    def publish_cmd(self, linear_x, angular_z):
        msg = Twist()

        msg.linear.x = linear_x
        msg.angular.z = angular_z

        self.cmd_pub.publish(msg)

    def wait_for_clock(self):
        """等待 Gazebo 的仿真时钟有效。"""

        while rclpy.ok():
            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

            if self.get_clock().now().nanoseconds > 0:
                return

    def run_segment(self, linear_x, angular_z, duration):
        """按照给定速度运行指定的仿真时间。"""

        start_time = self.get_clock().now()
        last_publish_time = start_time

        while rclpy.ok():

            # 持续处理 /clock
            rclpy.spin_once(
                self,
                timeout_sec=0.01
            )

            now = self.get_clock().now()

            elapsed = (
                now - start_time
            ).nanoseconds / 1e9

            if elapsed >= duration:
                break

            since_last_publish = (
                now - last_publish_time
            ).nanoseconds / 1e9

            if since_last_publish >= self.publish_period:

                self.publish_cmd(
                    linear_x,
                    angular_z
                )

                last_publish_time = now

    def drive_straight(self, distance, speed):
        """直行指定距离。"""

        duration = distance / speed

        self.get_logger().info(
            f'Straight: distance={distance:.3f} m, '
            f'speed={speed:.3f} m/s, '
            f'duration={duration:.3f} s'
        )

        self.run_segment(
            linear_x=speed,
            angular_z=0.0,
            duration=duration
        )

    def turn_left_90(self, speed, radius):
        """按照指定半径完成 90° 左转。"""

        angular_velocity = speed / radius

        turn_angle = math.pi / 2.0

        duration = turn_angle / angular_velocity

        steering_angle = math.atan(0.28 / radius)

        self.get_logger().info(
            f'Left turn: radius={radius:.3f} m, '
            f'angular_velocity={angular_velocity:.3f} rad/s, '
            f'steering_angle={math.degrees(steering_angle):.2f} deg, '
            f'duration={duration:.3f} s'
        )

        self.run_segment(
            linear_x=speed,
            angular_z=angular_velocity,
            duration=duration
        )

    def stop(self):
        """发送多次停止命令。"""

        for _ in range(5):

            self.publish_cmd(
                0.0,
                0.0
            )

            rclpy.spin_once(
                self,
                timeout_sec=0.02
            )


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        '--side',
        type=float,
        default=2.0,
        help='straight side length in meters'
    )

    parser.add_argument(
        '--speed',
        type=float,
        default=0.2,
        help='linear speed in m/s'
    )

    parser.add_argument(
        '--turn-radius',
        type=float,
        default=1.0,
        help='turn radius in meters'
    )

    args = parser.parse_args()

    if args.side <= 0.0:
        parser.error('--side must be greater than 0')

    if args.speed <= 0.0:
        parser.error('--speed must be greater than 0')

    if args.turn_radius <= 0.0:
        parser.error('--turn-radius must be greater than 0')

    rclpy.init()

    node = SquareDriver()

    try:
        node.wait_for_clock()

        # 第一段：直行一条边
        node.drive_straight(
            distance=args.side,
            speed=args.speed
        )

        # 第二段：左转 90°
        node.turn_left_90(
            speed=args.speed,
            radius=args.turn_radius
        )

    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()