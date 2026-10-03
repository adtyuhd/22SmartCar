#!/usr/bin/env python3

import argparse
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist


WHEELBASE = 0.28
MAX_STEERING_ANGLE = math.radians(30.0)

# Gazebo 实测标定：
# R = 0.6 m, v = 0.2 m/s 时，
# 理论 4.712 s 会明显过转，
# 实测约 3.93 s 对应真实 90°。
TURN_TIME_SCALE = 0.834


class SquareDriver(Node):

    def __init__(self):
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

        # 20 Hz
        self.publish_period = 0.05

    def publish_cmd(self, linear_x, angular_z):
        msg = Twist()

        msg.linear.x = linear_x
        msg.angular.z = angular_z

        self.cmd_pub.publish(msg)

    def wait_for_clock(self):
        """
        等待 Gazebo /clock 有效。
        """

        while rclpy.ok():

            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

            if self.get_clock().now().nanoseconds > 0:
                return

    def run_segment(self, linear_x, angular_z, duration):
        """
        按照给定速度运行指定的仿真时间。
        """

        # 一开始立刻发送一次命令
        self.publish_cmd(
            linear_x,
            angular_z
        )

        start_time = self.get_clock().now()
        last_publish_time = start_time

        while rclpy.ok():

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
        """
        直行指定距离。
        """

        duration = distance / speed

        self.get_logger().info(
            f'Straight: '
            f'distance={distance:.3f} m, '
            f'speed={speed:.3f} m/s, '
            f'duration={duration:.3f} s'
        )

        self.run_segment(
            linear_x=speed,
            angular_z=0.0,
            duration=duration
        )

    def turn_left_90(self, speed, radius):
        """
        左转 90°。
        """

        angular_velocity = speed / radius

        turn_angle = math.pi / 2.0

        theoretical_duration = (
            turn_angle / angular_velocity
        )

        calibrated_duration = (
            theoretical_duration
            * TURN_TIME_SCALE
        )

        steering_angle = math.atan(
            WHEELBASE / radius
        )

        self.get_logger().info(
            f'Left turn: '
            f'radius={radius:.3f} m, '
            f'angular_velocity={angular_velocity:.3f} rad/s, '
            f'steering_angle={math.degrees(steering_angle):.2f} deg, '
            f'theoretical_duration={theoretical_duration:.3f} s, '
            f'calibrated_duration={calibrated_duration:.3f} s'
        )

        self.run_segment(
            linear_x=speed,
            angular_z=angular_velocity,
            duration=calibrated_duration
        )

    def stop(self):
        """
        连续发送停止命令。
        """

        for _ in range(5):

            self.publish_cmd(
                0.0,
                0.0
            )

            rclpy.spin_once(
                self,
                timeout_sec=0.02
            )


def start_recorder(output_dir):
    """
    启动 record_traj.py。

    output_dir 例如：
        data/test_run

    最终生成：
        data/test_run/run_01_odom.csv
        data/test_run/run_01_truth.csv
    """

    os.makedirs(
        output_dir,
        exist_ok=True
    )

    script_dir = os.path.dirname(
        os.path.abspath(__file__)
    )

    recorder_script = os.path.join(
        script_dir,
        'record_traj.py'
    )

    output_prefix = os.path.join(
        output_dir,
        'run_01'
    )

    process = subprocess.Popen([
        sys.executable,
        recorder_script,
        '--output-prefix',
        output_prefix
    ])

    return process


def stop_recorder(process):
    """
    给记录器发送 Ctrl+C 等价的 SIGINT，
    让它正常关闭 CSV 文件。
    """

    if process is None:
        return

    if process.poll() is None:

        process.send_signal(
            signal.SIGINT
        )

        try:
            process.wait(
                timeout=5
            )

        except subprocess.TimeoutExpired:

            process.terminate()
            process.wait()


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        '--side',
        type=float,
        default=2.0,
        help='outer square side length in meters'
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
        default=0.6,
        help='corner turn radius in meters'
    )

    parser.add_argument(
        '--out',
        type=str,
        default=None,
        help='directory for recorded CSV files'
    )

    args = parser.parse_args()

    if args.side <= 0.0:
        parser.error(
            '--side must be greater than 0'
        )

    if args.speed <= 0.0:
        parser.error(
            '--speed must be greater than 0'
        )

    if args.turn_radius <= 0.0:
        parser.error(
            '--turn-radius must be greater than 0'
        )

    straight_length = (
        args.side
        - 2.0 * args.turn_radius
    )

    if straight_length <= 0.0:
        parser.error(
            '--turn-radius is too large '
            'for the selected --side'
        )

    steering_angle = math.atan(
        WHEELBASE / args.turn_radius
    )

    if steering_angle > MAX_STEERING_ANGLE:
        parser.error(
            'turn radius is too small: '
            f'requires steering angle '
            f'{math.degrees(steering_angle):.2f} deg, '
            f'but limit is 30.00 deg'
        )

    recorder_process = None

    rclpy.init()

    node = SquareDriver()

    try:

        node.wait_for_clock()

        node.get_logger().info(
            '========== PATH PARAMETERS =========='
        )

        node.get_logger().info(
            f'Outer side length: '
            f'{args.side:.3f} m'
        )

        node.get_logger().info(
            f'Turn radius: '
            f'{args.turn_radius:.3f} m'
        )

        node.get_logger().info(
            f'Straight length: '
            f'{straight_length:.3f} m'
        )

        node.get_logger().info(
            f'Turn time scale: '
            f'{TURN_TIME_SCALE:.3f}'
        )

        # 如果给了 --out，就自动启动记录器
        if args.out is not None:

            node.get_logger().info(
                '========== START RECORDER =========='
            )

            recorder_process = start_recorder(
                args.out
            )

            # 给记录器一点时间完成 ROS 节点初始化
            time.sleep(1.0)

        # 跑一圈
        for i in range(4):

            node.get_logger().info(
                f'========== SIDE {i + 1}/4 =========='
            )

            node.drive_straight(
                distance=straight_length,
                speed=args.speed
            )

            node.turn_left_90(
                speed=args.speed,
                radius=args.turn_radius
            )

        node.stop()

        # 停车后再多记录一小段
        time.sleep(1.0)

        node.get_logger().info(
            '========== ONE LAP FINISHED =========='
        )

    finally:

        node.stop()

        stop_recorder(
            recorder_process
        )

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()