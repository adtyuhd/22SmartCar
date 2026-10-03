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
from std_srvs.srv import Empty


WHEELBASE = 0.28
MAX_STEERING_ANGLE = math.radians(30.0)

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

        self.reset_client = self.create_client(
            Empty,
            '/reset_world'
        )

        self.publish_period = 0.05

    def publish_cmd(
        self,
        linear_x,
        angular_z
    ):
        msg = Twist()

        msg.linear.x = linear_x
        msg.angular.z = angular_z

        self.cmd_pub.publish(msg)

    def wait_for_clock(self):
        while rclpy.ok():

            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

            if (
                self.get_clock()
                .now()
                .nanoseconds
                > 0
            ):
                return

    def wait_sim_duration(
        self,
        duration
    ):
        start_time = None

        while rclpy.ok():

            rclpy.spin_once(
                self,
                timeout_sec=0.02
            )

            now = self.get_clock().now()

            if now.nanoseconds <= 0:
                continue

            if start_time is None:
                start_time = now
                continue

            elapsed = (
                now - start_time
            ).nanoseconds / 1e9

            if elapsed >= duration:
                return

    def reset_world(self):
        self.stop()

        while not (
            self.reset_client.wait_for_service(
                timeout_sec=1.0
            )
        ):
            self.get_logger().info(
                'Waiting for /reset_world ...'
            )

        request = Empty.Request()

        future = (
            self.reset_client.call_async(
                request
            )
        )

        rclpy.spin_until_future_complete(
            self,
            future
        )

        if future.exception() is not None:

            raise RuntimeError(
                f'/reset_world failed: '
                f'{future.exception()}'
            )

        self.stop()

    def run_segment(
        self,
        linear_x,
        angular_z,
        duration
    ):
        # 先获取最新的 /clock
        rclpy.spin_once(
            self,
            timeout_sec=0.02
        )

        start_time = (
            self.get_clock().now()
        )

        last_publish_time = (
            start_time
        )

        self.publish_cmd(
            linear_x,
            angular_z
        )

        while rclpy.ok():

            rclpy.spin_once(
                self,
                timeout_sec=0.01
            )

            now = (
                self.get_clock().now()
            )

            elapsed = (
                now - start_time
            ).nanoseconds / 1e9

            if elapsed >= duration:
                break

            since_last_publish = (
                now - last_publish_time
            ).nanoseconds / 1e9

            if (
                since_last_publish
                >= self.publish_period
            ):

                self.publish_cmd(
                    linear_x,
                    angular_z
                )

                last_publish_time = now

    def drive_straight(
        self,
        distance,
        speed
    ):
        duration = (
            distance / speed
        )

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

    def turn_left_90(
        self,
        speed,
        radius
    ):
        angular_velocity = (
            speed / radius
        )

        turn_angle = (
            math.pi / 2.0
        )

        theoretical_duration = (
            turn_angle
            / angular_velocity
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
            f'angular_velocity='
            f'{angular_velocity:.3f} rad/s, '
            f'steering_angle='
            f'{math.degrees(steering_angle):.2f} deg, '
            f'theoretical_duration='
            f'{theoretical_duration:.3f} s, '
            f'calibrated_duration='
            f'{calibrated_duration:.3f} s'
        )

        self.run_segment(
            linear_x=speed,
            angular_z=angular_velocity,
            duration=calibrated_duration
        )

    def stop(self):
        for _ in range(5):

            self.publish_cmd(
                0.0,
                0.0
            )

            rclpy.spin_once(
                self,
                timeout_sec=0.02
            )


def start_recorder(
    output_dir,
    lap_number
):
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
        f'run_{lap_number:02d}'
    )

    ready_file = os.path.join(
        output_dir,
        f'.run_{lap_number:02d}_ready'
    )

    # 防止上一次运行留下 ready 文件
    if os.path.exists(
        ready_file
    ):
        os.remove(
            ready_file
        )

    process = subprocess.Popen([
        sys.executable,
        recorder_script,
        '--output-prefix',
        output_prefix,
        '--ready-file',
        ready_file
    ])

    return (
        process,
        ready_file
    )


def wait_for_recorder_ready(
    node,
    process,
    ready_file,
    timeout=10.0
):
    """
    等到 recorder 已经真正收到
    odom 和 Gazebo truth。
    """

    start_wall_time = (
        time.monotonic()
    )

    while rclpy.ok():

        rclpy.spin_once(
            node,
            timeout_sec=0.05
        )

        if os.path.exists(
            ready_file
        ):

            node.get_logger().info(
                'Recorder confirmed ready.'
            )

            return

        if process.poll() is not None:

            raise RuntimeError(
                'Trajectory recorder '
                'exited before becoming ready.'
            )

        elapsed = (
            time.monotonic()
            - start_wall_time
        )

        if elapsed >= timeout:

            raise RuntimeError(
                'Timed out waiting for '
                'trajectory recorder.'
            )


def stop_recorder(
    process,
    ready_file=None
):
    if process is not None:

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

    if (
        ready_file
        and os.path.exists(
            ready_file
        )
    ):
        os.remove(
            ready_file
        )


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        '--laps',
        type=int,
        default=1,
        help='number of laps'
    )

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

    if args.laps <= 0:
        parser.error(
            '--laps must be greater than 0'
        )

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
        - 2.0
        * args.turn_radius
    )

    if straight_length <= 0.0:
        parser.error(
            '--turn-radius is too large '
            'for the selected --side'
        )

    steering_angle = math.atan(
        WHEELBASE
        / args.turn_radius
    )

    if (
        steering_angle
        > MAX_STEERING_ANGLE
    ):
        parser.error(
            'turn radius is too small: '
            f'requires steering angle '
            f'{math.degrees(steering_angle):.2f} deg, '
            f'but limit is 30.00 deg'
        )

    recorder_process = None
    ready_file = None

    rclpy.init()

    node = SquareDriver()

    try:

        node.wait_for_clock()

        node.get_logger().info(
            '========== PATH PARAMETERS =========='
        )

        node.get_logger().info(
            f'Laps: {args.laps}'
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

        for lap in range(
            1,
            args.laps + 1
        ):

            node.get_logger().info(
                f'========== LAP '
                f'{lap}/{args.laps} =========='
            )

            node.get_logger().info(
                'Resetting world...'
            )

            node.reset_world()

            # 等车辆物理状态稳定
            node.wait_sim_duration(
                1.0
            )

            if args.out is not None:

                (
                    recorder_process,
                    ready_file
                ) = start_recorder(
                    args.out,
                    lap
                )

                # 关键：
                # 不再固定等待 1 秒。
                # recorder 真正收到两种数据后
                # 才开始运动。
                wait_for_recorder_ready(
                    node,
                    recorder_process,
                    ready_file
                )

            for side_index in range(4):

                node.get_logger().info(
                    f'----- LAP {lap} '
                    f'SIDE '
                    f'{side_index + 1}/4 -----'
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

            node.wait_sim_duration(
                0.5
            )

            stop_recorder(
                recorder_process,
                ready_file
            )

            recorder_process = None
            ready_file = None

            node.get_logger().info(
                f'========== LAP {lap} '
                f'FINISHED =========='
            )

        node.get_logger().info(
            '========== ALL LAPS FINISHED =========='
        )

    finally:

        node.stop()

        stop_recorder(
            recorder_process,
            ready_file
        )

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()