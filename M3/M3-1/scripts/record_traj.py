#!/usr/bin/env python3

import argparse
import csv
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry


def normalize_angle(angle):
    """把角度归一化到 (-pi, pi]。"""

    while angle <= -math.pi:
        angle += 2.0 * math.pi

    while angle > math.pi:
        angle -= 2.0 * math.pi

    return angle


def quaternion_to_yaw(x, y, z, w):
    """四元数转换为 yaw，单位 rad。"""

    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z)
    )


class TrajectoryRecorder(Node):

    def __init__(self, output_dir, run_id):
        super().__init__('trajectory_recorder')

        # 使用 Gazebo 发布的 /clock。
        self.set_parameters([
            Parameter(
                'use_sim_time',
                Parameter.Type.BOOL,
                True
            )
        ])

        self.output_dir = output_dir
        self.run_id = run_id

        os.makedirs(self.output_dir, exist_ok=True)

        self.odom_path = os.path.join(
            self.output_dir,
            f'run_{self.run_id:02d}_odom.csv'
        )

        self.truth_path = os.path.join(
            self.output_dir,
            f'run_{self.run_id:02d}_truth.csv'
        )

        self.odom_file = open(
            self.odom_path,
            'w',
            newline=''
        )

        self.truth_file = open(
            self.truth_path,
            'w',
            newline=''
        )

        self.odom_writer = csv.writer(self.odom_file)
        self.truth_writer = csv.writer(self.truth_file)

        # 题目要求的固定四列表头。
        self.odom_writer.writerow([
            't',
            'x',
            'y',
            'theta'
        ])

        self.truth_writer.writerow([
            't',
            'x',
            'y',
            'theta'
        ])

        self.odom_file.flush()
        self.truth_file.flush()

        # -------------------------
        # 最新数据
        # -------------------------

        self.latest_odom = None
        self.latest_truth = None

        # 正式记录是否已经开始。
        self.recording_started = False

        # 记录开始时的 Gazebo 仿真时间。
        self.record_start_time = None

        # 上一次真正写入 CSV 的相对时间。
        # 用来避免重复时间戳。
        self.last_recorded_t = None

        # -------------------------
        # Subscribers
        # -------------------------

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.truth_sub = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.truth_callback,
            10
        )

        # 10 Hz 采样。
        self.timer = self.create_timer(
            0.1,
            self.record_callback
        )

        self.sample_count = 0

        self.get_logger().info(
            f'Recorder started for run {self.run_id:02d}.'
        )

        self.get_logger().info(
            f'Odom output: {self.odom_path}'
        )

        self.get_logger().info(
            f'Truth output: {self.truth_path}'
        )

        self.get_logger().info(
            'Waiting for /odom and /gazebo/model_states...'
        )

    def odom_callback(self, msg):
        """保存最新 /odom。"""

        self.latest_odom = msg

    def truth_callback(self, msg):
        """从 ModelStates 中找到 smart_car。"""

        try:
            index = msg.name.index('smart_car')
        except ValueError:
            return

        if index >= len(msg.pose):
            return

        self.latest_truth = msg.pose[index]

    def get_sim_time(self):
        """返回当前 Gazebo 仿真时间，单位秒。"""

        now = self.get_clock().now()

        if now.nanoseconds <= 0:
            return None

        return now.nanoseconds / 1e9

    def start_recording_if_ready(self):
        """等两路数据都准备好后才正式开始记录。"""

        if self.recording_started:
            return

        if self.latest_odom is None:
            return

        if self.latest_truth is None:
            return

        current_time = self.get_sim_time()

        if current_time is None:
            return

        self.record_start_time = current_time
        self.recording_started = True
        self.last_recorded_t = None

        self.get_logger().info(
            f'Start recording at sim time '
            f'{self.record_start_time:.3f} s.'
        )

    def record_callback(self):
        """每 0.1 秒记录一次 odom 和 truth。"""

        self.start_recording_if_ready()

        if not self.recording_started:
            return

        current_time = self.get_sim_time()

        if current_time is None:
            return

        # 相对于本次记录开始时刻的时间。
        t = current_time - self.record_start_time

        # 防止同一个仿真时间重复写入。
        if self.last_recorded_t is not None:
            if t <= self.last_recorded_t:
                return

        # 我们要求两路数据都存在，
        # 这样 odom 和 truth 才能共享完全相同的时间戳。
        if self.latest_odom is None:
            return

        if self.latest_truth is None:
            return

        # -------------------------
        # /odom
        # -------------------------

        odom_pose = self.latest_odom.pose.pose

        odom_theta = quaternion_to_yaw(
            odom_pose.orientation.x,
            odom_pose.orientation.y,
            odom_pose.orientation.z,
            odom_pose.orientation.w
        )

        odom_theta = normalize_angle(
            odom_theta
        )

        # -------------------------
        # Gazebo truth
        # -------------------------

        truth_pose = self.latest_truth

        truth_theta = quaternion_to_yaw(
            truth_pose.orientation.x,
            truth_pose.orientation.y,
            truth_pose.orientation.z,
            truth_pose.orientation.w
        )

        truth_theta = normalize_angle(
            truth_theta
        )

        # -------------------------
        # 写入两个 CSV
        # -------------------------

        self.odom_writer.writerow([
            f'{t:.3f}',
            f'{odom_pose.position.x:.6f}',
            f'{odom_pose.position.y:.6f}',
            f'{odom_theta:.6f}'
        ])

        self.truth_writer.writerow([
            f'{t:.3f}',
            f'{truth_pose.position.x:.6f}',
            f'{truth_pose.position.y:.6f}',
            f'{truth_theta:.6f}'
        ])

        self.odom_file.flush()
        self.truth_file.flush()

        self.last_recorded_t = t
        self.sample_count += 1

    def close_files(self):
        """关闭 CSV 文件。"""

        if not self.odom_file.closed:
            self.odom_file.close()

        if not self.truth_file.closed:
            self.truth_file.close()


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            'Record /odom and Gazebo truth trajectories '
            'to CSV.'
        )
    )

    parser.add_argument(
        '--out',
        required=True,
        help='Output directory.'
    )

    parser.add_argument(
        '--run',
        type=int,
        default=1,
        help='Run number, default: 1.'
    )

    return parser.parse_args()


def main():
    args = parse_args()

    rclpy.init()

    recorder = TrajectoryRecorder(
        output_dir=args.out,
        run_id=args.run
    )

    try:
        rclpy.spin(recorder)

    except KeyboardInterrupt:
        pass

    finally:
        recorder.close_files()
        recorder.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
