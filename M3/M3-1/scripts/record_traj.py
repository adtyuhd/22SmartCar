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
from std_msgs.msg import String


def normalize_angle(angle):
    while angle <= -math.pi:
        angle += 2.0 * math.pi

    while angle > math.pi:
        angle -= 2.0 * math.pi

    return angle


def quaternion_to_yaw(x, y, z, w):
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z)
    )


class TrajectoryRecorder(Node):

    def __init__(self, output_dir, run_id):
        super().__init__('trajectory_recorder')

        self.set_parameters([
            Parameter(
                'use_sim_time',
                Parameter.Type.BOOL,
                True
            )
        ])

        self.output_dir = output_dir
        self.run_id = run_id

        os.makedirs(
            self.output_dir,
            exist_ok=True
        )

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

        self.odom_writer = csv.writer(
            self.odom_file
        )

        self.truth_writer = csv.writer(
            self.truth_file
        )

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

        self.latest_odom = None
        self.latest_truth = None

        self.recording_started = False
        self.recording_finished = False

        self.record_start_time = None
        self.last_recorded_t = None

        self.sample_count = 0

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

        self.control_sub = self.create_subscription(
            String,
            '/experiment_control',
            self.control_callback,
            10
        )

        # 10 Hz 统一采样。
        self.timer = self.create_timer(
            0.1,
            self.record_callback
        )

        self.get_logger().info(
            f'Recorder ready for run '
            f'{self.run_id:02d}.'
        )

        self.get_logger().info(
            f'Odom output: {self.odom_path}'
        )

        self.get_logger().info(
            f'Truth output: {self.truth_path}'
        )

        self.get_logger().info(
            'Waiting for /odom and '
            '/gazebo/model_states...'
        )

        self.get_logger().info(
            'Then waiting for START signal '
            'from square_driver.'
        )

    def odom_callback(self, msg):
        self.latest_odom = msg

    def truth_callback(self, msg):
        try:
            index = msg.name.index(
                'smart_car'
            )

        except ValueError:
            return

        if index >= len(msg.pose):
            return

        self.latest_truth = msg.pose[index]

    def get_sim_time(self):
        now = self.get_clock().now()

        if now.nanoseconds <= 0:
            return None

        return now.nanoseconds / 1e9

    def data_ready(self):
        return (
            self.latest_odom is not None
            and self.latest_truth is not None
        )

    def control_callback(self, msg):

        if msg.data == 'START':

            # READY 阶段会收到多个 START。
            # 只处理第一个。
            if self.recording_started:
                return

            if not self.data_ready():
                self.get_logger().warning(
                    'START received, but odom/truth '
                    'data are not ready yet.'
                )
                return

            current_time = self.get_sim_time()

            if current_time is None:
                return

            self.record_start_time = current_time
            self.last_recorded_t = None
            self.recording_started = True

            self.get_logger().info(
                'START received.'
            )

            self.get_logger().info(
                'Recording started at sim time '
                f'{self.record_start_time:.3f} s.'
            )

            # START 时立即写入第一条记录，
            # 因此第一条数据的 t 尽量接近 0。
            self.write_sample(
                current_time
            )

        elif msg.data == 'STOP':

            if not self.recording_started:
                return

            if self.recording_finished:
                return

            current_time = self.get_sim_time()

            if current_time is not None:
                self.write_sample(
                    current_time
                )

            self.recording_finished = True

            self.get_logger().info(
                'STOP received.'
            )

            self.get_logger().info(
                f'Recording finished with '
                f'{self.sample_count} samples.'
            )

            self.get_logger().info(
                'Recorder will shut down automatically.'
            )

    def write_sample(self, current_time):

        if not self.recording_started:
            return

        if self.record_start_time is None:
            return

        if not self.data_ready():
            return

        t = (
            current_time
            - self.record_start_time
        )

        if t < 0.0:
            return

        if (
            self.last_recorded_t is not None
            and t <= self.last_recorded_t
        ):
            return

        odom_pose = self.latest_odom.pose.pose

        odom_theta = normalize_angle(
            quaternion_to_yaw(
                odom_pose.orientation.x,
                odom_pose.orientation.y,
                odom_pose.orientation.z,
                odom_pose.orientation.w
            )
        )

        truth_pose = self.latest_truth

        truth_theta = normalize_angle(
            quaternion_to_yaw(
                truth_pose.orientation.x,
                truth_pose.orientation.y,
                truth_pose.orientation.z,
                truth_pose.orientation.w
            )
        )

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

    def record_callback(self):

        if self.recording_finished:
            return

        if not self.recording_started:
            return

        current_time = self.get_sim_time()

        if current_time is None:
            return

        self.write_sample(
            current_time
        )

    def close_files(self):

        if not self.odom_file.closed:
            self.odom_file.close()

        if not self.truth_file.closed:
            self.truth_file.close()


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            'Record /odom and Gazebo truth '
            'trajectories to CSV.'
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

    if args.run <= 0:
        raise SystemExit(
            '--run must be greater than 0.'
        )

    rclpy.init()

    recorder = TrajectoryRecorder(
        output_dir=args.out,
        run_id=args.run
    )

    try:
        while (
            rclpy.ok()
            and not recorder.recording_finished
        ):
            rclpy.spin_once(
                recorder,
                timeout_sec=0.1
            )

    except KeyboardInterrupt:
        pass

    finally:
        recorder.close_files()
        recorder.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
