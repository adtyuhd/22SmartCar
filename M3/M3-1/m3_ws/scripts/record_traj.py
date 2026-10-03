#!/usr/bin/env python3

import argparse
import csv
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates


def quaternion_to_yaw(q):
    """
    Quaternion -> yaw

    返回范围约为 [-pi, pi]
    """

    siny_cosp = 2.0 * (
        q.w * q.z
        + q.x * q.y
    )

    cosy_cosp = 1.0 - 2.0 * (
        q.y * q.y
        + q.z * q.z
    )

    return math.atan2(
        siny_cosp,
        cosy_cosp
    )


class TrajectoryRecorder(Node):

    def __init__(
        self,
        output_prefix,
        model_name
    ):
        super().__init__(
            'trajectory_recorder'
        )

        self.set_parameters([
            Parameter(
                'use_sim_time',
                Parameter.Type.BOOL,
                True
            )
        ])

        self.model_name = model_name
        self.start_time = None
        self.model_index = None

        # ------------------------------
        # 输出文件
        # ------------------------------

        odom_path = (
            output_prefix
            + '_odom.csv'
        )

        truth_path = (
            output_prefix
            + '_truth.csv'
        )

        output_dir = os.path.dirname(
            output_prefix
        )

        if output_dir:

            os.makedirs(
                output_dir,
                exist_ok=True
            )

        self.odom_file = open(
            odom_path,
            'w',
            newline='',
            buffering=1
        )

        self.truth_file = open(
            truth_path,
            'w',
            newline='',
            buffering=1
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

        # ------------------------------
        # ROS subscriptions
        # ------------------------------

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            50
        )

        self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.truth_callback,
            50
        )

        self.get_logger().info(
            f'Recording odom to: '
            f'{odom_path}'
        )

        self.get_logger().info(
            f'Recording truth to: '
            f'{truth_path}'
        )

    def get_relative_time(self):
        """
        使用 ROS 仿真时间。
        """

        now = self.get_clock().now()

        if now.nanoseconds <= 0:
            return None

        if self.start_time is None:
            self.start_time = now

        elapsed = (
            now - self.start_time
        ).nanoseconds / 1e9

        return elapsed

    def odom_callback(self, msg):
        t = self.get_relative_time()

        if t is None:
            return

        pose = msg.pose.pose

        x = pose.position.x
        y = pose.position.y

        theta = quaternion_to_yaw(
            pose.orientation
        )

        self.odom_writer.writerow([
            f'{t:.6f}',
            f'{x:.9f}',
            f'{y:.9f}',
            f'{theta:.9f}'
        ])

    def truth_callback(self, msg):
        t = self.get_relative_time()

        if t is None:
            return

        if self.model_index is None:

            try:

                self.model_index = (
                    msg.name.index(
                        self.model_name
                    )
                )

                self.get_logger().info(
                    f'Found Gazebo model '
                    f'"{self.model_name}" '
                    f'at index '
                    f'{self.model_index}'
                )

            except ValueError:

                self.get_logger().warning(
                    f'Gazebo model '
                    f'"{self.model_name}" '
                    f'not found'
                )

                return

        if self.model_index >= len(
            msg.pose
        ):
            self.model_index = None
            return

        pose = msg.pose[
            self.model_index
        ]

        x = pose.position.x
        y = pose.position.y

        theta = quaternion_to_yaw(
            pose.orientation
        )

        self.truth_writer.writerow([
            f'{t:.6f}',
            f'{x:.9f}',
            f'{y:.9f}',
            f'{theta:.9f}'
        ])

    def close_files(self):
        """
        刷新并关闭 CSV。
        """

        self.odom_file.flush()
        self.truth_file.flush()

        self.odom_file.close()
        self.truth_file.close()


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        '--output-prefix',
        type=str,
        default='data/run',
        help='output path prefix'
    )

    parser.add_argument(
        '--model-name',
        type=str,
        default='smart_car',
        help='Gazebo model name'
    )

    args = parser.parse_args()

    rclpy.init()

    node = TrajectoryRecorder(
        output_prefix=args.output_prefix,
        model_name=args.model_name
    )

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    except RuntimeError as exc:
        """
        ROS 2 Humble 中进程收到 SIGINT 时，
        executor 有可能正处于 take_message()。

        此时偶尔会抛出：
        Unable to convert call argument to Python object

        数据已经通过 line buffering 写入 CSV，
        finally 中再 flush/close。
        """

        if (
            'Unable to convert call argument '
            'to Python object'
            not in str(exc)
        ):
            raise

    finally:

        node.close_files()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()