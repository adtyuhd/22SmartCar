#!/usr/bin/env python3

import statistics

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, Image

from message_filters import (
    Subscriber,
    ApproximateTimeSynchronizer,
)


class SyncTestNode(Node):
    """
    用来研究 /scan 和 /camera/image_raw 的近似时间同步。

    当前节点只研究：
      1. ApproximateTimeSynchronizer
      2. slop
      3. queue_size
      4. 成功配对后的时间差

    暂时不做：
      - TF 坐标变换
      - LaserScan 转 3D 点
      - 相机投影
    """

    def __init__(self):
        super().__init__('sync_test_node')

        # ----------------------------------------------------------
        # 1. ROS 参数
        # ----------------------------------------------------------

        # slop 单位是秒。
        #
        # 例如：
        #
        #   0.02 = 20 ms
        #   0.05 = 50 ms
        #
        self.declare_parameter('slop', 0.05)

        # 题目建议 >= 50。
        self.declare_parameter('queue_size', 50)

        self.slop = (
            self.get_parameter('slop')
            .get_parameter_value()
            .double_value
        )

        self.queue_size = (
            self.get_parameter('queue_size')
            .get_parameter_value()
            .integer_value
        )

        # ----------------------------------------------------------
        # 2. 实验统计数据
        # ----------------------------------------------------------

        # 成功同步的 pair 数量
        self.pair_count = 0

        # 保存每一对消息的时间差，单位 ms
        self.time_diffs_ms = []

        # ----------------------------------------------------------
        # 3. 创建 message_filters 订阅器
        # ----------------------------------------------------------

        self.scan_sub = Subscriber(
            self,
            LaserScan,
            '/scan',
        )

        self.image_sub = Subscriber(
            self,
            Image,
            '/camera/image_raw',
        )

        # ----------------------------------------------------------
        # 4. 创建 ApproximateTimeSynchronizer
        # ----------------------------------------------------------
        #
        # 它会维护两个队列：
        #
        #   /scan 队列
        #   /camera/image_raw 队列
        #
        # 然后尝试从中找到时间差不超过 slop 的一对消息。
        #
        self.synchronizer = ApproximateTimeSynchronizer(
            [
                self.scan_sub,
                self.image_sub,
            ],
            queue_size=self.queue_size,
            slop=self.slop,
        )

        # 成功配出一对以后，调用 synchronized_callback。
        self.synchronizer.registerCallback(
            self.synchronized_callback
        )

        self.get_logger().info(
            'Approximate time synchronizer started'
        )

        self.get_logger().info(
            f'  slop       = {self.slop:.3f} s '
            f'({self.slop * 1000.0:.1f} ms)'
        )

        self.get_logger().info(
            f'  queue_size = {self.queue_size}'
        )

    @staticmethod
    def stamp_to_seconds(stamp):
        """
        把 ROS 时间戳：

            stamp.sec
            stamp.nanosec

        转换成秒。

        例如：

            sec = 1
            nanosec = 500000000

        得到：

            1.5 秒
        """

        return (
            float(stamp.sec)
            + float(stamp.nanosec) * 1e-9
        )

    def synchronized_callback(self, scan_msg, image_msg):
        """
        只有同步器认为这一帧 scan 和这一帧 image
        可以组成一对时，才会进入这个函数。
        """

        self.pair_count += 1

        # LaserScan 自己携带的采样时间
        scan_time = self.stamp_to_seconds(
            scan_msg.header.stamp
        )

        # Image 自己携带的采样时间
        image_time = self.stamp_to_seconds(
            image_msg.header.stamp
        )

        # 两者时间差
        diff_seconds = abs(
            scan_time - image_time
        )

        diff_ms = diff_seconds * 1000.0

        self.time_diffs_ms.append(diff_ms)

        self.get_logger().info(
            f'pair #{self.pair_count:03d} | '
            f'scan={scan_time:.6f} s | '
            f'image={image_time:.6f} s | '
            f'dt={diff_ms:.3f} ms'
        )

    def print_summary(self):
        """
        打印本次 slop 实验的统计结果。
        """

        print()
        print('=' * 60)
        print('Synchronization experiment summary')
        print('=' * 60)

        print(
            f'slop       : '
            f'{self.slop * 1000.0:.1f} ms'
        )

        print(
            f'queue_size : '
            f'{self.queue_size}'
        )

        print(
            f'pairs      : '
            f'{self.pair_count}'
        )

        # 一对都没有匹配到时，不能计算统计量。
        if not self.time_diffs_ms:
            print('No synchronized pairs were found.')
            print('=' * 60)
            return

        median_ms = statistics.median(
            self.time_diffs_ms
        )

        mean_ms = statistics.mean(
            self.time_diffs_ms
        )

        min_ms = min(
            self.time_diffs_ms
        )

        max_ms = max(
            self.time_diffs_ms
        )

        print(
            f'median dt  : '
            f'{median_ms:.3f} ms'
        )

        print(
            f'mean dt    : '
            f'{mean_ms:.3f} ms'
        )

        print(
            f'min dt     : '
            f'{min_ms:.3f} ms'
        )

        print(
            f'max dt     : '
            f'{max_ms:.3f} ms'
        )

        print('=' * 60)
        print()


def main(args=None):
    rclpy.init(args=args)

    node = SyncTestNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        # Ctrl+C 后先输出实验统计结果
        node.print_summary()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()