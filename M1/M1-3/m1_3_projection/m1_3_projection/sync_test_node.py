#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, Image

from message_filters import (
    Subscriber,
    ApproximateTimeSynchronizer,
)


class SyncTestNode(Node):
    """
    用于观察 /scan 与 /camera/image_raw 的近似时间同步效果。

    本节点暂时不做：
      - TF
      - 激光点转换
      - 图像投影

    只做一件事情：

        /scan
             \
              -> ApproximateTimeSynchronizer
             /
        /camera/image_raw

    每成功配对一次，就打印两条消息的时间戳以及时间差。
    """

    def __init__(self):
        super().__init__('sync_test_node')

        # ----------------------------------------------------------
        # ROS 参数
        # ----------------------------------------------------------
        #
        # slop：
        #   允许两条消息之间最大的时间差，单位是秒。
        #
        # 默认 0.05 秒，也就是 50 ms。
        #
        self.declare_parameter('slop', 0.05)
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

        # 用来统计成功匹配了多少对
        self.pair_count = 0

        # 用于累计时间差，后面可以计算平均值等
        self.time_diffs_ms = []

        # ----------------------------------------------------------
        # message_filters Subscriber
        # ----------------------------------------------------------
        #
        # 注意这里没有使用普通的：
        #
        #   self.create_subscription(...)
        #
        # 因为我们想让 message_filters 接管消息，
        # 再把两路消息进行时间匹配。
        #
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
        # ApproximateTimeSynchronizer
        # ----------------------------------------------------------
        #
        # fs:
        #   需要同步的多个订阅器。
        #
        # queue_size:
        #   每一路最多保存多少条等待匹配的消息。
        #
        # slop:
        #   两条消息最大允许时间差，单位：秒。
        #
        self.synchronizer = ApproximateTimeSynchronizer(
            [
                self.scan_sub,
                self.image_sub,
            ],
            queue_size=self.queue_size,
            slop=self.slop,
        )

        # 一旦找到可以配成一对的 scan 和 image，
        # 就调用 synchronized_callback。
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
        ROS 时间戳：

            sec
            nanosec

        转换成浮点秒：

            sec + nanosec / 1e9
        """

        return (
            float(stamp.sec)
            + float(stamp.nanosec) * 1e-9
        )

    def synchronized_callback(self, scan_msg, image_msg):
        """
        只有 ApproximateTimeSynchronizer 成功匹配到：

            1 个 LaserScan
            +
            1 个 Image

        才会进入这个函数。
        """

        self.pair_count += 1

        scan_time = self.stamp_to_seconds(
            scan_msg.header.stamp
        )

        image_time = self.stamp_to_seconds(
            image_msg.header.stamp
        )

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


def main(args=None):
    rclpy.init(args=args)

    node = SyncTestNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()