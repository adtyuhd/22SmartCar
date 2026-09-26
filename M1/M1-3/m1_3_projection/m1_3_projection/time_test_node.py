#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan


class TimeTestNode(Node):
    """
    用来演示 ROS 2 的 use_sim_time。

    比较：

        node clock
        scan.header.stamp

    当 rosbag 使用 --clock 播放时：

        use_sim_time = true
            node clock 使用 /clock

        use_sim_time = false
            node clock 使用系统时间
    """

    def __init__(self):
        super().__init__('time_test_node')

        self.scan_count = 0

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10,
        )

        use_sim_time = (
            self.get_parameter('use_sim_time')
            .get_parameter_value()
            .bool_value
        )

        self.get_logger().info(
            f'use_sim_time = {use_sim_time}'
        )

        self.get_logger().info(
            'Waiting for /scan...'
        )

    @staticmethod
    def stamp_to_seconds(stamp):
        return (
            float(stamp.sec)
            + float(stamp.nanosec) * 1e-9
        )

    def scan_callback(self, scan_msg):
        self.scan_count += 1

        node_now = (
            self.get_clock()
            .now()
            .nanoseconds
            / 1e9
        )

        scan_stamp = self.stamp_to_seconds(
            scan_msg.header.stamp
        )

        difference = (
            node_now - scan_stamp
        )

        print(
            f'scan #{self.scan_count:03d} | '
            f'node_clock={node_now:.6f} s | '
            f'scan_stamp={scan_stamp:.6f} s | '
            f'difference={difference:.6f} s'
        )


def main(args=None):
    rclpy.init(args=args)

    node = TimeTestNode()

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