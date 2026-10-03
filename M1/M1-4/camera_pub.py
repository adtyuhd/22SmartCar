#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""相机节点 —— 终端1运行

发布:
    /camera/image_raw
    640x480 bgr8
    目标 30Hz
"""

import time

import numpy as np
import rclpy

from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image


IMAGE_IN = "/camera/image_raw"

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

CAMERA_HZ = 30.0
QUEUE_DEPTH = 30


def make_qos():
    return QoSProfile(
        depth=QUEUE_DEPTH,
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
    )


class CameraSim(Node):
    def __init__(self):
        super().__init__("camera_sim")

        self.pub = self.create_publisher(
            Image,
            IMAGE_IN,
            make_qos(),
        )

        self.bridge = CvBridge()

        self.seq = 0

        # -----------------------------
        # 最近 3 秒发布频率统计
        # -----------------------------
        self.count = 0
        self.t_start = time.monotonic()

        # -----------------------------
        # 预计算基础图案
        # -----------------------------
        x = np.arange(
            IMAGE_WIDTH,
            dtype=np.uint16,
        )

        y = np.arange(
            IMAGE_HEIGHT,
            dtype=np.uint16,
        ).reshape(-1, 1)

        self.base_pattern = (
            (x + y * 13) & 0xFF
        ).astype(np.uint8)

        # 图像缓冲区只创建一次
        self.frame = np.empty(
            (
                IMAGE_HEIGHT,
                IMAGE_WIDTH,
                3,
            ),
            dtype=np.uint8,
        )

        # 第三个通道固定为 60
        self.frame[:, :, 2] = 60

        # 30Hz 相机定时器
        self.timer = self.create_timer(
            1.0 / CAMERA_HZ,
            self.on_timer,
        )

        # 每 3 秒打印当前窗口频率
        self.report_timer = self.create_timer(
            3.0,
            self.report,
        )

    def capture(self):
        """
        原来的公式：

        off = (y * 13 + seq) & 0xFF
        value = (x + off) & 0xFF

        等价于：

        value = (x + y*13 + seq) & 0xFF

        使用 NumPy 向量化完成。
        """

        seq_offset = np.uint8(
            self.seq & 0xFF
        )

        np.add(
            self.base_pattern,
            seq_offset,
            out=self.frame[:, :, 0],
        )

        self.frame[:, :, 1] = (
            self.frame[:, :, 0]
        )

        return self.frame

    def on_timer(self):
        self.seq += 1

        frame = self.capture()

        msg = self.bridge.cv2_to_imgmsg(
            frame,
            encoding="bgr8",
        )

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = "camera_link"

        self.pub.publish(msg)

        self.count += 1

    def report(self):
        now = time.monotonic()

        elapsed = (
            now - self.t_start
        )

        if elapsed <= 0:
            return

        hz = (
            self.count / elapsed
        )

        self.get_logger().info(
            "[CAMERA] 最近 %.2f 秒发布 %d 帧, %.2f Hz (目标 %.1f Hz)"
            % (
                elapsed,
                self.count,
                hz,
                CAMERA_HZ,
            )
        )

        # 关键：
        # 每次打印以后重新开始统计，
        # 不再使用从启动以来的累计平均值。
        self.count = 0
        self.t_start = now


def main():
    rclpy.init()

    node = CameraSim()

    try:
        rclpy.spin(node)

    except (
        KeyboardInterrupt,
        rclpy.executors.ExternalShutdownException,
    ):
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()