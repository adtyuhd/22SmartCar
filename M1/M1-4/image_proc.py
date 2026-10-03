#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""图像处理与控制节点 —— 终端2 运行"""

import time

import numpy as np
import rclpy

from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image


IMAGE_IN = "/camera/image_raw"
IMAGE_OUT = "/perception/result"
CMD_OUT = "/control/cmd"

CONTROL_HZ = 20.0


def make_image_sub_qos():
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
    )


def make_output_qos():
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
    )


class ImageProc(Node):
    def __init__(self):
        super().__init__("image_proc")

        self.sub = self.create_subscription(
            Image,
            IMAGE_IN,
            self.on_image,
            make_image_sub_qos(),
        )

        self.pub = self.create_publisher(
            Image,
            IMAGE_OUT,
            make_output_qos(),
        )

        self.bridge = CvBridge()

        self.count = 0

        self.convert_ms = 0.0
        self.process_ms = 0.0
        self.build_ms = 0.0
        self.publish_ms = 0.0
        self.total_ms = 0.0

        self.t_start = time.monotonic()

        self.report_timer = self.create_timer(
            3.0,
            self.report,
        )

    def on_image(self, msg):
        callback_start = time.perf_counter()

        # =============================
        # 1. ROS Image -> NumPy
        # =============================
        t0 = time.perf_counter()

        gray = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="mono8",
        )

        t1 = time.perf_counter()

        # =============================
        # 2. 图像处理
        # =============================

        work = np.array(
            gray,
            dtype=np.int16,
            copy=True,
        )

        # 提亮 +20
        work += 20

        np.clip(
            work,
            0,
            255,
            out=work,
        )

        # 对比度增强
        # (v - 128) * 3 // 2 + 128
        work -= 128
        work *= 3
        work //= 2
        work += 128

        np.clip(
            work,
            0,
            255,
            out=work,
        )

        payload = work.astype(
            np.uint8
        )

        t2 = time.perf_counter()

        # =============================
        # 3. NumPy -> ROS Image
        # =============================

        out = self.bridge.cv2_to_imgmsg(
            payload,
            encoding="mono8",
        )

        # 必须保留输入时间戳
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id

        t3 = time.perf_counter()

        # =============================
        # 4. 发布
        # =============================

        self.pub.publish(out)

        t4 = time.perf_counter()

        # =============================
        # 性能统计
        # =============================

        self.count += 1

        self.convert_ms += (
            t1 - t0
        ) * 1000.0

        self.process_ms += (
            t2 - t1
        ) * 1000.0

        self.build_ms += (
            t3 - t2
        ) * 1000.0

        self.publish_ms += (
            t4 - t3
        ) * 1000.0

        self.total_ms += (
            time.perf_counter()
            - callback_start
        ) * 1000.0

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

        if self.count > 0:
            convert_avg = (
                self.convert_ms
                / self.count
            )

            process_avg = (
                self.process_ms
                / self.count
            )

            build_avg = (
                self.build_ms
                / self.count
            )

            publish_avg = (
                self.publish_ms
                / self.count
            )

            total_avg = (
                self.total_ms
                / self.count
            )

        else:
            convert_avg = 0.0
            process_avg = 0.0
            build_avg = 0.0
            publish_avg = 0.0
            total_avg = 0.0

        self.get_logger().info(
            "[PROC] %.2f Hz | "
            "convert %.2f ms | "
            "process %.2f ms | "
            "build %.2f ms | "
            "publish %.2f ms | "
            "total %.2f ms"
            % (
                hz,
                convert_avg,
                process_avg,
                build_avg,
                publish_avg,
                total_avg,
            )
        )

        self.count = 0

        self.convert_ms = 0.0
        self.process_ms = 0.0
        self.build_ms = 0.0
        self.publish_ms = 0.0
        self.total_ms = 0.0

        self.t_start = now


class ControlLoop(Node):
    def __init__(self):
        super().__init__("control_loop")

        self.pub = self.create_publisher(
            Twist,
            CMD_OUT,
            make_output_qos(),
        )

        self.timer = self.create_timer(
            1.0 / CONTROL_HZ,
            self.on_timer,
        )

    def on_timer(self):
        cmd = Twist()

        cmd.linear.x = 0.10
        cmd.angular.z = 0.05

        self.pub.publish(cmd)


def main():
    rclpy.init()

    nodes = [
        ImageProc(),
        ControlLoop(),
    ]

    # 这一步只改变执行器：
    # 单线程 -> 两个 worker 的多线程执行器
    executor = MultiThreadedExecutor(
        num_threads=2,
    )

    for node in nodes:
        executor.add_node(node)

    try:
        executor.spin()

    except (
        KeyboardInterrupt,
        rclpy.executors.ExternalShutdownException,
    ):
        pass

    finally:
        executor.shutdown()

        for node in nodes:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()