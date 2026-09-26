#!/usr/bin/env python3

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import LaserScan

from tf2_ros import (
    Buffer,
    TransformListener,
    TransformException,
)


class TransformTestNode(Node):
    """
    学习节点：

        LaserScan
            ↓
        极坐标 (r, theta)
            ↓
        laser_link 中的 (x, y, z)
            ↓
        TF2
            ↓
        camera_link 中的 (x, y, z)

    本节点只打印一帧中若干具有代表性的激光点。
    暂时不做图像投影。
    """

    def __init__(self):
        super().__init__('transform_test_node')

        # ----------------------------------------------------------
        # 1. TF2 Buffer
        # ----------------------------------------------------------
        #
        # Buffer：
        #   存储接收到的 TF 坐标关系。
        #
        # TransformListener：
        #   自动监听 /tf 和 /tf_static，
        #   并把收到的变换放进 Buffer。
        #
        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        # ----------------------------------------------------------
        # 2. 订阅 LaserScan
        # ----------------------------------------------------------
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10,
        )

        # 成功打印一帧后就不再重复打印。
        self.finished = False

        self.get_logger().info(
            'Transform test node started'
        )

        self.get_logger().info(
            'Waiting for /scan and TF...'
        )

    def scan_callback(self, scan_msg):
        """
        收到 LaserScan 后：

        1. 查 laser_link -> camera_link TF
        2. 选几个代表性激光点
        3. 极坐标转 Cartesian
        4. 应用 TF
        5. 打印结果
        """

        if self.finished:
            return

        # LaserScan 自己告诉我们它属于哪个坐标系。
        source_frame = scan_msg.header.frame_id

        target_frame = 'camera_link'

        # ----------------------------------------------------------
        # 3. 查询 TF
        # ----------------------------------------------------------
        #
        # 注意参数顺序：
        #
        # lookup_transform(
        #     target,
        #     source,
        #     time,
        # )
        #
        # 我们现在要：
        #
        # laser_link 中的点
        #       ↓
        # camera_link 中的点
        #
        # 所以：
        #
        # target = camera_link
        # source = laser_link
        #
        scan_time = Time.from_msg(
            scan_msg.header.stamp
        )

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                scan_time,
                timeout=Duration(seconds=1.0),
            )

        except TransformException as ex:
            self.get_logger().warning(
                f'Cannot transform '
                f'{source_frame} -> {target_frame}: {ex}'
            )

            return

        # ----------------------------------------------------------
        # 4. 打印当前 TF
        # ----------------------------------------------------------

        t = transform.transform.translation
        q = transform.transform.rotation

        print()
        print('=' * 78)
        print('LaserScan -> camera_link transform test')
        print('=' * 78)

        print(
            f'source frame : {source_frame}'
        )

        print(
            f'target frame : {target_frame}'
        )

        print()

        print(
            'TF translation: '
            f'[{t.x:.3f}, {t.y:.3f}, {t.z:.3f}]'
        )

        print(
            'TF quaternion : '
            f'[{q.x:.6f}, {q.y:.6f}, '
            f'{q.z:.6f}, {q.w:.6f}]'
        )

        print()

        # ----------------------------------------------------------
        # 5. 选一些有代表性的 beam
        # ----------------------------------------------------------
        #
        # 180 根雷达线中：
        #
        #   0       最右边附近
        #   N/4     右前方
        #   N/2     正前方附近
        #   3N/4    左前方
        #   N-1     最左边附近
        #
        n = len(scan_msg.ranges)

        candidate_indices = [
            0,
            n // 4,
            n // 2 - 1,
            n // 2,
            3 * n // 4,
            n - 1,
        ]

        print(
            f'{"i":>4} '
            f'{"angle(deg)":>12} '
            f'{"range(m)":>10} '
            f'{"laser xyz":>25} '
            f'{"camera xyz":>25}'
        )

        print('-' * 90)

        for i in candidate_indices:

            if i < 0 or i >= n:
                continue

            r = float(
                scan_msg.ranges[i]
            )

            # ------------------------------------------------------
            # 6. 检查距离是否有效
            # ------------------------------------------------------

            if not math.isfinite(r):
                continue

            if (
                r < scan_msg.range_min
                or r > scan_msg.range_max
            ):
                continue

            # ------------------------------------------------------
            # 7. 计算这一束激光的角度
            # ------------------------------------------------------

            theta = (
                scan_msg.angle_min
                + i * scan_msg.angle_increment
            )

            # ------------------------------------------------------
            # 8. LaserScan 极坐标 -> laser_link Cartesian
            # ------------------------------------------------------

            x_laser = r * math.cos(theta)
            y_laser = r * math.sin(theta)
            z_laser = 0.0

            # ------------------------------------------------------
            # 9. laser_link -> camera_link
            # ------------------------------------------------------

            (
                x_camera,
                y_camera,
                z_camera,
            ) = self.transform_point(
                x_laser,
                y_laser,
                z_laser,
                transform,
            )

            print(
                f'{i:>4d} '
                f'{math.degrees(theta):>12.3f} '
                f'{r:>10.3f} '
                f'({x_laser:>6.3f},'
                f'{y_laser:>6.3f},'
                f'{z_laser:>6.3f}) '
                f'({x_camera:>6.3f},'
                f'{y_camera:>6.3f},'
                f'{z_camera:>6.3f})'
            )

        print('=' * 78)

        print()
        print(
            'For the default extrinsics, we expect approximately:'
        )

        print(
            '  x_camera = x_laser + 0.05'
        )

        print(
            '  y_camera = y_laser'
        )

        print(
            '  z_camera = z_laser - 0.10'
        )

        print()

        print(
            'This relationship is only for the current zero-rotation '
            'extrinsics.'
        )

        print(
            'The code itself uses the quaternion from TF, so it also '
            'supports non-zero camera/laser rotations.'
        )

        print('=' * 78)
        print()

        self.finished = True

    @staticmethod
    def transform_point(
        x,
        y,
        z,
        transform,
    ):
        """
        对一个三维点应用 geometry_msgs/TransformStamped：

            P_target = R * P_source + t

        其中：

            R = quaternion 对应的旋转矩阵
            t = translation
        """

        q = transform.transform.rotation
        t = transform.transform.translation

        qx = float(q.x)
        qy = float(q.y)
        qz = float(q.z)
        qw = float(q.w)

        # ----------------------------------------------------------
        # 四元数先归一化
        # ----------------------------------------------------------

        norm = math.sqrt(
            qx * qx
            + qy * qy
            + qz * qz
            + qw * qw
        )

        if norm == 0.0:
            raise ValueError(
                'Received invalid zero-length quaternion'
            )

        qx /= norm
        qy /= norm
        qz /= norm
        qw /= norm

        # ----------------------------------------------------------
        # quaternion -> 3x3 rotation matrix
        # ----------------------------------------------------------

        r00 = 1.0 - 2.0 * (
            qy * qy + qz * qz
        )

        r01 = 2.0 * (
            qx * qy - qz * qw
        )

        r02 = 2.0 * (
            qx * qz + qy * qw
        )

        r10 = 2.0 * (
            qx * qy + qz * qw
        )

        r11 = 1.0 - 2.0 * (
            qx * qx + qz * qz
        )

        r12 = 2.0 * (
            qy * qz - qx * qw
        )

        r20 = 2.0 * (
            qx * qz - qy * qw
        )

        r21 = 2.0 * (
            qy * qz + qx * qw
        )

        r22 = 1.0 - 2.0 * (
            qx * qx + qy * qy
        )

        # ----------------------------------------------------------
        # R * P
        # ----------------------------------------------------------

        rx = (
            r00 * x
            + r01 * y
            + r02 * z
        )

        ry = (
            r10 * x
            + r11 * y
            + r12 * z
        )

        rz = (
            r20 * x
            + r21 * y
            + r22 * z
        )

        # ----------------------------------------------------------
        # R * P + t
        # ----------------------------------------------------------

        x_target = rx + float(t.x)
        y_target = ry + float(t.y)
        z_target = rz + float(t.z)

        return (
            x_target,
            y_target,
            z_target,
        )


def main(args=None):
    rclpy.init(args=args)

    node = TransformTestNode()

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