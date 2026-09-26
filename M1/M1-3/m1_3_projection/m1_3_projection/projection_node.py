#!/usr/bin/env python3

import math
import os

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

import yaml

from PIL import Image as PilImage

from message_filters import (
    Subscriber,
    ApproximateTimeSynchronizer,
)

from sensor_msgs.msg import (
    CameraInfo,
    Image,
    LaserScan,
)

from tf2_ros import (
    Buffer,
    TransformException,
    TransformListener,
)


class ProjectionNode(Node):
    """
    M1-3 激光雷达到相机图像投影节点。

    功能：

        /scan
        /camera/image_raw
                ↓
        ApproximateTimeSynchronizer
                ↓
        LaserScan 极坐标 -> laser_link
                ↓
        TF2 -> camera_link
                ↓
        camera_link -> optical coordinates
                ↓
        pinhole projection
                ↓
        距离着色
                ↓
        /projection/debug_image

    同时保存最多 max_images 张 projection_*.png。
    """

    def __init__(self):
        super().__init__('projection_node')

        # ==========================================================
        # 1. ROS 参数
        # ==========================================================

        self.declare_parameter(
            'extrinsics_file',
            'extrinsics.yaml',
        )

        self.declare_parameter(
            'output_dir',
            '.',
        )

        self.declare_parameter(
            'max_images',
            3,
        )

        self.declare_parameter(
            'slop',
            0.02,
        )

        self.declare_parameter(
            'queue_size',
            50,
        )

        self.extrinsics_file = (
            self.get_parameter('extrinsics_file')
            .get_parameter_value()
            .string_value
        )

        self.output_dir = (
            self.get_parameter('output_dir')
            .get_parameter_value()
            .string_value
        )

        self.max_images = (
            self.get_parameter('max_images')
            .get_parameter_value()
            .integer_value
        )

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

        if self.max_images < 0:
            raise ValueError(
                'max_images must be >= 0'
            )

        # ==========================================================
        # 2. YAML
        # ==========================================================

        self.extrinsics_file = os.path.abspath(
            self.extrinsics_file
        )

        config = self.load_config(
            self.extrinsics_file
        )

        frames = config['frames']

        self.camera_frame = frames['camera_link']

        intrinsics = config['camera_intrinsics']

        self.fx = float(
            intrinsics['fx']
        )

        self.fy = float(
            intrinsics['fy']
        )

        self.cx = float(
            intrinsics['cx']
        )

        self.cy = float(
            intrinsics['cy']
        )

        self.image_width = int(
            intrinsics['width']
        )

        self.image_height = int(
            intrinsics['height']
        )

        # ==========================================================
        # 3. 输出目录
        # ==========================================================

        self.output_dir = os.path.abspath(
            self.output_dir
        )

        os.makedirs(
            self.output_dir,
            exist_ok=True,
        )

        # ==========================================================
        # 4. TF2
        # ==========================================================

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        # ==========================================================
        # 5. CameraInfo
        # ==========================================================

        self.camera_info_received = False

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10,
        )

        # ==========================================================
        # 6. ApproximateTimeSynchronizer
        # ==========================================================

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

        self.synchronizer = ApproximateTimeSynchronizer(
            [
                self.scan_sub,
                self.image_sub,
            ],
            queue_size=self.queue_size,
            slop=self.slop,
        )

        self.synchronizer.registerCallback(
            self.synchronized_callback
        )

        # ==========================================================
        # 7. Publisher
        # ==========================================================

        self.debug_image_pub = self.create_publisher(
            Image,
            '/projection/debug_image',
            10,
        )

        # ==========================================================
        # 8. 状态
        # ==========================================================

        self.pair_count = 0
        self.saved_image_count = 0

        # 上一次保存图片对应的 scan 时间。
        self.last_saved_stamp_sec = None

        # 为了避免 3 张图片全部来自连续的 0.1 秒，
        # 默认让保存图片之间至少相隔 1 秒。
        #
        # 这不是传感器参数，也不是相机/雷达标定参数，
        # 只是 debug 输出采样策略。
        self.save_interval_sec = 1.0

        # ==========================================================
        # 9. 启动信息
        # ==========================================================

        self.get_logger().info(
            'Projection node started'
        )

        self.get_logger().info(
            f'Extrinsics file: '
            f'{self.extrinsics_file}'
        )

        self.get_logger().info(
            'Camera intrinsics: '
            f'fx={self.fx:.3f}, '
            f'fy={self.fy:.3f}, '
            f'cx={self.cx:.3f}, '
            f'cy={self.cy:.3f}'
        )

        self.get_logger().info(
            'Expected image size: '
            f'{self.image_width}x'
            f'{self.image_height}'
        )

        self.get_logger().info(
            f'ATS slop: '
            f'{self.slop * 1000.0:.1f} ms'
        )

        self.get_logger().info(
            f'ATS queue_size: '
            f'{self.queue_size}'
        )

        self.get_logger().info(
            f'Output directory: '
            f'{self.output_dir}'
        )

        self.get_logger().info(
            f'Maximum saved images: '
            f'{self.max_images}'
        )

    # ==============================================================
    # YAML
    # ==============================================================

    @staticmethod
    def load_config(path):
        if not os.path.isfile(path):
            raise FileNotFoundError(
                f'Extrinsics file does not exist: {path}'
            )

        with open(
            path,
            'r',
            encoding='utf-8',
        ) as f:
            config = yaml.safe_load(f)

        if not isinstance(config, dict):
            raise ValueError(
                'extrinsics.yaml root must be a dictionary'
            )

        required_sections = [
            'frames',
            'laser_link',
            'camera_link',
            'camera_intrinsics',
        ]

        for section in required_sections:
            if section not in config:
                raise ValueError(
                    f'Missing YAML section: {section}'
                )

        required_intrinsics = [
            'fx',
            'fy',
            'cx',
            'cy',
            'width',
            'height',
        ]

        for key in required_intrinsics:
            if key not in config['camera_intrinsics']:
                raise ValueError(
                    f'Missing camera intrinsic: {key}'
                )

        return config

    # ==============================================================
    # CameraInfo
    # ==============================================================

    def camera_info_callback(self, msg):
        if self.camera_info_received:
            return

        self.camera_info_received = True

        self.get_logger().info(
            'Received /camera/camera_info: '
            f'{msg.width}x{msg.height}, '
            f'frame={msg.header.frame_id}'
        )

        if (
            int(msg.width) != self.image_width
            or int(msg.height) != self.image_height
        ):
            self.get_logger().warning(
                'CameraInfo image size differs '
                'from extrinsics.yaml'
            )

    # ==============================================================
    # Main synchronized callback
    # ==============================================================

    def synchronized_callback(
        self,
        scan_msg,
        image_msg,
    ):
        self.pair_count += 1

        # ----------------------------------------------------------
        # 1. 输入图像检查
        # ----------------------------------------------------------

        if image_msg.encoding != 'rgb8':
            self.get_logger().warning(
                'Expected rgb8 input image, '
                f'but received {image_msg.encoding}'
            )
            return

        if (
            image_msg.width != self.image_width
            or image_msg.height != self.image_height
        ):
            self.get_logger().warning(
                'Image dimensions differ from '
                'extrinsics.yaml'
            )
            return

        # ----------------------------------------------------------
        # 2. TF lookup
        # ----------------------------------------------------------

        source_frame = scan_msg.header.frame_id

        scan_time = Time.from_msg(
            scan_msg.header.stamp
        )

        try:
            transform = self.tf_buffer.lookup_transform(
                self.camera_frame,
                source_frame,
                scan_time,
                timeout=Duration(seconds=0.1),
            )

        except TransformException as ex:
            self.get_logger().warning(
                f'TF lookup failed: {ex}'
            )
            return

        # ----------------------------------------------------------
        # 3. 原图副本
        # ----------------------------------------------------------

        output_data = bytearray(
            image_msg.data
        )

        projected_count = 0

        # ----------------------------------------------------------
        # 4. 遍历 LaserScan
        # ----------------------------------------------------------

        for i, range_value in enumerate(
            scan_msg.ranges
        ):
            r = float(
                range_value
            )

            if not math.isfinite(r):
                continue

            if (
                r < scan_msg.range_min
                or r > scan_msg.range_max
            ):
                continue

            theta = (
                scan_msg.angle_min
                + i * scan_msg.angle_increment
            )

            # laser_link
            x_laser = r * math.cos(theta)
            y_laser = r * math.sin(theta)
            z_laser = 0.0

            # laser_link -> camera_link
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

            # camera_link -> optical
            x_opt = -y_camera
            y_opt = -z_camera
            z_opt = x_camera

            # 相机后方点丢弃。
            if z_opt <= 0.0:
                continue

            # pinhole projection
            u_float = (
                self.fx * x_opt / z_opt
                + self.cx
            )

            v_float = (
                self.fy * y_opt / z_opt
                + self.cy
            )

            u = int(
                round(u_float)
            )

            v = int(
                round(v_float)
            )

            if (
                u < 0
                or u >= image_msg.width
                or v < 0
                or v >= image_msg.height
            ):
                continue

            # ------------------------------------------------------
            # 5. 根据距离得到 RGB
            # ------------------------------------------------------

            red, green, blue = self.range_to_rgb(
                r,
                scan_msg.range_min,
                scan_msg.range_max,
            )

            # ------------------------------------------------------
            # 6. 画点
            # ------------------------------------------------------

            self.draw_point(
                output_data,
                image_msg,
                u,
                v,
                red,
                green,
                blue,
            )

            projected_count += 1

        # ----------------------------------------------------------
        # 7. 构造 debug Image
        # ----------------------------------------------------------

        debug_msg = Image()

        debug_msg.header = image_msg.header

        debug_msg.height = image_msg.height
        debug_msg.width = image_msg.width

        debug_msg.encoding = 'rgb8'

        debug_msg.is_bigendian = image_msg.is_bigendian
        debug_msg.step = image_msg.step

        debug_msg.data = bytes(
            output_data
        )

        # ----------------------------------------------------------
        # 8. 发布
        # ----------------------------------------------------------

        self.debug_image_pub.publish(
            debug_msg
        )

        # ----------------------------------------------------------
        # 9. 按时间间隔保存 PNG
        # ----------------------------------------------------------

        saved_path = self.maybe_save_image(
            debug_msg
        )

        log_text = (
            f'pair #{self.pair_count:03d}: '
            f'projected {projected_count} '
            f'laser points'
        )

        if saved_path is not None:
            log_text += (
                f' | saved {saved_path}'
            )

        self.get_logger().info(
            log_text
        )

    # ==============================================================
    # Distance -> RGB
    # ==============================================================

    @staticmethod
    def range_to_rgb(
        distance,
        range_min,
        range_max,
    ):
        """
        将距离线性映射到：

            near -> red
            far  -> blue
        """

        denominator = (
            float(range_max)
            - float(range_min)
        )

        if denominator <= 0.0:
            return 255, 0, 0

        alpha = (
            float(distance)
            - float(range_min)
        ) / denominator

        alpha = max(
            0.0,
            min(
                1.0,
                alpha,
            ),
        )

        red = int(
            round(
                255.0 * (1.0 - alpha)
            )
        )

        green = 0

        blue = int(
            round(
                255.0 * alpha
            )
        )

        return (
            red,
            green,
            blue,
        )

    # ==============================================================
    # Drawing
    # ==============================================================

    @staticmethod
    def draw_point(
        image_data,
        image_msg,
        center_u,
        center_v,
        red,
        green,
        blue,
    ):
        """
        在 RGB8 图像上画 5x5 点。
        """

        radius = 2

        for dv in range(
            -radius,
            radius + 1,
        ):
            for du in range(
                -radius,
                radius + 1,
            ):
                u = center_u + du
                v = center_v + dv

                if (
                    u < 0
                    or u >= image_msg.width
                    or v < 0
                    or v >= image_msg.height
                ):
                    continue

                offset = (
                    v * image_msg.step
                    + u * 3
                )

                image_data[offset] = red
                image_data[offset + 1] = green
                image_data[offset + 2] = blue

    # ==============================================================
    # Save PNG
    # ==============================================================

    def maybe_save_image(
        self,
        image_msg,
    ):
        """
        最多保存 max_images 张图片。

        相邻保存图片之间至少相隔 save_interval_sec，
        保证不是连续几帧几乎完全一样的结果。
        """

        if (
            self.saved_image_count
            >= self.max_images
        ):
            return None

        stamp_sec = (
            float(image_msg.header.stamp.sec)
            + float(
                image_msg.header.stamp.nanosec
            ) * 1e-9
        )

        if self.last_saved_stamp_sec is not None:
            elapsed = (
                stamp_sec
                - self.last_saved_stamp_sec
            )

            if elapsed < self.save_interval_sec:
                return None

        next_index = (
            self.saved_image_count + 1
        )

        filename = (
            f'projection_{next_index:03d}.png'
        )

        output_path = os.path.join(
            self.output_dir,
            filename,
        )

        # ROS rgb8 数据中每行实际占 image_msg.step bytes。
        #
        # 当前生成器的 RGB8 是紧密排列的：
        #
        #     step = width * 3
        #
        # 为避免悄悄处理错误格式，这里显式检查。
        expected_step = (
            image_msg.width * 3
        )

        if image_msg.step != expected_step:
            self.get_logger().warning(
                'Cannot save PNG because RGB8 '
                f'step={image_msg.step}, '
                f'expected {expected_step}'
            )

            return None

        pil_image = PilImage.frombytes(
            'RGB',
            (
                image_msg.width,
                image_msg.height,
            ),
            bytes(
                image_msg.data
            ),
        )

        pil_image.save(
            output_path
        )

        self.saved_image_count += 1
        self.last_saved_stamp_sec = stamp_sec

        return output_path

    # ==============================================================
    # TF transformation
    # ==============================================================

    @staticmethod
    def transform_point(
        x,
        y,
        z,
        transform,
    ):
        """
        P_target = R * P_source + t
        """

        q = transform.transform.rotation
        t = transform.transform.translation

        qx = float(q.x)
        qy = float(q.y)
        qz = float(q.z)
        qw = float(q.w)

        norm = math.sqrt(
            qx * qx
            + qy * qy
            + qz * qz
            + qw * qw
        )

        if norm == 0.0:
            raise ValueError(
                'Received zero-length quaternion'
            )

        qx /= norm
        qy /= norm
        qz /= norm
        qw /= norm

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

        return (
            rx + float(t.x),
            ry + float(t.y),
            rz + float(t.z),
        )


def main(args=None):
    rclpy.init(args=args)

    node = ProjectionNode()

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