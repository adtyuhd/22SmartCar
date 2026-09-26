#!/usr/bin/env python3

import math
import os

import rclpy
from rclpy.node import Node

import yaml

from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class TfStaticPublisher(Node):
    """
    从 extrinsics.yaml 读取传感器外参，并发布静态 TF：

        map
         |
        odom
         |
        base_link
        /       \
    laser_link  camera_link
    """

    def __init__(self):
        super().__init__('tf_static_publisher')

        # --------------------------------------------------------------
        # 1. 声明 ROS 参数
        # --------------------------------------------------------------
        #
        # 默认值按照题目接口契约要求为：
        #
        #   extrinsics_file = extrinsics.yaml
        #
        # 运行时可以用：
        #
        #   --ros-args -p extrinsics_file:=/path/to/extrinsics.yaml
        #
        # 来覆盖。
        #
        self.declare_parameter('extrinsics_file', 'extrinsics.yaml')

        extrinsics_file = (
            self.get_parameter('extrinsics_file')
            .get_parameter_value()
            .string_value
        )

        # 如果用户传进来的是相对路径，
        # 就相对于启动节点时的当前工作目录解析。
        extrinsics_file = os.path.abspath(extrinsics_file)

        self.get_logger().info(
            f'Loading extrinsics from: {extrinsics_file}'
        )

        # --------------------------------------------------------------
        # 2. 读取 YAML
        # --------------------------------------------------------------
        config = self.load_extrinsics(extrinsics_file)

        # --------------------------------------------------------------
        # 3. 从 YAML 获取 frame 名称
        # --------------------------------------------------------------
        frames = config['frames']

        frame_map = frames['map']
        frame_odom = frames['odom']
        frame_base = frames['base_link']
        frame_laser = frames['laser_link']
        frame_camera = frames['camera_link']

        # --------------------------------------------------------------
        # 4. 读取 laser_link 外参
        # --------------------------------------------------------------
        laser_config = config['laser_link']

        laser_parent = laser_config['parent']
        laser_xyz = laser_config['xyz']
        laser_rpy = laser_config['rpy']

        # --------------------------------------------------------------
        # 5. 读取 camera_link 外参
        # --------------------------------------------------------------
        camera_config = config['camera_link']

        camera_parent = camera_config['parent']
        camera_xyz = camera_config['xyz']
        camera_rpy = camera_config['rpy']

        # --------------------------------------------------------------
        # 6. 做一些基本检查
        # --------------------------------------------------------------
        #
        # 题目要求的树是：
        #
        #   base_link -> laser_link
        #   base_link -> camera_link
        #
        # 因此这里检查 YAML 中的 parent 是否符合要求。
        #
        if laser_parent != frame_base:
            raise ValueError(
                f'laser_link parent should be "{frame_base}", '
                f'but YAML contains "{laser_parent}"'
            )

        if camera_parent != frame_base:
            raise ValueError(
                f'camera_link parent should be "{frame_base}", '
                f'but YAML contains "{camera_parent}"'
            )

        # --------------------------------------------------------------
        # 7. 创建 StaticTransformBroadcaster
        # --------------------------------------------------------------
        #
        # 普通动态 TF 使用 TransformBroadcaster，
        # 静态安装关系使用 StaticTransformBroadcaster。
        #
        # 它会发布到：
        #
        #   /tf_static
        #
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # 所有静态 TF 使用同一个时间戳。
        stamp = self.get_clock().now().to_msg()

        transforms = []

        # --------------------------------------------------------------
        # 8. map -> odom
        # --------------------------------------------------------------
        #
        # 当前合成数据没有提供车辆地图定位与里程计运动，
        # 因此这里使用单位变换。
        #
        transforms.append(
            self.make_transform(
                parent=frame_map,
                child=frame_odom,
                xyz=[0.0, 0.0, 0.0],
                rpy=[0.0, 0.0, 0.0],
                stamp=stamp,
            )
        )

        # --------------------------------------------------------------
        # 9. odom -> base_link
        # --------------------------------------------------------------
        #
        # 当前合成数据没有模拟车辆运动，
        # 同样使用单位变换。
        #
        transforms.append(
            self.make_transform(
                parent=frame_odom,
                child=frame_base,
                xyz=[0.0, 0.0, 0.0],
                rpy=[0.0, 0.0, 0.0],
                stamp=stamp,
            )
        )

        # --------------------------------------------------------------
        # 10. base_link -> laser_link
        # --------------------------------------------------------------
        #
        # 注意：
        # 这里绝对不能写死 [0.15, 0.0, 0.20]。
        #
        # 必须使用刚才从 extrinsics.yaml 读取的 laser_xyz
        # 和 laser_rpy。
        #
        transforms.append(
            self.make_transform(
                parent=laser_parent,
                child=frame_laser,
                xyz=laser_xyz,
                rpy=laser_rpy,
                stamp=stamp,
            )
        )

        # --------------------------------------------------------------
        # 11. base_link -> camera_link
        # --------------------------------------------------------------
        transforms.append(
            self.make_transform(
                parent=camera_parent,
                child=frame_camera,
                xyz=camera_xyz,
                rpy=camera_rpy,
                stamp=stamp,
            )
        )

        # --------------------------------------------------------------
        # 12. 一次发布全部静态 TF
        # --------------------------------------------------------------
        self.static_broadcaster.sendTransform(transforms)

        self.get_logger().info('Static TF tree published:')
        self.get_logger().info(
            f'  {frame_map} -> {frame_odom}'
        )
        self.get_logger().info(
            f'  {frame_odom} -> {frame_base}'
        )
        self.get_logger().info(
            f'  {frame_base} -> {frame_laser} '
            f'xyz={laser_xyz} rpy={laser_rpy}'
        )
        self.get_logger().info(
            f'  {frame_base} -> {frame_camera} '
            f'xyz={camera_xyz} rpy={camera_rpy}'
        )

    def load_extrinsics(self, path):
        """
        读取并检查 extrinsics.yaml。
        """

        if not os.path.isfile(path):
            raise FileNotFoundError(
                f'Extrinsics file does not exist: {path}'
            )

        with open(path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)

        if not isinstance(config, dict):
            raise ValueError(
                'extrinsics.yaml root must be a YAML dictionary'
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
                    f'Missing required section in YAML: {section}'
                )

        return config

    def make_transform(
        self,
        parent,
        child,
        xyz,
        rpy,
        stamp,
    ):
        """
        根据 parent / child / xyz / rpy 创建 TransformStamped。

        xyz:
            [x, y, z]，单位：米

        rpy:
            [roll, pitch, yaw]，单位：弧度
        """

        if len(xyz) != 3:
            raise ValueError(
                f'xyz for {child} must contain exactly 3 values'
            )

        if len(rpy) != 3:
            raise ValueError(
                f'rpy for {child} must contain exactly 3 values'
            )

        x, y, z = [float(v) for v in xyz]
        roll, pitch, yaw = [float(v) for v in rpy]

        qx, qy, qz, qw = self.rpy_to_quaternion(
            roll,
            pitch,
            yaw,
        )

        transform = TransformStamped()

        # header.frame_id 是父坐标系
        transform.header.frame_id = parent

        # child_frame_id 是子坐标系
        transform.child_frame_id = child

        transform.header.stamp = stamp

        # 平移
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z

        # 旋转：ROS TF 使用四元数
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        return transform

    @staticmethod
    def rpy_to_quaternion(roll, pitch, yaw):
        """
        将 roll / pitch / yaw 转换成四元数。

        输入单位：弧度

        返回：
            qx, qy, qz, qw
        """

        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return qx, qy, qz, qw


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = TfStaticPublisher()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()