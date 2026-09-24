#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class SquareDriver(Node):

    def __init__(self):
        super().__init__('square_driver')

        # 使用 Gazebo 发布的 /clock 作为时间源。
        self.set_parameters([
            Parameter('use_sim_time', Parameter.Type.BOOL, True)
        ])

        # -------------------------
        # Publisher / Subscriber
        # -------------------------

        # 向小车发送速度命令。
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # 订阅 /odom，用来读取车辆当前姿态。
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # 最近一次 /odom 给出的 yaw。
        # 单位：rad
        self.current_theta = None

        # 每次转弯开始时的 yaw。
        self.turn_start_theta = None

        # -------------------------
        # 路径参数
        # -------------------------

        # 目标圆角方形外包络边长：2.0 m。
        self.side_length = 2.0

        # 车辆线速度：0.2 m/s。
        self.linear_speed = 0.2

        # 圆角转弯半径：0.5 m。
        self.turn_radius = 0.5

        # 对于 2 m × 2 m 的圆角方形：
        #
        # straight_length
        # = side_length - 2 * turn_radius
        # = 2.0 - 2 * 0.5
        # = 1.0 m
        self.straight_length = (
            self.side_length - 2.0 * self.turn_radius
        )

        # 直线段理论时间：
        #
        # t = distance / speed
        #   = 1.0 / 0.2
        #   = 5.0 s
        self.straight_duration = (
            self.straight_length / self.linear_speed
        )

        # 转弯角速度：
        #
        # omega = v / R
        #       = 0.2 / 0.5
        #       = 0.4 rad/s
        self.angular_speed = (
            self.linear_speed / self.turn_radius
        )

        # 每个圆角目标转角：90 度。
        self.turn_angle = math.pi / 2.0

        # 理论转弯时间：
        #
        # (pi / 2) / 0.4
        # ≈ 3.927 s
        self.theoretical_turn_duration = (
            self.turn_angle / self.angular_speed
        )

        # 根据前面的实际实验进行校准。
        #
        # 第一次：
        # theoretical ≈ 3.927 s
        # actual turn ≈ 77.09 deg
        #
        # 校准：
        # 3.927 * 90 / 77.09 ≈ 4.585 s
        #
        # 第二次实验：
        # actual turn ≈ 89.58 deg
        # error ≈ -0.42 deg
        self.turn_duration = 4.585

        # -------------------------
        # 一圈控制参数
        # -------------------------

        # 一个完整圆角方形有 4 个转弯。
        self.total_corners = 4

        # 已经完成多少个转弯。
        self.completed_corners = 0

        # -------------------------
        # 状态机
        # -------------------------

        # 状态变化：
        #
        # WAIT_CLOCK
        #     ↓
        # STRAIGHT
        #     ↓
        # TURN
        #     ↓
        # STRAIGHT
        #     ↓
        # TURN
        #     ↓
        # ...
        #     ↓
        # 第 4 个 TURN
        #     ↓
        # STOP
        self.state = 'WAIT_CLOCK'

        # 当前状态开始的仿真时间。
        self.state_start_time = None

        # 10 Hz 定时器。
        self.timer = self.create_timer(
            0.1,
            self.timer_callback
        )

        self.get_logger().info(
            'Square driver started.'
        )

        self.get_logger().info(
            'Waiting for Gazebo /clock...'
        )

        self.get_logger().info(
            f'Side length: {self.side_length:.2f} m'
        )

        self.get_logger().info(
            f'Straight length: {self.straight_length:.2f} m'
        )

        self.get_logger().info(
            f'Straight duration: {self.straight_duration:.3f} s'
        )

        self.get_logger().info(
            f'Turn duration: {self.turn_duration:.3f} s'
        )

    def odom_callback(self, msg):
        """读取 /odom，并把四元数转换成 yaw。"""

        q = msg.pose.pose.orientation

        x = q.x
        y = q.y
        z = q.z
        w = q.w

        self.current_theta = math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z)
        )

    def publish_cmd(self, linear_x, angular_z):
        """向 /cmd_vel 发布速度命令。"""

        msg = Twist()

        msg.linear.x = linear_x
        msg.angular.z = angular_z

        self.cmd_pub.publish(msg)

    def normalize_angle(self, angle):
        """把角度归一化到 (-pi, pi]。"""

        while angle <= -math.pi:
            angle += 2.0 * math.pi

        while angle > math.pi:
            angle -= 2.0 * math.pi

        return angle

    def change_state(self, new_state):
        """切换状态，并重新记录状态开始时间。"""

        self.state = new_state
        self.state_start_time = self.get_clock().now()

        self.get_logger().info(
            f'State: {self.state}'
        )

        # 每次刚进入 TURN，都记录转弯起始角度。
        if new_state == 'TURN':

            if self.current_theta is not None:
                self.turn_start_theta = self.current_theta

                self.get_logger().info(
                    'Turn start theta: '
                    f'{math.degrees(self.turn_start_theta):.2f} deg'
                )

    def finish_turn(self):
        """完成一次转弯，打印结果，并决定下一步。"""

        self.completed_corners += 1

        self.get_logger().info(
            f'Corner {self.completed_corners}/'
            f'{self.total_corners} completed.'
        )

        # 如果有有效的 /odom 数据，
        # 就计算这一弯实际转了多少度。
        if (
            self.turn_start_theta is not None
            and self.current_theta is not None
        ):
            actual_turn = self.normalize_angle(
                self.current_theta
                - self.turn_start_theta
            )

            turn_error = self.normalize_angle(
                actual_turn
                - self.turn_angle
            )

            self.get_logger().info(
                'Actual turn: '
                f'{math.degrees(actual_turn):.2f} deg'
            )

            self.get_logger().info(
                'Turn error: '
                f'{math.degrees(turn_error):+.2f} deg'
            )

        # 四个转弯都完成了：
        # 一圈结束，停车。
        if self.completed_corners >= self.total_corners:

            self.change_state('STOP')

            self.get_logger().info(
                'One lap completed.'
            )

        # 否则进入下一条直线。
        else:

            self.change_state('STRAIGHT')

    def timer_callback(self):
        """根据当前状态决定车辆应该做什么。"""

        now = self.get_clock().now()

        # -------------------------
        # 状态 0：等待 Gazebo 时钟
        # -------------------------

        if self.state == 'WAIT_CLOCK':

            self.publish_cmd(
                0.0,
                0.0
            )

            if now.nanoseconds > 0:
                self.change_state('STRAIGHT')

            return

        elapsed = (
            now - self.state_start_time
        ).nanoseconds / 1e9

        # -------------------------
        # 状态 1：直行
        # -------------------------

        if self.state == 'STRAIGHT':

            self.publish_cmd(
                self.linear_speed,
                0.0
            )

            if elapsed >= self.straight_duration:
                self.change_state('TURN')

        # -------------------------
        # 状态 2：左转
        # -------------------------

        elif self.state == 'TURN':

            self.publish_cmd(
                self.linear_speed,
                self.angular_speed
            )

            if elapsed >= self.turn_duration:
                self.finish_turn()

        # -------------------------
        # 状态 3：停车
        # -------------------------

        elif self.state == 'STOP':

            self.publish_cmd(
                0.0,
                0.0
            )


def main(args=None):
    rclpy.init(args=args)

    node = SquareDriver()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        # 无论程序如何退出，都先发送停车命令。
        node.publish_cmd(
            0.0,
            0.0
        )

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
