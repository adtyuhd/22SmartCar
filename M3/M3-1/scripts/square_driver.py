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

        # 订阅里程计，用来读取车辆当前姿态。
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # 最近一次从 /odom 得到的 yaw。
        # 单位：rad
        self.current_theta = None

        # 转弯开始时的 yaw。
        self.turn_start_theta = None

        # -------------------------
        # 测试运动参数
        # -------------------------

        # 线速度：0.2 m/s。
        self.linear_speed = 0.2

        # 先直行 2 秒。
        # 理论距离 = 0.2 * 2.0 = 0.4 m。
        self.straight_duration = 2.0

        # 计划转弯半径：0.5 m。
        self.turn_radius = 0.5

        # omega = v / R
        # 0.2 / 0.5 = 0.4 rad/s
        self.angular_speed = (
            self.linear_speed / self.turn_radius
        )

        # 目标转角：90 度。
        self.turn_angle = math.pi / 2.0

        # 理论转弯时间：
        # (pi / 2) / 0.4 ≈ 3.927 s
        self.theoretical_turn_duration = (
            self.turn_angle / self.angular_speed
        )

        # 第一次实验结果：
        # 理论时间约 3.927 s，
        # /odom 测得实际转角约 77.09 deg。
        #
        # 根据比例进行第一次经验校准：
        #
        # 3.927 * 90 / 77.09 ≈ 4.585 s
        #
        # 注意：
        # 这里不是凭感觉调整，而是根据实验测量结果计算。
        self.turn_duration = 4.585

        # -------------------------
        # 状态机
        # -------------------------

        # WAIT_CLOCK -> STRAIGHT -> TURN -> STOP
        self.state = 'WAIT_CLOCK'

        # 当前状态的开始时间。
        self.state_start_time = None

        # 防止 STOP 状态重复打印测量结果。
        self.result_printed = False

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

    def odom_callback(self, msg):
        """读取 /odom，并把四元数转换成 yaw。"""

        q = msg.pose.pose.orientation

        x = q.x
        y = q.y
        z = q.z
        w = q.w

        # 四元数 -> yaw
        theta = math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z)
        )

        self.current_theta = theta

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

        # 刚进入 TURN 时，记录转弯起始角度。
        if new_state == 'TURN':

            if self.current_theta is not None:
                self.turn_start_theta = self.current_theta

                self.get_logger().info(
                    'Turn start theta: '
                    f'{math.degrees(self.turn_start_theta):.2f} deg'
                )

        # 刚进入 STOP 时，计算实际转角。
        elif new_state == 'STOP':

            self.print_turn_result()

    def print_turn_result(self):
        """打印本次转弯的实际角度和误差。"""

        if self.result_printed:
            return

        if self.turn_start_theta is None:
            self.get_logger().warning(
                'No turn start theta available.'
            )
            return

        if self.current_theta is None:
            self.get_logger().warning(
                'No current odom theta available.'
            )
            return

        turn_end_theta = self.current_theta

        # 实际转角：
        # 转弯结束角度 - 转弯开始角度
        actual_turn = self.normalize_angle(
            turn_end_theta - self.turn_start_theta
        )

        # 转角误差：
        # 实际转角 - 目标转角
        turn_error = self.normalize_angle(
            actual_turn - self.turn_angle
        )

        self.get_logger().info(
            'Turn end theta: '
            f'{math.degrees(turn_end_theta):.2f} deg'
        )

        self.get_logger().info(
            'Actual turn: '
            f'{math.degrees(actual_turn):.2f} deg'
        )

        self.get_logger().info(
            'Target turn: '
            f'{math.degrees(self.turn_angle):.2f} deg'
        )

        self.get_logger().info(
            'Turn error: '
            f'{math.degrees(turn_error):+.2f} deg'
        )

        self.result_printed = True

    def timer_callback(self):
        """根据当前状态决定车辆应该做什么。"""

        now = self.get_clock().now()

        # -------------------------
        # 状态 0：等待 Gazebo 时钟
        # -------------------------

        if self.state == 'WAIT_CLOCK':

            # 在收到有效 /clock 前保持停车。
            self.publish_cmd(
                0.0,
                0.0
            )

            if now.nanoseconds > 0:
                self.change_state('STRAIGHT')

            return

        # 从这里开始，
        # state_start_time 一定已经被设置。
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
                self.change_state('STOP')

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
        # 程序退出前再发送一次停车命令。
        node.publish_cmd(
            0.0,
            0.0
        )

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
