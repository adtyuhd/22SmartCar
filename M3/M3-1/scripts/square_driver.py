#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SquareDriver(Node):

    def __init__(self):
        super().__init__('square_driver')

        # 创建 /cmd_vel 发布器。
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # 测试参数：
        # 以 0.2 m/s 的速度直行 2 秒。
        # 理论距离 = 0.2 * 2.0 = 0.4 m。
        self.linear_speed = 0.2
        self.drive_duration = 2.0

        # 记录程序开始驾驶的 ROS 时间。
        self.start_time = self.get_clock().now()

        # 标记这次测试是否已经结束。
        self.finished = False

        # 每 0.1 秒调用一次回调函数，即 10 Hz。
        self.timer = self.create_timer(
            0.1,
            self.timer_callback
        )

        self.get_logger().info('Square driver started.')
        self.get_logger().info(
            'Test: drive straight at 0.2 m/s for 2.0 s.'
        )

    def publish_cmd(self, linear_x, angular_z):
        """向 /cmd_vel 发布速度命令。"""

        msg = Twist()

        msg.linear.x = linear_x
        msg.angular.z = angular_z

        self.cmd_pub.publish(msg)

    def timer_callback(self):
        """根据已经经过的时间决定继续直行还是停车。"""

        # 当前 ROS 时间。
        now = self.get_clock().now()

        # Duration 转换成秒。
        elapsed = (now - self.start_time).nanoseconds / 1e9

        if elapsed < self.drive_duration:
            # 前 2 秒持续直行。
            self.publish_cmd(
                self.linear_speed,
                0.0
            )

        else:
            # 2 秒后持续发送停车命令。
            self.publish_cmd(
                0.0,
                0.0
            )

            # 日志只打印一次。
            if not self.finished:
                self.finished = True

                self.get_logger().info(
                    f'Straight test finished after {elapsed:.3f} s.'
                )


def main(args=None):
    rclpy.init(args=args)

    node = SquareDriver()

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
