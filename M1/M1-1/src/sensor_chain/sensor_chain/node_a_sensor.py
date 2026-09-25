import rclpy
from rclpy.node import Node

from sensor_interfaces.msg import SensorData


class SensorPublisher(Node):

    def __init__(self):
        super().__init__('node_a_sensor')

        self.publisher_ = self.create_publisher(
            SensorData,
            '/sensor_data',
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.timer_callback
        )

    def timer_callback(self):
        msg = SensorData()

        msg.distance = 0.5
        msg.unit = 'm'
        msg.stamp = self.get_clock().now().to_msg()
        msg.status = 0

        self.publisher_.publish(msg)

        self.get_logger().info(
            f'Published distance: {msg.distance:.3f} m'
        )


def main(args=None):
    rclpy.init(args=args)

    node = SensorPublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()