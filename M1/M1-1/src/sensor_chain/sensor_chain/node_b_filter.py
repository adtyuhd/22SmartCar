import rclpy
from rclpy.node import Node

from sensor_interfaces.msg import SensorData
from std_msgs.msg import Float32


class SensorFilter(Node):

    def __init__(self):
        super().__init__('node_b_filter')

        self.subscription = self.create_subscription(
            SensorData,
            '/sensor_data',
            self.sensor_callback,
            10
        )

        self.publisher_ = self.create_publisher(
            Float32,
            '/processed_distance',
            10
        )

        self.declare_parameter('alpha', 0.3)

        self.filtered_value = None

    def sensor_callback(self, msg):

        if msg.status != 0:
            self.get_logger().warning(
                f'Invalid sensor data: status={msg.status}'
            )
            return

        alpha = self.get_parameter('alpha').value

        if self.filtered_value is None:
            self.filtered_value = msg.distance
        else:
            self.filtered_value = (
                alpha * msg.distance
                + (1.0 - alpha) * self.filtered_value
            )

        output_msg = Float32()
        output_msg.data = float(self.filtered_value)

        self.publisher_.publish(output_msg)

        self.get_logger().info(
            f'Raw: {msg.distance:.3f} m -> '
            f'Filtered: {self.filtered_value:.3f} m '
            f'(alpha={alpha:.2f})'
        )


def main(args=None):
    rclpy.init(args=args)

    node = SensorFilter()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()