import random

import rclpy
from rclpy.node import Node

from sensor_interfaces.msg import SensorData
from sensor_interfaces.srv import TriggerAlarm


class SensorPublisher(Node):

    def __init__(self):
        super().__init__('node_a_sensor')

        self.publisher_ = self.create_publisher(
            SensorData,
            'sensor_data',
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.timer_callback
        )

        self.alarm_service = self.create_service(
            TriggerAlarm,
            'trigger_alarm',
            self.alarm_callback
        )

    def timer_callback(self):
        msg = SensorData()

        msg.distance = float(
            0.4 + random.uniform(-0.2, 0.2)
        )
        msg.unit = 'm'
        msg.stamp = self.get_clock().now().to_msg()
        msg.status = 0

        self.publisher_.publish(msg)

        self.get_logger().info(
            f'Published distance: {msg.distance:.3f} m'
        )

    def alarm_callback(self, request, response):
        self.get_logger().warning(
            f'[ALARM] 距离过近！distance={request.distance:.3f} m'
        )

        response.success = True
        response.message = 'Alarm triggered'

        return response


def main(args=None):
    rclpy.init(args=args)

    node = SensorPublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()