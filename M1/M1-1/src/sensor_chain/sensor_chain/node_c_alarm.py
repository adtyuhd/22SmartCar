import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from sensor_interfaces.srv import TriggerAlarm


class AlarmClient(Node):

    def __init__(self):
        super().__init__('node_c_alarm')

        self.declare_parameter('alarm_threshold', 0.3)
        self.declare_parameter('service_timeout', 1.0)

        self.subscription = self.create_subscription(
            Float32,
            '/processed_distance',
            self.distance_callback,
            10
        )

        self.alarm_client = self.create_client(
            TriggerAlarm,
            '/trigger_alarm'
        )

        self.pending_future = None
        self.request_start_time = None

        self.timeout_timer = self.create_timer(
            0.1,
            self.check_alarm_request
        )

    def distance_callback(self, msg):
        if self.pending_future is not None:
            return

        threshold = self.get_parameter(
            'alarm_threshold'
        ).value

        if msg.data < threshold:
            request = TriggerAlarm.Request()
            request.distance = msg.data

            self.pending_future = (
                self.alarm_client.call_async(request)
            )

            self.request_start_time = (
                self.get_clock().now()
            )

            self.get_logger().warning(
                f'Alarm requested: {msg.data:.3f} m'
            )

    def check_alarm_request(self):
        if self.pending_future is None:
            return

        if self.pending_future.done():
            try:
                response = self.pending_future.result()

                if response.success:
                    self.get_logger().info(
                        f'Alarm service success: {response.message}'
                    )
                else:
                    self.get_logger().warning(
                        f'Alarm service failed: {response.message}'
                    )

            except Exception as e:
                self.get_logger().error(
                    f'Alarm service error: {e}'
                )

            self.pending_future = None
            self.request_start_time = None
            return

        now = self.get_clock().now()

        elapsed = (
            now - self.request_start_time
        ).nanoseconds / 1e9

        timeout = self.get_parameter(
            'service_timeout'
        ).value

        if elapsed >= timeout:
            self.get_logger().warning(
                '报警服务超时'
            )

            self.pending_future = None
            self.request_start_time = None


def main(args=None):
    rclpy.init(args=args)

    node = AlarmClient()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()