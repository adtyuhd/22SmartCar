
#!/usr/bin/env python3

"""
M3-2 激光 SLAM 自动巡场程序。

默认只预览路线，不驱动车辆。

示例：
    python3 drive_route.py
    python3 drive_route.py --run --test-seconds 8
    python3 drive_route.py --run
"""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


# ==========================================
# 1. 车辆与控制参数
# ==========================================

WHEELBASE = 0.28
MAX_STEERING_ANGLE = math.radians(30.0)

STRAIGHT_SPEED = 0.25
TURN_SPEED = 0.12

TURN_RADIUS = 0.8
POINT_STEP = 0.10
LOOKAHEAD = 0.55
GOAL_TOLERANCE = 0.25

MAX_RUN_TIME = 600.0


# ==========================================
# 2. 数学辅助函数
# ==========================================

def normalize_angle(angle):
    """将角度限制到 [-pi, pi]。"""

    return math.atan2(
        math.sin(angle),
        math.cos(angle)
    )


def smoothstep(t):
    """生成两端斜率为零的平滑插值系数。"""

    return t * t * (3.0 - 2.0 * t)


def quaternion_to_yaw(q):
    """从四元数求出平面航向角。"""

    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


# ==========================================
# 3. 路线生成
# ==========================================

def create_route():
    """
    生成以小车初始位置为原点的路线。

    每个点：
        (x, y, slow)

    slow=True 代表低速区。
    """

    points = []

    def add_point(x, y, slow=False):

        if points:
            px, py, old_slow = points[-1]

            if math.hypot(x - px, y - py) < 1e-8:
                points[-1] = (
                    px,
                    py,
                    old_slow or slow
                )
                return

        points.append((x, y, slow))

    def add_line(x0, y0, x1, y1, slow=False):
        """把直线分成间距不超过 POINT_STEP 的点。"""

        length = math.hypot(
            x1 - x0,
            y1 - y0
        )

        count = max(
            1,
            math.ceil(length / POINT_STEP)
        )

        for i in range(count + 1):
            t = i / count

            add_point(
                x0 + (x1 - x0) * t,
                y0 + (y1 - y0) * t,
                slow
            )

    def add_arc(cx, cy, radius, a0, a1):
        """生成逆时针转弯圆弧。"""

        length = radius * abs(a1 - a0)

        count = max(
            1,
            math.ceil(length / POINT_STEP)
        )

        for i in range(count + 1):
            t = i / count
            angle = a0 + (a1 - a0) * t

            add_point(
                cx + radius * math.cos(angle),
                cy + radius * math.sin(angle),
                True
            )

    def add_transition(x0, x1, y0, y1):
        """生成窄通道前后的平滑横向偏移。"""

        length = abs(x1 - x0)

        count = max(
            1,
            math.ceil(length / POINT_STEP)
        )

        for i in range(count + 1):
            t = i / count

            add_point(
                x0 + (x1 - x0) * t,
                y0 + (y1 - y0) * smoothstep(t),
                True
            )

    # 南侧走廊：向东
    add_line(
        0.0, 0.0,
        6.25, 0.0
    )

    # 东南角
    add_arc(
        6.25, 0.8,
        TURN_RADIUS,
        -math.pi / 2.0,
        0.0
    )

    # 东侧走廊：向北
    add_line(
        7.05, 0.8,
        7.05, 9.3
    )

    # 东北角
    add_arc(
        6.25, 9.3,
        TURN_RADIUS,
        0.0,
        math.pi / 2.0
    )

    # 北侧走廊：向西
    add_line(
        6.25, 10.10,
        4.5, 10.10
    )

    # 进入窄通道
    add_transition(
        4.5, 2.7,
        10.10, 9.625
    )

    # 穿越窄通道
    add_line(
        2.7, 9.625,
        1.3, 9.625,
        slow=True
    )

    # 离开窄通道
    add_transition(
        1.3, -0.5,
        9.625, 10.10
    )

    add_line(
        -0.5, 10.10,
        -6.25, 10.10
    )

    # 西北角
    add_arc(
        -6.25, 9.3,
        TURN_RADIUS,
        math.pi / 2.0,
        math.pi
    )

    # 西侧走廊：向南
    add_line(
        -7.05, 9.3,
        -7.05, 0.8
    )

    # 西南角
    add_arc(
        -6.25, 0.8,
        TURN_RADIUS,
        math.pi,
        3.0 * math.pi / 2.0
    )

    # 返回南侧起点
    add_line(
        -6.25, 0.0,
        0.0, 0.0
    )

    return points


# ==========================================
# 4. ROS 2 自动驾驶节点
# ==========================================

class DriveRoute(Node):

    def __init__(self):

        super().__init__("m3_2_drive_route")

        # 避免重复声明 ROS 2 已有的参数。
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter(
                "use_sim_time",
                True
            )

        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

        self.current_pose = None
        self.initial_pose = None
        self.last_odom_time = None

    def odom_callback(self, msg):
        """更新里程计反馈。"""

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        yaw = quaternion_to_yaw(
            msg.pose.pose.orientation
        )

        self.current_pose = (x, y, yaw)
        self.last_odom_time = time.monotonic()

        if self.initial_pose is None:
            self.initial_pose = self.current_pose

    def get_local_pose(self):
        """把 odom 位姿转换到初始小车坐标系。"""

        x, y, yaw = self.current_pose
        x0, y0, yaw0 = self.initial_pose

        dx = x - x0
        dy = y - y0

        c = math.cos(yaw0)
        s = math.sin(yaw0)

        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy

        local_yaw = normalize_angle(
            yaw - yaw0
        )

        return local_x, local_y, local_yaw

    def publish_cmd(self, linear, angular):
        """向底盘发送速度指令。"""

        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)

        self.cmd_pub.publish(msg)

    def stop(self):
        """
        发送多次零速度命令。

        必须在 rclpy.shutdown() 之前调用。
        """

        if not rclpy.ok():
            self.get_logger().warning(
                "ROS 2 已关闭，无法发布停车指令"
            )
            return

        for _ in range(10):
            self.publish_cmd(0.0, 0.0)
            time.sleep(0.05)

        self.get_logger().info(
            "已发送零速度停车指令"
        )

    def wait_for_odometry(self):
        """等待第一帧 /odom。"""

        start = time.monotonic()

        while rclpy.ok():

            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

            if self.current_pose is not None:
                return

            if time.monotonic() - start > 15.0:
                raise RuntimeError(
                    "15 秒内没有收到 /odom"
                )

    def follow_route(self, points, test_seconds=None):
        """
        Pure Pursuit 路线跟踪。

        test_seconds:
            None -> 跑完整路线
            数字 -> 指定秒数后主动结束
        """

        self.wait_for_odometry()

        self.get_logger().info(
            "已收到 /odom，开始跟踪回环路线"
        )

        if test_seconds is not None:
            self.get_logger().info(
                f"短测模式：运行 {test_seconds:.1f} 秒后停车"
            )

        progress = 0
        start_time = time.monotonic()
        last_log_time = start_time

        max_curvature = (
            math.tan(MAX_STEERING_ANGLE)
            / WHEELBASE
        )

        while rclpy.ok():

            rclpy.spin_once(
                self,
                timeout_sec=0.05
            )

            now = time.monotonic()
            elapsed = now - start_time

            # 新增：程序内部计时，到时正常结束。
            if (
                test_seconds is not None
                and elapsed >= test_seconds
            ):
                self.get_logger().info(
                    "短测时间到，结束路径跟踪"
                )
                break

            if elapsed >= MAX_RUN_TIME:
                raise RuntimeError(
                    "巡场超过最长允许时间"
                )

            # 里程计断流保护。
            if (
                self.last_odom_time is None
                or now - self.last_odom_time > 2.0
            ):
                raise RuntimeError(
                    "/odom 超过 2 秒未更新"
                )

            x, y, yaw = self.get_local_pose()

            # 从当前进度向前寻找最近路线点。
            search_end = min(
                len(points),
                progress + 50
            )

            nearest = min(
                range(progress, search_end),
                key=lambda i: math.hypot(
                    points[i][0] - x,
                    points[i][1] - y
                )
            )

            progress = max(progress, nearest)

            # 判断是否完成整圈。
            goal_distance = math.hypot(
                points[-1][0] - x,
                points[-1][1] - y
            )

            if (
                progress >= len(points) - 6
                and goal_distance < GOAL_TOLERANCE
            ):
                self.get_logger().info(
                    "已完成一圈回环路线"
                )
                break

            # 选择前视目标点。
            target = progress

            while target < len(points) - 1:

                tx, ty, _ = points[target]

                distance = math.hypot(
                    tx - x,
                    ty - y
                )

                if distance >= LOOKAHEAD:
                    break

                target += 1

            tx, ty, _ = points[target]

            dx = tx - x
            dy = ty - y

            distance = max(
                0.01,
                math.hypot(dx, dy)
            )

            # 目标点相对于车头的方向误差。
            alpha = normalize_angle(
                math.atan2(dy, dx) - yaw
            )

            # Pure Pursuit 曲率。
            curvature = (
                2.0 * math.sin(alpha)
                / distance
            )

            curvature = max(
                -max_curvature,
                min(max_curvature, curvature)
            )

            # 弯道和窄通道减速。
            slow_end = min(
                len(points),
                target + 5
            )

            slow_area = any(
                p[2]
                for p in points[progress:slow_end]
            )

            speed = (
                TURN_SPEED
                if slow_area
                else STRAIGHT_SPEED
            )

            # 将曲率转换成角速度。
            angular = speed * curvature

            self.publish_cmd(
                speed,
                angular
            )

            if now - last_log_time >= 5.0:

                self.get_logger().info(
                    f"路线进度 {progress + 1}/"
                    f"{len(points)}，"
                    f"位置 ({x:.2f}, {y:.2f})，"
                    f"速度 {speed:.2f} m/s"
                )

                last_log_time = now


# ==========================================
# 5. 程序入口
# ==========================================

def main():

    parser = argparse.ArgumentParser(
        description="M3-2 SLAM 自动巡场"
    )

    parser.add_argument(
        "--run",
        action="store_true",
        help="实际驱动车辆"
    )

    parser.add_argument(
        "--test-seconds",
        type=float,
        default=None,
        help="自动运行指定秒数后停车"
    )

    args = parser.parse_args()

    if (
        args.test_seconds is not None
        and args.test_seconds <= 0.0
    ):
        parser.error(
            "--test-seconds 必须大于 0"
        )

    points = create_route()

    min_radius = (
        WHEELBASE
        / math.tan(MAX_STEERING_ANGLE)
    )

    print("========== M3-2 路线信息 ==========")
    print(f"路线点数量：{len(points)}")
    print(f"设计转弯半径：{TURN_RADIUS:.2f} m")
    print(f"理论最小转弯半径：{min_radius:.2f} m")
    print(f"直线速度：{STRAIGHT_SPEED:.2f} m/s")
    print(f"慢速区速度：{TURN_SPEED:.2f} m/s")
    print(f"起点：{points[0][:2]}")
    print(f"终点：{points[-1][:2]}")

    if TURN_RADIUS < min_radius:
        raise ValueError(
            "设计转弯半径小于车辆理论下限"
        )

    if not args.run:
        print("预览完成：小车不会运动")
        return

    # 不让 ROS 2 的默认 SIGINT 处理器提前关闭上下文。
    # 这样 Ctrl+C 会由 Python 的 KeyboardInterrupt
    # 交给下方的 finally 先执行停车。
    rclpy.init(
        signal_handler_options=SignalHandlerOptions.NO
    )

    node = None

    try:
        node = DriveRoute()

        node.follow_route(
            points,
            test_seconds=args.test_seconds
        )

    except KeyboardInterrupt:
        print("用户中断，准备停车")

    except Exception as error:
        print(f"自动驾驶异常：{error}")

    finally:
        # 先停车，再销毁节点、关闭 ROS 2。
        if node is not None:
            try:
                node.stop()
            finally:
                node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()