#!/usr/bin/env python3
"""M3-2 仅用 SLAM、雷达完成巡场；雷达墙面反馈辅助纠偏和防撞。"""

import argparse
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformListener, TransformException


# ==========================================
# 1. 车辆参数
# ==========================================

WHEELBASE = 0.28
MAX_STEERING = math.radians(30.0)

CAR_LENGTH = 0.45
CAR_WIDTH = 0.22

STRAIGHT_SPEED = 0.16
TURN_SPEED = 0.08

TURN_RADIUS = 0.8
POINT_STEP = 0.10
LOOKAHEAD = 0.45

MIN_CLEARANCE = 0.12

SPAWN_X = 0.0
SPAWN_Y = -5.05

MAX_RUN_SECONDS = 600.0

# 定位由 slam_toolbox 的 map -> odom 和轮式里程计 odom -> base_footprint 融合而来。
# 控制节点只读取 ROS TF，不订阅 /gazebo/model_states，不读取仿真真值。
TF_WAIT_SECONDS = 15.0
TF_TIMEOUT_SECONDS = 0.30
TF_MAX_AGE_SECONDS = 1.5

# 仅使用真实传感器 /scan 检查前方是否有障碍物。
SCAN_MAX_AGE_SECONDS = 1.0
FRONT_STOP_DISTANCE = 0.45
SIDE_STOP_DISTANCE = 0.22

# 在宽度规则的直线走廊，以真实激光两侧墙距辅助纠偏。
# 不使用 /gazebo/model_states，也不使用世界真值。
WALL_LATERAL_GAIN = 0.16
WALL_HEADING_GAIN = 0.40
WALL_MAX_CORRECTION = 0.25


# ==========================================
# 2. 数学辅助函数
# ==========================================

def normalize_angle(angle):
    return math.atan2(
        math.sin(angle),
        math.cos(angle)
    )


def smoothstep(t):
    return t * t * (3.0 - 2.0 * t)


# ==========================================
# 3. 创建固定巡场路线
# ==========================================

def create_route():
    """
    返回路线点列表。

    每个点为：
        (x, y, slow)

    slow=True 表示低速路段。
    """

    points = []

    def add_point(x, y, slow=False):

        if points:
            px, py, previous_slow = points[-1]

            if math.hypot(x - px, y - py) < 1e-8:
                points[-1] = (
                    px,
                    py,
                    previous_slow or slow
                )
                return

        points.append((x, y, slow))

    def add_line(x0, y0, x1, y1, slow=False):

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

    def add_arc(cx, cy, radius, start, end):

        length = radius * abs(end - start)

        count = max(
            1,
            math.ceil(length / POINT_STEP)
        )

        for i in range(count + 1):
            t = i / count
            angle = start + (end - start) * t

            add_point(
                cx + radius * math.cos(angle),
                cy + radius * math.sin(angle),
                True
            )

    def add_transition(x0, x1, y0, y1):

        count = max(
            1,
            math.ceil(abs(x1 - x0) / POINT_STEP)
        )

        for i in range(count + 1):
            t = i / count

            add_point(
                x0 + (x1 - x0) * t,
                y0 + (y1 - y0) * smoothstep(t),
                True
            )

    # 南侧走廊
    add_line(0, 0, 6.25, 0)

    # 东南角
    add_arc(
        6.25, 0.8, TURN_RADIUS,
        -math.pi / 2, 0
    )

    # 东侧走廊
    add_line(7.05, 0.8, 7.05, 9.3)

    # 东北角
    add_arc(
        6.25, 9.3, TURN_RADIUS,
        0, math.pi / 2
    )

    # 北侧走廊
    add_line(6.25, 10.10, 4.5, 10.10)

    # 进入窄通道
    add_transition(
        4.5, 2.7,
        10.10, 9.625
    )

    # 穿过窄通道
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
        -6.25, 9.3, TURN_RADIUS,
        math.pi / 2, math.pi
    )

    # 西侧走廊
    add_line(-7.05, 9.3, -7.05, 0.8)

    # 西南角
    add_arc(
        -6.25, 0.8, TURN_RADIUS,
        math.pi, 3 * math.pi / 2
    )

    # 返回起点
    add_line(-6.25, 0, 0, 0)

    return points


# ==========================================
# 4. ROS TF / SLAM 定位
# ==========================================

def quaternion_to_yaw(q):
    """由 ROS 四元数计算平面航向角。"""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


# ==========================================
# 5. 读取场景障碍物
# ==========================================

def get_obstacles():
    """从 scene.world 读取固定障碍物碰撞盒。"""

    world_path = (
        Path(__file__).resolve().parent.parent
        / "worlds"
        / "scene.world"
    )

    with open(
        world_path,
        encoding="utf-8-sig"
    ) as file:
        root = ET.fromstring(
            file.read().lstrip()
        )

    world = root.find("world")

    if world is None:
        raise RuntimeError(
            "scene.world 没有 world 元素"
        )

    obstacles = []

    for model in world.findall("model"):

        name = model.get(
            "name",
            "unknown"
        )

        model_pose = [
            float(v)
            for v in model.findtext(
                "pose",
                "0 0 0 0 0 0"
            ).split()
        ]

        for link in model.findall("link"):

            link_pose = [
                float(v)
                for v in link.findtext(
                    "pose",
                    "0 0 0 0 0 0"
                ).split()
            ]

            for collision in link.findall("collision"):

                box = collision.find(
                    "geometry/box"
                )

                if box is None:
                    continue

                size = [
                    float(v)
                    for v in box.findtext(
                        "size"
                    ).split()
                ]

                collision_pose = [
                    float(v)
                    for v in collision.findtext(
                        "pose",
                        "0 0 0 0 0 0"
                    ).split()
                ]

                # 当前世界的固定障碍物均无旋转。
                if any(
                    abs(pose[j]) > 1e-6
                    for pose in (
                        model_pose,
                        link_pose,
                        collision_pose
                    )
                    for j in (3, 4, 5)
                ):
                    raise RuntimeError(
                        "碰撞检查暂不支持旋转障碍物"
                    )

                x = (
                    model_pose[0]
                    + link_pose[0]
                    + collision_pose[0]
                )

                y = (
                    model_pose[1]
                    + link_pose[1]
                    + collision_pose[1]
                )

                obstacles.append((
                    name,
                    x - size[0] / 2,
                    x + size[0] / 2,
                    y - size[1] / 2,
                    y + size[1] / 2
                ))

    return obstacles


# ==========================================
# 6. 计算小车与障碍物的距离
# ==========================================

def get_clearance(x, y, yaw, obstacles):
    """
    使用车身旋转后的轴对齐包围盒计算距离。

    这是保守近似，并非精确碰撞检测。
    """

    half_length = CAR_LENGTH / 2
    half_width = CAR_WIDTH / 2

    hx = (
        abs(math.cos(yaw)) * half_length
        + abs(math.sin(yaw)) * half_width
    )

    hy = (
        abs(math.sin(yaw)) * half_length
        + abs(math.cos(yaw)) * half_width
    )

    left = x - hx
    right = x + hx
    bottom = y - hy
    top = y + hy

    nearest_name = None
    nearest_distance = float("inf")

    for name, bx0, bx1, by0, by1 in obstacles:

        dx = max(
            bx0 - right,
            left - bx1,
            0.0
        )

        dy = max(
            by0 - top,
            bottom - by1,
            0.0
        )

        distance = math.hypot(dx, dy)

        if distance < nearest_distance:
            nearest_distance = distance
            nearest_name = name

    return nearest_distance, nearest_name


# ==========================================
# 7. 自动驾驶节点
# ==========================================

class RouteDriver(Node):

    def __init__(self):

        super().__init__(
            "m3_2_drive_route",
            parameter_overrides=[
                Parameter("use_sim_time", Parameter.Type.BOOL, True)
            ]
        )

        self.publisher = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        # TF 只来源于 slam_toolbox 和底盘 /odom；不使用 Gazebo 真值。
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=True
        )

        self.latest_scan = None
        self.scan_received_at = None
        self.create_subscription(
            LaserScan,
            "/scan",
            self.on_scan,
            qos_profile_sensor_data
        )

    def on_scan(self, msg):
        self.latest_scan = msg
        self.scan_received_at = time.monotonic()

    @staticmethod
    def sample_scan(msg, direction, width):
        """读取雷达指定方向的有效测距，单位为米。"""
        result = []
        for i, distance in enumerate(msg.ranges):
            if not (math.isfinite(distance) and msg.range_min <= distance <= msg.range_max):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            delta = normalize_angle(angle - direction)
            if abs(delta) <= width:
                result.append(float(distance))
        return result

    def get_scan_info(self):
        """雷达超时、缺测时立即停止；返回前、左、右距离和墙面偏航角。"""
        if self.latest_scan is None or self.scan_received_at is None:
            raise RuntimeError("/scan 尚未收到激光数据，安全停车")
        age = time.monotonic() - self.scan_received_at
        if age > SCAN_MAX_AGE_SECONDS:
            raise RuntimeError(f"/scan 已 {age:.2f} 秒没有更新，安全停车")

        msg = self.latest_scan
        front = self.sample_scan(msg, 0.0, math.radians(35))
        left = self.sample_scan(msg, math.pi / 2, math.radians(8))
        right = self.sample_scan(msg, -math.pi / 2, math.radians(8))
        if not front or not left or not right:
            raise RuntimeError("/scan 前方或侧方有效测距不足，安全停车")

        front_distance = min(front)
        # 用中央值排除一两束噪声反射。
        left_distance = sorted(left)[len(left) // 2]
        right_distance = sorted(right)[len(right) // 2]
        if front_distance < FRONT_STOP_DISTANCE:
            raise RuntimeError(f"雷达前方障碍物仅 {front_distance:.2f} m，安全停车")
        if min(left_distance, right_distance) < SIDE_STOP_DISTANCE:
            raise RuntimeError(
                f"雷达侧方障碍物过近：左 {left_distance:.2f} m，"
                f"右 {right_distance:.2f} m，安全停车"
            )

        # 沿正交侧墙的两束斜向雷达估计真实偏航，不依赖 /odom 的偏航。
        headings = []
        for sign in (1, -1):
            angle_front = sign * math.radians(60)
            angle_back = sign * math.radians(120)
            samples_front = self.sample_scan(msg, angle_front, math.radians(2))
            samples_back = self.sample_scan(msg, angle_back, math.radians(2))
            if not samples_front or not samples_back:
                continue
            rf = sorted(samples_front)[len(samples_front) // 2]
            rb = sorted(samples_back)[len(samples_back) // 2]
            if not (0.25 <= rf <= 2.5 and 0.25 <= rb <= 2.5):
                continue
            xf, yf = rf * math.cos(angle_front), rf * math.sin(angle_front)
            xb, yb = rb * math.cos(angle_back), rb * math.sin(angle_back)
            heading_error = -math.atan2(yf - yb, xf - xb)
            if abs(heading_error) <= math.radians(25):
                headings.append(heading_error)

        heading = None
        if headings:
            heading = sorted(headings)[len(headings) // 2]
        return front_distance, left_distance, right_distance, heading

    @staticmethod
    def use_corridor_assist(x, y):
        """只在双侧墙面规则的直线区域使用墙距纠偏，转弯和窄门跳过。"""
        # 路线以出生点为原点，y=0 对应世界 y=-5.05。
        if 0.3 <= x <= 5.7 and abs(y) <= 0.9:
            return True  # 南侧出发走廊
        if 6.6 <= x <= 7.4 and 1.45 <= y <= 8.65:
            return True  # 东侧走廊
        if abs(y - 10.10) <= 0.6 and (3.3 <= x <= 5.7 or -5.7 <= x <= -1.5):
            return True  # 北侧：避开窄通道
        if -7.4 <= x <= -6.6 and 1.5 <= y <= 8.65:
            if 1.9 <= y <= 3.8 or 6.0 <= y <= 8.0:
                return False  # 西侧凸起物附近不用对称墙距
            return True
        if abs(y) <= 0.6 and (-5.7 <= x <= -4.6 or -2.9 <= x <= -1.5):
            return True  # 南侧返回路段，避开凸起物
        return False

    def get_slam_pose(self):
        """返回 map 坐标系下的小车 (x, y, yaw)；过期或丢失时安全停车。"""
        try:
            tf = self.tf_buffer.lookup_transform(
                "map",
                "base_footprint",
                Time(),
                timeout=Duration(seconds=TF_TIMEOUT_SECONDS)
            )
        except TransformException as exc:
            raise RuntimeError(
                "没有可用的 map -> base_footprint 变换，"
                "请先启动 slam_toolbox 并检查 /tf：" + str(exc)
            ) from exc

        # 与 Gazebo 使用同一仿真时钟，禁止用系统墙钟比较 /tf 的时间戳。
        stamp = Time.from_msg(tf.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        if age > TF_MAX_AGE_SECONDS or age < -TF_MAX_AGE_SECONDS:
            raise RuntimeError(
                f"SLAM 位姿时间戳异常（相差 {age:.2f} 秒），安全停车"
            )

        t = tf.transform.translation
        q = tf.transform.rotation
        x, y, yaw = t.x, t.y, quaternion_to_yaw(q)
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise RuntimeError("SLAM 位姿数据无效，安全停车")
        return x, y, yaw

    def wait_for_slam_pose(self):
        """等待 SLAM 开始发布 map->odom；初始位置与航向必须正确。"""
        deadline = time.monotonic() + TF_WAIT_SECONDS
        last_error = None
        while rclpy.ok() and time.monotonic() < deadline:
            try:
                x, y, yaw = self.get_slam_pose()
                if (
                    math.hypot(x, y) > 0.30
                    or abs(normalize_angle(yaw)) > math.radians(20)
                ):
                    raise RuntimeError(
                        f"SLAM 起点位姿 ({x:.2f}, {y:.2f}, "
                        f"{math.degrees(yaw):.1f}°) 偏离路线原点；"
                        "请重新启动 Gazebo 和 SLAM"
                    )
                return
            except RuntimeError as exc:
                last_error = exc
                # 等待变换正常出现；检查起点错误也会在超时后报出。
                time.sleep(0.2)
        raise RuntimeError(
            f"等待 SLAM 定位超时：{last_error}"
        )

    def command(self, linear, angular):
        """发送速度指令。"""

        message = Twist()

        message.linear.x = float(linear)
        message.angular.z = float(angular)

        self.publisher.publish(message)

    def stop(self):
        """在 ROS 2 关闭前发布停车指令。"""

        if not rclpy.ok():
            return

        for _ in range(15):
            self.command(0.0, 0.0)
            time.sleep(0.05)

        self.get_logger().info(
            "已发送停车指令"
        )

    def follow(
        self,
        points,
        obstacles,
        test_seconds
    ):
        """使用激光 SLAM 的地图位姿跟踪路线。"""

        start = time.monotonic()
        last_log = start
        progress = 0

        max_curvature = (
            math.tan(MAX_STEERING)
            / WHEELBASE
        )

        while rclpy.ok():

            now = time.monotonic()
            elapsed = now - start

            if (
                test_seconds is not None
                and elapsed >= test_seconds
            ):
                self.get_logger().info(
                    "短测时间到，结束路径跟踪"
                )
                break

            if elapsed >= MAX_RUN_SECONDS:
                raise RuntimeError(
                    "巡场运行超时"
                )

            # SLAM 估计的 map 坐标从小车出生位置附近的 (0,0) 开始。
            # 仅为查询设计图纸中的障碍物，才换算成世界的固定坐标。
            x, y, yaw = self.get_slam_pose()
            front, left, right, wall_heading = self.get_scan_info()
            world_x = x + SPAWN_X
            world_y = y + SPAWN_Y

            clearance, obstacle_name = (
                get_clearance(
                    world_x,
                    world_y,
                    yaw,
                    obstacles
                )
            )

            if clearance < MIN_CLEARANCE:
                raise RuntimeError(
                    f"距离 {obstacle_name} 仅 "
                    f"{clearance:.3f} m，"
                    "触发安全停车"
                )

            # 找到当前位置附近的路线点。
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

            progress = max(
                progress,
                nearest
            )

            # 判断是否已经完成一圈。
            goal_distance = math.hypot(
                points[-1][0] - x,
                points[-1][1] - y
            )

            if (
                progress >= len(points) - 6
                and goal_distance < 0.25
            ):
                self.get_logger().info(
                    "已完成一圈巡场路线"
                )
                break

            # 选择前视目标点。
            target = progress

            while target < len(points) - 1:

                tx, ty, _ = points[target]

                if (
                    math.hypot(
                        tx - x,
                        ty - y
                    )
                    >= LOOKAHEAD
                ):
                    break

                target += 1

            tx, ty, _ = points[target]

            dx = tx - x
            dy = ty - y

            distance = max(
                0.01,
                math.hypot(dx, dy)
            )

            alpha = normalize_angle(
                math.atan2(dy, dx) - yaw
            )

            # Pure Pursuit 曲率公式。
            curvature = (
                2.0 * math.sin(alpha)
                / distance
            )

            curvature = max(
                -max_curvature,
                min(max_curvature, curvature)
            )

            slow_end = min(
                len(points),
                target + 6
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

            angular = speed * curvature

            # SLAM 里程计可能漏报车辆的真实偏航。
            # 使用 /scan 测得的两侧墙距和墙面角度修正直线航向。
            assist = 0.0
            if self.use_corridor_assist(x, y) and wall_heading is not None:
                # 左距小于右距：已偏向左侧墙，应向右打方向。
                lateral_error = left - right
                assist = (
                    WALL_LATERAL_GAIN * lateral_error
                    - WALL_HEADING_GAIN * wall_heading
                )
                assist = max(-WALL_MAX_CORRECTION, min(WALL_MAX_CORRECTION, assist))
                angular += assist

            # 再次限制总曲率，避免下发过大的转向指令。
            max_angular = speed * max_curvature
            angular = max(-max_angular, min(max_angular, angular))

            self.command(
                speed,
                angular
            )

            if now - last_log >= 5.0:

                self.get_logger().info(
                    f"进度 {progress + 1}/"
                    f"{len(points)}，"
                    f"SLAM 估计位置 "
                    f"({world_x:.2f}, "
                    f"{world_y:.2f})，"
                    f"SLAM 估计航向 "
                    f"{math.degrees(yaw):.1f}°，"
                    f"估计障碍物距离 {clearance:.2f} m，"
                    f"雷达前/左/右 {front:.2f}/"
                    f"{left:.2f}/{right:.2f} m，"
                    f"雷达转向修正 {assist:+.3f} rad/s"
                )

                last_log = now

            time.sleep(0.10)


# ==========================================
# 8. 程序入口
# ==========================================

def main():

    parser = argparse.ArgumentParser(
        description="M3-2 激光 SLAM 定位巡场"
    )

    parser.add_argument(
        "--run",
        action="store_true",
        help="实际驱动小车"
    )

    parser.add_argument(
        "--test-seconds",
        type=float,
        default=None,
        help="指定短测秒数"
    )

    args = parser.parse_args()

    if (
        args.test_seconds is not None
        and args.test_seconds <= 0
    ):
        parser.error(
            "--test-seconds 必须大于 0"
        )

    points = create_route()
    obstacles = get_obstacles()

    print(
        "========== M3-2 路线检查 =========="
    )

    print(
        f"路线点数量：{len(points)}"
    )

    print(
        f"障碍物数量：{len(obstacles)}"
    )

    print(
        f"直行速度：{STRAIGHT_SPEED} m/s"
    )

    print(
        f"转弯速度：{TURN_SPEED} m/s"
    )

    print(
        "反馈来源：SLAM TF + /scan 激光纠偏和防撞"
    )

    print(
        f"定位超时停车阈值：{TF_MAX_AGE_SECONDS} 秒"
    )
    print(
        f"雷达前方停车阈值：{FRONT_STOP_DISTANCE} m"
    )

    if not args.run:
        print(
            "预览完成，小车不会运动"
        )
        return

    rclpy.init(
        signal_handler_options=(
            SignalHandlerOptions.NO
        )
    )

    node = None

    try:
        node = RouteDriver()

        # 首先等待 SLAM 稳定建立地图坐标系，再让小车移动。
        node.wait_for_slam_pose()
        time.sleep(0.7)

        node.get_logger().info(
            "开始使用 SLAM 位姿跟踪路线（不使用 Gazebo 真值）"
        )

        node.follow(
            points,
            obstacles,
            args.test_seconds
        )

    except KeyboardInterrupt:
        print(
            "用户中断，准备停车"
        )

    except Exception as error:
        print(
            f"自动驾驶停止：{error}"
        )

    finally:

        if node is not None:

            try:
                node.stop()
            finally:
                node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()