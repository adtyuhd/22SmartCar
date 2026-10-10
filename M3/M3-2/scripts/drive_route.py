
#!/usr/bin/env python3
"""M3-2 Gazebo 真值反馈巡场程序。"""

import argparse
import math
import subprocess
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist


# 车辆几何
WHEELBASE = 0.28
MAX_STEERING = math.radians(30.0)
CAR_LENGTH = 0.45
CAR_WIDTH = 0.22

# 降低速度，增加实际跟踪安全余量
STRAIGHT_SPEED = 0.16
TURN_SPEED = 0.08

TURN_RADIUS = 0.8
POINT_STEP = 0.10
LOOKAHEAD = 0.45

# 车身距障碍物小于此值时主动停车
MIN_CLEARANCE = 0.12

# Gazebo 世界中的出生位置
SPAWN_X = 0.0
SPAWN_Y = -5.05

MAX_RUN_SECONDS = 600.0


def normalize_angle(angle):
    return math.atan2(
        math.sin(angle),
        math.cos(angle)
    )


def smoothstep(t):
    return t * t * (3.0 - 2.0 * t)


def create_route():
    """创建与原程序相同的环形路线。"""

    points = []

    def add_point(x, y, slow=False):
        if points:
            px, py, previous_slow = points[-1]
            if math.hypot(x - px, y - py) < 1e-8:
                points[-1] = (
                    px, py, previous_slow or slow
                )
                return
        points.append((x, y, slow))

    def add_line(x0, y0, x1, y1, slow=False):
        length = math.hypot(x1 - x0, y1 - y0)
        count = max(1, math.ceil(length / POINT_STEP))

        for i in range(count + 1):
            t = i / count
            add_point(
                x0 + (x1 - x0) * t,
                y0 + (y1 - y0) * t,
                slow
            )

    def add_arc(cx, cy, radius, start, end):
        length = radius * abs(end - start)
        count = max(1, math.ceil(length / POINT_STEP))

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

    # 南侧：向东
    add_line(0, 0, 6.25, 0)

    # 东南角
    add_arc(
        6.25, 0.8, TURN_RADIUS,
        -math.pi / 2, 0
    )

    # 东侧：向北
    add_line(7.05, 0.8, 7.05, 9.3)

    # 东北角
    add_arc(
        6.25, 9.3, TURN_RADIUS,
        0, math.pi / 2
    )

    # 北侧：向西
    add_line(6.25, 10.10, 4.5, 10.10)

    # 北侧窄通道
    add_transition(
        4.5, 2.7, 10.10, 9.625
    )

    add_line(
        2.7, 9.625,
        1.3, 9.625,
        slow=True
    )

    add_transition(
        1.3, -0.5, 9.625, 10.10
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

    # 西侧：向南
    add_line(-7.05, 9.3, -7.05, 0.8)

    # 西南角
    add_arc(
        -6.25, 0.8, TURN_RADIUS,
        math.pi, 3 * math.pi / 2
    )

    # 返回起点
    add_line(-6.25, 0, 0, 0)

    return points


def get_gazebo_pose():
    """
    查询 Gazebo 中 smart_car 的真实位置。

    gz model -p 返回：
    x y z roll pitch yaw
    """

    result = subprocess.run(
        ["gz", "model", "-m", "smart_car", "-p"],
        capture_output=True,
        text=True,
        timeout=3.0,
        check=True
    )

    values = [
        float(value)
        for value in result.stdout.split()
    ]

    if len(values) != 6:
        raise RuntimeError(
            "无法解析 Gazebo 真实位姿："
            + result.stdout.strip()
        )

    x, y, z, roll, pitch, yaw = values

    if not all(math.isfinite(v) for v in values):
        raise RuntimeError("Gazebo 位姿包含无效数值")

    return x, y, yaw


def get_obstacles():
    """从 scene.world 读取固定障碍物碰撞盒。"""

    world_path = (
        Path(__file__).resolve().parent.parent
        / "worlds" / "scene.world"
    )

    with open(
        world_path,
        encoding="utf-8-sig"
    ) as file:
        root = ET.fromstring(file.read().lstrip())

    world = root.find("world")

    if world is None:
        raise RuntimeError("scene.world 没有 world 元素")

    obstacles = []

    for model in world.findall("model"):
        name = model.get("name", "unknown")

        model_pose = [
            float(v)
            for v in model.findtext(
                "pose", "0 0 0 0 0 0"
            ).split()
        ]

        for link in model.findall("link"):
            link_pose = [
                float(v)
                for v in link.findtext(
                    "pose", "0 0 0 0 0 0"
                ).split()
            ]

            for collision in link.findall("collision"):
                box = collision.find("geometry/box")

                if box is None:
                    continue

                size = [
                    float(v)
                    for v in box.findtext("size").split()
                ]

                collision_pose = [
                    float(v)
                    for v in collision.findtext(
                        "pose", "0 0 0 0 0 0"
                    ).split()
                ]

                # 当前场景使用的是无旋转长方体。
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
                        "静态碰撞检查暂不支持旋转障碍物"
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


def get_clearance(x, y, yaw, obstacles):
    """
    使用旋转车身的外接矩形估计安全间隙。

    这是保守的近似值。
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


class RouteDriver(Node):

    def __init__(self):
        super().__init__("m3_2_drive_route")

        self.publisher = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

    def command(self, linear, angular):
        message = Twist()
        message.linear.x = float(linear)
        message.angular.z = float(angular)
        self.publisher.publish(message)

    def stop(self):
        if not rclpy.ok():
            return

        for _ in range(15):
            self.command(0.0, 0.0)
            time.sleep(0.05)

        self.get_logger().info("已发送停车指令")

    def follow(self, points, obstacles, test_seconds):
        """使用 Gazebo 真实位置进行 Pure Pursuit。"""

        start = time.monotonic()
        last_log = start
        progress = 0

        max_curvature = (
            math.tan(MAX_STEERING) / WHEELBASE
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
                raise RuntimeError("巡场运行超时")

            # 核心修复：直接读取真实位姿。
            world_x, world_y, yaw = get_gazebo_pose()

            # 转换成原路线使用的局部坐标。
            x = world_x - SPAWN_X
            y = world_y - SPAWN_Y

            clearance, obstacle_name = get_clearance(
                world_x, world_y, yaw, obstacles
            )

            if clearance < MIN_CLEARANCE:
                raise RuntimeError(
                    f"距离 {obstacle_name} 仅 "
                    f"{clearance:.3f} m，触发安全停车"
                )

            # 搜索附近最近路线点。
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

                if math.hypot(tx - x, ty - y) >= LOOKAHEAD:
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

            curvature = (
                2.0 * math.sin(alpha) / distance
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

            self.command(speed, angular)

            if now - last_log >= 5.0:
                self.get_logger().info(
                    f"进度 {progress + 1}/{len(points)}，"
                    f"Gazebo 位置 ({world_x:.2f}, "
                    f"{world_y:.2f})，"
                    f"真实航向 {math.degrees(yaw):.1f}°，"
                    f"最近障碍物距离 {clearance:.2f} m"
                )

                last_log = now

            time.sleep(0.10)


def main():
    parser = argparse.ArgumentParser(
        description="M3-2 Gazebo 真实位姿巡场"
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
        parser.error("--test-seconds 必须大于 0")

    points = create_route()
    obstacles = get_obstacles()

    print("========== M3-2 路线检查 ==========")
    print(f"路线点数量：{len(points)}")
    print(f"障碍物数量：{len(obstacles)}")
    print(f"直行速度：{STRAIGHT_SPEED} m/s")
    print(f"转弯速度：{TURN_SPEED} m/s")
    print("反馈来源：Gazebo 真实位姿")

    if not args.run:
        print("预览完成，小车不会运动")
        return

    # 启动前确认小车已经恢复到出生位置。
    x, y, yaw = get_gazebo_pose()

    if (
        math.hypot(x - SPAWN_X, y - SPAWN_Y) > 0.30
        or abs(normalize_angle(yaw)) > math.radians(20)
    ):
        raise RuntimeError(
            "小车不在出生位置，请先重新启动 Gazebo"
        )

    rclpy.init(
        signal_handler_options=SignalHandlerOptions.NO
    )

    node = None

    try:
        node = RouteDriver()

        # 给 ROS 2 发布器短暂的发现时间。
        time.sleep(0.7)

        node.get_logger().info(
            "开始使用 Gazebo 真实位姿跟踪路线"
        )

        node.follow(
            points,
            obstacles,
            args.test_seconds
        )

    except KeyboardInterrupt:
        print("用户中断，准备停车")

    except Exception as error:
        print(f"自动驾驶停止：{error}")

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