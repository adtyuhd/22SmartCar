#!/usr/bin/env python3
"""Gazebo collision + ray-visible box. Never publishes costmap data."""
import argparse
import math
import time


def sdf_box(size):
    return f'''<sdf version="1.6"><model name="temporary_obstacle"><static>true</static>
<link name="box"><collision name="collision"><geometry><box><size>{size} {size} 1</size>
</box></geometry></collision><visual name="visual"><geometry><box><size>{size} {size} 1</size>
</box></geometry><material><ambient>1 0.15 0.05 1</ambient><diffuse>1 0.15 0.05 1</diffuse>
</material></visual></link></model></sdf>'''


def spawn(node, xy, size, name="m34_obstacle"):
    from gazebo_msgs.srv import SpawnEntity
    import rclpy
    client = node.create_client(SpawnEntity, "/spawn_entity")
    if not client.wait_for_service(timeout_sec=10):
        raise RuntimeError("/spawn_entity unavailable")
    req = SpawnEntity.Request()
    req.name, req.xml, req.reference_frame = name, sdf_box(size), "world"
    req.initial_pose.position.x, req.initial_pose.position.y = xy
    req.initial_pose.position.z = 0.5
    req.initial_pose.orientation.w = 1.0
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10)
    if not future.done() or not future.result().success:
        raise RuntimeError(str(future.result() if future.done() else "spawn timeout"))
    return future.result().status_message


def coordinates(value):
    values = tuple(float(v) for v in value.split(","))
    if len(values) != 2 or not all(math.isfinite(v) for v in values):
        raise argparse.ArgumentTypeError("expected finite x,y")
    return values


def main():
    import os
    import rclpy
    from rclpy.parameter import Parameter
    from action_msgs.msg import GoalStatusArray
    from nav_msgs.msg import Odometry
    p = argparse.ArgumentParser(description="Wait for accepted navigation, then spawn a world-frame box")
    p.add_argument("--obstacle-at", type=coordinates, required=True)
    p.add_argument("--delay", type=float, default=3.0)
    p.add_argument("--obstacle-size", type=float, default=0.8)
    a = p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '33':
        p.error('use the isolated Gazebo ROS_DOMAIN_ID=33 environment')
    if not 2 <= a.delay <= 4 or not math.isfinite(a.obstacle_size) or a.obstacle_size <= 0:
        p.error("delay must be 2..4 simulation seconds; size must be positive")
    rclpy.init()
    node = rclpy.create_node("m34_inject")
    node.set_parameters([Parameter("use_sim_time", value=True)])
    accepted = []
    active = []
    positions = []
    def status(msg):
        active[:] = [s for s in msg.status_list if s.status == 2]
        if not accepted and active:
            accepted.append(node.get_clock().now().nanoseconds / 1e9)
            positions.clear()
    def odom(msg):
        if accepted:
            p=msg.pose.pose.position
            positions.append((p.x,p.y))
    node.create_subscription(GoalStatusArray, "/navigate_to_pose/_action/status", status, 10)
    node.create_subscription(Odometry, '/odom', odom, 10)
    deadline = time.monotonic() + 120
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
            if accepted and not active:
                raise RuntimeError('navigation ended before injection; no obstacle spawned')
            if accepted and node.get_clock().now().nanoseconds / 1e9 - accepted[0] >= a.delay:
                if len(positions)<2 or math.dist(positions[0],positions[-1])<.05:
                    raise RuntimeError('car has not moved 0.05m; no obstacle spawned')
                print(spawn(node, a.obstacle_at, a.obstacle_size), flush=True)
                return
        raise RuntimeError("no active navigation within 120 wall seconds")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
