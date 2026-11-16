#!/usr/bin/env python3
"""同一 rosbag、同一参数，仅切换 do_loop_closing；在独立 ROS 域重建地图。

source /opt/ros/humble/setup.bash
/usr/bin/python3 scripts/compare_loop.py --bag bags/某次完整录包
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.serialization import deserialize_message, serialize_message
import rosbag2_py
from slam_toolbox.srv import SerializePoseGraph
from tf2_msgs.msg import TFMessage
import yaml

ROOT = Path(__file__).resolve().parent.parent


def filter_bag(source, target):
    if target.exists():
        raise FileExistsError(target)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(source), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(target), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    names = {'/scan', '/odom', '/tf', '/tf_static', '/clock'}
    for topic in reader.get_all_topics_and_types():
        if topic.name in names:
            writer.create_topic(topic)
    counts = dict.fromkeys(names, 0)
    removed = 0
    while reader.has_next():
        name, data, stamp = reader.read_next()
        if name not in names:
            continue
        if name == '/tf':
            msg = deserialize_message(data, TFMessage)
            keep = [t for t in msg.transforms if not
                    (t.header.frame_id == 'map' and t.child_frame_id == 'odom')]
            removed += len(msg.transforms) - len(keep)
            if not keep:
                continue
            msg.transforms = keep
            data = serialize_message(msg)
        writer.write(name, data, stamp)
        counts[name] += 1
    del writer
    return {'input_message_counts': counts, 'removed_map_to_odom': removed}


def stop(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            process.terminate()
            process.wait(timeout=10)


def run_variant(label, config, filtered, out, rate):
    run = out / label
    run.mkdir()
    prefix = out / 'maps' / ('map_with_' + label)
    env = dict(os.environ, ROS_LOG_DIR=str(run / 'ros_logs'))
    env['PATH'] = '/usr/bin:' + env.get('PATH', '')
    with (run / 'slam.log').open('w') as log:
        process = subprocess.Popen([
            '/opt/ros/humble/lib/slam_toolbox/sync_slam_toolbox_node',
            '--ros-args', '--params-file', str(config)], stdout=log,
            stderr=subprocess.STDOUT, env=env)
        node = rclpy.create_node('comparison_capture')
        try:
            client = node.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
            if not client.wait_for_service(timeout_sec=30):
                raise RuntimeError('SLAM 服务未启动；检查 slam.log')
            with (run / 'bag_play.log').open('w') as playback_log:
                subprocess.run(['/opt/ros/humble/bin/ros2', 'bag', 'play',
                                str(filtered), '--rate', str(rate), '--delay', '2'],
                               stdout=playback_log, stderr=subprocess.STDOUT,
                               env=env, check=True, timeout=700/rate + 60)
            # 同步后端排空队列；保存前后位姿图节点数由 graph_check 核验。
            time.sleep(12)
            request = SerializePoseGraph.Request()
            request.filename = str(run / 'graph')
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=45)
            if not future.done() or future.result().result != 0:
                raise RuntimeError('位姿图保存失败')
            with (run / 'map_save.log').open('w') as save_log:
                subprocess.run([
                    '/opt/ros/humble/lib/nav2_map_server/map_saver_cli',
                    '-f', str(prefix), '--fmt', 'pgm', '--ros-args',
                    '-p', 'save_map_timeout:=15.0', '-p', 'free_thresh_default:=0.25',
                    '-p', 'occupied_thresh_default:=0.65'], check=True,
                    stdout=save_log, stderr=subprocess.STDOUT, env=env, timeout=30)
            subprocess.run(['/opt/ros/humble/bin/ros2', 'param', 'dump', '/slam_toolbox',
                            '--output-dir', str(run)], check=True,
                           stdout=subprocess.DEVNULL, env=env, timeout=30)
            print(label + ': 地图、位姿图、运行参数已保存', flush=True)
        finally:
            node.destroy_node()
            stop(process)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--out-dir', type=Path, default=ROOT/'results/loop_comparison')
    parser.add_argument('--rate', type=float, default=2.0)
    parser.add_argument('--domain', type=int, default=181)
    args = parser.parse_args()
    if args.rate <= 0:
        parser.error('--rate 必须为正数')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ.setdefault('ROS_LOG_DIR', '/tmp/m3_2_compare_ros_logs')
    out = args.out_dir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    (out/'maps').mkdir()
    filtered = out/'input_bag'
    manifest = filter_bag(args.bag.resolve(), filtered)
    manifest.update(source_bag=str(args.bag.resolve()), replay_rate=args.rate,
                    ros_domain_id=args.domain, backend='sync_slam_toolbox_node',
                    variable='do_loop_closing',
                    source_sha256={p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                   for p in sorted(args.bag.glob('*.db3'))})
    base = yaml.safe_load((ROOT/'config/slam_params.yaml').read_text())
    configs = {}
    for label, enabled in [('loop_on', True), ('loop_off', False)]:
        params = yaml.safe_load(yaml.safe_dump(base))
        params['slam_toolbox']['ros__parameters']['do_loop_closing'] = enabled
        configs[label] = out/('params_'+label+'.yaml')
        configs[label].write_text(yaml.safe_dump(params, sort_keys=False))
    (out/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    rclpy.init()
    try:
        for label, config in configs.items():
            print('开始 ' + label, flush=True)
            run_variant(label, config, filtered, out, args.rate)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
