#!/usr/bin/env python3
"""用真实 Nav2 map_server 加载两组地图。"""
import json
import os
from pathlib import Path
import signal
import subprocess
import rclpy
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.srv import LoadMap

ROOT=Path(__file__).resolve().parent.parent

def main():
    os.environ['ROS_DOMAIN_ID']='182'
    os.environ['ROS_LOCALHOST_ONLY']='1'
    os.environ['ROS_LOG_DIR']='/tmp/m3_2_map_check_logs'
    out=ROOT/'results/loop_comparison'
    with (out/'map_server.log').open('w') as log:
        process=subprocess.Popen(['/opt/ros/humble/lib/nav2_map_server/map_server',
            '--ros-args','-p','yaml_filename:='+str(ROOT/'maps/map_with_loop_on.yaml')],
            stdout=log,stderr=subprocess.STDOUT)
        rclpy.init()
        node=rclpy.create_node('comparison_map_check')
        try:
            change=node.create_client(ChangeState,'/map_server/change_state')
            if not change.wait_for_service(timeout_sec=30):
                raise RuntimeError('map_server 未启动')
            req=ChangeState.Request();req.transition.id=1
            future=change.call_async(req)
            rclpy.spin_until_future_complete(node,future,timeout_sec=30)
            if not future.done() or not future.result().success:
                raise RuntimeError('map_server 配置失败')
            req=ChangeState.Request();req.transition.id=3
            future=change.call_async(req)
            rclpy.spin_until_future_complete(node,future,timeout_sec=30)
            if not future.done() or not future.result().success:
                raise RuntimeError('map_server 激活失败')
            load=node.create_client(LoadMap,'/map_server/load_map')
            if not load.wait_for_service(timeout_sec=10):
                raise RuntimeError('load_map 服务不可用')
            results={}
            for label in ('loop_on','loop_off'):
                req=LoadMap.Request()
                req.map_url=str(ROOT/'maps'/('map_with_'+label+'.yaml'))
                future=load.call_async(req)
                rclpy.spin_until_future_complete(node,future,timeout_sec=30)
                if not future.done() or future.result().result!=0:
                    raise RuntimeError(label+' 地图加载失败')
                msg=future.result()
                results[label]=dict(result=msg.result,width=msg.map.info.width,
                    height=msg.map.info.height,resolution=msg.map.info.resolution,
                    cells=len(msg.map.data))
            (out/'map_server_check.json').write_text(json.dumps(results,indent=2)+'\n')
            print(json.dumps(results,indent=2))
        finally:
            node.destroy_node();rclpy.shutdown()
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.terminate();process.wait(timeout=10)

if __name__=='__main__':
    main()
