#!/usr/bin/env python3
"""Isolated Gazebo/Nav2 trials; complete restart per attempt; append-only evidence."""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]


def bootstrap():
    if os.environ.get('M34_READY') == '1':
        return
    runtime = ROOT/'results/runtime'
    runtime.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env.update(PATH='/usr/bin:/bin:/usr/sbin:/sbin:'+env.get('PATH',''),
               PYTHONNOUSERSITE='1', PYTHONDONTWRITEBYTECODE='1', M34_READY='1',
               ROS_DOMAIN_ID='33', RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
               GAZEBO_MASTER_URI='http://127.0.0.1:11346',
               ROS_LOG_DIR=str(ROOT/'results/ros-log'), TMPDIR=str(runtime),
               MPLCONFIGDIR=str(ROOT/'results/mpl'), GAZEBO_LOG_PATH=str(runtime),
               XDG_CACHE_HOME=str(runtime/'cache'), GAZEBO_MODEL_DATABASE_URI='')
    env.update(LIBGL_ALWAYS_SOFTWARE='1',QT_X11_NO_MITSHM='1')
    # Source existing overlays read-only; never build in another module.
    command = ('source /opt/ros/humble/setup.bash\n'
               'source "$1/../M3-1/m3_ws/install/setup.bash"\n'
               'source "$1/../M3-3-deps/install/local_setup.bash"\n'
               'shift\nexec /usr/bin/python3 "$@"')
    os.execve('/bin/bash', ['bash','-c',command,'m34',str(ROOT),str(Path(__file__).resolve()),*sys.argv[1:]],env)


def args():
    from inject_obstacle import coordinates
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--planners',default='A,B')
    p.add_argument('--runs',type=int,default=10)
    p.add_argument('--obstacle-at',type=coordinates,default=(2.5,-5.05),help='Gazebo world x,y')
    p.add_argument('--delay',type=float,default=3.0,help='simulation seconds after goal acceptance')
    p.add_argument('--obstacle-size',type=float,default=.6)
    p.add_argument('--goal',default='5.5,0,0',help='map x,y,yaw degrees')
    p.add_argument('--timeout',type=float,default=90,help='navigation simulation seconds')
    p.add_argument('--out',type=Path,default=ROOT/'results')
    p.add_argument('--baseline',action='store_true',help='no obstacle; record reference length')
    p.add_argument('--check',action='store_true',help='launch and check interfaces only')
    p.add_argument('--inspect',action='store_true',help='read live costmap / TF diagnostics without launching')
    p.add_argument('--campaign',action='store_true',help='fresh complete reference/comparison/tuning/boundary campaign')
    p.add_argument('--audit-existing',action='store_true',help='with --campaign, check existing real stages and regenerate artifacts without rerunning simulation')
    a=p.parse_args()
    a.planners=a.planners.split(',')
    a.goal=tuple(float(v) for v in a.goal.split(','))
    a.out=a.out.resolve()
    if not a.out.is_relative_to(ROOT): p.error('all output must stay inside M3-4')
    if not set(a.planners)<= {'A','B','B_tuned'}: p.error('planners: A,B,B_tuned')
    if len(set(a.planners))!=len(a.planners): p.error('duplicate planner names are not allowed')
    if 'B_tuned' in a.planners and len(a.planners)>1 and not a.baseline: p.error('run tuned comparison separately')
    if a.audit_existing and not a.campaign: p.error('--audit-existing requires --campaign')
    if len(a.goal)!=3 or not all(math.isfinite(v) for v in (*a.goal,*a.obstacle_at,a.obstacle_size,a.delay,a.timeout)):
        p.error('finite coordinates and numeric arguments required')
    if not 2<=a.delay<=4 or a.runs<1 or a.obstacle_size<=0 or a.timeout<=0: p.error('invalid delay/runs/size/timeout')
    return a


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid,signal.SIGINT)
        try: process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGTERM)
            try: process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL); process.wait()


def trial(a, planner, run, folder):
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from nav2_msgs.action import NavigateToPose
    from nav2_msgs.msg import BehaviorTreeLog
    from nav_msgs.msg import Odometry, OccupancyGrid, Path as NavPath
    from gazebo_msgs.msg import ModelStates
    from sensor_msgs.msg import LaserScan, Image
    from rcl_interfaces.msg import Log
    from tf2_ros import Buffer, TransformListener
    from rclpy.time import Time
    from inject_obstacle import spawn
    from eval_metrics import metrics, wrap
    import xml.etree.ElementTree as ET
    static_boxes=[]
    for model in ET.parse(ROOT.parent/'M3-2/worlds/scene.world').findall('.//world/model'):
        size=model.find('link/collision/geometry/box/size')
        if size is not None:
            pose=[float(v) for v in model.findtext('pose').split()]
            dims=[float(v) for v in size.text.split()]
            static_boxes.append((model.attrib['name'],pose[0],pose[1],dims[0]/2,dims[1]/2))

    config=ROOT/'config'/({'A':'planner_a_params.yaml','B':'planner_b_params.yaml','B_tuned':'planner_b_tuned.yaml'}[planner])
    command=['ros2','launch',str(ROOT.parent/'M3-3/launch/bringup.launch.py'),
             'gui:=false','rviz:=false','nav2_params:='+str(config)]
    row=dict(planner=planner,run=run,kind='baseline' if a.baseline else 'obstacle',success=0,
             reason='infrastructure_error',nav_sim_s='',nav_wall_s='',path_m='',extra_path_m='',
             smoothness_rad_m='',heading_rad_m='',path_source='gazebo_truth',obstacle_x=a.obstacle_at[0],obstacle_y=a.obstacle_at[1],
             obstacle_size=a.obstacle_size,delay_requested=a.delay,delay_actual='',scan_detected=0,
             path_intersection=0,collision=0,events=str((folder/'events.jsonl').relative_to(a.out)),
             config_sha256=hashlib.sha256(config.read_bytes()).hexdigest())
    (folder/'manifest.json').write_text(json.dumps(dict(command=command,argv=sys.argv,config=config.read_text(),
             map_sha256=hashlib.sha256((ROOT.parent/'M3-2/maps/map.pgm').read_bytes()).hexdigest(),
             metric='sum(abs(delta atan(0.28*kappa)))/distance; independent truth at 0.05m',
             script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
             start_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime())),indent=2))
    logfile=(folder/'launch.log').open('w')
    process=subprocess.Popen(command,stdout=logfile,stderr=subprocess.STDOUT,start_new_session=True)
    node=None
    try:
        rclpy.init()
        class Recorder(Node):
            def __init__(self):
                super().__init__('m34_benchmark')
                self.set_parameters([Parameter('use_sim_time',value=True)])
                self.fp=(folder/'events.jsonl').open('x')
                self.active=False; self.odom=[]; self.truth=[]; self.pose=None; self.scan=None
                self.plans=[]; self.injected=False; self.collision=False; self.detected=False
                self.image=None; self.saved=set(); self.saved_hashes=set(); self.begin=None; self.last_record={}
                self.initial_truth=None; self.initial_map=None
                self.client=ActionClient(self,NavigateToPose,'/navigate_to_pose')
                self.buffer=Buffer(); self.listener=TransformListener(self.buffer,self)
                self.create_subscription(Odometry,'/odom',self.on_odom,10)
                self.create_subscription(ModelStates,'/gazebo/model_states',self.on_truth,10)
                self.create_subscription(NavPath,'/plan',self.on_plan,10)
                self.create_subscription(LaserScan,'/scan',self.on_scan,qos_profile_sensor_data)
                self.create_subscription(Image,'/camera/image_raw',self.on_image,qos_profile_sensor_data)
                self.create_subscription(Log,'/rosout',self.on_log,100)
                self.create_subscription(BehaviorTreeLog,'/behavior_tree_log',self.on_bt,100)
                qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
                for topic in ('/global_costmap/costmap','/local_costmap/costmap'):
                    self.create_subscription(OccupancyGrid,topic,lambda msg,t=topic:self.on_costmap(msg,t),qos)
            def sim(self): return self.get_clock().now().nanoseconds/1e9
            def record(self,event,**kw):
                self.fp.write(json.dumps(dict(event=event,sim=self.sim(),wall=time.monotonic(),active=self.active,**kw))+'\n'); self.fp.flush()
            @staticmethod
            def value(p):
                q=p.orientation
                return [p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
            def on_odom(self,msg):
                v=self.value(msg.pose.pose)
                if self.active: self.odom.append(v)
                self.record('odom',value=v,v=msg.twist.twist.linear.x,w=msg.twist.twist.angular.z)
            def on_truth(self,msg):
                if 'smart_car' not in msg.name: return
                self.pose=self.value(msg.pose[msg.name.index('smart_car')])
                if self.active:
                    self.truth.append(self.pose)
                    boxes=static_boxes+([('injected',*a.obstacle_at,a.obstacle_size/2,a.obstacle_size/2)] if self.injected else [])
                    # SAT: URDF body rectangle vs physical world rectangles, including static walls.
                    for name,ox,oy,sx,sy in boxes:
                        x,y,h=self.pose
                        dx,dy=ox-x,oy-y; c,t=math.cos(h),math.sin(h)
                        overlap=(abs(dx)<.225*abs(c)+.11*abs(t)+sx and
                                 abs(dy)<.225*abs(t)+.11*abs(c)+sy and
                                 abs(dx*c+dy*t)<.225+sx*abs(c)+sy*abs(t) and
                                 abs(-dx*t+dy*c)<.11+sx*abs(t)+sy*abs(c))
                        if overlap and not self.collision:
                            self.collision=True; self.record('collision_geometry',value=self.pose,object=name)
                self.record('truth',value=self.pose)
            def on_plan(self,msg):
                pp=[[p.pose.position.x,p.pose.position.y] for p in msg.poses]
                self.plans.append(pp); self.record('plan',frame=msg.header.frame_id,points=pp)
            def on_log(self,msg):
                if 'Twist message without stamped is deprecated' in msg.msg:
                    return
                if msg.level>=30 or any(v in msg.msg.lower() for v in ('failed','recovery','collision')):
                    self.record('rosout',name=msg.name,level=msg.level,message=msg.msg,
                                stamp=msg.stamp.sec+msg.stamp.nanosec/1e9)
            def on_bt(self,msg):
                for event in msg.event_log:
                    self.record('bt_status',name=event.node_name,previous=event.previous_status,
                                current=event.current_status,
                                stamp=event.timestamp.sec+event.timestamp.nanosec/1e9)
            def on_costmap(self,msg,topic):
                if self.sim()-self.last_record.get(topic,-1e9)<1: return
                self.last_record[topic]=self.sim()
                self.record('costmap',topic=topic,frame=msg.header.frame_id,resolution=msg.info.resolution,
                            width=msg.info.width,height=msg.info.height,
                            origin=[msg.info.origin.position.x,msg.info.origin.position.y],data=list(msg.data))
            def on_image(self,msg):
                if self.image is None:
                    self.record('camera_available',encoding=msg.encoding,width=msg.width,height=msg.height)
                self.image=msg
            def snapshot(self,label):
                if self.image is None or label in self.saved: return
                from PIL import Image as PILImage
                msg=self.image
                stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
                if self.sim()-stamp>.5: return
                if self.sim()-self.last_record.get('snapshot_'+label,-1e9)<.1: return
                self.last_record['snapshot_'+label]=self.sim()
                if msg.encoding not in ('rgb8','bgr8'): return
                im=PILImage.frombytes('RGB',(msg.width,msg.height),bytes(msg.data),'raw',
                                     'RGB' if msg.encoding=='rgb8' else 'BGR',msg.step)
                digest=hashlib.sha256(im.tobytes()).hexdigest()
                if digest in self.saved_hashes: return
                if label=='02_detected' and sum(r>30 and g<r*.5 and b<r*.5 for r,g,b in im.getdata())<50: return
                im.save(folder/(label+'.png')); self.saved.add(label); self.saved_hashes.add(digest)
                self.record('camera_frame',label=label,image_stamp=stamp,image_frame=msg.header.frame_id)
            def on_scan(self,msg):
                self.scan=msg
                if not self.injected or self.pose is None: return
                try:
                    tf=self.buffer.lookup_transform('world','lidar_link',Time())
                except Exception:
                    # world TF is intentionally not published; independent Gazebo body pose + URDF lidar offset.
                    x,y,h=self.pose
                    # Original lidar_joint has zero planar translation from base_link.
                else:
                    v=tf.transform; x,y=v.translation.x,v.translation.y
                    q=v.rotation; h=math.atan2(2*q.w*q.z,1-2*q.z*q.z)
                hits=[]
                for i,d in enumerate(msg.ranges):
                    if not math.isfinite(d): continue
                    angle=h+msg.angle_min+i*msg.angle_increment
                    px,py=x+d*math.cos(angle),y+d*math.sin(angle)
                    if abs(px-a.obstacle_at[0])<a.obstacle_size/2+.08 and abs(py-a.obstacle_at[1])<a.obstacle_size/2+.08:
                        hits.append([px,py])
                if hits and not self.detected:
                    self.detected=True; self.record('scan_detection',hits=hits)
                    self.snapshot('02_detected')
                if self.active and self.sim()-self.last_record.get('scan',-1e9)>.5:
                    self.last_record['scan']=self.sim()
                    self.record('scan',angle_min=msg.angle_min,angle_increment=msg.angle_increment,
                                ranges=[float(d) if math.isfinite(d) else None for d in msg.ranges])
        node=Recorder()
        deadline=time.monotonic()+90
        ready=False
        while time.monotonic()<deadline and process.poll() is None:
            rclpy.spin_once(node,timeout_sec=.05)
            text=(folder/'launch.log').read_text(errors='replace')
            if 'process has died' in text:
                raise RuntimeError('launch child died; inspect launch.log')
            if ('Managed nodes are active' in text and node.scan is not None and node.pose is not None
                and node.client.server_is_ready() and node.image is not None):
                try: node.buffer.lookup_transform('map','base_footprint',Time())
                except Exception: continue
                ready=True; break
        if not ready: raise RuntimeError('startup readiness timeout or launch exit')
        # Settle localization at the initial pose before accepting the goal.
        until=node.sim()+2; wall=time.monotonic()+15
        while node.sim()<until and time.monotonic()<wall: rclpy.spin_once(node,timeout_sec=.02)
        node.record('ready',pose=node.pose)
        node.initial_truth=list(node.pose)
        initial_tf=node.buffer.lookup_transform('map','base_footprint',Time()).transform
        iq=initial_tf.rotation
        node.initial_map=[initial_tf.translation.x,initial_tf.translation.y,
                          math.atan2(2*iq.w*iq.z,1-2*iq.z*iq.z)]
        node.record('initial_alignment',world=node.initial_truth,map=node.initial_map)
        if a.check:
            row['reason']='interfaces_ready'; return row
        goal=NavigateToPose.Goal(); goal.pose.header.frame_id='map'; goal.pose.header.stamp=node.get_clock().now().to_msg()
        goal.pose.pose.position.x,goal.pose.pose.position.y=a.goal[:2]
        goal.pose.pose.orientation.z=math.sin(math.radians(a.goal[2])/2)
        goal.pose.pose.orientation.w=math.cos(math.radians(a.goal[2])/2)
        future=node.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node,future,timeout_sec=10)
        if not future.done() or not future.result().accepted: raise RuntimeError('goal not accepted')
        handle=future.result(); result=handle.get_result_async()
        node.active=True; start=node.sim(); wall_start=time.monotonic(); node.begin=start
        node.record('goal_accepted',goal=a.goal)
        while not result.done() and node.sim()-start<a.timeout and time.monotonic()-wall_start<a.timeout*3:
            rclpy.spin_once(node,timeout_sec=.01)
            elapsed=node.sim()-start
            if elapsed>=1: node.snapshot('01_approach')
            if node.detected: node.snapshot('02_detected')
            if not a.baseline and not node.injected and elapsed>=a.delay:
                # Validate intersection with the accepted global path in map coordinates using measured TF alignment.
                tf=node.buffer.lookup_transform('map','base_footprint',Time())
                q=tf.transform.rotation
                mh=math.atan2(2*q.w*q.z,1-2*q.z*q.z); wh=node.pose[2]; angle=mh-wh
                wx,wy=node.pose[:2]; tx=tf.transform.translation.x; ty=tf.transform.translation.y
                ox,oy=a.obstacle_at
                mx=tx+math.cos(angle)*(ox-wx)-math.sin(angle)*(oy-wy)
                my=ty+math.sin(angle)*(ox-wx)+math.cos(angle)*(oy-wy)
                pp=node.plans[-1] if node.plans else []
                row['path_intersection']=int(any(abs(x-mx)<=a.obstacle_size/2 and abs(y-my)<=a.obstacle_size/2 for x,y in pp))
                if not row['path_intersection']: raise RuntimeError('obstacle does not intersect accepted plan; choose --obstacle-at on route')
                if metrics(node.odom)['path_m'] < .05:
                    raise RuntimeError('car not in motion before injection')
                node.record('spawn_request',xy=a.obstacle_at,size=a.obstacle_size,elapsed=elapsed,map_xy=[mx,my])
                spawn(node,a.obstacle_at,a.obstacle_size)
                node.injected=True; row['delay_actual']=node.sim()-start
                node.record('spawn_complete',elapsed=row['delay_actual'])
            if node.injected and node.pose:
                if abs(node.pose[1]-a.obstacle_at[1])>.3: node.snapshot('03_detour')
                if node.pose[0]>a.obstacle_at[0]+a.obstacle_size/2+.4: node.snapshot('04_passed')
            if node.collision: break
        elapsed=node.sim()-start; wall_elapsed=time.monotonic()-wall_start
        status=result.result().status if result.done() else None
        node.active=False
        if not result.done():
            cancel=handle.cancel_goal_async(); rclpy.spin_until_future_complete(node,cancel,timeout_sec=3)
            rclpy.spin_until_future_complete(node,result,timeout_sec=5)
        row.update(nav_sim_s=elapsed,nav_wall_s=wall_elapsed,collision=int(node.collision),scan_detected=int(node.detected))
        valid=a.baseline or (node.injected and node.detected and row['path_intersection'] and 2<=row['delay_actual']<=4)
        goal_verified=False
        if status==4:
            try:
                tf=node.buffer.lookup_transform('map','base_footprint',Time()).transform
                q=tf.rotation; yaw=math.atan2(2*q.w*q.z,1-2*q.z*q.z)
                position_error=math.hypot(tf.translation.x-a.goal[0],tf.translation.y-a.goal[1])
                yaw_error=abs(wrap(yaw-math.radians(a.goal[2])))
                goal_verified=position_error<=.2 and yaw_error<=.4
                angle=node.initial_map[2]-node.initial_truth[2]
                dx=a.goal[0]-node.initial_map[0]; dy=a.goal[1]-node.initial_map[1]
                expected=[node.initial_truth[0]+math.cos(angle)*dx+math.sin(angle)*dy,
                          node.initial_truth[1]-math.sin(angle)*dx+math.cos(angle)*dy]
                truth_error=math.hypot(node.pose[0]-expected[0],node.pose[1]-expected[1])
                goal_verified=goal_verified and truth_error<=.3
                node.record('goal_verification',position_error=position_error,yaw_error=yaw_error,
                            truth_position_error=truth_error,expected_world=expected,truth=node.pose)
            except Exception:
                pass
        row['success']=int(status==4 and goal_verified and not node.collision and valid)
        row['reason']=('collision_geometry' if node.collision else 'invalid_obstacle_evidence' if not valid else
                       'reached' if status==4 and goal_verified else 'goal_verification_failed' if status==4 else
                       'timeout' if status is None else f'action_status_{status}')
        row.update(metrics(node.truth))
        node.record('result',**row); node.active=False
        return row
    except Exception as error:
        row['reason']=f'infrastructure_error: {error}'
        (folder/'error.txt').write_text(str(error))
        return row
    finally:
        if node:
            node.fp.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        stop(process); logfile.close()


def main():
    bootstrap()
    a=args(); a.out.mkdir(parents=True,exist_ok=True)
    if a.campaign:
        if a.out==ROOT/'results' and not a.audit_existing:
            a.out=a.out/f'reproduction-{time.time_ns()}'
            a.out.mkdir()
        if not a.audit_existing and ((a.out/'benchmark.csv').exists() or (a.out/'baseline.csv').exists()):
            raise SystemExit('campaign requires a fresh output directory; existing evidence is never overwritten')
        common=['--out',str(a.out),'--obstacle-at',','.join(map(str,a.obstacle_at)),
                '--obstacle-size',str(a.obstacle_size),'--delay',str(a.delay),
                '--goal',','.join(map(str,a.goal)),'--timeout',str(a.timeout)]
        def execute(extra,base=common):
            if not a.audit_existing:
                subprocess.run(['/usr/bin/python3',str(Path(__file__).resolve()),*base,*extra],check=True)
        def check_stage(path,counts):
            from collections import Counter
            rr=list(csv.DictReader(path.open()))
            if Counter(r['planner'] for r in rr)!=Counter(counts):
                raise SystemExit(f'incomplete stage: {path}')
            for r in rr:
                events=path.parent/r['events']
                if (not events.exists() or r['scan_detected']!='1' or r['path_intersection']!='1'
                    or not r['delay_actual'] or not 2<=float(r['delay_actual'])<=4):
                    raise SystemExit(f'invalid injection evidence in stage: {path}')
            print(f'CHECKED {path}: {len(rr)} real attempts',flush=True)
        execute(['--check','--planners','A','--runs','1'])
        execute(['--baseline','--planners','A,B,B_tuned','--runs','1'])
        refs=list(csv.DictReader((a.out/'baseline.csv').open()))
        if len(refs)!=3 or not all(r['success']=='1' for r in refs):
            raise SystemExit('reference stage failed; do not proceed to comparison')
        execute(['--planners','A,B','--runs','10'])
        check_stage(a.out/'benchmark.csv',{'A':10,'B':10})
        from eval_metrics import summarize,plots,reports
        original=list(csv.DictReader((a.out/'benchmark.csv').open()))
        summary=summarize(original)
        if any(summary[g]['attempts']!=10 for g in ('A','B')):
            raise SystemExit('comparison stage has not completed ten attempts per scheme')
        if summary['B']['successes']>summary['A']['successes']:
            raise SystemExit('B was not poorer in this campaign; reassess tuning rather than claim unsupported improvement')
        execute(['--planners','B_tuned','--runs','10'])
        check_stage(a.out/'benchmark_tuned.csv',{'B_tuned':10})
        boundaries=[('large_0p8',['--obstacle-size','0.8']),
                    ('blocked_1p6',['--obstacle-size','1.6']),
                    ('near_1p4',['--obstacle-at','1.4,-5.05'])]
        for label,extra in boundaries:
            execute(['--planners','A','--runs','3','--out',str(a.out/'boundaries'/label),*extra])
            check_stage(a.out/'boundaries'/label/'benchmark.csv',{'A':3})
        plots(a.out); audit=reports(a.out)
        print(json.dumps({'output':str(a.out),'audit':audit},indent=2),flush=True)
        return 0 if all(audit.values()) else 3
    if a.inspect:
        import rclpy
        from rclpy.qos import QoSProfile,DurabilityPolicy
        from nav_msgs.msg import OccupancyGrid
        from tf2_ros import Buffer,TransformListener
        from rclpy.time import Time
        rclpy.init(); n=rclpy.create_node('m34_inspect')
        b=Buffer(); listener=TransformListener(b,n)
        def cb(msg):
            cells=[]
            for i,v in enumerate(msg.data):
                x=msg.info.origin.position.x+(i%msg.info.width+.5)*msg.info.resolution
                y=msg.info.origin.position.y+(i//msg.info.width+.5)*msg.info.resolution
                if abs(x-a.obstacle_at[0])<.5 and abs(y)<.5 and v>0: cells.append((x,y,v))
            print('COSTMAP',msg.info.width,msg.info.height,'near obstacle nonzero',len(cells),'sample',cells[:6],flush=True)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for topic in ('/global_costmap/costmap','/local_costmap/costmap'):
            n.create_subscription(OccupancyGrid,topic,cb,qos)
        until=time.monotonic()+6
        while time.monotonic()<until: rclpy.spin_once(n,timeout_sec=.1)
        for target,source in [('map','lidar_link'),('map','base_footprint')]:
            try: print(target,source,b.lookup_transform(target,source,Time()))
            except Exception as e: print(e)
        n.destroy_node(); rclpy.shutdown(); return 0
    path=a.out/('baseline.csv' if a.baseline else 'benchmark_tuned.csv' if a.planners==['B_tuned'] else 'benchmark.csv')
    existing=list(csv.DictReader(path.open())) if path.exists() else []
    if not a.check and any(r['planner'] in a.planners for r in existing):
        raise SystemExit(f'{path} already contains this planner; use a fresh --out to avoid mixing batches')
    reference_file=a.out/'baseline.csv'
    references=list(csv.DictReader(reference_file.open())) if reference_file.exists() else []
    for planner in a.planners:
        for run in range(1,a.runs+1):
            folder=a.out/'raw'/f'{time.time_ns()}_{planner}_{run:02}'
            folder.mkdir(parents=True)
            print(f'START {planner} {run}/{a.runs}: {folder}',flush=True)
            row=trial(a,planner,run,folder)
            # Baseline is planner-specific and only measured successful runs count.
            refs=[float(r['path_m']) for r in references if r['kind']=='baseline' and r['planner']==planner
                  and str(r['success'])=='1']
            if not a.baseline and refs and row['path_m']!='': row['extra_path_m']=float(row['path_m'])-sum(refs)/len(refs)
            if not a.check:
                with path.open('a') as fp:
                    writer=csv.DictWriter(fp,fieldnames=list(row))
                    if fp.tell()==0: writer.writeheader()
                    writer.writerow(row)
                existing.append(row)
            print(json.dumps(row,ensure_ascii=False),flush=True)
            if row['reason'].startswith('infrastructure_error'):
                return 2  # Do not turn infrastructure failures into a fake 10-trial campaign.
    return 0


if __name__=='__main__':
    raise SystemExit(main())
