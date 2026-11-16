#!/usr/bin/env python3
"""量化两张回环对照地图，并导出重访墙的两次扫描拟合图。

source /opt/ros/humble/setup.bash
/usr/bin/python3 -s scripts/analyze_loop.py
"""
import argparse
import csv
from collections import deque
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess

os.environ.setdefault('MPLCONFIGDIR', '/tmp/m3_2_matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml
from eval_map import read_pgm, wall_fit, narrow_gap, free_component_sizes, make_figures

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT/'results/loop_comparison'


def boundary_check(w, h, p):
    seen = {i for i, v in enumerate(p) if v >= 250}
    queue = deque(seen)
    edge = []
    while queue:
        i = queue.popleft()
        x, y = i % w, i // w
        if x in (0, w-1) or y in (0, h-1):
            edge.append((x, y))
        for j in (i-1 if x else -1, i+1 if x+1 < w else -1,
                  i-w if y else -1, i+w if y+1 < h else -1):
            if j >= 0 and p[j] > 60 and j not in seen:
                seen.add(j)
                queue.append(j)
    return dict(reachable_nonoccupied_edge_count=len(edge),
                reachable_free_edge_count=sum(p[y*w+x] >= 250 for x,y in edge),
                reachable_edge_pixels=edge)


def occupied_islands(w, h, p):
    unseen = {i for i,v in enumerate(p) if v <= 60}
    components = []
    while unseen:
        start = unseen.pop()
        queue = [start]
        members = [start]
        while queue:
            i = queue.pop()
            x,y = i%w,i//w
            for j in (i-1 if x else -1,i+1 if x+1<w else -1,
                      i-w if y else -1,i+w if y+1<h else -1):
                if j in unseen:
                    unseen.remove(j)
                    queue.append(j)
                    members.append(j)
        xs=[i%w for i in members]; ys=[i//w for i in members]
        components.append(dict(cells=len(members),bbox_pixels=[min(xs),min(ys),max(xs),max(ys)]))
    return sorted(components,key=lambda c:c['cells'],reverse=True)


def compile_reader():
    exe = OUT/'compare_graph'
    includes = ['rclcpp', 'rcutils', 'rcl', 'rmw', 'rcl_yaml_param_parser',
                'rosidl_runtime_c', 'rosidl_runtime_cpp', 'rosidl_typesupport_interface',
                'rcpputils', 'builtin_interfaces', 'rcl_interfaces',
                'libstatistics_collector', 'statistics_msgs', 'tracetools']
    cmd = ['g++', '-std=c++17', str(ROOT/'scripts/compare_graph.cpp'),
           '-I/usr/include/eigen3', '-I/opt/ros/humble/include']
    cmd += ['-I/opt/ros/humble/include/'+d for d in includes]
    cmd += ['-L/opt/ros/humble/lib', '-Wl,-rpath,/opt/ros/humble/lib',
            '-lkartoSlamToolbox', '-lboost_serialization', '-ltbb', '-lrclcpp',
            '-o', str(exe)]
    subprocess.run(cmd, check=True)
    return exe


def rows(path):
    with path.open() as f:
        return [{k: float(v) for k,v in row.items()} for row in csv.DictReader(f)]


def revisit_wall(nodes, points):
    # 同一南侧外墙；在两次实际扫描的重叠视野选 ROI，不调整 SLAM 坐标。
    times = {int(n['id']): n['time'] for n in nodes}
    groups = [[], []]
    ids = [set(), set()]
    for p in points:
        t = times[int(p['id'])]
        if -2 <= p['x'] <= 2 and -1.5 <= p['y'] <= -0.5:
            group = 0 if t <= 50 else (1 if t >= 415 else None)
            if group is not None:
                groups[group].append((p['x'], p['y']))
                ids[group].add(int(p['id']))
    fits = []
    for ps in groups:
        if len(ps) < 30:
            raise RuntimeError('重访墙扫描点不足，请检查 points.csv 和实际地图坐标')
        a = np.array(ps)
        slope, intercept = np.polyfit(a[:,0], a[:,1], 1)
        residual = (a[:,1]-slope*a[:,0]-intercept)/math.sqrt(1+slope*slope)
        fits.append(dict(points=len(ps), slope=float(slope), intercept=float(intercept),
                         fit_rms_m=float(np.sqrt(np.mean(residual**2)))))
    first, last = fits
    denominator = math.sqrt(1+first['slope']**2)
    at_center = abs(last['intercept']-first['intercept'])/denominator
    ends = [abs((last['slope']-first['slope'])*x+
                last['intercept']-first['intercept'])/denominator for x in (-2,2)]
    windows = [[min(times[i] for i in group), max(times[i] for i in group)] for group in ids]
    return dict(roi_m=[-2,2,-1.5,-0.5], time_windows_s=windows,
                node_ids=[sorted(s) for s in ids], fits=fits,
                wall_separation_at_x0_m=at_center,
                max_fitted_separation_over_roi_m=max(ends)), groups


def main():
    global OUT
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out-dir', type=Path, default=OUT)
    parser.add_argument('--map-dir', type=Path, default=ROOT/'maps')
    args = parser.parse_args()
    OUT = args.out_dir.resolve()
    exe = compile_reader()
    results = {}
    figure, axes = plt.subplots(1,2,figsize=(12,5), constrained_layout=True)
    detail, daxes = plt.subplots(1,2,figsize=(12,4), constrained_layout=True)
    for col,label in enumerate(('loop_on','loop_off')):
        run = OUT/label
        with (run/'graph_check.txt').open('w') as log:
            subprocess.run([str(exe), str(run/'graph.posegraph'), str(run)],
                           check=True, stdout=log)
        nodes = rows(run/'nodes.csv')
        edges = rows(run/'edges.csv')
        wall, groups = revisit_wall(nodes, rows(run/'points.csv'))
        pgm = args.map_dir/('map_with_'+label+'.pgm')
        meta = yaml.safe_load(pgm.with_suffix('.yaml').read_text())
        w,h,p = read_pgm(pgm)
        r = meta['resolution']
        walls = [wall_fit(w,h,p,r,'central_north',0.173),
                 wall_fit(w,h,p,r,'central_south',0.818)]
        gap = narrow_gap(w,h,p,r)
        components = free_component_sizes(w,h,p)
        make_figures(run,w,h,p,walls,gap)
        counts = {n: sum(fn(v) for v in p) for n,fn in [
            ('occupied',lambda v:v<=60), ('free',lambda v:v>=250),
            ('unknown',lambda v:60<v<250)]}
        long_edges = [[int(e['source']),int(e['target'])] for e in edges
                      if abs(e['source']-e['target'])>20]
        # 诊断 ID 间隔，结合实际端点位置和轨迹判断是否跨回环。
        results[label] = dict(width=w,height=h,resolution=r,counts=counts,
            pgm_bytes=pgm.stat().st_size, pgm_sha256=hashlib.sha256(pgm.read_bytes()).hexdigest(),
            graph_nodes=len(nodes), graph_edges=len(edges), long_edges=long_edges,
            first_scan_time=nodes[0]['time'],last_scan_time=nodes[-1]['time'],
            walls=[{k:v for k,v in a.items() if k!='samples'} for a in walls],
            narrow_passage={k:v for k,v in gap.items() if k!='boundary_pixels'},
            free_components=len(components),
            largest_free_component_ratio=components[0]/counts['free'],
            boundary=boundary_check(w,h,p), revisit_wall=wall)
        results[label]['occupied_components'] = occupied_islands(w,h,p)
        origin = meta['origin']
        axes[col].imshow(np.frombuffer(p,dtype=np.uint8).reshape(h,w),cmap='gray',
                         vmin=0,vmax=254,extent=[origin[0],origin[0]+w*r,origin[1],origin[1]+h*r])
        axes[col].plot([n['x'] for n in nodes],[n['y'] for n in nodes],color='#1875bd',lw=0.8)
        for a,b in long_edges:
            lookup = {int(n['id']): n for n in nodes}
            axes[col].plot([lookup[a]['x'],lookup[b]['x']],
                           [lookup[a]['y'],lookup[b]['y']],color='red',lw=3)
        axes[col].set(title=label.replace('_',' ').upper(),xlabel='map x (m)',ylabel='map y (m)')
        for i,(ps,fit,color) in enumerate(zip(groups,wall['fits'],['#1578bf','#e36b20'])):
            a=np.array(ps)
            reference=wall['fits'][0]
            normalizer=math.sqrt(1+reference['slope']**2)
            residual=(a[:,1]-reference['slope']*a[:,0]-reference['intercept'])/normalizer
            daxes[col].scatter(a[:,0],residual,s=3,alpha=.35,c=color,
                               label='First pass' if i==0 else 'Revisit')
            xx=np.array([-2,2])
            yy=((fit['slope']-reference['slope'])*xx+fit['intercept']-reference['intercept'])/normalizer
            daxes[col].plot(xx,yy,c=color,lw=2)
        daxes[col].set(title=f"{label}: separation at x=0: {wall['wall_separation_at_x0_m']*100:.2f} cm",
                       xlabel='map x (m)',ylabel='Offset from first fitted wall (m)',
                       xlim=(-2,2),ylim=(-.07,.05))
        daxes[col].legend()
        daxes[col].grid(alpha=.2)
    figure.savefig(OUT/'maps_comparison.png',dpi=180)
    detail.savefig(OUT/'revisit_wall_comparison.png',dpi=180)
    (OUT/'metrics.json').write_text(json.dumps(results,indent=2)+'\n')
    print(json.dumps(results,indent=2))


if __name__=='__main__':
    main()
