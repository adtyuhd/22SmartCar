#!/usr/bin/env python3
"""Metrics from measured simulator body truth, resampled at 0.05m.

Smoothness = sum |delta steering angle| / distance, steering=atan(L*kappa).
Also report heading variation per metre, which is NOT steering variation.
"""
import argparse
import csv
import json
import math
import statistics
import hashlib
from pathlib import Path


def wrap(x):
    return math.atan2(math.sin(x), math.cos(x))


def metrics(points, wheelbase=0.28, spacing=0.05):
    if len(points) < 2:
        return {"path_m": 0.0, "smoothness_rad_m": None, "heading_rad_m": None}
    length = 0.0
    sampled = [points[0]]
    next_s = spacing
    for a, b in zip(points, points[1:]):
        ds = math.hypot(b[0]-a[0], b[1]-a[1])
        if ds < 1e-8:
            continue
        while next_s <= length + ds:
            f = (next_s-length)/ds
            sampled.append([a[0]+f*(b[0]-a[0]), a[1]+f*(b[1]-a[1]),
                            a[2]+f*wrap(b[2]-a[2])])
            next_s += spacing
        length += ds
    angles = [math.atan(wheelbase*wrap(b[2]-a[2])/spacing)
              for a, b in zip(sampled, sampled[1:])]
    return {"path_m": length,
            "smoothness_rad_m": sum(abs(b-a) for a,b in zip(angles,angles[1:]))/length if length else None,
            "heading_rad_m": sum(abs(wrap(b[2]-a[2])) for a,b in zip(sampled,sampled[1:]))/length if length else None}


def plots(directory):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    root = Path(directory)
    rows = list(csv.DictReader((root/"benchmark.csv").open()))
    if (root/'benchmark_tuned.csv').exists():
        rows += list(csv.DictReader((root/'benchmark_tuned.csv').open()))
    groups = sorted(set(r["planner"] for r in rows if r["kind"] == "obstacle"))
    fig, axes = plt.subplots(1, 4, figsize=(15,4))
    for ax, key, title in zip(axes, ("success", "nav_sim_s", "extra_path_m", "smoothness_rad_m"),
                              ("Success / all attempts", "Time (sim s), successes", "Extra distance (m), successes", "Steering variation (rad/m), successes")):
        vals=[]
        for g in groups:
            rr=[r for r in rows if r['planner']==g and r['kind']=='obstacle']
            valid=[float(r[key]) for r in rr if r[key] and (key=='success' or r['success']=='1')]
            vals.append(sum(valid)/len(valid) if valid else float('nan'))
        ax.bar(groups,[0 if math.isnan(v) else v for v in vals])
        ax.set_title(title,fontsize=9)
        if all(math.isnan(v) or v==0 for v in vals): ax.set_ylim(0,1)
        if key=='success': ax.set_ylim(0,1.12)
        for i,v in enumerate(vals):
            ax.text(i,.03 if math.isnan(v) else v, 'N/A' if math.isnan(v) else f'{v:.3f}',ha='center',va='bottom',fontsize=9)
    fig.tight_layout(); fig.savefig(root/'metrics.png',dpi=160); plt.close(fig)
    fig,ax=plt.subplots(figsize=(10,4))
    import xml.etree.ElementTree as ET
    from matplotlib.patches import Rectangle
    world=Path(__file__).resolve().parents[2]/'M3-2/worlds/scene.world'
    for model in ET.parse(world).findall('.//world/model'):
        size=model.find('link/collision/geometry/box/size')
        if size is not None:
            x,y,*_=map(float,model.findtext('pose').split()); sx,sy,*_=map(float,size.text.split())
            ax.add_patch(Rectangle((x-sx/2,y-sy/2),sx,sy,color='gray',alpha=.25))
    colors=plt.rcParams['axes.prop_cycle'].by_key()['color']
    for g in groups:
        rr=[r for r in rows if r['planner']==g and r['kind']=='obstacle']
        chosen=next((r for r in rr if r['success']=='1'),rr[0])
        events=[json.loads(l) for l in (root/chosen['events']).read_text().splitlines()]
        pp=[e['value'] for e in events if e['event']=='truth' and e.get('active')]
        if pp: ax.plot([p[0] for p in pp],[p[1] for p in pp],color=colors[groups.index(g)],label=f"{g}: trial {chosen['run']} ({chosen['reason']})")
        for r in rr:
            if r is chosen: continue
            other=[json.loads(l) for l in (root/r['events']).read_text().splitlines()]
            pts=[e['value'] for e in other if e['event']=='truth' and e.get('active')]
            if pts: ax.plot([p[0] for p in pts],[p[1] for p in pts],alpha=.18,color=colors[groups.index(g)])
    if rows:
        r=next((r for r in rows if r['kind']=='obstacle'),None)
        if r:
            s=float(r['obstacle_size']); x=float(r['obstacle_x']); y=float(r['obstacle_y'])
            ax.add_patch(Rectangle((x-s/2,y-s/2),s,s,color='red',alpha=.4))
    ax.set(xlabel='Gazebo world x (m)',ylabel='Gazebo world y (m)'); ax.set_aspect('equal'); ax.legend(fontsize=8)
    pts=[json.loads(l)['value'] for r in rows for l in (root/r['events']).read_text().splitlines()
         if '"event": "truth"' in l and json.loads(l).get('active')]
    if pts:
        ax.set_xlim(min(p[0] for p in pts)-.4,max(p[0] for p in pts)+.4)
        ax.set_ylim(min(p[1] for p in pts)-.6,max(p[1] for p in pts)+.6)
    fig.tight_layout(); fig.savefig(root/'trajectories.png',dpi=160); plt.close(fig)


def summarize(rows):
    result={}
    for planner in sorted({r['planner'] for r in rows}):
        rr=[r for r in rows if r['planner']==planner]
        successes=[r for r in rr if r['success']=='1']
        result[planner]={'attempts':len(rr),'successes':len(successes),
                         'reasons':{reason:sum(r['reason']==reason for r in rr) for reason in {r['reason'] for r in rr}}}
        for key in ('nav_sim_s','nav_wall_s','path_m','extra_path_m','smoothness_rad_m'):
            vv=[float(r[key]) for r in successes if r[key]!='']
            result[planner][key]=statistics.mean(vv) if vv else None
    return result


def replan_evidence(root, rows):
    data=[]
    for row in rows:
        events=[json.loads(l) for l in (root/row['events']).read_text().splitlines()]
        failures=[e for e in events if e['event']=='bt_status' and e['name']=='FollowPath' and e['current']=='FAILURE']
        failure=failures[0] if failures else None
        plans=[e for e in events if e['event']=='plan']
        after=[e for e in plans if failure and e['wall']>failure['wall']]
        computes=[e for e in events if failure and e['wall']>failure['wall'] and e['event']=='bt_status'
                  and e['name']=='ComputePathToPose' and e['current']=='RUNNING']
        data.append({'planner':row['planner'],'run':row['run'],'events':row['events'],
                     'success':row['success'],'local_failure_observed':bool(failure),
                     'failure_sim':failure['sim'] if failure else None,
                     'replan_start_wall_delay_s':computes[0]['wall']-failure['wall'] if computes else None,
                     'new_plan_wall_delay_s':after[0]['wall']-failure['wall'] if after else None,
                     'new_plan_sim_delay_s':after[0]['sim']-failure['sim'] if after else None})
    return data


def reports(root):
    root=Path(root)
    rows=list(csv.DictReader((root/'benchmark.csv').open()))
    tuned=list(csv.DictReader((root/'benchmark_tuned.csv').open())) if (root/'benchmark_tuned.csv').exists() else []
    summary=summarize(rows+tuned)
    (root/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False))
    replans=replan_evidence(root,rows+tuned)
    (root/'replanning.json').write_text(json.dumps(replans,indent=2,ensure_ascii=False))
    def fmt(value): return 'N/A（无成功试次）' if value is None else f'{value:.3f}'
    table='|方案|成功/尝试|成功均耗时 sim s|墙钟 s|绕行额外 m|转向变化 rad/m|\n|---|---:|---:|---:|---:|---:|\n'
    for g,s in summary.items():
        table+=f"|{g}|{s['successes']}/{s['attempts']}|{fmt(s['nav_sim_s'])}|{fmt(s['nav_wall_s'])}|{fmt(s['extra_path_m'])}|{fmt(s['smoothness_rad_m'])}|\n"
    baseline=[dict(r) for r in csv.DictReader((root/'baseline.csv').open())] if (root/'baseline.csv').exists() else []
    checks={f'{g}_10_attempts':sum(r['planner']==g for r in rr)==10 for g,rr in [('A',rows),('B',rows),('B_tuned',tuned)]}
    checks['all_injections_valid']=all(r['delay_actual'] and 2<=float(r['delay_actual'])<=4 and r['scan_detected']=='1'
                                     and r['path_intersection']=='1' for r in rows+tuned)
    checks['all_raw_results_match']=True
    checks['all_metrics_recompute']=True
    checks['all_trial_configs_match']=True
    checks['car_moved_before_injection']=True
    goals=set()
    for r in rows+tuned:
        events=[json.loads(l) for l in (root/r['events']).read_text().splitlines()]
        outcomes=[e for e in events if e['event']=='result']
        if len(outcomes)!=1 or int(outcomes[0]['success'])!=int(r['success']): checks['all_raw_results_match']=False
        measured=metrics([e['value'] for e in events if e['event']=='truth' and e.get('active')])
        for key in ('path_m','smoothness_rad_m','heading_rad_m'):
            if measured[key] is not None and (not r[key] or abs(measured[key]-float(r[key]))>1e-8):
                checks['all_metrics_recompute']=False
        manifest=json.loads((root/r['events']).with_name('manifest.json').read_text())
        if hashlib.sha256(manifest['config'].encode()).hexdigest()!=r['config_sha256']:
            checks['all_trial_configs_match']=False
        goals.update(tuple(e['goal']) for e in events if e['event']=='goal_accepted')
        spawn_time=next((e['wall'] for e in events if e['event']=='spawn_request'),None)
        before=[e['value'] for e in events if e['event']=='truth' and e.get('active') and spawn_time and e['wall']<spawn_time]
        if len(before)<2 or math.dist(before[0][:2],before[-1][:2])<.05: checks['car_moved_before_injection']=False
    checks['formal_scene_consistent']=len(goals)==1 and len({(r['obstacle_x'],r['obstacle_y'],r['obstacle_size'],r['delay_requested']) for r in rows+tuned})==1
    checks['all_trial_directories_unique']=len({r['events'] for r in rows+tuned})==len(rows+tuned)
    checks['successful_metrics_have_baselines']=all(r['extra_path_m']!='' for r in rows+tuned if r['success']=='1')
    checks['configuration_frozen']=all(len({r['config_sha256'] for r in rows+tuned if r['planner']==g})==1 for g in summary)
    checks['baseline_three_successful']=len(baseline)==3 and {r['planner'] for r in baseline}=={'A','B','B_tuned'} and all(r['success']=='1' for r in baseline)
    (root/'audit.json').write_text(json.dumps(checks,indent=2,ensure_ascii=False))
    # Do not accept a single frozen image saved four times, or a detection frame without the box.
    def valid_images(row):
        from PIL import Image
        files=[(root/row['events']).parent/(n+'.png') for n in ('01_approach','02_detected','03_detour','04_passed')]
        if not all(p.exists() for p in files): return False
        if len({hashlib.sha256(p.read_bytes()).digest() for p in files})!=4: return False
        pixels=Image.open(files[1]).convert('RGB').getdata()
        return sum(r>30 and g<r*.5 and b<r*.5 for r,g,b in pixels)>50
    chosen=next((r for r in rows+tuned if r['success']=='1' and valid_images(r)),None)
    if chosen:
        import shutil
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(1,4,figsize=(16,3.4))
        for ax,name in zip(axes,('01_approach','02_detected','03_detour','04_passed')):
            src=(root/chosen['events']).parent/(name+'.png'); shutil.copyfile(src,root/(name+'.png'))
            ax.imshow(plt.imread(src)); ax.set_title(name); ax.axis('off')
        fig.tight_layout(); fig.savefig(root/'process_sequence.png',dpi=140); plt.close(fig)
    frame_table='尚无满足内容核验的四帧过程，不能标为通过。'
    if chosen:
        events=[json.loads(l) for l in (root/chosen['events']).read_text().splitlines()]
        accepted=next(e['sim'] for e in events if e['event']=='goal_accepted')
        frame_table='|帧|目标接受后 sim s|最近真值 world x,y|\n|---|---:|---|\n'
        for e in (e for e in events if e['event']=='camera_frame'):
            truth=min((t for t in events if t['event']=='truth'),key=lambda t:abs(t['wall']-e['wall']))
            frame_table+=f"|{e['label']}|{e['sim']-accepted:.2f}|({truth['value'][0]:.3f},{truth['value'][1]:.3f})|\n"
    checks['four_real_camera_frames']=chosen is not None
    failures=[]
    for boundary in sorted((root/'boundaries').glob('*/benchmark.csv')) if (root/'boundaries').exists() else []:
        rr=list(csv.DictReader(boundary.open()))
        for r in rr:
            r['events']=str((boundary.parent/r['events']).relative_to(root))
        failures.extend(rr)
    failure_summary=summarize(failures) if failures else {}
    boundary_text='# 失效边界：实测触发实验\n\n有限重复只能支持该场景中的观测，不证明任意噪声下必然失败。导航终止状态 6=ABORTED；超时单独记录。\n\n'
    boundary_text+='|场景|次数|成功|障碍世界坐标|尺寸 m|实际注入 sim s|终止耗时 sim s|终止原因|\n|---|---:|---:|---|---:|---|---|---|\n'
    for boundary in sorted((root/'boundaries').glob('*/benchmark.csv')) if (root/'boundaries').exists() else []:
        rr=list(csv.DictReader(boundary.open())); r=rr[0]
        delays=[float(r['delay_actual']) for r in rr if r['delay_actual']]
        durations=[float(r['nav_sim_s']) for r in rr if r['nav_sim_s']]
        delay_range=f'{min(delays):.2f}–{max(delays):.2f}' if delays else 'N/A'
        time_range=f'{min(durations):.2f}–{max(durations):.2f}' if durations else 'N/A'
        boundary_text+=f"|{boundary.parent.name}|{len(rr)}|{sum(int(r['success']) for r in rr)}|({r['obstacle_x']},{r['obstacle_y']})|{r['obstacle_size']}|{delay_range}|{time_range}|{', '.join(sorted({r['reason'] for r in rr}))}|\n"
    if not failures: boundary_text+='\n尚未运行边界实验；不能声明已验证失效边界。\n'
    else: boundary_text+='\n与标准条件0.60m、x=2.5m的A方案9/10相比，上述记录给出了可脚本重现的失效触发条件。当前尺寸只采样0.60/0.80/1.60m、距离只采样1.4/2.5m，不构成连续精确阈值搜索；不能声称更小障碍必成功或任意更近位置必失败。\n'
    boundary_text+='\n命令、配置快照、每次轨迹、激光和行为树事件保存在各场景 raw/ 中；不得把基础设施失败当作算法边界。\n'
    boundary_text+='\n南侧通道几何宽度 2.10m。中心障碍留下的单侧几何宽度=(2.10-size)/2；物理车宽0.22m，导航 footprint 宽0.30m。膨胀半径0.40m是代价衰减带，不能直接当成不可通行硬边界。前进转弯半径≥0.60m，纵向准备距离也影响可行性。\n'
    boundary_text+='\n1.60m障碍留下单侧0.25m，小于导航footprint宽0.30m，因此南侧就近绕行不满足配置中的车体净宽；这不证明整个环形地图无解。可能存在更长的全局绕行，超时只表示本配置在90仿真秒预算内未到达。近距离场景检验的是前进转弯准备距离，不能简单归因为障碍面积。\n'
    (root/'failure_boundary.md').write_text(boundary_text)
    boundary_replans=replan_evidence(root,failures)
    (root/'boundary_replanning.json').write_text(json.dumps(boundary_replans,indent=2,ensure_ascii=False))
    tune='# 参数调优记录\n\n共同感知修复在正式比较前完成，不计作 B 的性能调优。A=速度相关短前视0.30–0.50m；B=固定长前视1.50m。B_tuned 保持固定前视，仅改一个参数。\n\n'
    tune+='|轮次|参数|本实验原值|新值|依据|\n|---|---|---:|---:|---|\n|1|FollowPath.lookahead_dist|1.50m|0.40m|长前视跨过绕行弯道而切向障碍；缩短跟踪点以跟随 Hybrid-A* 曲率。保持速度、碰撞检测、膨胀、全局规划及目标不变。|\n\n'+table
    tune+='\n这里的“原值”是 B 的冻结实验配置，并非宣称 Nav2 上游默认值为1.50m。失败试次不进入成功耗时/绕行/平滑度均值；原始行仍完整保留。调优未保证在其他障碍尺寸和位置也有效。\n'
    (root/'tuning_log.md').write_text(tune)
    s_a=summary.get('A',{}); s_b=summary.get('B',{})
    side_counts={}
    for r in rows+tuned:
        events=[json.loads(l) for l in (root/r['events']).read_text().splitlines()]
        ys=[e['value'][1]-float(r['obstacle_y']) for e in events if e['event']=='truth' and e.get('active')]
        if not ys: continue
        side='北侧' if max(ys)>abs(min(ys)) else '南侧'
        key=r['planner']+' '+side
        side_counts.setdefault(key,[0,0]); side_counts[key][0]+=int(r['success']); side_counts[key][1]+=1
    side_text='；'.join(f'{key} {s}/{n}' for key,(s,n) in side_counts.items())
    first_events=[json.loads(l) for l in (root/rows[0]['events']).read_text().splitlines()]
    measured_goal=next(e['goal'] for e in first_events if e['event']=='goal_accepted')
    measured_scene=f"map目标 {tuple(measured_goal)}（yaw为度），world障碍 ({rows[0]['obstacle_x']},{rows[0]['obstacle_y']})，边长{rows[0]['obstacle_size']}m，请求延迟{rows[0]['delay_requested']}s"
    better='A' if s_a.get('successes',0)>=s_b.get('successes',0) else 'B'
    worse='B' if better=='A' else 'A'
    def rank(g):
        s=summary[g]
        return (-s['successes'],s['smoothness_rad_m'] if s['smoothness_rad_m'] is not None else math.inf,
                s['nav_sim_s'] if s['nav_sim_s'] is not None else math.inf)
    selected=min(summary,key=rank)
    gain=(summary[better]['successes']/summary[better]['attempts']-summary[worse]['successes']/summary[worse]['attempts'])*100
    reference=next((r for r in baseline if r['planner']==better and r['success']=='1'),None)
    cost_text=''
    if reference and summary[better]['nav_sim_s'] is not None:
        cost_text=(f"与该方案自身无障碍参考相比，成功均耗时增加 {summary[better]['nav_sim_s']-float(reference['nav_sim_s']):.3f}s，"
                   f"路径增加 {summary[better]['extra_path_m']:.3f}m，平滑度由 {float(reference['smoothness_rad_m']):.3f} 升为 {summary[better]['smoothness_rad_m']:.3f}rad/m。"
                   '这是避障的时间、路程和转向代价；无障碍参考仅1次，不能据此估计统计显著性。')
    tuned_text=''
    if 'B_tuned' in summary and summary['B_tuned']['nav_sim_s'] is not None and s_a.get('nav_sim_s') is not None:
        st=summary['B_tuned']
        tuned_text=(f"调优将B的成功数从 {s_b['successes']}/{s_b['attempts']} 提升为 {st['successes']}/{st['attempts']}。"
                    f"与A相比，B_tuned成功数少 {s_a['successes']-st['successes']} 次；仅比较成功试次，耗时差为 {st['nav_sim_s']-s_a['nav_sim_s']:+.3f}s、"
                    f"额外路径差 {st['extra_path_m']-s_a['extra_path_m']:+.3f}m、平滑度差 {st['smoothness_rad_m']-s_a['smoothness_rad_m']:+.3f}rad/m。"
                    '这体现了更平顺的成功轨迹与本批到达可靠性之间的取舍，不能把成功子样本上的改善解释为所有尝试都更快。')
    has_failure=[r for r in replans+boundary_replans if r['local_failure_observed']]
    has_output=[r for r in has_failure if r['new_plan_wall_delay_s'] is not None]
    replan_text=(f"观察到 {len(has_failure)} 个包含 FollowPath→FAILURE 的试次，其中 {len(has_output)} 个在失败后再次收到 /plan。"
                 +(f"首次失败到下一条路径的墙钟延迟范围 {min(r['new_plan_wall_delay_s'] for r in has_output):.3f}–{max(r['new_plan_wall_delay_s'] for r in has_output):.3f}s。" if has_output else '没有可报告的新路径延迟。'))
    report=f'''# M3-4 实测分析报告

## 实验与数据口径

ROS2 Humble + Gazebo Classic 11；只读复用 M3-1 车型、M3-2 地图/world 和 M3-3 bringup。每次完整重启全栈，重新定位；本批场景为{measured_scene}，障碍高1m，在接受目标并已有运动后由spawn service生成。实体同时具有 collision 和 visual；原始 /scan 产生障碍层，不向代价地图写数据。两方案仅改变 RPP 前视策略。

成功要求 Nav2 SUCCEEDED、最终 TF 位置误差≤0.20m/航向≤0.40rad、相对初始 map/world 对齐的真值位置误差≤0.30m、未检测到车体与世界方盒相交，并具有延迟2–4秒、路径交叉和激光检测证据。碰撞诊断是原 URDF 0.45×0.22m 车体与 world 方盒的20Hz真值几何相交检查，不是接触力传感器；不保证捕捉采样间的瞬时接触。

长度来自独立 Gazebo 车体真值轨迹，真值仅用于评估。每方案成功无障碍参考路径的均值用于减法；本次参考各1次，因此未估计参考路径方差。耗时从目标接受到终止，主表用仿真秒并附墙钟秒，排除启动、定位与退出时间。成功均值只取成功试次；失败原始行的路程是未完成的部分轨迹，不能解释为有效绕行。

轨迹按0.05m弧长重采样，κ=wrap(Δyaw)/0.05，δ=atan(0.28κ)，平滑度=Σ|Δδ|/总路程（rad/m）。另存 heading_rad_m=Σ|Δyaw|/路程供诊断，两者不可混称。该指标是依据轨迹的等效单轨转角估计，包含物理噪声，未用指令曲线冒充实测转角。

{table}

原始正式记录：[benchmark.csv](results/benchmark.csv)，调优：[benchmark_tuned.csv](results/benchmark_tuned.csv)，无障碍参考：[baseline.csv](results/baseline.csv)。每行关联独立事件文件及包含完整参数、地图校验值、脚本校验值的 manifest。试跑与正式批次分开，不挑选试次。

## ① 哪种综合更好，代价是什么

按正式成功数优先，{better} 优于 {worse}，本批成功率相差 {gain:.1f} 个百分点；具体四指标见上表。若某方案无成功试次，其成功条件下的耗时/绕行/平滑度不可定义，不能用失败后提前停车带来的短耗时或小转角宣称更好。{cost_text}

{tuned_text}

## ② 失效条件与触发实验

{boundary_text.split(chr(10),1)[1]}

长前视方案的正式失败原因见 summary.json 与每次 rosout/行为树；尚未测试移动障碍速度边界，不能推断其预测能力。早期试跑还暴露了 scan.max_obstacle_height 默认0.0过滤掉高度0.135m激光点的问题；显式设为2.0后，实测可见障碍进入代价地图并触发碰撞预警。早期失败保存在 pilot*，不计入正式结果。

按实际轨迹相对障碍中心的最大侧向偏移分组，正式样本观测为：{side_text}。这是对轨迹的事后诊断，并非随机分配的绕行侧对照；它提示局部可用空间、地图/定位差异与全局路径分支也参与失效，不能把剩余失败只归因于前视距离。原始轨迹均保留在叠加图中。

## ③ 局部失败后有没有全局重规划，延迟多少

{replan_text}

使用 /behavior_tree_log 中 FollowPath 的 FAILURE 作为局部失败判据，随后关联 ComputePathToPose RUNNING 与 /plan 回调。墙钟采用同一采集进程 time.monotonic；sim 由 /clock 回调，约0.1s分辨率。Nav2 还会按1Hz周期规划，所以“失败后有路径”不能证明是失败因果触发，更不等于已经脱困。new_plan_* 为 null 表示之后未观察到新路径，不填0。replanning.json 与 boundary_replanning.json 列出每次失败、重规划启动/输出延迟及最终成功状态；最终未到达即未脱困。

## ④ 比赛场景的选择与参数

比赛假设为同一地图南侧2.10m通道，障碍约0.60m、在离车足够远的路径中段延迟2–4s出现；不包含迎面高速移动物。把调优方案也纳入后，本批优先选择 **{selected}**：先比较成功数，同成功数时比较成功轨迹平滑度，再比较耗时；该选择只针对本报告数据，不是显著性结论。A：RPP desired_linear_vel=0.22、速度相关前视0.30–0.50m、lookahead_time=1.5；B_tuned：固定lookahead_dist=0.40m。两者碰撞预测1.0s、cost_scaling_dist=0.40、膨胀0.40m；Smac DUBIN 与最终 Twist 限幅半径0.60m。保留碰撞检测和禁止原地旋转/倒车约束。边界场景下应停车并允许任务失败，不能用关闭检测获得好看的到达率。

## 图与逐项验收

![轨迹](results/trajectories.png)
![四指标](results/metrics.png)
![真实相机过程](results/process_sequence.png)

自动审计见 results/audit.json；四帧来源为 {chosen['events'] if chosen else '尚无满足条件的正式成功试次'}。逐项状态见 README 验收表。仿真有限次实测不替代真实车辆或考官现场两次随机位置测试，后两者尚未进行。

{frame_table}
'''
    destination=root.parent/'report.md' if root.name=='results' else root/'report.md'
    if root.name!='results':
        report=report.replace('(results/','(')
    destination.write_text(report)
    checks['failure_boundary_measured']=len(failures)>0
    boundary_files=list((root/'boundaries').glob('*/benchmark.csv')) if (root/'boundaries').exists() else []
    checks['boundary_three_conditions_x3']=len(boundary_files)==3 and all(len(list(csv.DictReader(p.open())))==3 for p in boundary_files)
    checks['boundary_injections_valid']=bool(failures) and all(r['scan_detected']=='1' and r['path_intersection']=='1'
        and r['delay_actual'] and 2<=float(r['delay_actual'])<=4 for r in failures)
    (root/'audit.json').write_text(json.dumps(checks,indent=2,ensure_ascii=False))
    return checks


def main():
    p=argparse.ArgumentParser()
    p.add_argument('directory',type=Path)
    p.add_argument('--report',action='store_true')
    a=p.parse_args()
    if not a.directory.resolve().is_relative_to(Path(__file__).resolve().parents[1]): p.error('outputs must remain inside M3-4')
    plots(a.directory)
    if a.report: print(json.dumps(reports(a.directory),indent=2,ensure_ascii=False))


if __name__=='__main__':
    main()
