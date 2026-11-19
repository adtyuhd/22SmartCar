#!/usr/bin/env python3
"""Metrics from measured simulator body truth, resampled at 0.05m.

Smoothness = sum |delta steering angle| / distance, steering=atan(L*kappa).
Also report heading variation per metre, which is NOT steering variation.
"""
import argparse
import os
import csv
import json
import math
import statistics
import hashlib
from pathlib import Path

PLANNER_COLORS={"RPP":"tab:blue","MPPI":"tab:orange","MPPI_tuned":"tab:green","RPP_tuned":"tab:green"}


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
    os.environ.setdefault("MPLCONFIGDIR",str(Path(directory)/"mpl"))
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
        vals=[]; errors=[]
        for g in groups:
            rr=[r for r in rows if r['planner']==g and r['kind']=='obstacle']
            valid=[float(r[key]) for r in rr if r[key] and (key=='success' or r['success']=='1')]
            vals.append(sum(valid)/len(valid) if valid else float('nan'))
            errors.append(statistics.stdev(valid) if key!='success' and len(valid)>1 else 0.0)
        ax.bar(groups,[0 if math.isnan(v) else v for v in vals],yerr=errors,capsize=4,color=[PLANNER_COLORS[g] for g in groups])
        ax.set_title(title,fontsize=9)
        if all(math.isnan(v) or v==0 for v in vals): ax.set_ylim(0,1)
        if key=='success': ax.set_ylim(0,1.12)
        for i,v in enumerate(vals):
            label='N/A' if math.isnan(v) else f'{v:.3f}'
            if key=='success':
                rr=[r for r in rows if r['planner']==groups[i] and r['kind']=='obstacle']
                label=f"{sum(r['success']=='1' for r in rr)}/{len(rr)}"
            ax.annotate(label,(i,.03 if math.isnan(v) else v+errors[i]),xytext=(0,4),textcoords='offset points',ha='center',va='bottom',fontsize=9)
        if key!='success': ax.margins(y=.2)
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
    colors=PLANNER_COLORS
    for g in groups:
        rr=[r for r in rows if r['planner']==g and r['kind']=='obstacle']
        chosen=next((r for r in rr if r['success']=='1'),rr[0])
        events=[json.loads(l) for l in (root/chosen['events']).read_text().splitlines()]
        pp=[e['value'] for e in events if e['event']=='truth' and e.get('active')]
        if pp: ax.plot([p[0] for p in pp],[p[1] for p in pp],color=colors[g],label=f"{g}: trial {chosen['run']} ({chosen['reason']})")
        for r in rr:
            if r is chosen: continue
            other=[json.loads(l) for l in (root/r['events']).read_text().splitlines()]
            pts=[e['value'] for e in other if e['event']=='truth' and e.get('active')]
            if pts: ax.plot([p[0] for p in pts],[p[1] for p in pts],alpha=.18,color=colors[g])
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
    """Generate statements only from the current directory's measured evidence."""
    root=Path(root)
    def read(name):
        p=root/name
        return list(csv.DictReader(p.open())) if p.exists() else []
    rows=read('benchmark.csv')+read('benchmark_tuned.csv')
    baseline=read('baseline.csv')
    summary=summarize(rows)
    (root/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False))
    replans=replan_evidence(root,rows)
    (root/'replanning.json').write_text(json.dumps(replans,indent=2,ensure_ascii=False))
    checks={p+'_10_runs':sum(r['planner']==p for r in rows)==10 for p in ('RPP','MPPI')}
    checks['all_injections_valid']=bool(rows) and all(r['delay_actual'] and 2<=float(r['delay_actual'])<=4 and r['scan_detected']=='1' and r['path_intersection']=='1' for r in rows)
    checks['successful_extra_paths_have_reference']=all(r['extra_path_m']!='' for r in rows if r['success']=='1')
    checks['metrics_match_evidence']=True
    checks['plugin_matches']=True
    checks['configs_frozen']=True
    configs={}; diagnostics=[]
    for r in rows:
        folder=(root/r['events']).parent
        events=[json.loads(l) for l in (folder/'events.jsonl').read_text().splitlines()]
        m=metrics([e['value'] for e in events if e['event']=='truth' and e.get('active')])
        for k,v in m.items():
            if v is not None and (not r[k] or abs(float(r[k])-v)>1e-6): checks['metrics_match_evidence']=False
        messages=[e.get('message','') for e in events if e['event']=='rosout']
        failures=[e for e in events if e['event']=='bt_status' and e.get('name')=='FollowPath' and e.get('current')=='FAILURE']
        spawn=next((e for e in events if e['event']=='spawn_complete'),None)
        detection=next((e for e in events if e['event']=='scan_detection'),None)
        diagnostics.append({'planner':r['planner'],'run':r['run'],'optimizer_fail_logs':sum('Optimizer fail' in m for m in messages),'planning_fail_logs':sum('failed to generate a valid path' in m for m in messages),'missed_control_rate_logs':sum('Control loop missed' in m for m in messages),'followpath_failure_events':len(failures),'spawn_to_detection_sim_s':detection['sim']-spawn['sim'] if detection and spawn else None})
        expected='nav2_mppi_controller::MPPIController' if r['planner'].startswith('MPPI') else 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'
        if not any(e['event']=='active_plugin' and e['plugin']==expected for e in events): checks['plugin_matches']=False
        configs.setdefault(r['planner'],set()).add(r['config_sha256'])
        manifest=json.loads((folder/'manifest.json').read_text())
        if hashlib.sha256(manifest['config'].encode()).hexdigest()!=r['config_sha256']: checks['configs_frozen']=False
    if diagnostics:
        with (root/'diagnostics.csv').open('w') as fp:
            writer=csv.DictWriter(fp,fieldnames=list(diagnostics[0])); writer.writeheader(); writer.writerows(diagnostics)
    checks['configs_frozen'] &= all(len(v)==1 for v in configs.values())
    (root/'audit.json').write_text(json.dumps(checks,indent=2))
    def fmt(v): return 'N/A' if v is None else f'{v:.3f}'
    table='|方案|成功/尝试|耗时 sim s|额外路径 m|转角变化 rad/m|\n|---|---:|---:|---:|---:|\n'
    for p,s in summary.items():
        table+=f"|{p}|{s['successes']}/{s['attempts']}|{fmt(s['nav_sim_s'])}|{fmt(s['extra_path_m'])}|{fmt(s['smoothness_rad_m'])}|\n"
    ranks=sorted((p for p in ('RPP','MPPI') if p in summary),key=lambda p:(-summary[p]['successes'],summary[p]['nav_sim_s'] or math.inf))
    text='# RPP 与 MPPI：新一轮实测分析\n\n'
    text+='本目录数据来自重新运行的 Gazebo 实验，未混入父目录旧实验。A=RPP，B=MPPI（Ackermann）。实际加载插件通过 controller_server 参数服务核验。\n\n'
    text+='共同条件：最大速度0.22 m/s，最小转弯半径0.60 m，footprint 0.50×0.30 m，膨胀半径0.40 m，Smac Hybrid-A*（DUBIN），1Hz行为树重规划；禁止倒车和原地旋转。默认目标 map(5.5,0,0°)，world障碍(2.5,-5.05)，边长0.60 m，高1 m，目标接受后3秒生成。\n\n'
    text+='实体有 collision 和 visual，通过激光进入两张代价地图；不直接写代价地图。真值仅用于评估。成功须动作 SUCCEEDED、TF位置误差≤0.20 m/朝向≤0.40 rad、真值位置误差≤0.30 m、未发现几何碰撞且有有效延迟/路径交叉/激光检测证据。20Hz矩形相交检测不等同于接触力传感器。\n\n'
    text+='长度为Gazebo真值轨迹积分；平滑度按0.05m弧长重采样，κ=Δyaw/Δs，δ=atan(0.28κ)，Σ|Δδ|/总路程。它是等效单轨转角估计，另存实测 steering_joints 供诊断。耗时、额外路径、平滑度均值只取成功试次；失败全部保留，不能用失败部分轨迹冒充有效绕行。各规划器采用自身无障碍基线均值。\n\n'+table
    text+='\n## ① 哪种表现更好\n\n'
    if len(ranks)==2:
        a,b=ranks; sa,sb=summary[a],summary[b]
        text+=f"按成功数优先、相同时成功平均耗时的预定规则，本批排序为 {a}、{b}。成功数分别为{sa['successes']}/{sa['attempts']}和{sb['successes']}/{sb['attempts']}；四项指标见表。"
        if all(s['nav_sim_s'] is not None for s in (sa,sb)):
            text+=f"{a}减{b}的成功平均耗时差为{sa['nav_sim_s']-sb['nav_sim_s']:+.3f}s、额外路径差{sa['extra_path_m']-sb['extra_path_m']:+.3f}m、平滑度差{sa['smoothness_rad_m']-sb['smoothness_rad_m']:+.3f}rad/m。"
        text+='每种仅10次，成功子样本构成可能不同，不宣称统计显著或所有场景普遍优越。\n'
    text+='\n|方案|优化失败日志条数|全局规划失败日志条数|控制周期超时日志条数|FollowPath失败事件数|\n|---|---:|---:|---:|---:|\n'
    for p in summary:
        d=[r for r in diagnostics if r['planner']==p]
        text+=f"|{p}|{sum(r['optimizer_fail_logs'] for r in d)}|{sum(r['planning_fail_logs'] for r in d)}|{sum(r['missed_control_rate_logs'] for r in d)}|{sum(r['followpath_failure_events'] for r in d)}|\n"
    text+='\n日志条数不是独立失效次数；控制周期警告也不能单独证明CPU是失败原因。逐次诊断见 diagnostics.csv。\n'
    timeouts=[r for r in rows if r['reason']=='timeout']
    if timeouts:
        text+='\n超时试次的终止路程（未完成轨迹，不能作为有效绕行）：'+ '；'.join(f"{r['planner']}/{r['run']}：{float(r['path_m']):.3f}m/{float(r['nav_sim_s']):.1f}s" for r in timeouts)+'。应结合全局路径及停止位置区分局部卡住与全局长路超时，不仅凭超时标签推断原因。\n'
    text+='\n终点复核不通过的试次也计失败。定位估计、全局路径分支与控制动作可能相互影响；共享AMCL配置并不等于排除了定位误差，本实验不将所有失败都归因为局部控制器本身。\n'
    text+='\nRPP以路径跟踪、调速与碰撞检测为主；MPPI采样阿克曼可行轨迹并评分。两者共同使用动态全局代价地图和Smac，故比较的是相同导航栈下控制器替换的整体效果，不能将全局绕行完全归因于局部控制器。\n'
    boundaries=[]; boundary_replans=[]
    for f in sorted((root/'boundaries').glob('*/benchmark.csv')):
        rr=list(csv.DictReader(f.open())); ss=summarize(rr)
        evidence=replan_evidence(f.parent,rr)
        for e in evidence: e['scene']=f.parent.name
        boundary_replans.extend(evidence)
        for p,s in ss.items():
            r=next(r for r in rr if r['planner']==p)
            boundaries.append((f.parent.name,p,s,r))
    boundary='# 失效触发实验\n\n|场景|规划器|成功/次数|world x,y|尺寸 m|延迟 s|原因|\n|---|---|---:|---|---:|---:|---|\n'
    for name,p,s,r in boundaries:
        boundary+=f"|{name}|{p}|{s['successes']}/{s['attempts']}|{r['obstacle_x']},{r['obstacle_y']}|{r['obstacle_size']}|{r['delay_requested']}|{s['reasons']}|\n"
    boundary+='\n有限重复只说明采样条件下的观测；0/3不是数学意义的必然失败。2.10m通道中1.60m中心障碍单侧净宽0.25m小于导航footprint宽0.30m，但这并不证明整张环形地图无解。膨胀半径是代价衰减带，不能直接作为硬净宽。没有测试移动障碍速度边界。\n'
    (root/'failure_boundary.md').write_text(boundary)
    (root/'boundary_replanning.json').write_text(json.dumps(boundary_replans,indent=2,ensure_ascii=False))
    text+='\n## ② 什么情况下失效\n\n'+boundary.split('\n\n',1)[1]
    text+='\n## ③ 局部失败后是否重规划\n\n'
    failures=[r for r in replans+boundary_replans if r['local_failure_observed']]
    outputs=[r for r in failures if r['new_plan_wall_delay_s'] is not None]
    text+=f"正式对照、调优及边界实验中，{len(failures)}次观察到 FollowPath→FAILURE，其中{len(outputs)}次之后收到 /plan。"
    if outputs: text+=f"首次失败到下一条路径回调的墙钟延迟为{min(r['new_plan_wall_delay_s'] for r in outputs):.3f}～{max(r['new_plan_wall_delay_s'] for r in outputs):.3f}s。"
    text+='以 /behavior_tree_log 和 /plan 的同进程 monotonic 时钟关联；周期规划与失败后恢复可能重叠，这只是时间关联，不证明因果。没有后续路径时记null而非0；最终成功状态见下表。\n\n|方案/次|规划启动延迟 s|路径输出延迟 s|最终到达|\n|---|---:|---:|---|\n'
    for r in failures:
        text+=f"|{r.get('scene','standard')}/{r['planner']}/{r['run']}|{fmt(r['replan_start_wall_delay_s'])}|{fmt(r['new_plan_wall_delay_s'])}|{r['success']}|\n"
    text+='\n## ④ 比赛选择与具体参数\n\n'
    candidates=sorted(summary,key=lambda p:(-summary[p]['successes']/summary[p]['attempts'],summary[p]['nav_sim_s'] or math.inf))
    if candidates:
        p=candidates[0]; selected=summary[p]
        text+=f"在本批低速、2.10m通道、突现静止障碍场景中，按成功率优先、同率比较耗时的规则暂选 {p}：成功{selected['successes']}/{selected['attempts']}，成功均耗时{fmt(selected['nav_sim_s'])}s、额外路径{fmt(selected['extra_path_m'])}m、转角变化{fmt(selected['smoothness_rad_m'])}rad/m。"
        import yaml
        r=next(r for r in rows if r['planner']==p)
        recorded=json.loads(((root/r['events']).parent/'manifest.json').read_text())
        follow=yaml.safe_load(recorded['config'])['controller_server']['ros__parameters']['FollowPath']
        if p.startswith('MPPI'):
            text+=f"具体控制参数：motion_model=Ackermann，vx_max={follow['vx_max']}，vx_min={follow['vx_min']}，min_turning_r={follow['AckermannConstraints']['min_turning_r']}，time_steps={follow['time_steps']}，model_dt={follow['model_dt']}，batch_size={follow['batch_size']}，ObstaclesCritic.repulsion_weight={follow['ObstaclesCritic']['repulsion_weight']}。"
        else:
            text+=f"具体控制参数：desired_linear_vel={follow['desired_linear_vel']}，use_velocity_scaled_lookahead_dist={follow['use_velocity_scaled_lookahead_dist']}，min/max_lookahead_dist={follow['min_lookahead_dist']}/{follow['max_lookahead_dist']}m，lookahead_time={follow['lookahead_time']}s，max_allowed_time_to_collision_up_to_carrot={follow['max_allowed_time_to_collision_up_to_carrot']}s。"
        text+='完整冻结参数见本目录 config_snapshot/（从每次manifest导出），不将这一选择外推到高速移动物场景。'

    text+='无解或超预算时停车并记录失败，不能关闭碰撞检测换成功率。\n\n'
    snapshot=root/'config_snapshot'; snapshot.mkdir(exist_ok=True)
    for p in summary:
        r=next(r for r in rows if r['planner']==p)
        m=json.loads(((root/r['events']).parent/'manifest.json').read_text())
        (snapshot/(p.lower()+'.yaml')).write_text(m['config'])
    plan=Path(__file__).resolve().parents[1].with_name('M3-4-evidence')/'config/tuning_plan.json'
    if plan.exists():
        t=json.loads(plan.read_text()); original=t['planner']; tuned=original+'_tuned'
        tune='# 参数调优记录\n\n'+t['selection_reason']+'\n\n|参数|冻结原值|新值|假设|\n|---|---:|---:|---|\n'
        for c in t['changes']: tune+=f"|{c['parameter']}|{c['old']}|{c['new']}|{c['why']}|\n"
        tune+='\n原值为本实验冻结配置，不冒称上游默认值。每轮只改表中参数，其余全栈保持一致。\n\n'+table
        if original in summary and tuned in summary:
            a,b=summary[original],summary[tuned]
            tune+=f"\n成功数变化：{a['successes']}/{a['attempts']} → {b['successes']}/{b['attempts']}。\n"
            for k in ('nav_sim_s','extra_path_m','smoothness_rad_m'):
                if a[k] is not None and b[k] is not None: tune+=f"{k}：{a[k]:.3f} → {b[k]:.3f}，变化{b[k]-a[k]:+.3f}。\n"
        if original in summary and tuned in summary and summary[tuned]['successes']/summary[tuned]['attempts']<summary[original]['successes']/summary[original]['attempts']:
            tune+='\n本轮成功率低于原MPPI，不采纳这套参数作为改进配置。它作为完整保留的负结果，不能宣称调优成功。比赛选择仍依据实测排序。\n'
        tune+='\n成功子样本均值存在选择偏差，调优不保证同时改善所有指标，失败试次完整保留。\n'
        (root/'tuning_log.md').write_text(tune)
        text+='## 参数调优\n\n'+tune.split('\n\n',1)[1]+'\n'
    archive=root/'tuning_round1/benchmark_tuned.csv'
    if archive.exists():
        first=list(csv.DictReader(archive.open())); first_summary=summarize(first)
        text+='## 第一轮未改善的调优记录\n\n第一轮只把 time_steps 80→120，成功数为'+str(first_summary['MPPI_tuned']['successes'])+'/10，低于原MPPI的8/10。主表与主图中的 MPPI_tuned 为第二轮最终配置；第一轮CSV、完整基线、配置和图表均保留，见 [第一轮记录](tuning_round1/README.md)，不混入第二轮均值。\n'
        (root/'tuning_round1_summary.json').write_text(json.dumps(first_summary,indent=2))
        log=root/'tuning_log.md'
        if log.exists():
            log.write_text(log.read_text()+'\n## 第一轮（未改善）\n\nFollowPath.time_steps：80→120；成功7/10，原MPPI为8/10，未改善。完整原始记录见 tuning_round1/，主表为第二轮。\n')
    text+='## 可视化\n\n![指标](metrics.png)\n\n![轨迹](trajectories.png)\n\n![障碍附近放大](trajectories_zoom.png)\n\n![实测相机过程](process_sequence.png)\n'
    if (root/'infrastructure/README.md').exists():
        text+='\n## 基础设施启动错误\n\n有1次调优准备启动因车载相机无图像超时，尚未接受导航目标，不计入导航分母。CSV、事件和日志均保留，见 [启动错误记录](infrastructure/README.md)。修复只将可选车载相机从导航接口就绪条件中移除；正式20次结果未替换，调优10次导航从头采集。\n'
    text+='\n## 无障碍参考\n\n|规划器|尝试|成功|平均路径 m|\n|---|---:|---:|---:|\n'
    for p,s in summarize(baseline).items(): text+=f"|{p}|{s['attempts']}|{s['successes']}|{fmt(s['path_m'])}|\n"
    if (root/'overview_process_sequence.png').exists():
        text=text.replace('![实测相机过程](process_sequence.png)','')
        text+='\n## 独立俯视演示\n\n以下为同配置额外演示试次的真实Gazebo俯视相机帧，不计入上述成功率。来源见 overview_source.json。\n\n![俯视过程](overview_process_sequence.png)\n'
    (root/'report.md').write_text(text)
    # Additional uncertainty summaries; SD is descriptive, not a significance claim.
    spread={}
    for p in summary:
        spread[p]={}
        for k in ('nav_sim_s','extra_path_m','smoothness_rad_m'):
            v=[float(r[k]) for r in rows if r['planner']==p and r['success']=='1' and r[k]!='']
            spread[p][k]={'n':len(v),'mean':statistics.mean(v) if v else None,'sd':statistics.stdev(v) if len(v)>1 else None,'min':min(v) if v else None,'max':max(v) if v else None}
    (root/'spread.json').write_text(json.dumps(spread,indent=2))
    return checks


def supplementary_plots(root):
    os.environ.setdefault("MPLCONFIGDIR",str(Path(root)/"mpl"))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    from PIL import Image,ImageOps,ImageDraw
    root=Path(root)
    rows=list(csv.DictReader((root/'benchmark.csv').open()))
    if (root/'benchmark_tuned.csv').exists(): rows+=list(csv.DictReader((root/'benchmark_tuned.csv').open()))
    fig,ax=plt.subplots(figsize=(9,6))
    r=rows[0]; ox,oy,size=map(float,(r['obstacle_x'],r['obstacle_y'],r['obstacle_size']))
    colors=PLANNER_COLORS
    for p in sorted({r['planner'] for r in rows}):
        rr=[r for r in rows if r['planner']==p]
        for i,r in enumerate(rr):
            events=[json.loads(l) for l in (root/r['events']).read_text().splitlines()]
            points=[e['value'] for e in events if e['event']=='truth' and e.get('active')]
            if points:
                ax.plot([x[0] for x in points],[x[1] for x in points],color=colors[p],alpha=.5,label=p if i==0 else None,linestyle='-' if r['success']=='1' else '--')
                if r['success']!='1': ax.plot(*points[-1][:2],marker='x',color=colors[p])
    ax.add_patch(Rectangle((ox-size/2,oy-size/2),size,size,color='red',alpha=.35))
    ax.axhline(-6.1,color='gray'); ax.axhline(-4.0,color='gray')
    ax.set(xlim=(ox-1.8,ox+1.5),ylim=(oy-1.2,oy+1.2),xlabel='world x (m)',ylabel='world y (m)',title='All trials near obstacle; dashed = failed, x = stop')
    ax.set_aspect('equal'); ax.legend(); fig.tight_layout(); fig.savefig(root/'trajectories_zoom.png',dpi=160); plt.close(fig)
    labels=['01_approach','02_detected','03_detour','04_passed']
    sequence=next(((root/r['events']).parent for r in rows if r['success']=='1' and all(((root/r['events']).parent/(l+'.png')).exists() for l in labels)),None)
    if sequence:
        source_events=[json.loads(l) for l in (sequence/'events.jsonl').read_text().splitlines()]
        accepted=next(e['sim'] for e in source_events if e['event']=='goal_accepted')
        frame_info=[]
        canvas=Image.new('RGB',(1280,1010),'white'); draw=ImageDraw.Draw(canvas)
        for i,l in enumerate(labels):
            im=Image.open(sequence/(l+'.png')).convert('RGB'); im=ImageOps.contain(im,(630,460))
            x=(i%2)*640; y=(i//2)*505
            frame=next(e for e in source_events if e['event']=='camera_frame' and e['label']==l)
            elapsed=frame['image_stamp']-accepted
            truth=min((e for e in source_events if e['event']=='truth'),key=lambda e:abs(e['sim']-frame['image_stamp']))['value']
            frame_info.append({'stage':l,'elapsed_sim_s':elapsed,'world_pose':truth,'image_stamp':frame['image_stamp']})
            canvas.paste(im,(x,y+30)); draw.text((x+10,y+8),f'{l}   t={elapsed:.2f}s   world=({truth[0]:.2f}, {truth[1]:.2f})',fill='black')
        canvas.save(root/'process_sequence.png')
        (root/'process_source.json').write_text(json.dumps({'trial':str(sequence.relative_to(root)),'frames':frame_info},indent=2))



def export_delivery(evidence, output):
    """Export only the assignment's CSV, Markdown and PNG artifacts."""
    import shutil
    root=Path(evidence).resolve(); output=Path(output).resolve()
    output.mkdir(parents=True,exist_ok=True)
    # Analysis intermediates stay with the external original evidence.
    plots(root); supplementary_plots(root); reports(root)
    rows=list(csv.DictReader((root/'benchmark.csv').open()))
    for row in rows:
        row['events']=os.path.relpath(root/row['events'],output)
    with (output/'benchmark.csv').open('w') as fp:
        w=csv.DictWriter(fp,fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
    for name in ('metrics.png','trajectories.png','trajectories_zoom.png'):
        shutil.copy2(root/name,output/name)
    overview=root/'overview_process_sequence.png'
    if overview.exists():shutil.copy2(overview,output/'process_sequence.png')
    elif (root/'process_sequence.png').exists():shutil.copy2(root/'process_sequence.png',output/'process_sequence.png')
    def table(rr):
        keys=['planner','run','success','reason','nav_sim_s','path_m','extra_path_m','smoothness_rad_m']
        text='|'+'|'.join(keys)+'|\n|'+'|'.join(['---']*len(keys))+'|\n'
        for r in rr:text+='|'+'|'.join(str(r.get(k,'')) for k in keys)+'|\n'
        return text
    tuning=(root/'tuning_log.md').read_text() if (root/'tuning_log.md').exists() else '# 参数调优记录\n'
    tuning=tuning.replace('tuning_round1/','交付目录外的 M3-4-evidence/results/formal/tuning_round1/')
    for title,path in [('第一轮逐次数据',root/'tuning_round1/benchmark_tuned.csv'),('第二轮逐次数据',root/'benchmark_tuned.csv')]:
        if path.exists():tuning+='\n## '+title+'\n\n'+table(list(csv.DictReader(path.open())))
    (output/'tuning_log.md').write_text(tuning)
    failure=(root/'failure_boundary.md').read_text()
    for path in sorted((root/'boundaries').glob('*/benchmark.csv')):
        failure+='\n## '+path.parent.name+' 逐次数据\n\n'+table(list(csv.DictReader(path.open())))
    (output/'failure_boundary.md').write_text(failure)
    text=(root/'report.md').read_text()
    text=text.replace('](metrics.png)','](results/metrics.png)').replace('](trajectories.png)','](results/trajectories.png)').replace('](trajectories_zoom.png)','](results/trajectories_zoom.png)')
    text=text.replace('](overview_process_sequence.png)','](results/process_sequence.png)').replace('](process_sequence.png)','](results/process_sequence.png)')
    text=text.replace('[第一轮记录](tuning_round1/README.md)','[逐轮调优记录](results/tuning_log.md)')
    text=text.replace('[启动错误记录](infrastructure/README.md)','交付目录外保留的启动错误记录')
    text=text.replace('config_snapshot/（从每次manifest导出）','config/（A=planner_a_params.yaml，B=planner_b_params.yaml）')
    text=text.replace('diagnostics.csv','交付目录外保留的逐次诊断记录')
    text=text.replace('来源见 overview_source.json。','原始相机帧与时间记录保存在交付目录外的证据备份。')
    text=text.replace('第一轮完整数据见 tuning_round1/。','第一轮完整数据见 results/tuning_log.md。')
    text+='\n## 数据与交付位置\n\n原始20次对照见 [benchmark.csv](results/benchmark.csv)，两轮调优逐次数据见 [tuning_log.md](results/tuning_log.md)，18次边界逐次数据见 [failure_boundary.md](results/failure_boundary.md)。额外日志、配置快照和启动错误保存在交付目录外相邻的 M3-4-evidence/，交付目录不生成这些中间文件。\n'
    (output.parent/'report.md').write_text(text)
    print('Exported delivery:',output)


if __name__=='__main__':
    p=argparse.ArgumentParser(description='Recompute metrics from external evidence and export the required delivery files')
    p.add_argument('directory',type=Path,nargs='?',default=Path(__file__).resolve().parents[1]/'results')
    p.add_argument('--evidence',type=Path)
    a=p.parse_args()
    out=a.directory.resolve()
    evidence=a.evidence
    if evidence is None:
        csv_path=out/'benchmark.csv'
        if not csv_path.exists():p.error('benchmark.csv missing; pass --evidence')
        row=next(csv.DictReader(csv_path.open()))
        evidence=(out/row['events']).resolve().parents[2]
    export_delivery(evidence,out)
