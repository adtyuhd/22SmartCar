# M3-4：动态障碍局部规划与避障

## 运行约定

环境：ROS2 Humble、Gazebo Classic、已安装RPP/MPPI，以及相邻M3-1/M3-2/M3-3/M3-3-deps。需要可用DISPLAY。同一时间只运行一组（ROS域33，Gazebo端口11346）。

进入本目录后直接使用题目约定的命令。

```bash
# 基准测试：两种规划器，各10次
python3 scripts/run_benchmark.py --planners A,B --runs 10 \
  --obstacle-at 2.5,-5.05 --delay 3.0 --out results/

# 注入演示：跑1次
python3 scripts/run_benchmark.py --planners A --runs 1 \
  --obstacle-at 2.5,-5.05 --delay 3.0

# 现场随机障碍：替换为考官指定的世界坐标，跑2次
python3 scripts/run_benchmark.py --planners A --runs 2 \
  --obstacle-at 2.8,-5.05 --delay 2.5
```

A=RPP，B=MPPI（Ackermann）。三条命令都会打开Gazebo和RViz，接口就绪后自动发送目标，不需要回车，也不需要在RViz另发目标。每次从初始状态启动，导航开始且车已移动后，延迟指定秒数生成真实Gazebo障碍。

障碍使用Gazebo世界坐标，必须处于本次导航路径上。题目中的3.0,1.5为参数格式示例，不能直接当作当前地图的测试位置。本场景实测位置为2.5,-5.05，默认目标为map坐标5.5,0、朝向0度；map y≈0对应world y≈-5.05。改目标可用 `--goal x,y,yaw`，朝向单位为度。

每次调用建立独立批次，重复运行不会混入上次记录。首先执行指定次数的障碍测试；无障碍基线优先复用配置哈希、目标相同的3次成功实测参考，缺少时在障碍测试完成后补测各3次参考，再计算绕行长度。`--runs`统计障碍尝试次数，基线不计入成功率分母。终端分别标注“障碍测试”和“无障碍参考”，两阶段均显示Gazebo、RViz窗口。

完整的A/B各10次批次会更新results中的benchmark.csv和图表。单次演示、现场2次记录保存到交付目录外的 `../M3-4-evidence/runs/`，不会覆盖已有20次正式结果；终端会打印本次记录路径。

## 交付内容与实测

- scripts：inject_obstacle.py、run_benchmark.py、eval_metrics.py。
- config：planner_a_params.yaml、planner_b_params.yaml、planner_b_tuned.yaml。
- results：benchmark.csv、tuning_log.md、failure_boundary.md和PNG。
- report.md：四问分析报告。

已有正式数据：A为10/10、31.42秒；B为8/10、33.66秒（耗时只统计成功试次）。两轮调优为7/10和3/10，均未改善。planner_b_tuned.yaml保存第二轮测试配置，不作为比赛推荐配置。逐次数据见两份记录；原始证据、历史试跑和启动错误在交付目录外的M3-4-evidence保留。

重算当前交付数据：

```bash
PYTHONDONTWRITEBYTECODE=1 python3 scripts/eval_metrics.py results/
```

平滑度按0.05m弧长重采样，以δ=atan(0.28κ)计算单位距离转角变化；实际长度来自Gazebo真值，真值只用于评估。耗时、额外路径和平滑度均值只取成功试次，失败全部保留。

原始证据目录 `../M3-4-evidence/` 为本机保留数据，不随本次交付提交；完整逐次指标保存在交付内的 CSV 和 Markdown 中。新机器运行时会自动建立新的证据目录。

若 M3-3 启动文件使用 `m3_3_navigation` 包，须先完成 M3-3 构建；本脚本会自动加载其 `install/local_setup.bash`，无需在终端手动 source。
