# 复盘字段

| 字段 | 数据来源和计算 |
|---|---|
| 结果、总耗时 | M5-1 task_report.json 的 result 和 total_elapsed_s，单位为秒。 |
| 子任务 | steps 中的 name、status、elapsed_s、retries、degraded。 |
| nav_time | 已执行的 nav 开头子任务耗时之和。 |
| nav_success_rate | 成功导航子任务数 ÷ 已执行导航子任务数 × 100%。 |
| path_optimality | 最优参考路径长度 ÷ 实际路径长度 × 100%。累计长度取各自最大值。 |
| perception_fps / confidence_mean | MCAP 指标采样值的算术均值。 |
| cpu_peak / memory_peak / temperature_peak | cpu_percent / memory_mb / temperature_c 的最大采样值。 |
| 上一把 | 更早且来源不同的最近一份复盘。差值为本把减上一把，故障按 ID 比较。 |

## 话题和时间对齐

- 白名单见 scripts/parse_mcap.py 的 TOPICS。读取 JSON 或封装 JSON 的 ROS 2 String。索引标明无关块时跳过整块。缺索引时遍历块，仍只解码白名单消息。坏包、缺话题和编码不匹配会列在数据质量中。
- MCAP log_time 使用纳秒。故障事件优先使用自带的 wall_time 或 timestamp（Unix 秒或带时区的 ISO 8601）。否则在 /m5/review/clock 的两个已知时间锚点间插值。无可靠墙钟时保留 MCAP 相对时间。仿真时钟发生回拨时应分段录制。
- M5-1 报告的 started_at 若存在，用于运行排序。只有 elapsed_s 时只能确定耗时。无开始锚点时可回退到系统录包时间或文件修改时间。
- 故障归因遵循「现象 → 动作 → 代价」。节点日志支持逐行 JSON，包括 event=fault_response、fault_id、event_id、wall_time、action、cost。同 ID 可直接关联，无 ID 时使用 10 秒内同类故障的唯一响应。切换日志 details 也可通过 fault_id 和 cost 提供证据。
- M5-3 switch_log.csv 的成功切换用于展示本把和上一把的参数、模型变化。文件缺失和没有成功切换分别显示。只有明确标注 fault_events_complete=true 或观测到故障流时才比较故障增减。
- 数据缺失显示为 —。按录制顺序复盘到同一输出目录，可自动取到上一把。
