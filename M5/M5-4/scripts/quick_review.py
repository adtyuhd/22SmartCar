"""Review one run and automatically compare with the preceding report."""
import argparse
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import statistics
import sys

from parse_mcap import read_mcap

ROOT = Path(__file__).resolve().parents[1]
METRICS = {
    "total_time": "s", "nav_time": "s", "nav_success_rate": "%",
    "path_optimality": "%", "perception_fps": "Hz", "confidence_mean": "0–1",
    "cpu_peak": "%", "memory_peak": "MB", "temperature_peak": "°C",
}


def number(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and abs(x) < float("inf")


def epoch(value):
    if number(value):
        return value
    if isinstance(value, str):
        try:
            dt = datetime.fromisoformat(value.replace("Z", "+00:00"))
            return dt.timestamp() if dt.tzinfo else None
        except ValueError:
            pass
    return None


def fmt(value):
    return f"{value:.3f}" if number(value) else ("—" if value is None else str(value))


def cell(value):
    return fmt(value).replace("|", r"\|").replace("\n", " ")


def review(mcap, logs, switch_log, task_report, output, previous_dir=None):
    messages, issues, skipped = read_mcap(mcap)
    task = {}
    try:
        task = json.loads(task_report.read_text())
        if not isinstance(task, dict):
            raise ValueError("预期为 JSON 对象")
    except (OSError, ValueError) as exc:
        task = {}
        issues.append(f"任务报告不可用：{exc}")
    steps = [step for step in (task.get("steps") or []) if isinstance(step, dict)]
    switches = []
    switch_known = False
    try:
        with switch_log.open(newline="", encoding="utf-8-sig") as file:
            reader = csv.DictReader(file)
            if not {"ts", "op", "target", "to_value", "result"} <= set(reader.fieldnames or []):
                raise ValueError("缺少 M5-3 切换日志列")
            switches = [row for row in reader if row["result"] in {"ok", "success"}]
            switch_known = True
    except (OSError, ValueError, csv.Error) as exc:
        issues.append(f"切换记录不可用：{exc}")

    samples, faults, actions, anchors = [], [], [], []
    for ns, topic, msg in messages:
        if topic.endswith("/clock"):
            wall = epoch(msg.get("wall_time"))
            if wall is not None:
                anchors.append((ns, wall))
        elif topic.endswith("/metrics"):
            samples.append(msg)
        elif topic.endswith("/faults") and not msg.get("self_healed"):
            faults.append((ns, msg))
    if logs.is_dir():
        for path in sorted(logs.rglob("*")):
            if not path.is_file() or path.suffix not in {".log", ".jsonl", ".txt"}:
                continue
            try:
                with path.open(errors="replace") as file:
                    for line in file:
                        # Structured JSON events may follow a ROS logger prefix.
                        if "{" not in line or len(line) > 65536:
                            continue
                        try:
                            event, _ = json.JSONDecoder().raw_decode(line[line.index("{"):])
                        except ValueError:
                            continue
                        if event.get("event") == "fault":
                            faults.append((None, event))
                        elif event.get("event") == "fault_response":
                            actions.append(event)
            except OSError as exc:
                issues.append(f"节点日志读取失败：{exc}")
    else:
        issues.append("节点日志目录缺失")
    anchors.sort()

    def wall_time(ns, msg):
        wall = epoch(msg.get("wall_time", msg.get("timestamp")))
        if wall is not None or ns is None or not anchors:
            return wall
        before = max((a for a in anchors if a[0] <= ns), default=anchors[0])
        after = min((a for a in anchors if a[0] >= ns), default=anchors[-1])
        if before[0] != after[0]:
            return before[1] + (ns - before[0]) * (after[1] - before[1]) / (after[0] - before[0])
        return before[1] if before[0] == ns else None

    nav = [step for step in steps if step.get("name", "").lower().startswith(("nav", "navigate"))]
    completed_nav = [s for s in nav if str(s.get("status")).upper() not in {"SKIPPED", "NONE"}]
    nav_times = [s["elapsed_s"] for s in completed_nav if number(s.get("elapsed_s"))]

    def aggregate(key, reducer):
        values = [s[key] for s in samples if number(s.get(key))]
        return reducer(values) if values else None

    context = task.get("final_context") or {}
    actual, optimal = (aggregate(k, max) for k in ("actual_path_m", "optimal_path_m"))
    metrics = {
        "total_time": task.get("total_elapsed_s"),
        "nav_time": sum(nav_times) if nav_times else None,
        "nav_success_rate": 100 * sum(str(s.get("status")).upper() in {"SUCCESS", "SUCCEEDED"} for s in completed_nav) / len(completed_nav) if completed_nav else None,
        "path_optimality": 100 * optimal / actual if actual and optimal is not None else None,
        "perception_fps": aggregate("perception_fps", statistics.mean),
        "confidence_mean": aggregate("confidence_mean", statistics.mean),
        "cpu_peak": aggregate("cpu_percent", max),
        "memory_peak": aggregate("memory_mb", max),
        "temperature_peak": aggregate("temperature_c", max),
    }
    for key in ("nav_time", "nav_success_rate", "path_optimality"):
        if metrics[key] is None and isinstance(context, dict) and number(context.get(key)):
            metrics[key] = context[key]

    chains = {}
    seen = set()
    for ns, event in faults:
        kind = event.get("fault_id") or event.get("type")
        if not kind or (kind, ns, epoch(event.get("timestamp"))) in seen:
            continue
        seen.add((kind, ns, epoch(event.get("timestamp"))))
        wall = wall_time(ns, event)
        label = datetime.fromtimestamp(wall, timezone.utc).isoformat(timespec="seconds") if wall is not None else (f"MCAP t={ns / 1e9:.3f}s" if ns is not None else "时间未知")
        evidence = event.get("evidence") or event.get("details") or "故障话题触发"
        responses = []
        for action in actions:
            action_time = epoch(action.get("wall_time", action.get("timestamp")))
            same_id = event.get("event_id") and event["event_id"] == action.get("event_id")
            close = wall is not None and action_time is not None and 0 <= action_time - wall <= 10
            if action.get("fault_id") == kind and (same_id or close):
                responses.append(action)
        response = responses[0] if len(responses) == 1 else {}
        action = event.get("action") or response.get("action")
        cost = event.get("cost") or response.get("cost")
        if not action:
            for row in switches:
                when = epoch(row.get("ts"))
                try:
                    detail = json.loads(row.get("details") or "{}")
                except ValueError:
                    continue
                if (isinstance(detail, dict) and detail.get("fault_id") == kind and
                        wall is not None and when is not None and 0 <= when - wall <= 10):
                    action = f"{row['op']} {row['target']} → {row['to_value']}"
                    cost = detail.get("cost")
                    break
        chains.setdefault(kind, []).append(
            f"{kind}@{label} ({evidence}) → {action or '处理动作证据不足'} → {cost or '代价未测得'}"
        )

    status = str(task.get("result", "UNKNOWN")).upper()
    result = {
        "recorded_at": epoch(task.get("started_at")) or (anchors[0][1] if anchors else None)
                       or (min((ns / 1e9 for ns, _, _ in messages), default=0)
                           if messages and messages[0][0] >= 1_000_000_000_000_000_000
                           else mcap.stat().st_mtime if mcap.exists() else 0),
        "source": str(mcap.resolve()), "status": status, "metrics": metrics,
        "faults": sorted(chains), "fault_data_known": any(topic.endswith("/faults") for _, topic, _ in messages)
        or bool(faults) or any(sample.get("fault_events_complete") is True for sample in samples),
        "switches": switches if switch_known else None,
    }
    previous = None
    for path in (previous_dir or output).glob("review_*.md"):
        try:
            with path.open() as file:
                line = file.readline()
            if not line.startswith("<!-- m5-4 "):
                continue
            old = json.loads(line[len("<!-- m5-4 "):-5])
            key = old["recorded_at"], old["source"]
            if old["source"] != result["source"] and key < (result["recorded_at"], result["source"]):
                if previous is None or key > (previous["recorded_at"], previous["source"]):
                    previous = old
        except (OSError, ValueError, KeyError, TypeError):
            continue
    lines = [f"# 运行复盘 · {mcap.name}", "", f"**{status}** · 总耗时 {fmt(metrics['total_time'])} s", "",
             "## 子任务", "", "| 步骤 | 状态 | 耗时 (s) | 重试 | 降级 |",
             "|---|---|---:|---:|---|"]
    lines.extend("| " + " | ".join(cell(s.get(k)) for k in
                 ("name", "status", "elapsed_s", "retries", "degraded")) + " |" for s in steps)
    lines += ["", "## 关键指标", "", "| 指标 | 当前 | 上一把 | 差值 |", "|---|---:|---:|---:|"]
    for key, unit in METRICS.items():
        value = metrics[key]
        old = previous["metrics"].get(key) if previous else None
        delta = f"{value - old:+.3f}" if number(value) and number(old) else "—"
        lines.append(f"| {key} ({unit}) | {fmt(value)} | {fmt(old)} | {delta} |")
    lines.extend(["", "## 故障根因", ""])
    lines.extend(f"- {chain}" for values in chains.values() for chain in values)
    if not chains:
        lines.append("未见故障事件。" if result["fault_data_known"] else "缺少故障事件来源。")
    lines.extend(["", "## 上一把与变更", ""])
    if previous:
        lines.append(f"- 上一把：{Path(previous['source']).name}")
        if result["fault_data_known"] and previous.get("fault_data_known"):
            lines.append(f"- 新增故障：{', '.join(sorted(set(chains) - set(previous['faults']))) or '无'}")
            lines.append(f"- 消失故障：{', '.join(sorted(set(previous['faults']) - set(chains))) or '无'}")
        else:
            lines.append("- 故障数据不足，无法确认新增或消失。")
    else:
        lines.append("- 暂无更早的已复盘记录。")
    for label, rows in [("本把", result["switches"])] + ([("上一把", previous["switches"])] if previous else []):
        if rows:
            for row in rows:
                lines.append(f"- {label} {row['ts']} {row['op']} {row['target']}：{row.get('from_value') or '未知'} → {row['to_value']}")
        else:
            lines.append(f"- {label}切换：{'无成功切换' if rows is not None else '记录缺失'}")
    missing = [k for k, v in metrics.items() if not number(v)]
    if missing:
        issues.append("无有效数据的指标：" + ", ".join(missing))
    if not anchors:
        issues.append("未录制同步锚点；仅用事件自带墙钟关联跨源记录")
    lines += ["", "## 数据质量", "", f"- 跳过的无关索引块：{skipped}"]
    lines.extend(f"- {issue}" for issue in issues)
    output.mkdir(parents=True, exist_ok=True)
    report = output / f"review_{datetime.now(timezone.utc):%Y%m%dT%H%M%S_%fZ}.md"
    metadata = json.dumps(result, ensure_ascii=False).replace("--", r"\u002d\u002d")
    report.write_text("<!-- m5-4 " + metadata + " -->\n\n" + "\n".join(lines) + "\n")
    color = ("\033[32m" if status == "SUCCESS" else "\033[31m") if sys.stdout.isatty() else ""
    reset = "\033[0m" if color else ""
    print(f"{color}{status}{reset} · {fmt(metrics['total_time'])} s · {report}")
    print(" · ".join(f"{k}={fmt(v)}" for k, v in metrics.items() if k != "total_time"))
    for values in chains.values():
        for chain in values:
            print(chain)
    for issue in issues:
        print(f"提示：{issue}")
    result["report"] = str(report)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mcap", type=Path, required=True)
    parser.add_argument("--logs", type=Path)
    parser.add_argument("--switch-log", type=Path)
    parser.add_argument("--task-report", type=Path)
    parser.add_argument("--out", type=Path, default=ROOT / "outputs")
    args = parser.parse_args()
    parent = args.mcap.parent
    try:
        review(args.mcap, args.logs or parent / "logs", args.switch_log or parent / "switch_log.csv",
               args.task_report or parent / "task_report.json", args.out)
    except (OSError, ValueError, TypeError) as exc:
        parser.exit(1, f"复盘失败：{exc}\n")


if __name__ == "__main__":
    main()
