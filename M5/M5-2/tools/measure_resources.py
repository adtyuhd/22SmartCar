#!/usr/bin/env python3
"""连续测量 M5-2 小车端与主机端进程的 CPU、内存占用。"""

import argparse
import json
import os
import sys
import time
from datetime import datetime
from pathlib import Path

import psutil


ROOT = Path(__file__).resolve().parents[1]

TARGETS = {
    "aggregator": {
        "suffix": "dashboard_aggregator.py",
        "label": "小车端聚合节点",
        "cpu_limit": 2.0,
        "memory_limit_mb": 50.0,
    },
    "host": {
        "suffix": "host_side/app.py",
        "label": "主机端 Web 应用",
        "cpu_limit": 5.0,
        "memory_limit_mb": 200.0,
    },
}


def find_processes(target_names):
    """根据实际启动的 Python 脚本路径查找目标进程。"""
    found = []

    for proc in psutil.process_iter(["pid", "cmdline"]):
        try:
            if proc.pid == os.getpid():
                continue

            cmdline = proc.info.get("cmdline") or []
            tokens = [str(arg).replace("\\", "/") for arg in cmdline]

            for name in target_names:
                suffix = TARGETS[name]["suffix"]

                if any(token.endswith(suffix) for token in tokens):
                    found.append({
                        "target": name,
                        "pid": proc.pid,
                        "process": psutil.Process(proc.pid),
                    })

        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue

    return found


def sample_processes(processes, duration, interval):
    """连续采样，返回各进程的 CPU 和 RSS 内存数据。"""
    records = []

    for item in processes:
        proc = item["process"]

        try:
            proc.cpu_percent(interval=None)

            records.append({
                "target": item["target"],
                "pid": item["pid"],
                "process": proc,
                "cpu_samples": [],
                "memory_samples_mb": [],
            })

        except (psutil.NoSuchProcess, psutil.AccessDenied):
            print(f"警告：无法读取 PID {item['pid']}")

    if not records:
        return []

    print(f"\n开始连续采样 {duration} 秒，采样间隔 {interval} 秒。")
    print("请保持两个被测进程运行，不要关闭对应终端。\n")

    start = time.monotonic()
    deadline = start + duration

    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()

        if remaining > 0:
            time.sleep(min(interval, remaining))

        for record in records:
            proc = record["process"]

            try:
                cpu = proc.cpu_percent(interval=None)
                memory_mb = proc.memory_info().rss / (1024 ** 2)

                record["cpu_samples"].append(cpu)
                record["memory_samples_mb"].append(memory_mb)

            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue

        elapsed = min(duration, time.monotonic() - start)
        print(f"\r采样进度：{elapsed:.0f}/{duration} 秒", end="", flush=True)

    print("\n采样结束。")
    return records


def summarize(records, duration, interval):
    """计算平均值、峰值，并按 README 阈值判断。"""
    results = []

    for record in records:
        name = record["target"]
        config = TARGETS[name]

        cpu = record["cpu_samples"]
        memory = record["memory_samples_mb"]

        if not cpu or not memory:
            results.append({
                "target": name,
                "label": config["label"],
                "pid": record["pid"],
                "status": "ERROR",
                "reason": "采样期间未能获取有效数据",
            })
            continue

        cpu_avg = sum(cpu) / len(cpu)
        memory_avg = sum(memory) / len(memory)
        memory_peak = max(memory)

        cpu_pass = cpu_avg < config["cpu_limit"]
        memory_pass = memory_avg < config["memory_limit_mb"]

        results.append({
            "target": name,
            "label": config["label"],
            "pid": record["pid"],
            "duration_s": duration,
            "interval_s": interval,
            "sample_count": len(cpu),
            "cpu_avg_percent": round(cpu_avg, 3),
            "memory_avg_mb": round(memory_avg, 3),
            "memory_peak_mb": round(memory_peak, 3),
            "cpu_limit_percent": config["cpu_limit"],
            "memory_limit_mb": config["memory_limit_mb"],
            "cpu_pass": cpu_pass,
            "memory_pass": memory_pass,
            "status": "PASS" if cpu_pass and memory_pass else "FAIL",
        })

    return results


def main():
    parser = argparse.ArgumentParser(
        description="M5-2 进程资源连续采样工具"
    )

    parser.add_argument(
        "--target",
        choices=["aggregator", "host", "both"],
        default="both",
        help="测量目标，默认同时测量两个进程",
    )

    parser.add_argument(
        "--duration",
        type=int,
        default=60,
        help="连续采样时长，单位秒，默认 60",
    )

    parser.add_argument(
        "--interval",
        type=float,
        default=1.0,
        help="采样间隔，单位秒，默认 1",
    )

    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="可选的 JSON 报告输出路径",
    )

    args = parser.parse_args()

    if args.duration <= 0 or args.interval <= 0:
        parser.error("--duration 和 --interval 必须大于 0")

    targets = (
        list(TARGETS)
        if args.target == "both"
        else [args.target]
    )

    print("========== M5-2 资源占用测量 ==========")
    print(f"测量时长：{args.duration} 秒")
    print(f"采样间隔：{args.interval} 秒")

    processes = find_processes(targets)

    for name in targets:
        matches = [p for p in processes if p["target"] == name]

        if not matches:
            print(f"未找到进程：{TARGETS[name]['label']}")
            print("请先启动对应程序，再运行本脚本。")
            return 2

        for item in matches:
            print(
                f"找到 {TARGETS[name]['label']}："
                f"PID={item['pid']}"
            )

    records = sample_processes(
        processes,
        args.duration,
        args.interval,
    )

    results = summarize(
        records,
        args.duration,
        args.interval,
    )

    report = {
        "timestamp": datetime.now().astimezone().isoformat(),
        "duration_s": args.duration,
        "interval_s": args.interval,
        "results": results,
    }

    print("\n========== 测量结果 ==========")

    for result in results:
        print(f"\n目标：{result['label']}（PID {result['pid']}）")

        if result["status"] == "ERROR":
            print(f"结果：{result['reason']}")
            continue

        print(f"CPU 平均值：{result['cpu_avg_percent']:.3f}%")
        print(f"内存平均值：{result['memory_avg_mb']:.3f} MB")
        print(f"内存峰值：{result['memory_peak_mb']:.3f} MB")
        print(
            f"CPU 阈值：< {result['cpu_limit_percent']}% —— "
            f"{'PASS' if result['cpu_pass'] else 'FAIL'}"
        )
        print(
            f"内存阈值：< {result['memory_limit_mb']} MB —— "
            f"{'PASS' if result['memory_pass'] else 'FAIL'}"
        )
        print(f"综合结果：{result['status']}")

    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(report, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        print(f"\nJSON 报告已保存：{args.output}")

    return 0 if results and all(
        result["status"] == "PASS" for result in results
    ) else 1


if __name__ == "__main__":
    sys.exit(main())