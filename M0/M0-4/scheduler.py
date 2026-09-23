import argparse
import json
import random
import sys
import time
from pathlib import Path

import yaml


# ---------- 彩色日志 ----------
class Color:
    GREEN = "\033[92m"
    RED = "\033[91m"
    YELLOW = "\033[93m"
    BLUE = "\033[94m"
    RESET = "\033[0m"


def status_print(message, status):
    colors = {
        "SUCCESS": Color.GREEN,
        "FAILED": Color.RED,
        "RETRY": Color.YELLOW,
        "SKIPPED": Color.BLUE,
        "TIMEOUT": Color.RED,
    }

    # 只有真正的终端才输出 ANSI 颜色
    if sys.stdout.isatty():
        color = colors.get(status, "")

        print(
            f"{color}"
            f"{message}"
            f"{Color.RESET}"
        )

    else:
        # 输出重定向到文件时不用颜色
        print(message)


# ---------- 命令行参数 ----------
def parse_args():
    parser = argparse.ArgumentParser(
        description="Task scheduler simulator"
    )

    parser.add_argument(
        "--config",
        required=True,
        help="path to tasks.yaml, tasks.yml, or tasks.json",
    )

    parser.add_argument(
        "--timeout",
        type=float,
        default=None,
        help="global timeout in seconds, overrides config timeout",
    )

    parser.add_argument(
        "--report",
        default="report.json",
        help="report output path (default: report.json)",
    )

    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="random seed for reproducible results",
    )

    return parser.parse_args()


# ---------- 加载配置 ----------
def load_config(path):
    path = Path(path)

    suffix = path.suffix.lower()

    if suffix in [".yaml", ".yml"]:
        with open(path, "r") as f:
            config = yaml.safe_load(f)

    elif suffix == ".json":
        with open(path, "r") as f:
            config = json.load(f)

    else:
        raise ValueError(
            "unsupported config format: "
            "use .yaml, .yml, or .json"
        )

    return config


# ---------- 检查配置 ----------
def validate_config(config):
    if not isinstance(config, dict):
        raise ValueError(
            "config must be an object"
        )

    if "tasks" not in config:
        raise ValueError(
            "config must contain 'tasks'"
        )

    if not isinstance(
        config["tasks"],
        list,
    ):
        raise ValueError(
            "'tasks' must be a list"
        )

    if len(config["tasks"]) == 0:
        raise ValueError(
            "task list cannot be empty"
        )

    names = set()

    for task in config["tasks"]:
        if not isinstance(task, dict):
            raise ValueError(
                "each task must be an object"
            )

        required_fields = [
            "name",
            "duration",
            "success_rate",
            "dependencies",
        ]

        for field in required_fields:
            if field not in task:
                raise ValueError(
                    f"task is missing "
                    f"required field: {field}"
                )

        name = task["name"]

        if name in names:
            raise ValueError(
                f"duplicate task name: {name}"
            )

        names.add(name)

        duration = task["duration"]
        success_rate = task["success_rate"]
        dependencies = task["dependencies"]

        if (
            not isinstance(
                duration,
                (int, float),
            )
            or duration < 0
        ):
            raise ValueError(
                f"task '{name}': "
                f"duration must be >= 0"
            )

        if (
            not isinstance(
                success_rate,
                (int, float),
            )
            or not 0 <= success_rate <= 1
        ):
            raise ValueError(
                f"task '{name}': "
                f"success_rate must be "
                f"between 0 and 1"
            )

        if not isinstance(
            dependencies,
            list,
        ):
            raise ValueError(
                f"task '{name}': "
                f"dependencies must be a list"
            )

    for task in config["tasks"]:
        name = task["name"]

        for dependency in task["dependencies"]:
            if dependency not in names:
                raise ValueError(
                    f"task '{name}' depends on "
                    f"unknown task: {dependency}"
                )


# ---------- 拓扑排序 ----------
def topological_sort(tasks):
    indegree = {}
    graph = {}

    for task in tasks:
        name = task["name"]

        indegree[name] = len(
            task["dependencies"]
        )

        graph[name] = []

    for task in tasks:
        name = task["name"]

        for dependency in task["dependencies"]:
            graph[dependency].append(name)

    ready = []

    for name in indegree:
        if indegree[name] == 0:
            ready.append(name)

    order = []

    while ready:
        current = ready.pop(0)

        order.append(current)

        for next_task in graph[current]:
            indegree[next_task] -= 1

            if indegree[next_task] == 0:
                ready.append(next_task)

    if len(order) != len(tasks):
        raise ValueError(
            "dependency cycle detected"
        )

    return order


# ---------- 时间监控 ----------
def elapsed_since(start_time):
    return time.monotonic() - start_time


def is_timeout(start_time, timeout):
    if timeout is None:
        return False

    return (
        elapsed_since(start_time)
        >= timeout
    )


# ---------- 执行单个任务 ----------
def run_task(
    task,
    program_start,
    timeout,
):
    name = task["name"]
    duration = task["duration"]
    success_rate = task["success_rate"]

    max_attempts = 3

    started_at = None
    ended_at = None

    for attempt in range(
        1,
        max_attempts + 1,
    ):

        if is_timeout(
            program_start,
            timeout,
        ):
            return (
                "TIMEOUT",
                attempt - 1,
                started_at,
                ended_at,
            )

        if started_at is None:
            started_at = time.time()

        print(
            f"Running {name}, "
            f"attempt {attempt}"
        )

        time.sleep(duration)

        ended_at = time.time()

        if is_timeout(
            program_start,
            timeout,
        ):
            status_print(
                f"{name}: TIMEOUT",
                "TIMEOUT",
            )

            return (
                "TIMEOUT",
                attempt,
                started_at,
                ended_at,
            )

        value = random.random()

        if value < success_rate:
            status_print(
                f"{name}: SUCCESS",
                "SUCCESS",
            )

            return (
                "SUCCESS",
                attempt,
                started_at,
                ended_at,
            )

        status_print(
            f"{name}: FAILED",
            "FAILED",
        )

        if attempt < max_attempts:
            status_print(
                f"{name}: RETRY",
                "RETRY",
            )

    return (
        "SKIPPED",
        max_attempts,
        started_at,
        ended_at,
    )


# ---------- 写报告 ----------
def write_report(path, report):
    path = Path(path)

    with open(
        path,
        "w",
        encoding="utf-8",
    ) as f:
        json.dump(
            report,
            f,
            indent=2,
            ensure_ascii=False,
        )


# ---------- 主程序 ----------
def main():
    args = parse_args()

    if args.seed is not None:
        random.seed(args.seed)

    config = load_config(
        args.config
    )

    validate_config(config)

    if args.timeout is not None:
        if args.timeout <= 0:
            raise ValueError(
                "--timeout must be "
                "greater than 0"
            )

        timeout = args.timeout

    else:
        timeout = config.get(
            "timeout"
        )

    order = topological_sort(
        config["tasks"]
    )

    task_map = {}

    for task in config["tasks"]:
        task_map[
            task["name"]
        ] = task

    print("execution order:")

    for name in order:
        print(name)

    print()

    program_start = time.monotonic()

    statuses = {}
    results = {}

    for name in order:

        if is_timeout(
            program_start,
            timeout,
        ):
            status_print(
                "Global scheduler "
                "status: TIMEOUT",
                "TIMEOUT",
            )
            break

        task = task_map[name]

        dependency_failed = False

        for dependency in task["dependencies"]:
            if (
                statuses[dependency]
                != "SUCCESS"
            ):
                dependency_failed = True
                break

        if dependency_failed:
            status = "SKIPPED"
            attempts = 0
            started_at = None
            ended_at = None

            status_print(
                f"{name}: SKIPPED because "
                f"dependency failed",
                "SKIPPED",
            )

        else:
            (
                status,
                attempts,
                started_at,
                ended_at,
            ) = run_task(
                task,
                program_start,
                timeout,
            )

        statuses[name] = status

        results[name] = {
            "name": name,
            "status": status,
            "attempts": attempts,
            "duration": task["duration"],
            "started_at": started_at,
            "ended_at": ended_at,
        }

        print(
            f"final: {name} = {status}, "
            f"attempts = {attempts}"
        )

        if status == "TIMEOUT":
            status_print(
                "Global scheduler "
                "status: TIMEOUT",
                "TIMEOUT",
            )
            break

    total_duration = elapsed_since(
        program_start
    )

    report = {
        "timeout": any(
            result["status"]
            == "TIMEOUT"
            for result
            in results.values()
        ),
        "total_duration": round(
            total_duration,
            2,
        ),
        "tasks": list(
            results.values()
        ),
    }

    write_report(
        args.report,
        report,
    )

    print()

    print(
        f"Total duration: "
        f"{total_duration:.2f}s"
    )

    print(
        f"Report written to: "
        f"{args.report}"
    )


# ---------- 程序入口 ----------
if __name__ == "__main__":
    try:
        main()

    except FileNotFoundError as e:
        print(
            f"Error: config file "
            f"not found: {e.filename}"
        )
        sys.exit(1)

    except yaml.YAMLError as e:
        print(
            f"Error: invalid YAML: {e}"
        )
        sys.exit(1)

    except json.JSONDecodeError as e:
        print(
            f"Error: invalid JSON: {e}"
        )
        sys.exit(1)

    except ValueError as e:
        print(
            f"Error: {e}"
        )
        sys.exit(1)

