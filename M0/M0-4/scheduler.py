import argparse
import json
import random
import sys
import time
from pathlib import Path

import yaml


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
            "unsupported config format: use .yaml, .yml, or .json"
        )

    return config


# ---------- 检查配置 ----------
def validate_config(config):
    if not isinstance(config, dict):
        raise ValueError("config must be an object")

    if "tasks" not in config:
        raise ValueError("config must contain 'tasks'")

    if not isinstance(config["tasks"], list):
        raise ValueError("'tasks' must be a list")

    if len(config["tasks"]) == 0:
        raise ValueError("task list cannot be empty")

    names = set()

    # 第一轮：检查每个任务本身
    for task in config["tasks"]:
        if not isinstance(task, dict):
            raise ValueError("each task must be an object")

        required_fields = [
            "name",
            "duration",
            "success_rate",
            "dependencies",
        ]

        for field in required_fields:
            if field not in task:
                raise ValueError(
                    f"task is missing required field: {field}"
                )

        name = task["name"]

        # 任务名不能重复
        if name in names:
            raise ValueError(
                f"duplicate task name: {name}"
            )

        names.add(name)

        duration = task["duration"]
        success_rate = task["success_rate"]
        dependencies = task["dependencies"]

        # duration 必须是非负数字
        if (
            not isinstance(duration, (int, float))
            or duration < 0
        ):
            raise ValueError(
                f"task '{name}': duration must be >= 0"
            )

        # success_rate 必须在 0 ~ 1
        if (
            not isinstance(success_rate, (int, float))
            or not 0 <= success_rate <= 1
        ):
            raise ValueError(
                f"task '{name}': success_rate must be between 0 and 1"
            )

        # dependencies 必须是列表
        if not isinstance(dependencies, list):
            raise ValueError(
                f"task '{name}': dependencies must be a list"
            )

    # 第二轮：检查依赖是否存在
    for task in config["tasks"]:
        name = task["name"]

        for dependency in task["dependencies"]:
            if dependency not in names:
                raise ValueError(
                    f"task '{name}' depends on unknown task: {dependency}"
                )


# ---------- 拓扑排序 ----------
def topological_sort(tasks):
    indegree = {}
    graph = {}

    # 初始化
    for task in tasks:
        name = task["name"]

        # 当前任务有多少个前置依赖
        indegree[name] = len(task["dependencies"])

        # 当前任务完成后影响哪些任务
        graph[name] = []

    # 建立依赖关系
    for task in tasks:
        name = task["name"]

        for dependency in task["dependencies"]:
            graph[dependency].append(name)

    # 找到一开始没有依赖的任务
    ready = []

    for name in indegree:
        if indegree[name] == 0:
            ready.append(name)

    # 保存最终执行顺序
    order = []

    # 拓扑排序
    while ready:
        current = ready.pop(0)

        order.append(current)

        for next_task in graph[current]:
            indegree[next_task] -= 1

            if indegree[next_task] == 0:
                ready.append(next_task)

    # 如果没有处理完全部任务，说明存在依赖环
    if len(order) != len(tasks):
        raise ValueError(
            "dependency cycle detected"
        )

    return order


# ---------- 执行单个任务 ----------
def run_task(task):
    name = task["name"]
    duration = task["duration"]
    success_rate = task["success_rate"]

    max_attempts = 3

    for attempt in range(1, max_attempts + 1):
        print(
            f"Running {name}, attempt {attempt}"
        )

        # 模拟任务执行时间
        time.sleep(duration)

        # 产生 0 ~ 1 的随机数
        value = random.random()

        # 成功
        if value < success_rate:
            print(
                f"{name}: SUCCESS"
            )

            return "SUCCESS", attempt

        # 本次尝试失败
        print(
            f"{name}: FAILED"
        )

        # 前两次失败还可以重试
        if attempt < max_attempts:
            print(
                f"{name}: RETRY"
            )

    # 三次全部失败
    return "SKIPPED", max_attempts


# ---------- 主程序 ----------
def main():
    args = parse_args()

    # 设置随机种子
    if args.seed is not None:
        random.seed(args.seed)

    # 读取配置
    config = load_config(args.config)

    # 检查配置
    validate_config(config)

    # 根据依赖关系进行拓扑排序
    order = topological_sort(
        config["tasks"]
    )

    # 建立：
    # 任务名 -> 完整任务配置
    task_map = {}

    for task in config["tasks"]:
        task_map[task["name"]] = task

    # 打印拓扑排序结果
    print("execution order:")

    for name in order:
        print(name)

    print()

    # 保存每个已经处理任务的最终状态
    statuses = {}

    # 按拓扑顺序执行
    for name in order:
        task = task_map[name]

        # ---------- 检查前置依赖 ----------
        dependency_failed = False

        for dependency in task["dependencies"]:
            if statuses[dependency] != "SUCCESS":
                dependency_failed = True
                break

        # ---------- 前置任务失败 ----------
        if dependency_failed:
            status = "SKIPPED"
            attempts = 0

            print(
                f"{name}: SKIPPED because dependency failed"
            )

        # ---------- 前置任务全部成功 ----------
        else:
            status, attempts = run_task(task)

        # 保存最终状态
        statuses[name] = status

        print(
            f"final: {name} = {status}, "
            f"attempts = {attempts}"
        )


# ---------- 程序入口 ----------
if __name__ == "__main__":
    try:
        main()

    # 文件不存在
    except FileNotFoundError as e:
        print(
            f"Error: config file not found: {e.filename}"
        )
        sys.exit(1)

    # YAML 语法错误
    except yaml.YAMLError as e:
        print(
            f"Error: invalid YAML: {e}"
        )
        sys.exit(1)

    # JSON 语法错误
    except json.JSONDecodeError as e:
        print(
            f"Error: invalid JSON: {e}"
        )
        sys.exit(1)

    # 配置错误 / 依赖错误 / 环
    except ValueError as e:
        print(
            f"Error: {e}"
        )
        sys.exit(1)
