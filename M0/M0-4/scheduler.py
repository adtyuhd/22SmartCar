import argparse
import json
from pathlib import Path

import yaml


def parse_args():
    parser = argparse.ArgumentParser(
        description="Task scheduler simulator"
    )

    parser.add_argument(
        "--config",
        required=True,
        help="path to tasks.yaml, tasks.yml, or tasks.json",
    )

    return parser.parse_args()


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


def validate_config(config):
    # 整个配置必须是字典
    if not isinstance(config, dict):
        raise ValueError("config must be an object")

    # 必须有 tasks
    if "tasks" not in config:
        raise ValueError("config must contain 'tasks'")

    # tasks 必须是列表
    if not isinstance(config["tasks"], list):
        raise ValueError("'tasks' must be a list")

    # 任务列表不能为空
    if len(config["tasks"]) == 0:
        raise ValueError("task list cannot be empty")

    # 保存所有已经出现的任务名
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

        # 检查必填字段
        for field in required_fields:
            if field not in task:
                raise ValueError(
                    f"task is missing required field: {field}"
                )

        name = task["name"]

        # 检查任务名是否重复
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

        # success_rate 必须在 0~1
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

    # 第二轮：检查依赖的任务是否真的存在
    for task in config["tasks"]:
        name = task["name"]

        for dependency in task["dependencies"]:
            if dependency not in names:
                raise ValueError(
                    f"task '{name}' depends on unknown task: {dependency}"
                )


def main():
    args = parse_args()

    config = load_config(args.config)

    validate_config(config)

    print(config)


if __name__ == "__main__":
    main()
