import json
import sys
import time

import yaml

from task_fsm.context import TaskContext
from task_fsm.core import TaskScheduler
from task_fsm.logger import log
from task_fsm.parallel import ParallelExecutor
from task_fsm.registry import create_node


def build_node(config, ctx):
    if "parallel" in config:
        parallel_config = config["parallel"]

        branches = []

        for branch in parallel_config["branches"]:
            branch_node = create_node(
                branch["name"],
                branch["type"],
                branch.get("params", {}),
                ctx
            )
            branches.append(branch_node)

        return ParallelExecutor(
            config["name"],
            branches,
            parallel_config.get(
                "fail_policy",
                "all_must_succeed"
            )
        )

    return create_node(
        config["name"],
        config["type"],
        config.get("params", {}),
        ctx
    )


def load_task(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def save_report(report, path="task_report.json"):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(
            report,
            f,
            ensure_ascii=False,
            indent=2
        )


def main():
    if len(sys.argv) != 2:
        print("Usage: python run_task.py <task.yaml>")
        return 1

    task_path = sys.argv[1]

    config = load_task(task_path)

    task_id = config["task_id"]

    deadline = config.get("deadline")

    ctx = TaskContext()

    if deadline is not None:
        ctx.deadline = time.monotonic() + deadline

    nodes = [
        build_node(node_config, ctx)
        for node_config in config["nodes"]
    ]

    scheduler = TaskScheduler(nodes, ctx)

    start_time = time.monotonic()

    result = scheduler.run()

    total_elapsed = time.monotonic() - start_time

    report = {
        "task_id": task_id,
        "result": result,
        "total_elapsed_s": round(total_elapsed, 3),
        "steps": scheduler.step_records,
        "final_context": ctx.data
    }

    save_report(report)

    log(
        f"Task {task_id}: {result}",
        result
    )

    return 0 if result == "SUCCESS" else 1


if __name__ == "__main__":
    sys.exit(main())