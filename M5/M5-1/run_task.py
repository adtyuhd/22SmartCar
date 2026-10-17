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
    name = config.get("name")

    if not name:
        raise ValueError("Missing required field: name")

    params = {
        key: value
        for key, value in config.items()
        if key not in ("name", "type", "parallel")
    }

    if "parallel" in config:
        parallel = config["parallel"]

        branches_config = parallel.get("branches")

        if not isinstance(branches_config, list) or not branches_config:
            raise ValueError(f"Parallel '{name}' requires non-empty branches")

        fail_policy = parallel.get("fail_policy", "all_must_succeed")

        if fail_policy not in ("any_fail", "all_must_succeed"):
            raise ValueError(f"Invalid fail_policy: {fail_policy}")

        branches = [
            build_node(branch, ctx)
            for branch in branches_config
        ]

        node = ParallelExecutor(name, branches, fail_policy)
        node.params = params

        return node

    node_type = config.get("type")

    if not node_type:
        raise ValueError(f"Node '{name}' missing required field: type")

    return create_node(name, node_type, params, ctx)


def load_task(path):
    try:
        with open(path, "r", encoding="utf-8") as f:
            config = yaml.safe_load(f)
    except yaml.YAMLError as e:
        mark = getattr(e, "problem_mark", None)

        if mark is not None:
            raise ValueError(
                f"YAML syntax error at line {mark.line + 1}: "
                f"{getattr(e, 'problem', str(e))}"
            ) from e

        raise ValueError(f"Invalid YAML: {e}") from e

    if not isinstance(config, dict):
        raise ValueError("YAML root must be a dictionary")

    tasks = config.get("tasks")

    if not isinstance(tasks, list) or not tasks:
        raise ValueError("Missing or empty tasks list")

    return config


def save_report(report):
    with open("task_report.json", "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)


def main():
    if len(sys.argv) != 2:
        print("Usage: python run_task.py <task.yaml>")
        return 1

    try:
        config = load_task(sys.argv[1])

        task_id = config.get("task_id", "demo")

        deadline = config.get("deadline")

        if deadline is not None:
            if not isinstance(deadline, (int, float)) or deadline <= 0:
                raise ValueError("deadline must be a positive number")

            deadline = time.monotonic() + deadline

        ctx = TaskContext(task_id, deadline)

        nodes = [
            build_node(task_config, ctx)
            for task_config in config["tasks"]
        ]

        scheduler = TaskScheduler(nodes, ctx)

        start_time = time.monotonic()

        try:
            result = scheduler.run()
        except KeyboardInterrupt:
            result = "ABORTED"
            for node in nodes:
                try:
                    node.cancel()
                except Exception:
                    pass
            print("Task interrupted", file=sys.stderr)
        except Exception as e:
            result = "FAILED"
            for node in nodes:
                try:
                    node.cancel()
                except Exception:
                    pass
            print(f"Task execution error: {type(e).__name__}: {e}", file=sys.stderr)
        finally:
            report = {
                "task_id": task_id,
                "result": result,
                "total_elapsed_s": round(time.monotonic() - start_time, 3),
                "steps": scheduler.step_records,
                "final_context": ctx.data
            }
            save_report(report)

        log(f"Task {task_id}: {result}", result)

        return 0 if result == "SUCCESS" else 1

    except (ValueError, OSError) as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())