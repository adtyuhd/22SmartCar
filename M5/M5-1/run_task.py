from task_fsm.parallel import ParallelExecutor
from task_fsm.registry import create_node


def build_node(config, ctx):
    if "parallel" in config:
        parallel_config = config["parallel"]

        branches = []

        for branch in parallel_config["branches"]:
            node = create_node(
                branch["name"],
                branch["type"],
                branch.get("params", {}),
                ctx
            )
            branches.append(node)

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