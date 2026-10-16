from task_fsm.parallel import ParallelExecutor


class SuccessNode:
    def __init__(self, name):
        self.name = name
        self.params = {}
        self._cancelled = False

    def execute(self):
        return "SUCCESS"

    def cancel(self):
        self._cancelled = True

    def reset_cancel(self):
        self._cancelled = False


class FailedNode(SuccessNode):
    def execute(self):
        return "FAILED"


def test_parallel_any_fail():
    branches = [
        SuccessNode("a"),
        FailedNode("b"),
        SuccessNode("c"),
    ]

    parallel = ParallelExecutor(
        "parallel_test",
        branches,
        "any_fail"
    )

    assert parallel.execute() == "FAILED"


def test_parallel_all_must_succeed():
    branches = [
        SuccessNode("a"),
        SuccessNode("b"),
        FailedNode("c"),
    ]

    parallel = ParallelExecutor(
        "parallel_test",
        branches,
        "all_must_succeed"
    )

    assert parallel.execute() == "FAILED"


def test_parallel_all_success():
    branches = [
        SuccessNode("a"),
        SuccessNode("b"),
        SuccessNode("c"),
    ]

    parallel = ParallelExecutor(
        "parallel_test",
        branches,
        "all_must_succeed"
    )

    assert parallel.execute() == "SUCCESS"