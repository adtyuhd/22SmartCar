import time
import threading

from task_fsm.context import TaskContext
from task_fsm.core import TaskScheduler
from task_fsm.parallel import ParallelExecutor


# ==================== Mock Nodes ====================

class SuccessNode:
    def __init__(self, name, params=None):
        self.name = name
        self.params = params or {}
        self._cancelled = False
        self.count = 0

    def execute(self):
        self.count += 1
        return "SUCCESS"

    def cancel(self):
        self._cancelled = True

    def reset_cancel(self):
        self._cancelled = False


class FailedNode(SuccessNode):
    def execute(self):
        self.count += 1
        return "FAILED"


class RetryNode(SuccessNode):
    def execute(self):
        self.count += 1

        if self.count == 1:
            return "FAILED"

        return "SUCCESS"


class TimeoutNode(SuccessNode):
    def __init__(self, name, params=None):
        super().__init__(name, params)
        self.event = threading.Event()

    def execute(self):
        self.count += 1
        self.event.wait(1)
        return "SKIPPED"

    def cancel(self):
        super().cancel()
        self.event.set()

    def reset_cancel(self):
        super().reset_cancel()
        self.event.clear()


# ==================== Parallel Tests ====================

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


# ==================== Scheduler Tests ====================

def test_scheduler_success():
    nodes = [
        SuccessNode("a"),
        SuccessNode("b"),
        SuccessNode("c"),
    ]

    ctx = TaskContext("test")
    scheduler = TaskScheduler(nodes, ctx)

    assert scheduler.run() == "SUCCESS"
    assert scheduler.state == "SUCCESS"
    assert scheduler.current_index == 3
    assert len(scheduler.step_records) == 3


def test_scheduler_retry():
    node = RetryNode("retry_node", {"retry": 1})

    ctx = TaskContext("test")
    scheduler = TaskScheduler([node], ctx)

    assert scheduler.run() == "SUCCESS"
    assert node.count == 2
    assert scheduler.step_records[0]["retries"] == 1
    assert scheduler.step_records[0]["status"] == "SUCCESS"


def test_scheduler_retry_exhausted():
    node = FailedNode("failed_node", {"retry": 2})

    ctx = TaskContext("test")
    scheduler = TaskScheduler([node], ctx)

    assert scheduler.run() == "FAILED"
    assert node.count == 3
    assert scheduler.step_records[0]["retries"] == 2
    assert scheduler.step_records[0]["status"] == "FAILED"


def test_scheduler_fallback():
    nodes = [
        FailedNode("a", {
            "retry": 1,
            "on_fail": "fallback",
            "fallback": "b"
        }),
        SuccessNode("b"),
    ]

    ctx = TaskContext("test")
    scheduler = TaskScheduler(nodes, ctx)

    assert scheduler.run() == "SUCCESS"
    assert nodes[0].count == 2
    assert nodes[1].count == 1
    assert scheduler.step_records[0]["status"] == "FAILED"
    assert scheduler.step_records[1]["status"] == "SUCCESS"


def test_scheduler_skip():
    nodes = [
        FailedNode("a", {"on_fail": "skip"}),
        SuccessNode("b"),
    ]

    ctx = TaskContext("test")
    scheduler = TaskScheduler(nodes, ctx)

    assert scheduler.run() == "SUCCESS"
    assert scheduler.step_records[0]["status"] == "SKIPPED"
    assert scheduler.step_records[0]["degraded"] is True
    assert scheduler.step_records[1]["status"] == "SUCCESS"


def test_scheduler_timeout():
    node = TimeoutNode("timeout_node", {
        "timeout": 0.05,
        "on_fail": "skip"
    })

    ctx = TaskContext("test")
    scheduler = TaskScheduler([node], ctx)

    assert scheduler.run() == "SUCCESS"
    assert node._cancelled is True
    assert scheduler.step_records[0]["status"] == "SKIPPED"
    assert scheduler.step_records[0]["degraded"] is True


def test_scheduler_deadline():
    ctx = TaskContext(
        "test",
        deadline=time.monotonic() - 1
    )

    node = SuccessNode("a")
    scheduler = TaskScheduler([node], ctx)

    assert scheduler.run() == "TIMEOUT"
    assert scheduler.state == "FAILED"
    assert node.count == 0

def test_fallback_without_on_fail():
    class MockNode:
        def __init__(self, name, result, params=None):
            self.name = name
            self.result = result
            self.params = params or {}

        def execute(self):
            return self.result

        def cancel(self):
            pass

        def reset_cancel(self):
            pass

    nodes = [
        MockNode("a", "FAILED", {"fallback": "b"}),
        MockNode("b", "SUCCESS"),
    ]

    scheduler = TaskScheduler(nodes, TaskContext("fallback_test"))

    assert scheduler.run() == "SUCCESS"
    assert [step["name"] for step in scheduler.step_records] == ["a", "b"]
    assert scheduler.step_records[0]["status"] == "FAILED"
    assert scheduler.step_records[0]["degraded"] is True
    assert scheduler.step_records[1]["status"] == "SUCCESS"


def test_fallback_cycle_terminates():
    class MockNode:
        def __init__(self, name, fallback):
            self.name = name
            self.params = {"fallback": fallback}

        def execute(self):
            return "FAILED"

        def cancel(self):
            pass

        def reset_cancel(self):
            pass

    nodes = [
        MockNode("a", "b"),
        MockNode("b", "a"),
    ]

    scheduler = TaskScheduler(nodes, TaskContext("cycle_test"))

    assert scheduler.run() == "FAILED"
    assert len(scheduler.step_records) == 3
    assert all(
        step["status"] == "FAILED"
        for step in scheduler.step_records
    )