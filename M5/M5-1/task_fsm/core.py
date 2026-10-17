import copy
import time
from concurrent.futures import ThreadPoolExecutor, TimeoutError

from .logger import log


class TaskDeadlineExceeded(Exception):
    pass


class TaskScheduler:
    def __init__(self, nodes, ctx):
        self.nodes = nodes
        self.ctx = ctx
        self.state = "IDLE"
        self.current_index = 0
        self.step_records = []
        self.node_map = {node.name: i for i, node in enumerate(nodes)}

    def _set_state(self, state, reason=""):
        if state != self.state:
            suffix = f" ({reason})" if reason else ""
            log(f"[state] {self.state} -> {state}{suffix}", "STATE")
            self.state = state

    def _record_step(self, node, status, started, retries, degraded):
        elapsed = round(time.monotonic() - started, 3)
        record = {
            "name": node.name,
            "status": status,
            "elapsed_s": elapsed,
            "retries": retries,
            "degraded": degraded,
        }
        self.step_records.append(record)

        log(
            f"[step] {node.name}: {status}, "
            f"elapsed={elapsed}s, retries={retries}",
            status,
        )

        if degraded:
            log(
                f"[degraded] {node.name}: continuing with reduced guarantees",
                "DEGRADED",
            )

    def _snapshot_context(self):
        try:
            return copy.deepcopy(self.ctx.data)
        except Exception:
            return dict(self.ctx.data)

    def _log_context_changes(self, before, node_name):
        after = self.ctx.data

        for key in before.keys() - after.keys():
            log(f"[context] {node_name}: removed '{key}'", "CONTEXT")

        for key, value in after.items():
            if key not in before:
                log(
                    f"[context] {node_name}: set '{key}' = {value!r}",
                    "CONTEXT",
                )
            elif before[key] != value:
                log(
                    f"[context] {node_name}: updated '{key}' = {value!r}",
                    "CONTEXT",
                )

    def _sleep_with_deadline(self, seconds):
        if seconds <= 0:
            return

        deadline = self.ctx.deadline

        if deadline is None:
            time.sleep(seconds)
            return

        remaining = deadline - time.monotonic()

        if remaining <= 0:
            raise TaskDeadlineExceeded

        time.sleep(min(seconds, remaining))

        if time.monotonic() >= deadline:
            raise TaskDeadlineExceeded

    def _execute_node(self, node):
        timeout = node.params.get("timeout")
        deadline = self.ctx.deadline

        if deadline is not None:
            remaining = deadline - time.monotonic()

            if remaining <= 0:
                node.cancel()
                raise TaskDeadlineExceeded

            timeout = remaining if timeout is None else min(
                timeout, remaining
            )

        if timeout is None:
            return node.execute()

        with ThreadPoolExecutor(max_workers=1) as pool:
            future = pool.submit(node.execute)

            try:
                return future.result(timeout=timeout)

            except TimeoutError:
                log(
                    f"[timeout] node '{node.name}' exceeded {timeout}s",
                    "TIMEOUT",
                )
                node.cancel()

                try:
                    future.result()
                except Exception:
                    pass

                if deadline is not None and time.monotonic() >= deadline:
                    raise TaskDeadlineExceeded

                return "FAILED"

    def _deadline_result(self, node, started, retries, degraded):
        log(
            f"[timeout] task deadline reached at node '{node.name}'",
            "TIMEOUT",
        )
        self._record_step(node, "TIMEOUT", started, retries, degraded)
        self._set_state("FAILED", "task deadline exceeded")
        return "TIMEOUT"

    def run(self):
        self.current_index = 0
        self.step_records = []
        fallback_count = 0
        max_fallbacks = len(self.nodes)

        self._set_state("RUNNING", "task started")

        while self.current_index < len(self.nodes):
            node = self.nodes[self.current_index]
            started = time.monotonic()

            if (
                self.ctx.deadline is not None
                and started >= self.ctx.deadline
            ):
                return self._deadline_result(node, started, 0, False)

            retries = 0
            retry_limit = node.params.get("retry", 0)
            degraded = False

            while True:
                node.reset_cancel()
                self._set_state("RUNNING", f"executing {node.name}")
                before = self._snapshot_context()

                try:
                    result = self._execute_node(node)
                except TaskDeadlineExceeded:
                    self._log_context_changes(before, node.name)
                    return self._deadline_result(
                        node, started, retries, degraded
                    )

                self._log_context_changes(before, node.name)

                if result == "SUCCESS":
                    self._record_step(
                        node, result, started, retries, degraded
                    )
                    self._set_state("SUCCESS", f"{node.name} completed")
                    self.current_index += 1
                    break

                if result == "SKIPPED":
                    self._record_step(
                        node, result, started, retries, degraded
                    )
                    self._set_state("SKIPPED", f"{node.name} skipped")
                    self.current_index += 1
                    break

                if result != "FAILED":
                    self._record_step(
                        node, "FAILED", started, retries, degraded
                    )
                    self._set_state(
                        "FAILED",
                        f"invalid result from {node.name}: {result!r}",
                    )
                    return "FAILED"

                if retries < retry_limit:
                    retries += 1
                    self._set_state(
                        "RETRYING",
                        f"{node.name}, attempt {retries}/{retry_limit}",
                    )
                    log(
                        f"[retry] {node.name}: retry "
                        f"{retries}/{retry_limit}",
                        "RETRYING",
                    )

                    try:
                        self._sleep_with_deadline(
                            node.params.get("retry_interval", 0)
                        )
                    except TaskDeadlineExceeded:
                        return self._deadline_result(
                            node, started, retries, degraded
                        )

                    continue

                fallback = node.params.get("fallback")

                if fallback is not None:
                    if (
                        fallback not in self.node_map
                        or fallback_count >= max_fallbacks
                    ):
                        self._record_step(
                            node, "FAILED", started, retries, True
                        )
                        self._set_state(
                            "FAILED",
                            f"invalid fallback or fallback limit: {fallback}",
                        )
                        return "FAILED"

                    fallback_count += 1
                    self._record_step(
                        node, "FAILED", started, retries, True
                    )
                    self._set_state(
                        "FAILED",
                        f"{node.name} failed; applying fallback",
                    )
                    log(
                        f"[degraded] fallback: {node.name} -> {fallback}",
                        "DEGRADED",
                    )
                    self._set_state("RUNNING", f"fallback to {fallback}")
                    self.current_index = self.node_map[fallback]
                    break

                if node.params.get("on_fail", "abort") == "skip":
                    self._record_step(
                        node, "SKIPPED", started, retries, True
                    )
                    self._set_state(
                        "SKIPPED",
                        f"{node.name} skipped after failure",
                    )
                    self.current_index += 1
                    break

                self._record_step(
                    node, "FAILED", started, retries, degraded
                )
                self._set_state("FAILED", f"{node.name} failed")
                return "FAILED"

        self._set_state("SUCCESS", "all tasks processed")
        return "SUCCESS"