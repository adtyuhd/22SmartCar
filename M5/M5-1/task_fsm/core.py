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
        self.step_records.append({
            "name": node.name, "status": status, "elapsed_s": elapsed,
            "retries": retries, "degraded": degraded,
        })
        log(f"[step] {node.name}: {status}, elapsed={elapsed}s, retries={retries}", status)
        if degraded:
            log(f"[degraded] {node.name}: fallback or skip applied", "DEGRADED")

    def _log_context_changes(self, before, node):
        after = self.ctx.data
        for key in before.keys() - after.keys():
            log(f"[context] {node.name}: removed '{key}'", "CONTEXT")
        for key, value in after.items():
            if key not in before or before[key] != value:
                action = "set" if key not in before else "updated"
                log(f"[context] {node.name}: {action} '{key}' = {value!r}", "CONTEXT")

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

    def _execute_body(self, node):
        timeout = node.params.get("timeout")
        deadline = self.ctx.deadline
        if deadline is not None:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                node.cancel()
                raise TaskDeadlineExceeded
            timeout = remaining if timeout is None else min(timeout, remaining)
        if timeout is None:
            return node.execute()
        with ThreadPoolExecutor(max_workers=1) as pool:
            future = pool.submit(node.execute)
            try:
                return future.result(timeout=timeout)
            except TimeoutError:
                node.cancel()
                try:
                    future.result()
                except Exception:
                    pass
                if deadline is not None and time.monotonic() >= deadline:
                    raise TaskDeadlineExceeded
                return "FAILED"

    def _execute_node(self, node):
        setup = getattr(node, "setup", None)
        teardown = getattr(node, "teardown", None)
        setup_done = False
        result = None
        error = None
        try:
            if setup is not None:
                setup()
            setup_done = True
            result = self._execute_body(node)
        except Exception as exc:
            error = exc
        finally:
            if setup_done and teardown is not None:
                try:
                    teardown()
                except Exception as exc:
                    if error is None:
                        error = exc
        if error is not None:
            raise error
        return result

    def _deadline_result(self, node, started, retries, degraded):
        log(f"[timeout] task deadline reached at '{node.name}'", "TIMEOUT")
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
            if self.ctx.deadline is not None and started >= self.ctx.deadline:
                return self._deadline_result(node, started, 0, False)
            retries = 0
            retry_limit = node.params.get("retry", 0)
            degraded = False

            while True:
                node.reset_cancel()
                self._set_state("RUNNING", f"executing {node.name}")
                try:
                    before = copy.deepcopy(self.ctx.data)
                except Exception:
                    before = dict(self.ctx.data)
                try:
                    result = self._execute_node(node)
                except TaskDeadlineExceeded:
                    self._log_context_changes(before, node)
                    return self._deadline_result(node, started, retries, degraded)
                except Exception as exc:
                    log(f"[error] {node.name}: {type(exc).__name__}: {exc}", "FAILED")
                    result = "FAILED"
                self._log_context_changes(before, node)

                if result == "SUCCESS":
                    self._record_step(node, result, started, retries, degraded)
                    self._set_state("SUCCESS", f"{node.name} completed")
                    self.current_index += 1
                    break
                if result == "SKIPPED":
                    self._record_step(node, result, started, retries, degraded)
                    self._set_state("SKIPPED", f"{node.name} skipped")
                    self.current_index += 1
                    break
                if result != "FAILED":
                    self._record_step(node, "FAILED", started, retries, degraded)
                    self._set_state("FAILED", f"invalid result from {node.name}: {result!r}")
                    return "FAILED"
                if retries < retry_limit:
                    retries += 1
                    self._set_state("RETRYING", f"{node.name}, retry {retries}/{retry_limit}")
                    log(f"[retry] {node.name}: {retries}/{retry_limit}", "RETRYING")
                    try:
                        self._sleep_with_deadline(node.params.get("retry_interval", 0))
                    except TaskDeadlineExceeded:
                        return self._deadline_result(node, started, retries, degraded)
                    continue

                fallback = node.params.get("fallback")
                if fallback is not None:
                    if fallback not in self.node_map or fallback_count >= max_fallbacks:
                        self._record_step(node, "FAILED", started, retries, True)
                        self._set_state("FAILED", f"invalid fallback or limit: {fallback}")
                        return "FAILED"
                    fallback_count += 1
                    self._record_step(node, "FAILED", started, retries, True)
                    self._set_state("FAILED", f"{node.name} failed; fallback selected")
                    log(f"[degraded] fallback: {node.name} -> {fallback}", "DEGRADED")
                    self.current_index = self.node_map[fallback]
                    break
                if node.params.get("on_fail", "abort") == "skip":
                    self._record_step(node, "SKIPPED", started, retries, True)
                    self._set_state("SKIPPED", f"{node.name} skipped after failure")
                    self.current_index += 1
                    break
                self._record_step(node, "FAILED", started, retries, degraded)
                self._set_state("FAILED", f"{node.name} failed")
                return "FAILED"

        self._set_state("SUCCESS", "all tasks processed")
        return "SUCCESS"
