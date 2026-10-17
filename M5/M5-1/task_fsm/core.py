
import time
from concurrent.futures import ThreadPoolExecutor, TimeoutError


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

    def _record_step(self, node, status, started, retries=0, degraded=False):
        self.step_records.append({
            "name": node.name,
            "status": status,
            "elapsed_s": round(time.monotonic() - started, 3),
            "retries": retries,
            "degraded": degraded,
        })

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
        deadline = self.ctx.deadline
        timeout = node.params.get("timeout")

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

    def _handle_deadline(self, node, started, retries, degraded):
        self._record_step(node, "TIMEOUT", started, retries, degraded)
        self.state = "FAILED"
        return "TIMEOUT"

    def run(self):
        self.state = "RUNNING"
        self.current_index = 0
        self.step_records = []
        fallback_count = 0
        max_fallbacks = len(self.nodes)

        while self.current_index < len(self.nodes):
            node = self.nodes[self.current_index]
            started = time.monotonic()

            if self.ctx.deadline is not None and started >= self.ctx.deadline:
                return self._handle_deadline(node, started, 0, False)

            retries = 0
            retry_limit = node.params.get("retry", 0)
            degraded = False

            while True:
                node.reset_cancel()
                self.state = "RUNNING"

                try:
                    result = self._execute_node(node)
                except TaskDeadlineExceeded:
                    return self._handle_deadline(
                        node, started, retries, degraded
                    )

                if result == "SUCCESS":
                    self._record_step(node, "SUCCESS", started, retries, degraded)
                    self.state = "SUCCESS"
                    self.current_index += 1
                    break

                if result == "SKIPPED":
                    self._record_step(node, "SKIPPED", started, retries, degraded)
                    self.state = "SKIPPED"
                    self.current_index += 1
                    break

                if result != "FAILED":
                    self._record_step(node, "FAILED", started, retries, degraded)
                    self.state = "FAILED"
                    return "FAILED"

                if retries < retry_limit:
                    retries += 1
                    self.state = "RETRYING"

                    try:
                        self._sleep_with_deadline(
                            node.params.get("retry_interval", 0)
                        )
                    except TaskDeadlineExceeded:
                        return self._handle_deadline(
                            node, started, retries, degraded
                        )
                    continue

                fallback = node.params.get("fallback")
                if fallback is not None:
                    if fallback not in self.node_map:
                        self._record_step(node, "FAILED", started, retries, True)
                        self.state = "FAILED"
                        return "FAILED"

                    fallback_count += 1
                    if fallback_count > max_fallbacks:
                        self._record_step(node, "FAILED", started, retries, True)
                        self.state = "FAILED"
                        return "FAILED"

                    self._record_step(node, "FAILED", started, retries, True)
                    self.state = "FAILED"
                    self.current_index = self.node_map[fallback]
                    break

                if node.params.get("on_fail", "abort") == "skip":
                    self._record_step(node, "SKIPPED", started, retries, True)
                    self.state = "SKIPPED"
                    self.current_index += 1
                    break

                self._record_step(node, "FAILED", started, retries, degraded)
                self.state = "FAILED"
                return "FAILED"

        self.state = "SUCCESS"
        return "SUCCESS"
