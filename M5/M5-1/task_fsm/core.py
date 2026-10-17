
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

        self.node_map = {
            node.name: index
            for index, node in enumerate(nodes)
        }

    def _record_step(self, node, status, elapsed, retries, degraded):
        self.step_records.append({
            "name": node.name,
            "status": status,
            "elapsed_s": round(elapsed, 3),
            "retries": retries,
            "degraded": degraded
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
        node_timeout = node.params.get("timeout")
        deadline = self.ctx.deadline

        if deadline is not None:
            remaining = deadline - time.monotonic()

            if remaining <= 0:
                node.cancel()
                raise TaskDeadlineExceeded

            if node_timeout is None:
                timeout = remaining
            else:
                timeout = min(node_timeout, remaining)
        else:
            timeout = node_timeout

        if timeout is None:
            return node.execute()

        with ThreadPoolExecutor(max_workers=1) as executor:
            future = executor.submit(node.execute)

            try:
                return future.result(timeout=timeout)

            except TimeoutError:
                node.cancel()

                try:
                    future.result()
                except Exception:
                    pass

                if (
                    deadline is not None
                    and time.monotonic() >= deadline
                ):
                    raise TaskDeadlineExceeded

                return "FAILED"

    def run(self):
        self.state = "RUNNING"
        self.current_index = 0
        self.step_records = []

        fallback_count = 0
        max_fallbacks = len(self.nodes)

        while self.current_index < len(self.nodes):
            node = self.nodes[self.current_index]
            start_time = time.monotonic()

            if (
                self.ctx.deadline is not None
                and start_time >= self.ctx.deadline
            ):
                self._record_step(
                    node, "TIMEOUT", 0, 0, False
                )
                self.state = "FAILED"
                return "TIMEOUT"

            retry_count = 0
            retry_limit = node.params.get("retry", 0)
            degraded = False

            while True:
                node.reset_cancel()
                self.state = "RUNNING"

                try:
                    result = self._execute_node(node)

                except TaskDeadlineExceeded:
                    elapsed = time.monotonic() - start_time

                    self._record_step(
                        node,
                        "TIMEOUT",
                        elapsed,
                        retry_count,
                        degraded
                    )

                    self.state = "FAILED"
                    return "TIMEOUT"

                if result == "SUCCESS":
                    elapsed = time.monotonic() - start_time

                    self._record_step(
                        node,
                        "SUCCESS",
                        elapsed,
                        retry_count,
                        degraded
                    )

                    self.current_index += 1
                    break

                elif result == "SKIPPED":
                    elapsed = time.monotonic() - start_time

                    self._record_step(
                        node,
                        "SKIPPED",
                        elapsed,
                        retry_count,
                        degraded
                    )

                    self.current_index += 1
                    break

                elif result == "FAILED":
                    if retry_count < retry_limit:
                        retry_count += 1
                        self.state = "RETRYING"

                        try:
                            self._sleep_with_deadline(
                                node.params.get("retry_interval", 0)
                            )
                        except TaskDeadlineExceeded:
                            elapsed = time.monotonic() - start_time

                            self._record_step(
                                node,
                                "TIMEOUT",
                                elapsed,
                                retry_count,
                                degraded
                            )

                            self.state = "FAILED"
                            return "TIMEOUT"

                        continue

                    # 重试耗尽后，优先处理 fallback
                    fallback = node.params.get("fallback")

                    if fallback is not None:
                        if fallback not in self.node_map:
                            elapsed = time.monotonic() - start_time

                            self._record_step(
                                node,
                                "FAILED",
                                elapsed,
                                retry_count,
                                True
                            )

                            self.state = "FAILED"
                            return "FAILED"

                        fallback_count += 1

                        # 限制整个任务的 fallback 跳转次数
                        if fallback_count > max_fallbacks:
                            elapsed = time.monotonic() - start_time

                            self._record_step(
                                node,
                                "FAILED",
                                elapsed,
                                retry_count,
                                True
                            )

                            self.state = "FAILED"
                            return "FAILED"

                        elapsed = time.monotonic() - start_time

                        self._record_step(
                            node,
                            "FAILED",
                            elapsed,
                            retry_count,
                            True
                        )

                        self.current_index = self.node_map[fallback]
                        break

                    on_fail = node.params.get("on_fail", "abort")

                    if on_fail == "skip":
                        elapsed = time.monotonic() - start_time

                        self._record_step(
                            node,
                            "SKIPPED",
                            elapsed,
                            retry_count,
                            True
                        )

                        self.current_index += 1
                        break

                    elapsed = time.monotonic() - start_time

                    self._record_step(
                        node,
                        "FAILED",
                        elapsed,
                        retry_count,
                        degraded
                    )

                    self.state = "FAILED"
                    return "FAILED"

                else:
                    elapsed = time.monotonic() - start_time

                    self._record_step(
                        node,
                        "FAILED",
                        elapsed,
                        retry_count,
                        degraded
                    )

                    self.state = "FAILED"
                    return "FAILED"

        self.state = "SUCCESS"
        return "SUCCESS"
