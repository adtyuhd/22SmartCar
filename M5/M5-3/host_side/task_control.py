"""Independent task-control boundary. No M5-1 imports or scheduler mutation."""
import threading
from typing import Protocol


class SkipUnavailable(RuntimeError):
    pass


class SkipRejected(RuntimeError):
    pass


class SkipBackend(Protocol):
    def request_skip(self, *, task_id: str, node_id: str, reason: str) -> str:
        """Atomically verify running task/node and enqueue cancellation safely.

        Return a nonempty request ID only after acceptance. Raise SkipRejected
        for stale identity, unsafe cancellation or an uninterruptible node.
        Acceptance does not prove the node has stopped or been skipped.
        """
        ...


class TaskControl:
    def __init__(self, audit, backend: SkipBackend | None = None):
        self.audit = audit
        self.backend = backend
        self._lock = threading.Lock()

    def capabilities(self):
        return {
            "skip_available": self.backend is not None,
            "execution_confirmed": False,
            "message": (
                "仅可提交跳过请求，执行结果须由调度器确认"
                if self.backend is not None else
                "未接入可安全取消当前节点的调度器接口，不能跳过实际运行节点"
            ),
        }

    def request_skip(self, *, task_id, node_id, reason,
                     manual_confirmation, match_state, latched):
        details = {"task_id": task_id, "node_id": node_id, "reason": reason}
        with self._lock:
            if self.backend is None:
                self.audit.append(op="TASK_SKIP", source="ui", result="unavailable",
                                  details=details)
                raise SkipUnavailable(self.capabilities()["message"])
            if not manual_confirmation or match_state != "RUNNING" or latched:
                self.audit.append(op="TASK_SKIP", source="ui", result="rejected",
                                  details=details)
                raise SkipRejected("跳过需要人工确认、RUNNING 状态且无急停锁存")
            self.audit.append(op="TASK_SKIP", source="ui", result="requested",
                              details=details)
            request_id = self.backend.request_skip(**details)
            if not isinstance(request_id, str) or not request_id.strip():
                raise SkipRejected("调度器未返回有效请求标识；执行状态未知")
            self.audit.append(op="TASK_SKIP", source="ui", result="accepted",
                              details={**details, "request_id": request_id})
            return {"request_id": request_id, "request_sent": True,
                    "execution_confirmed": False,
                    "message": "跳过请求已接受，尚未确认实际节点已跳过"}
