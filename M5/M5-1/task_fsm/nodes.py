import threading
import time
class TaskNode:
    def __init__(self, name, params, ctx):
        self.name = name
        self.params = params
        self.ctx = ctx
        self._cancelled = False
        self.clients = {}

    def setup(self):
        pass

    def execute(self) -> str:
        raise NotImplementedError

    def cancel(self):
        self._cancelled = True

    def teardown(self):
        pass


class SpeakTTS(TaskNode):
    def execute(self) -> str:
        if self._cancelled:
            return "SKIPPED"

        text = self.params.get("text")
        client = self.params.get("client")

        if not text or client is None:
            return "FAILED"

        try:
            client(text)
            return "SUCCESS"
        except TimeoutError:
            return "FAILED"
        except Exception:
            return "FAILED"


class WaitForTrigger(TaskNode):
    def __init__(self, name, params, ctx):
        super().__init__(name, params, ctx)
        self._trigger_event = threading.Event()
        self._triggered = False

    def trigger(self):
        self._triggered = True
        self._trigger_event.set()

    def execute(self) -> str:
        if self._cancelled:
            return "SKIPPED"

        timeout = self.params.get("timeout", 30.0)
        deadline = time.monotonic() + timeout

        self._trigger_event.wait(timeout)

        if self._cancelled:
            return "SKIPPED"

        if self._triggered:
            return "SUCCESS"

        if time.monotonic() >= deadline:
            return "FAILED"

        return "FAILED"

    def cancel(self):
        self._cancelled = True
        self._trigger_event.set()

class NavigateTo(TaskNode):
    def execute(self) -> str:
        if self._cancelled:
            return "SKIPPED"

        pose = self.params.get("pose")

        if pose is None:
            key = self.params.get("pose_from_context")
            if key is not None:
                pose = self.ctx.data.get(key)

        client = self.params.get("client")

        if pose is None or client is None:
            return "FAILED"

        try:
            result = client(pose)

            if self._cancelled:
                return "SKIPPED"

            self.ctx.data["last_nav_result"] = result
            return "SUCCESS"
        except TimeoutError:
            return "FAILED"
        except Exception:
            return "FAILED"

class DetectTarget(TaskNode):
    def __init__(self, name, params, ctx):
        super().__init__(name, params, ctx)
        self._event = threading.Event()
        self._target = None

    def on_detection(self, detection):
        if self._cancelled or self._event.is_set():
            return

        target_class = self.params.get("target_class")
        min_confidence = self.params.get("min_confidence", 0)

        if (detection.get("class") == target_class
                and detection.get("confidence", 0) >= min_confidence
                and "id" in detection
                and "pose" in detection):
            self._target = detection
            self._event.set()

    def execute(self) -> str:
        if self._cancelled:
            return "SKIPPED"

        timeout = self.params.get("timeout", 15)
        self._event.wait(timeout)

        if self._cancelled:
            return "SKIPPED"

        if self._target is None:
            return "FAILED"

        self.ctx.data["target_pose"] = self._target["pose"]
        self.ctx.data["target_id"] = self._target["id"]

        return "SUCCESS"

    def cancel(self):
        super().cancel()
        self._event.set()

class ExecuteGrasp(TaskNode):
    def execute(self) -> str:
        if self._cancelled:
            return "SKIPPED"

        pose = self.params.get("grasp_pose")
        if pose is None:
            pose = self.ctx.data.get("grasp_pose")

        grasp_type = self.params.get("grasp_type")
        client = self.ctx.clients.get("grasp")

        if pose is None or not grasp_type or client is None:
            return "FAILED"

        result = client(pose, grasp_type)

        if self._cancelled:
            return "SKIPPED"

        if not isinstance(result, dict):
            return "FAILED"

        self.ctx.data["grasp_result"] = result

        if not result.get("success") or "object_pose" not in result:
            return "FAILED"

        self.ctx.data["object_pose"] = result["object_pose"]

        return "SUCCESS"