"""M5-3 小车端急停保护组件。

本文件不是独立 ROS 2 节点，不调用 rclpy.init()，
也不创建新的常驻进程。

将 EmergencyStopGuard 实例化到已有 ROS 2 节点中即可。

功能：
1. 只新增一个 /emergency_stop 订阅者。
2. 急停时锁存安全状态，严格请求停用底盘控制器。
3. 复位时要求人工确认，并保持底盘控制器停用。
4. 只有收到明确的人工恢复请求，才允许重新激活控制器。
5. 通过 /safety/status 返回执行结果。
"""

import json
import threading
from datetime import datetime, timezone

from controller_manager_msgs.srv import SwitchController
from std_msgs.msg import String


class EmergencyStopGuard:
    """附加到已有 ROS 2 节点的小车端急停保护组件。"""

    RESET_ALLOWED_STATES = {"STOPPED", "FAULT"}

    def __init__(
        self,
        node,
        controller_name="ackermann_steering_controller",
        switch_service="/controller_manager/switch_controller",
    ):
        if not controller_name:
            raise ValueError("controller_name 不能为空")

        self.node = node
        self.controller_name = controller_name
        self.switch_service = switch_service

        self._lock = threading.RLock()

        # 急停锁存状态。
        self._latched = False

        # True：已确认激活；False：已确认停用；None：状态未知。
        self._controller_active = None

        # 复位只解除锁存，不会自动激活控制器。
        self._awaiting_enable = False

        # 当前正在执行的控制器切换操作。
        self._transition_pending = None
        self._transition_token = 0

        # 同一次停用操作可能收到多个重复 STOP 请求。
        self._pending_stop_ids = []

        # 用于向主机报告执行结果。
        self.status_pub = node.create_publisher(
            String,
            "/safety/status",
            10,
        )

        # 调用已有控制器管理服务，不创建新的节点或进程。
        self.switch_client = node.create_client(
            SwitchController,
            switch_service,
        )

        # 本组件唯一新增的订阅者。
        self.command_sub = node.create_subscription(
            String,
            "/emergency_stop",
            self._on_command,
            10,
        )

        self.node.get_logger().info(
            "EmergencyStopGuard 已初始化："
            "订阅 /emergency_stop，"
            f"控制器={self.controller_name}，"
            f"服务={self.switch_service}"
        )

    @staticmethod
    def _timestamp():
        """生成带时区的 UTC 时间戳。"""
        return datetime.now(timezone.utc).isoformat(
            timespec="milliseconds"
        )

    def get_state(self):
        """返回线程安全的当前状态快照。"""
        with self._lock:
            return {
                "latched": self._latched,
                "controller_active": self._controller_active,
                "awaiting_enable": self._awaiting_enable,
                "transition_pending": self._transition_pending,
            }

    def _publish_status(
        self,
        state,
        request_id,
        controller_result,
        message,
    ):
        """向主机报告当前安全状态和操作结果。"""
        with self._lock:
            payload = {
                "state": state,
                "request_id": request_id,
                "controller_result": controller_result,
                "latched": self._latched,
                "controller_active": self._controller_active,
                "awaiting_enable": self._awaiting_enable,
                "message": message,
                "timestamp": self._timestamp(),
            }

        try:
            self.status_pub.publish(
                String(
                    data=json.dumps(
                        payload,
                        ensure_ascii=False,
                        sort_keys=True,
                    )
                )
            )
        except Exception as exc:
            self.node.get_logger().error(
                f"发布安全状态失败：{type(exc).__name__}: {exc}"
            )

    def _reject(self, action, request_id, message):
        """拒绝不满足安全条件的命令。"""
        if action == "RESET":
            state = "RESET_FAILED"
        elif action == "ENABLE":
            state = "ENABLE_FAILED"
        else:
            state = "COMMAND_REJECTED"

        self.node.get_logger().warning(message)

        self._publish_status(
            state=state,
            request_id=request_id,
            controller_result="failed",
            message=message,
        )

    def _on_command(self, msg):
        """处理 /emergency_stop 上的 JSON 命令。"""
        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self._reject(
                action="INVALID",
                request_id=None,
                message="急停命令不是有效的 JSON",
            )
            return

        if not isinstance(payload, dict):
            self._reject(
                action="INVALID",
                request_id=None,
                message="急停命令必须是 JSON 对象",
            )
            return

        action = str(payload.get("action", "")).upper()
        request_id = payload.get("request_id")

        if not isinstance(request_id, str) or not request_id.strip():
            self._reject(
                action=action,
                request_id=None,
                message="急停命令缺少有效 request_id",
            )
            return

        request_id = request_id.strip()

        if action == "STOP":
            self._handle_stop(request_id)

        elif action == "RESET":
            self._handle_reset(request_id, payload)

        elif action == "ENABLE":
            self._handle_enable(request_id, payload)

        else:
            self._reject(
                action="INVALID",
                request_id=request_id,
                message=f"不支持的急停动作：{action}",
            )

    def _begin_transition_locked(self, action):
        """开始一次新的控制器切换，并生成操作令牌。"""
        self._transition_token += 1
        self._transition_pending = action
        return self._transition_token

    def _handle_stop(self, request_id):
        """锁存急停，并请求停用底盘控制器。"""
        with self._lock:
            # 必须先锁存，再尝试调用服务。
            self._latched = True
            self._awaiting_enable = False

            if self._transition_pending == "STOP":
                # 重复 STOP 共享当前停用操作。
                self._pending_stop_ids.append(request_id)
                return

            if (
                self._transition_pending is None
                and self._controller_active is False
            ):
                # 已确认控制器停用，重复急停可以幂等应答。
                already_inactive = True
                token = None
            else:
                already_inactive = False

                # 让之前尚未完成的 ENABLE 等操作失效。
                token = self._begin_transition_locked("STOP")
                self._controller_active = None
                self._pending_stop_ids = [request_id]

        if already_inactive:
            self._publish_status(
                state="ESTOP_LATCHED",
                request_id=request_id,
                controller_result="ok",
                message="控制器已确认停用，急停锁存仍然有效",
            )
            return

        self.node.get_logger().warning(
            f"收到 STOP，开始停用控制器，request_id={request_id}"
        )

        self._start_switch(
            action="STOP",
            request_id=request_id,
            token=token,
        )

    def _handle_reset(self, request_id, payload):
        """人工复位只解除锁存，不自动重新激活控制器。"""
        manual_confirmation = payload.get("manual_confirmation")
        match_state = str(
            payload.get("match_state", "UNKNOWN")
        ).upper()

        with self._lock:
            if self._transition_pending is not None:
                reason = "控制器切换尚未完成，拒绝复位"

            elif not self._latched:
                reason = "当前没有急停锁存，拒绝重复复位"

            elif manual_confirmation is not True:
                reason = "复位必须经过明确的人工确认"

            elif match_state not in self.RESET_ALLOWED_STATES:
                reason = (
                    f"比赛状态 {match_state} 不允许复位；"
                    "只允许 STOPPED 或 FAULT"
                )

            elif self._controller_active is not False:
                reason = (
                    "尚未确认控制器处于停用状态，"
                    "不能安全解除急停锁存"
                )

            else:
                reason = None

                # 复位解除安全锁存，但不重新激活控制器。
                self._latched = False
                self._awaiting_enable = True
                self._controller_active = False

        if reason is not None:
            self._reject("RESET", request_id, reason)
            return

        self.node.get_logger().info(
            f"人工复位已确认，控制器仍保持停用，"
            f"request_id={request_id}"
        )

        self._publish_status(
            state="RESET_OK",
            request_id=request_id,
            controller_result="ok",
            message=(
                "急停锁存已解除；控制器仍保持停用，"
                "等待单独的人工恢复指令"
            ),
        )

    def _handle_enable(self, request_id, payload):
        """仅在人工明确恢复任务后，才允许重新激活控制器。"""
        manual_confirmation = payload.get("manual_confirmation")
        match_state = str(
            payload.get("match_state", "UNKNOWN")
        ).upper()

        with self._lock:
            if self._transition_pending is not None:
                reason = "另一项控制器切换尚未完成"

            elif self._latched:
                reason = "急停仍处于锁存状态，禁止激活控制器"

            elif not self._awaiting_enable:
                reason = "当前状态不允许执行控制器恢复"

            elif self._controller_active is not False:
                reason = "控制器当前状态不明确，拒绝激活"

            elif manual_confirmation is not True:
                reason = "控制器恢复必须经过明确的人工确认"

            elif match_state != "RUNNING":
                reason = (
                    "只有在任务已被明确恢复为 RUNNING 后，"
                    "才允许激活控制器"
                )

            else:
                reason = None

                token = self._begin_transition_locked("ENABLE")

                # 切换完成前，不能假定控制器仍处于停用状态。
                self._controller_active = None

        if reason is not None:
            self._reject("ENABLE", request_id, reason)
            return

        self.node.get_logger().info(
            f"收到人工恢复请求，开始激活控制器，"
            f"request_id={request_id}"
        )

        self._start_switch(
            action="ENABLE",
            request_id=request_id,
            token=token,
        )

    def _start_switch(self, action, request_id, token):
        """构造严格模式的控制器切换请求，并异步执行。"""
        try:
            if not self.switch_client.service_is_ready():
                self._finish_switch(
                    action=action,
                    request_id=request_id,
                    token=token,
                    success=False,
                    message=(
                        f"控制器管理服务不可用：{self.switch_service}"
                    ),
                )
                return

            request = SwitchController.Request()

            if action == "STOP":
                request.activate_controllers = []
                request.deactivate_controllers = [
                    self.controller_name
                ]

            elif action == "ENABLE":
                request.activate_controllers = [
                    self.controller_name
                ]
                request.deactivate_controllers = []

            else:
                raise ValueError(f"不支持的切换动作：{action}")

            request.strictness = SwitchController.Request.STRICT
            request.activate_asap = False

            # 设置有限的服务端切换等待时间。
            request.timeout.sec = 2
            request.timeout.nanosec = 0

            future = self.switch_client.call_async(request)

            future.add_done_callback(
                lambda completed,
                a=action,
                r=request_id,
                t=token: self._on_switch_done(
                    completed,
                    a,
                    r,
                    t,
                )
            )

        except Exception as exc:
            self._finish_switch(
                action=action,
                request_id=request_id,
                token=token,
                success=False,
                message=(
                    f"控制器切换请求异常："
                    f"{type(exc).__name__}: {exc}"
                ),
            )

    def _on_switch_done(
        self,
        future,
        action,
        request_id,
        token,
    ):
        """解析 ROS 2 服务结果。"""
        try:
            response = future.result()
            success = bool(response.ok)

            if success:
                message = (
                    "控制器已成功停用"
                    if action == "STOP"
                    else "控制器已成功激活"
                )
            else:
                message = (
                    "控制器管理服务返回 ok=False"
                )

        except Exception as exc:
            success = False
            message = (
                f"控制器服务调用失败："
                f"{type(exc).__name__}: {exc}"
            )

        self._finish_switch(
            action=action,
            request_id=request_id,
            token=token,
            success=success,
            message=message,
        )

    def _finish_switch(
        self,
        action,
        request_id,
        token,
        success,
        message,
    ):
        """完成控制器切换状态转换并发布反馈。"""
        with self._lock:
            # 如果 STOP 覆盖了之前的 ENABLE，旧结果不得覆盖新状态。
            if (
                token != self._transition_token
                or action != self._transition_pending
            ):
                self.node.get_logger().warning(
                    f"忽略过期的控制器切换结果："
                    f"action={action}, request_id={request_id}"
                )
                return

            self._transition_pending = None

            if action == "STOP":
                request_ids = list(self._pending_stop_ids)
                self._pending_stop_ids.clear()

                self._latched = True
                self._awaiting_enable = False

                if success:
                    self._controller_active = False
                    state = "ESTOP_LATCHED"
                    result = "ok"
                else:
                    # 切换失败后，不能假定控制器仍处于什么状态。
                    self._controller_active = None
                    state = "ESTOP_FAILED"
                    result = "failed"

            elif action == "ENABLE":
                request_ids = [request_id]

                if success:
                    self._controller_active = True
                    self._latched = False
                    self._awaiting_enable = False
                    state = "ENABLED"
                    result = "ok"
                else:
                    # 恢复失败时重新锁存，要求重新处理急停。
                    self._controller_active = None
                    self._latched = True
                    self._awaiting_enable = False
                    state = "ENABLE_FAILED"
                    result = "failed"

            else:
                return

        for current_id in request_ids:
            self._publish_status(
                state=state,
                request_id=current_id,
                controller_result=result,
                message=message,
            )

        if success:
            self.node.get_logger().info(
                f"{state}: {message}"
            )
        else:
            self.node.get_logger().error(
                f"{state}: {message}；急停保护状态不会被自动解除"
            )