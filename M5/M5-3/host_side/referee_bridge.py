
#!/usr/bin/env python3
"""M5-3 裁判协议适配器：状态同步、控制事件、急停联动和审计。"""

import argparse
import json
import math
import sys
import threading
import time
from datetime import datetime, timezone

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from audit import AuditLogger


KEY_COMMANDS = {
    "S": "START",
    "P": "PAUSE",
    "R": "RESUME",
    "E": "STOP",
}

CONTROL_TOPICS = {
    "START": "/task_start",
    "PAUSE": "/task_pause",
    "RESUME": "/task_resume",
    "STOP": "/task_stop",
}

SUPPORTED_COMMANDS = {
    "START",
    "PAUSE",
    "RESUME",
    "STOP",
    "PENALTY",
    "SCORE_QUERY",
}

RESET_ALLOWED_STATES = {"STOPPED", "FAULT"}


def utc_timestamp():
    """生成 UTC 时间戳。"""
    return datetime.now(timezone.utc).isoformat(
        timespec="milliseconds"
    )


def parse_payload(raw):
    """解析 JSON 对象或纯文本指令。"""
    if isinstance(raw, dict):
        payload = raw
    elif isinstance(raw, str):
        raw = raw.strip()
        if not raw:
            return {}

        try:
            payload = json.loads(raw)
        except json.JSONDecodeError:
            payload = {"command": raw}

        if isinstance(payload, str):
            payload = {"command": payload}

        if not isinstance(payload, dict):
            return {}
    else:
        return {}

    try:
        json.dumps(payload, ensure_ascii=False, allow_nan=False)
    except (TypeError, ValueError):
        return {}

    return payload


class RefereeBridge(Node):
    """裁判适配器及主机侧急停状态协调器。

    裁判输入：
        /referee/cmd

    急停联动输入：
        /safety/fault
        /emergency_stop
        /safety/status

    状态输出：
        /referee/status
        /task_state

    任务控制事件：
        /task_start
        /task_pause
        /task_resume
        /task_stop

    注意：
        发布任务控制事件不代表任务已经实际执行。
        本组件运行在主机侧，不创建额外的机器人进程。
    """

    def __init__(
        self,
        match_duration_s=180.0,
        audit_dir=None,
        audit_user=None,
    ):
        super().__init__("referee_bridge")

        if (
            not math.isfinite(match_duration_s)
            or match_duration_s <= 0
        ):
            raise ValueError("match_duration_s 必须是正数")

        self.match_duration_s = float(match_duration_s)
        self._lock = threading.RLock()

        self.state = "READY"
        self.phase = "READY"

        self.last_command = None
        self.last_source = "system"
        self.last_error = None

        self._sequence = 0
        self._deadline = None
        self._remaining_s = None
        self._paused_remaining_s = None

        self.penalties = []
        self.penalty_time_s = 0.0
        self.score = None

        # 急停锁存：普通 STOP 不能解除这个标志。
        self._emergency_latched = False
        self._emergency_request_id = None
        self._emergency_source = None
        self._emergency_reason = None

        # 用于关联控制器结果与复位请求。
        self._pending_stop_request_id = None
        self._pending_reset_id = None

        # 处理跨话题回调的到达顺序差异。
        # RESET_OK 可能先于对应的 /emergency_stop 回调到达。
        self._early_reset_results = {}

        # 避免重复记录同一条控制器结果。
        self._controller_status_seen = set()

        self.audit = AuditLogger(
            log_dir=audit_dir,
            user=audit_user,
        )

        # 原有裁判指令订阅。
        self.cmd_sub = self.create_subscription(
            String,
            "/referee/cmd",
            self._on_referee_command,
            10,
        )

        # 主机侧：接收安全模块的故障事件。
        self.safety_fault_sub = self.create_subscription(
            String,
            "/safety/fault",
            self._on_safety_fault,
            10,
        )

        # 主机侧：观察急停请求并关联 request_id。
        # 这不是小车端新增的订阅者。
        self.emergency_command_sub = self.create_subscription(
            String,
            "/emergency_stop",
            self._on_emergency_command,
            10,
        )

        # 主机侧：读取小车端的控制器切换及复位结果。
        self.safety_status_sub = self.create_subscription(
            String,
            "/safety/status",
            self._on_safety_status,
            10,
        )

        self.status_pub = self.create_publisher(
            String,
            "/referee/status",
            10,
        )

        self.task_state_pub = self.create_publisher(
            String,
            "/task_state",
            10,
        )

        self.control_publishers = {
            topic: self.create_publisher(String, topic, 10)
            for topic in CONTROL_TOPICS.values()
        }

        # 每秒刷新一次状态和倒计时。
        self.timer = self.create_timer(
            1.0,
            self._on_timer,
        )

        self.get_logger().info(
            "裁判适配器已启动："
            "订阅 /referee/cmd、/safety/fault、"
            "/emergency_stop、/safety/status；"
            "发布 /referee/status 和 /task_state"
        )

        self._publish_status()

    # ---------------------------------------------------------
    # 状态快照与发布
    # ---------------------------------------------------------

    def _remaining_locked(self):
        """计算剩余时间；调用时需要持有状态锁。"""
        if self.state == "RUNNING" and self._deadline is not None:
            return max(
                0,
                math.ceil(self._deadline - time.monotonic()),
            )

        if (
            self.state == "PAUSED"
            and self._paused_remaining_s is not None
        ):
            return max(
                0,
                math.ceil(self._paused_remaining_s),
            )

        return self._remaining_s

    def _snapshot_locked(self):
        """创建可以安全序列化的状态快照。"""
        return {
            "timestamp": utc_timestamp(),
            "sequence": self._sequence,
            "status": self.state,
            "phase": self.phase,
            "command": self.last_command,
            "source": self.last_source,
            "countdown_remaining_s": self._remaining_locked(),
            "penalty_time_s": round(self.penalty_time_s, 3),
            "penalties": list(self.penalties),
            "score": self.score,
            "error": self.last_error,
            "emergency_latched": self._emergency_latched,
            "emergency_request_id": self._emergency_request_id,
        }

    def _publish_snapshot(self, snapshot):
        """向裁判状态接口和任务状态接口发布同一份快照。"""
        payload = json.dumps(
            snapshot,
            ensure_ascii=False,
            allow_nan=False,
        )

        message = String(data=payload)
        self.status_pub.publish(message)
        self.task_state_pub.publish(message)

    def _publish_status(self):
        with self._lock:
            snapshot = self._snapshot_locked()

        self._publish_snapshot(snapshot)

    def _write_audit(
        self,
        op,
        source,
        result,
        from_state,
        to_state,
        details=None,
    ):
        """写审计日志；异常不会直接杀死 ROS 2 回调。"""
        try:
            self.audit.append(
                op=op,
                source=source,
                result=result,
                from_value=from_state,
                to_value=to_state,
                details=details,
            )
            return True
        except Exception as exc:
            self.get_logger().error(
                f"审计日志写入失败：{exc}"
            )
            return False

    def _publish_control(self, command, source, reason=None):
        """发布任务控制事件。"""
        topic = CONTROL_TOPICS.get(command)

        if topic is None:
            return

        payload = {
            "action": command,
            "source": source,
            "timestamp": utc_timestamp(),
        }

        if reason is not None:
            payload["reason"] = str(reason)

        try:
            self.control_publishers[topic].publish(
                String(
                    data=json.dumps(
                        payload,
                        ensure_ascii=False,
                        allow_nan=False,
                    )
                )
            )
            self.get_logger().info(
                f"已发布任务控制事件：{command} -> {topic}"
            )
        except Exception as exc:
            self.get_logger().error(
                f"发布任务控制事件失败：{command}，{exc}"
            )

    # ---------------------------------------------------------
    # 急停联动
    # ---------------------------------------------------------

    def _on_safety_fault(self, msg):
        """处理 /safety/fault 上的故障事件。"""
        payload = parse_payload(msg.data)

        if not payload:
            # 安全故障通道出现无法解析的消息时，采取保守策略。
            payload = {
                "type": "SAFETY_FAULT",
                "source": "safety_fault_topic",
                "reason": "收到无法解析的安全故障消息",
            }

        self._enter_emergency_fault(
            payload,
            default_source="safety_fault_topic",
        )

    def _on_emergency_command(self, msg):
        """观察主机发出的 STOP / RESET 请求。"""
        payload = parse_payload(msg.data)

        if not payload:
            self.get_logger().warning(
                "/emergency_stop 收到无法解析的消息"
            )
            return

        action = str(payload.get("action", "")).upper()
        request_id = payload.get("request_id")

        if not isinstance(request_id, str) or not request_id.strip():
            self.get_logger().warning(
                f"/emergency_stop 的 {action} 请求缺少 request_id"
            )
            return

        request_id = request_id.strip()

        if action == "STOP":
            with self._lock:
                self._pending_stop_request_id = request_id

            # 即使 /safety/fault 因异常没有送达，
            # STOP 请求本身也足以触发裁判侧故障锁存。
            self._enter_emergency_fault(
                {
                    "type": "EMERGENCY_STOP",
                    "state": "FAULT",
                    "request_id": request_id,
                    "source": payload.get("source", "emergency_stop"),
                    "reason": payload.get(
                        "reason",
                        "收到 /emergency_stop STOP 请求",
                    ),
                },
                default_source="emergency_stop",
            )
            return

        if action != "RESET":
            return

        early_result = None
        rejection = None

        with self._lock:
            if not self._emergency_latched:
                rejection = "当前没有急停锁存，忽略复位请求"

            elif payload.get("manual_confirmation") is not True:
                rejection = "复位请求缺少明确的人工确认"

            elif str(payload.get("match_state", "")).upper() not in (
                RESET_ALLOWED_STATES
            ):
                rejection = (
                    "比赛状态不允许复位；"
                    "只允许 STOPPED 或 FAULT"
                )

            else:
                self._pending_reset_id = request_id
                self.last_command = "EMERGENCY_RESET_REQUEST"
                self.last_source = str(
                    payload.get("source", "host")
                )
                self.last_error = self._emergency_reason
                self._sequence += 1

                # 可能已经收到小车端的快速响应。
                early_result = self._early_reset_results.pop(
                    request_id,
                    None,
                )

                snapshot = self._snapshot_locked()

        if rejection is not None:
            self.get_logger().warning(rejection)
            return

        self._publish_snapshot(snapshot)

        if early_result is not None:
            self._handle_reset_result(early_result)

    def _enter_emergency_fault(self, payload, default_source):
        """原子地进入急停 FAULT 状态并暂停任务。"""
        request_id = payload.get("request_id")

        if not isinstance(request_id, str) or not request_id.strip():
            request_id = None
        else:
            request_id = request_id.strip()

        source = str(payload.get("source", default_source))
        reason = str(
            payload.get("reason", "收到急停故障事件")
        )
        fault_type = str(
            payload.get("type", "SAFETY_FAULT")
        )

        with self._lock:
            # 同一请求可能同时通过 /emergency_stop 和
            # /safety/fault 到达；只处理一次。
            if (
                self._emergency_latched
                and request_id == self._emergency_request_id
            ):
                return

            old_state = self.state
            remaining = self._remaining_locked()

            if remaining is not None:
                self._remaining_s = remaining

            # 冻结倒计时，进入不可被普通 STOP 清除的 FAULT。
            self._deadline = None
            self._paused_remaining_s = None

            self._emergency_latched = True
            self._emergency_request_id = request_id
            self._emergency_source = source
            self._emergency_reason = reason

            # 新的急停使旧的待复位结果失效。
            self._pending_reset_id = None
            self._early_reset_results.clear()

            self.state = "FAULT"
            self.phase = "FAULT"
            self.last_command = "EMERGENCY_STOP"
            self.last_source = source
            self.last_error = reason
            self._sequence += 1

        # 先发暂停事件，不让审计写入延迟安全方向的控制。
        self._publish_control(
            "PAUSE",
            source="emergency_stop",
            reason=reason,
        )

        audit_ok = self._write_audit(
            op="EMERGENCY_STOP",
            source=source,
            result="fault",
            from_state=old_state,
            to_state="FAULT",
            details={
                "type": fault_type,
                "request_id": request_id,
                "reason": reason,
            },
        )

        with self._lock:
            if not audit_ok:
                self.last_error = (
                    f"{reason}；急停审计日志写入失败"
                )
                self._sequence += 1

            snapshot = self._snapshot_locked()

        self.get_logger().error(
            f"急停故障已锁存：{old_state} -> FAULT，"
            f"request_id={request_id}"
        )
        self._publish_snapshot(snapshot)

    def _on_safety_status(self, msg):
        """处理小车端返回的控制器和复位状态。"""
        payload = parse_payload(msg.data)

        if not payload:
            self.get_logger().warning(
                "/safety/status 收到无法解析的消息"
            )
            return

        state = str(payload.get("state", "")).upper()
        request_id = payload.get("request_id")

        if state in {"RESET_OK", "RESET_FAILED"}:
            if not isinstance(request_id, str) or not request_id:
                return

            with self._lock:
                if request_id != self._pending_reset_id:
                    # 跨话题回调可能乱序；暂存有限数量的结果。
                    self._early_reset_results[request_id] = payload

                    while len(self._early_reset_results) > 16:
                        oldest = next(iter(self._early_reset_results))
                        self._early_reset_results.pop(oldest)

                    return

            self._handle_reset_result(payload)
            return

        if state in {"ESTOP_LATCHED", "ESTOP_FAILED"}:
            self._record_controller_status(payload)

    def _handle_reset_result(self, payload):
        """只处理与当前复位请求匹配的小车端结果。"""
        state = str(payload.get("state", "")).upper()
        request_id = payload.get("request_id")
        controller_result = payload.get("controller_result")

        if state == "RESET_OK" and controller_result == "ok":
            self._complete_emergency_reset(payload)
        else:
            self._fail_emergency_reset(payload)

    def _complete_emergency_reset(self, payload):
        """审计成功后解除裁判故障锁存，但不恢复比赛。"""
        request_id = payload.get("request_id")

        with self._lock:
            if (
                not self._emergency_latched
                or request_id != self._pending_reset_id
            ):
                return

            old_state = self.state

            # 必须先记录复位结果；审计失败则继续保持 FAULT。
            audit_ok = self._write_audit(
                op="EMERGENCY_RESET",
                source="robot_side",
                result="ok",
                from_state=old_state,
                to_state="STOPPED",
                details={
                    "request_id": request_id,
                    "controller_result": payload.get(
                        "controller_result"
                    ),
                    "message": payload.get("message"),
                },
            )

            if not audit_ok:
                self.last_error = (
                    "小车端已返回 RESET_OK，"
                    "但裁判端审计失败；继续保持 FAULT"
                )
                self.last_source = "robot_side"
                self.last_command = "EMERGENCY_RESET"
                self._sequence += 1
                snapshot = self._snapshot_locked()

            else:
                # 解除裁判侧故障锁存，回到停止状态。
                # 绝不在这里发送 START、RESUME 或 ENABLE。
                self._emergency_latched = False
                self._emergency_request_id = None
                self._emergency_source = None
                self._emergency_reason = None

                self._pending_reset_id = None
                self._pending_stop_request_id = None

                self._deadline = None
                self._paused_remaining_s = None

                self.state = "STOPPED"
                self.phase = "STOPPED"
                self.last_command = "EMERGENCY_RESET"
                self.last_source = "robot_side"
                self.last_error = None
                self._sequence += 1

                self._early_reset_results.pop(request_id, None)
                snapshot = self._snapshot_locked()

        self._publish_snapshot(snapshot)

        if not audit_ok:
            self.get_logger().error(
                "复位审计失败：急停锁存仍然有效"
            )
            return

        self.get_logger().warning(
            "急停复位已确认；裁判状态为 STOPPED，"
            "比赛和底盘不会自动恢复"
        )

    def _fail_emergency_reset(self, payload):
        """复位失败时保持 FAULT，并记录失败结果。"""
        request_id = payload.get("request_id")
        reason = str(
            payload.get("message", "小车端拒绝或未能完成复位")
        )

        with self._lock:
            if (
                not self._emergency_latched
                or request_id != self._pending_reset_id
            ):
                return

            old_state = self.state

            audit_ok = self._write_audit(
                op="EMERGENCY_RESET",
                source="robot_side",
                result="failed",
                from_state=old_state,
                to_state="FAULT",
                details={
                    "request_id": request_id,
                    "robot_state": payload.get("state"),
                    "controller_result": payload.get(
                        "controller_result"
                    ),
                    "message": reason,
                },
            )

            self._pending_reset_id = None
            self.state = "FAULT"
            self.phase = "FAULT"
            self.last_command = "EMERGENCY_RESET"
            self.last_source = "robot_side"

            if audit_ok:
                self.last_error = reason
            else:
                self.last_error = (
                    f"{reason}；复位失败审计日志写入失败"
                )

            self._sequence += 1
            snapshot = self._snapshot_locked()

        self.get_logger().error(
            f"急停复位失败，继续保持 FAULT：{reason}"
        )
        self._publish_snapshot(snapshot)

    def _record_controller_status(self, payload):
        """记录小车端控制器停用结果。"""
        request_id = payload.get("request_id")
        robot_state = str(payload.get("state", "")).upper()
        controller_result = payload.get("controller_result")

        if not isinstance(request_id, str) or not request_id:
            return

        with self._lock:
            # 忽略不属于当前急停请求的旧反馈。
            if request_id not in (
                self._pending_stop_request_id,
                self._emergency_request_id,
            ):
                return

            key = (request_id, robot_state, controller_result)

            if key in self._controller_status_seen:
                return

            self._controller_status_seen.add(key)
            old_state = self.state

            audit_result = (
                "ok"
                if (
                    robot_state == "ESTOP_LATCHED"
                    and controller_result == "ok"
                )
                else "failed"
            )

            audit_ok = self._write_audit(
                op="EMERGENCY_STOP_CONTROLLER",
                source="robot_side",
                result=audit_result,
                from_state=old_state,
                to_state="FAULT",
                details={
                    "request_id": request_id,
                    "robot_state": robot_state,
                    "controller_result": controller_result,
                    "message": payload.get("message"),
                },
            )

            self.state = "FAULT"
            self.phase = "FAULT"
            self.last_source = "robot_side"
            self.last_command = "EMERGENCY_STOP_CONTROLLER"

            if audit_result == "failed":
                self.last_error = str(
                    payload.get(
                        "message",
                        "小车端控制器停用未成功",
                    )
                )
            elif not audit_ok:
                self.last_error = "控制器结果审计日志写入失败"
            else:
                self.last_error = self._emergency_reason

            self._sequence += 1
            snapshot = self._snapshot_locked()

        self._publish_snapshot(snapshot)

    # ---------------------------------------------------------
    # 裁判协议
    # ---------------------------------------------------------

    def _on_referee_command(self, msg):
        self.process_command(
            msg.data,
            source="referee_topic",
        )

    def process_command(self, raw, source="referee_topic"):
        """统一处理裁判指令并记录审计日志。"""
        payload = parse_payload(raw)

        command = str(
            payload.get(
                "command",
                payload.get("type", payload.get("action", "")),
            )
        ).strip().upper()

        control_command = None
        error = None

        with self._lock:
            old_state = self.state

            self.last_source = source
            self.last_command = command or None
            self.last_error = None

            if command == "START":
                if self._emergency_latched:
                    error = "急停锁存期间禁止 START"

                elif self.state not in ("READY", "STOPPED"):
                    error = f"当前状态 {self.state} 不允许 START"

                else:
                    duration = payload.get(
                        "countdown_s",
                        payload.get(
                            "duration_s",
                            self.match_duration_s,
                        ),
                    )

                    try:
                        duration = float(duration)
                    except (TypeError, ValueError):
                        duration = 0.0

                    if not math.isfinite(duration) or duration <= 0:
                        error = "START 的倒计时必须是正数"
                    else:
                        self.state = "RUNNING"
                        self.phase = str(
                            payload.get("phase", "RUNNING")
                        )

                        self._remaining_s = duration
                        self._deadline = (
                            time.monotonic() + duration
                        )
                        self._paused_remaining_s = None

                        self.penalties = []
                        self.penalty_time_s = 0.0
                        self.score = payload.get("score")

                        control_command = "START"

            elif command == "PAUSE":
                if self.state != "RUNNING":
                    error = f"当前状态 {self.state} 不允许 PAUSE"
                else:
                    remaining = self._remaining_locked()

                    self._remaining_s = remaining
                    self._paused_remaining_s = remaining
                    self._deadline = None
                    self.state = "PAUSED"

                    control_command = "PAUSE"

            elif command == "RESUME":
                if self._emergency_latched:
                    error = "急停锁存期间禁止 RESUME"

                elif self.state != "PAUSED":
                    error = f"当前状态 {self.state} 不允许 RESUME"

                else:
                    remaining = self._remaining_locked()

                    if remaining is None or remaining <= 0:
                        error = "倒计时已结束，不能继续比赛"
                    else:
                        self._deadline = (
                            time.monotonic() + remaining
                        )
                        self._remaining_s = remaining
                        self._paused_remaining_s = None
                        self.state = "RUNNING"

                        control_command = "RESUME"

            elif command == "STOP":
                if self._emergency_latched:
                    # 普通 STOP 可以停止任务，但不能清除急停 FAULT。
                    remaining = self._remaining_locked()

                    if remaining is not None:
                        self._remaining_s = remaining

                    self._deadline = None
                    self._paused_remaining_s = None
                    self.state = "FAULT"
                    self.phase = "FAULT"

                else:
                    self._remaining_s = self._remaining_locked()
                    self._deadline = None
                    self._paused_remaining_s = None

                    self.state = "STOPPED"
                    self.phase = str(
                        payload.get("phase", "STOPPED")
                    )

                control_command = "STOP"

            elif command == "PENALTY":
                penalty = payload.get(
                    "penalty_time_s",
                    payload.get("added_time_s", 0),
                )

                try:
                    penalty = float(penalty)
                except (TypeError, ValueError):
                    penalty = -1.0

                if not math.isfinite(penalty) or penalty < 0:
                    error = "加罚时间必须是非负数"
                else:
                    self.penalties.append({
                        "timestamp": utc_timestamp(),
                        "reason": str(payload.get("reason", "")),
                        "penalty_time_s": penalty,
                        "source": source,
                    })
                    self.penalty_time_s += penalty

            elif command == "SCORE_QUERY":
                if "score" in payload:
                    self.score = payload["score"]

            else:
                error = f"不支持的裁判指令：{command!r}"

            # 急停锁存期间不允许其他命令覆盖 FAULT 阶段。
            if (
                error is None
                and payload.get("phase") is not None
                and not self._emergency_latched
            ):
                self.phase = str(payload["phase"])

            if error is not None:
                self.last_error = error
            elif self._emergency_latched:
                self.last_error = self._emergency_reason
            else:
                self.last_error = None

            self._sequence += 1

            new_state = self.state
            snapshot = self._snapshot_locked()

        if error:
            self.get_logger().warning(error)

        details = {"command": command}

        if payload.get("reason") is not None:
            details["reason"] = str(payload["reason"])

        for key in (
            "countdown_s",
            "duration_s",
            "penalty_time_s",
            "added_time_s",
            "score",
            "request_id",
        ):
            if key in payload:
                details[key] = payload[key]

        op = (
            f"REFEREE_{command}"
            if command in SUPPORTED_COMMANDS
            else "REFEREE_UNKNOWN_COMMAND"
        )

        audit_ok = self._write_audit(
            op=op,
            source=source,
            result="ok" if error is None else "rejected",
            from_state=old_state,
            to_state=new_state,
            details=details,
        )

        if not audit_ok:
            with self._lock:
                audit_error = "审计日志写入失败"

                if error:
                    self.last_error = f"{error}；{audit_error}"
                else:
                    self.last_error = audit_error

                if self._emergency_latched:
                    # 即使审计失败，急停锁存也不能被解除。
                    self.state = "FAULT"
                    self.phase = "FAULT"
                    self._deadline = None
                    self._paused_remaining_s = None

                    if command in ("START", "RESUME"):
                        control_command = None

                elif (
                    command in ("PAUSE", "STOP")
                    and error is None
                ):
                    # 安全方向的控制事件仍然允许发布。
                    pass

                elif (
                    command in ("START", "RESUME")
                    and error is None
                ):
                    remaining = self._remaining_locked()
                    self._remaining_s = remaining
                    self._deadline = None
                    self._paused_remaining_s = None
                    self.state = "FAULT"
                    self.phase = "FAULT"
                    control_command = None

                else:
                    if self.state == "RUNNING":
                        self._remaining_s = self._remaining_locked()
                        control_command = "PAUSE"
                    else:
                        control_command = None

                    self._deadline = None
                    self._paused_remaining_s = None
                    self.state = "FAULT"
                    self.phase = "FAULT"

                self._sequence += 1
                snapshot = self._snapshot_locked()

        if control_command:
            self._publish_control(
                control_command,
                source,
                payload.get("reason"),
            )

        self._publish_snapshot(snapshot)

    # ---------------------------------------------------------
    # 倒计时看门狗
    # ---------------------------------------------------------

    def _on_timer(self):
        """刷新倒计时，并检测超时却未收到 STOP 的情况。"""
        countdown_fault = False

        with self._lock:
            old_state = self.state

            if (
                self.state == "RUNNING"
                and self._deadline is not None
                and time.monotonic() >= self._deadline
            ):
                self.state = "FAULT"
                self.phase = "FAULT"
                self._remaining_s = 0
                self._deadline = None
                self._paused_remaining_s = None

                self.last_error = (
                    "倒计时已结束，但没有收到裁判 STOP 指令"
                )

                countdown_fault = True

            self._sequence += 1
            snapshot = self._snapshot_locked()

        if countdown_fault:
            audit_ok = self._write_audit(
                op="COUNTDOWN_TIMEOUT",
                source="system",
                result="fault",
                from_state=old_state,
                to_state="FAULT",
                details={
                    "reason": "COUNTDOWN_EXPIRED_WITHOUT_STOP",
                },
            )

            if not audit_ok:
                with self._lock:
                    self.last_error = (
                        "倒计时结束且审计日志写入失败"
                    )
                    self._sequence += 1
                    snapshot = self._snapshot_locked()

            self.get_logger().error(
                "倒计时结束但未收到 STOP，"
                "已进入 FAULT 并请求暂停任务"
            )

            self._publish_control(
                "PAUSE",
                "countdown_watchdog",
                "COUNTDOWN_EXPIRED_WITHOUT_STOP",
            )

        self._publish_snapshot(snapshot)


def read_single_key():
    """交互式终端读取单个按键。"""
    if not sys.stdin.isatty():
        line = sys.stdin.readline()
        return line[:1] if line else ""

    import termios
    import tty

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(
            fd,
            termios.TCSADRAIN,
            old_settings,
        )


def keyboard_worker(node):
    """模拟裁判键盘：S/P/R/E；Q 退出。"""
    print(
        "\n模拟裁判：S=START，P=PAUSE，R=RESUME，"
        "E=STOP，Q=退出",
        flush=True,
    )

    while rclpy.ok():
        try:
            key = read_single_key()
        except (EOFError, KeyboardInterrupt):
            break
        except Exception as exc:
            node.get_logger().error(f"读取键盘失败：{exc}")
            break

        if not key:
            break

        if key == "\x03" or key.upper() == "Q":
            if rclpy.ok():
                rclpy.shutdown()
            break

        command = KEY_COMMANDS.get(key.upper())

        if command:
            print(f"\n[模拟裁判] {command}", flush=True)
            node.process_command(
                {"command": command},
                source="keyboard",
            )


def main(args=None):
    parser = argparse.ArgumentParser(
        description="M5-3 裁判协议适配器"
    )

    parser.add_argument(
        "--mode",
        choices=("topic", "keyboard"),
        default="topic",
        help="topic 接收 ROS 话题；keyboard 启用键盘模拟",
    )

    parser.add_argument(
        "--duration-s",
        type=float,
        default=180.0,
        help="默认比赛倒计时秒数",
    )

    options = parser.parse_args(args)

    if (
        not math.isfinite(options.duration_s)
        or options.duration_s <= 0
    ):
        parser.error("--duration-s 必须是正数")

    rclpy.init()
    node = RefereeBridge(
        match_duration_s=options.duration_s
    )

    if options.mode == "keyboard":
        threading.Thread(
            target=keyboard_worker,
            args=(node,),
            daemon=True,
        ).start()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
