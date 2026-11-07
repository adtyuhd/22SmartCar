
"""M5-3 主机侧分层急停组件。

本模块不创建独立 ROS 2 Node，而是附着到已有节点上，
供后续 operator_terminal.py 集成调用。

负责：
1. 指令层：发布零速度，并在锁定期间持续发布。
2. 话题层：向 /emergency_stop 发布 STOP / RESET 请求。
3. 记录层：复用 audit.py 的 AuditLogger。
4. 反馈层：接收 /safety/status，等待小车端确认。
5. 故障联动：发布 /safety/fault，供裁判适配器接入。

注意：
本模块本身不执行底盘控制器切换，也不代表已经完成实车急停验收。
"""

import json
import threading
import uuid
from datetime import datetime, timezone
from pathlib import Path

from geometry_msgs.msg import Twist
from std_msgs.msg import String

try:
    from .audit import AuditLogger
except ImportError:
    from audit import AuditLogger


class SafetyController:
    """附着到已有 ROS 2 节点的主机侧急停控制器。"""

    ALLOWED_RESET_STATES = {"STOPPED", "FAULT"}

    def __init__(
        self,
        node,
        audit_dir=None,
        audit_user="operator",
        zero_hz=20.0,
    ):
        if zero_hz <= 0:
            raise ValueError("zero_hz 必须大于 0")

        self.node = node
        self.audit_user = audit_user
        self._lock = threading.RLock()

        # 主机侧安全状态。
        self._latched = False
        self._robot_state = "UNKNOWN"
        self._controller_result = "unknown"

        # 用请求 ID 区分本次操作与过期的反馈。
        self._pending_stop_id = None
        self._pending_reset_id = None

        # 审计日志默认写入 M5-3 项目根目录。
        project_root = Path(__file__).resolve().parents[1]
        log_dir = Path(audit_dir) if audit_dir is not None else project_root

        self.audit = AuditLogger(
            log_dir=str(log_dir),
            user=audit_user,
        )

        # 指令层：常规底盘速度接口。
        self.cmd_vel_pub = node.create_publisher(
            Twist,
            "/cmd_vel",
            10,
        )

        # 话题层：小车端唯一新增的急停订阅者接收此话题。
        self.emergency_pub = node.create_publisher(
            String,
            "/emergency_stop",
            10,
        )

        # 故障事件：后续由裁判适配器接入，转换为 FAULT。
        self.fault_pub = node.create_publisher(
            String,
            "/safety/fault",
            10,
        )

        # 主机侧状态：供后续 Web 页面读取。
        self.host_status_pub = node.create_publisher(
            String,
            "/safety/host_status",
            10,
        )

        # 小车端反馈：确认急停与复位是否真正执行成功。
        self.robot_status_sub = node.create_subscription(
            String,
            "/safety/status",
            self._on_robot_status,
            10,
        )

        # 锁定期间持续发送零速度，而不是只发送一次。
        self.zero_timer = node.create_timer(
            1.0 / float(zero_hz),
            self._publish_zero_if_latched,
        )

        self.node.get_logger().info(
            "主机侧 SafetyController 已初始化；"
            "等待操作指令和小车端状态反馈"
        )

    @staticmethod
    def _timestamp():
        """生成带时区的 UTC 时间戳。"""
        return datetime.now(timezone.utc).isoformat(
            timespec="milliseconds"
        )

    def _write_audit(
        self,
        op,
        source,
        result,
        from_value,
        to_value,
        user=None,
        details=None,
    ):
        """写入审计日志。审计失败不能阻止急停请求发出。"""
        try:
            self.audit.append(
                op=op,
                source=source,
                result=result,
                from_value=from_value,
                to_value=to_value,
                user=user or self.audit_user,
                details=details or {},
            )
            return True
        except Exception as exc:
            self.node.get_logger().error(
                f"审计写入失败：{op}；{type(exc).__name__}: {exc}"
            )
            return False

    def _publish_zero(self):
        """发布一条零速度指令。"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        self.cmd_vel_pub.publish(msg)

    def _publish_zero_if_latched(self):
        """定时器回调：只要仍然锁定，就持续发送零速度。"""
        with self._lock:
            latched = self._latched

        if not latched:
            return

        try:
            self._publish_zero()
        except Exception as exc:
            self.node.get_logger().error(
                f"持续发布零速度失败：{type(exc).__name__}: {exc}"
            )

    def _publish_robot_command(
        self,
        action,
        request_id,
        source,
        user,
        reason="",
        manual_confirmation=False,
        match_state="UNKNOWN",
    ):
        """向小车端发送结构化命令。"""
        payload = {
            "action": action,
            "request_id": request_id,
            "source": source,
            "user": user or self.audit_user,
            "reason": reason,
            "manual_confirmation": bool(manual_confirmation),
            "match_state": str(match_state).upper(),
            "timestamp": self._timestamp(),
        }

        self.emergency_pub.publish(
            String(
                data=json.dumps(
                    payload,
                    ensure_ascii=False,
                    sort_keys=True,
                )
            )
        )

    def _publish_fault(self, request_id, source, reason):
        """发布急停故障事件，供裁判适配器后续接入。"""
        payload = {
            "type": "EMERGENCY_STOP",
            "state": "FAULT",
            "request_id": request_id,
            "source": source,
            "reason": reason,
            "timestamp": self._timestamp(),
        }

        self.fault_pub.publish(
            String(
                data=json.dumps(
                    payload,
                    ensure_ascii=False,
                    sort_keys=True,
                )
            )
        )

    def _publish_host_status(self, state, request_id=None, message=""):
        """发布主机侧状态，供 Web 终端显示。"""
        with self._lock:
            payload = {
                "state": state,
                "latched": self._latched,
                "robot_state": self._robot_state,
                "controller_result": self._controller_result,
                "request_id": request_id,
                "message": message,
                "timestamp": self._timestamp(),
            }

        self.host_status_pub.publish(
            String(
                data=json.dumps(
                    payload,
                    ensure_ascii=False,
                    sort_keys=True,
                )
            )
        )

    def trigger_emergency_stop(
        self,
        source="ui",
        user=None,
        reason="operator emergency stop",
        confirmed=False,
    ):
        """触发急停。

        语音来源必须显式确认，否则拒绝触发。
        返回 True 表示急停请求已发出，不代表底盘已确认停止。
        """
        source = str(source or "ui").strip()
        actor = user or self.audit_user

        with self._lock:
            was_latched = self._latched

        if source.lower().startswith("voice") and confirmed is not True:
            self._write_audit(
                op="EMERGENCY_STOP_REJECTED",
                source=source,
                result="confirmation_required",
                from_value=(
                    "ESTOP_LATCHED" if was_latched else "NORMAL"
                ),
                to_value=(
                    "ESTOP_LATCHED" if was_latched else "NORMAL"
                ),
                user=actor,
                details={
                    "reason": reason,
                    "message": "语音急停必须经过人工确认",
                },
            )
            self.node.get_logger().warning(
                "语音急停请求被拒绝：尚未经过人工确认"
            )
            return False

        request_id = uuid.uuid4().hex

        # 必须先锁定，再执行后续操作。
        with self._lock:
            self._latched = True
            self._pending_stop_id = request_id
            self._pending_reset_id = None

        # 第一层：立即发布零速度。
        try:
            self._publish_zero()
        except Exception as exc:
            self.node.get_logger().error(
                f"首次零速度发布失败：{type(exc).__name__}: {exc}"
            )

        # 第二层：向小车端发出急停请求。
        try:
            self._publish_robot_command(
                action="STOP",
                request_id=request_id,
                source=source,
                user=actor,
                reason=reason,
            )
        except Exception as exc:
            self.node.get_logger().error(
                f"急停话题发布失败：{type(exc).__name__}: {exc}"
            )

        # 故障事件独立发布，供裁判适配器转换为 FAULT。
        try:
            self._publish_fault(request_id, source, reason)
        except Exception as exc:
            self.node.get_logger().error(
                f"安全故障事件发布失败：{type(exc).__name__}: {exc}"
            )

        audit_ok = self._write_audit(
            op="EMERGENCY_STOP",
            source=source,
            result="requested",
            from_value=(
                "ESTOP_LATCHED" if was_latched else "NORMAL"
            ),
            to_value="ESTOP_LATCHED",
            user=actor,
            details={
                "request_id": request_id,
                "reason": reason,
                "audit_write_success": True,
            },
        )

        if not audit_ok:
            self.node.get_logger().error(
                "急停请求已发出，但审计写入失败；"
                "不能将此次操作视为完整成功"
            )

        self._publish_host_status(
            state="ESTOP_REQUESTED",
            request_id=request_id,
            message="急停请求已发出，等待小车端确认",
        )

        self.node.get_logger().warning(
            f"急停请求已发出，request_id={request_id}；"
            "尚未确认底盘停止"
        )

        return True

    def request_reset(
        self,
        manual_confirmation=False,
        match_state="UNKNOWN",
        source="ui",
        user=None,
    ):
        """请求人工复位。

        只有明确人工确认且比赛状态为 STOPPED 或 FAULT 时才允许请求。
        主机侧不会立即解除锁定，必须等待小车端返回 RESET_OK。
        """
        actor = user or self.audit_user
        match_state = str(match_state).upper()

        with self._lock:
            latched = self._latched
            from_state = (
                "ESTOP_LATCHED" if latched else "NORMAL"
            )

        if not latched:
            self.node.get_logger().warning(
                "复位请求被拒绝：当前没有急停锁定"
            )
            return False

        if manual_confirmation is not True:
            self._write_audit(
                op="EMERGENCY_RESET_REJECTED",
                source=source,
                result="manual_confirmation_required",
                from_value=from_state,
                to_value=from_state,
                user=actor,
                details={"match_state": match_state},
            )
            return False

        if match_state not in self.ALLOWED_RESET_STATES:
            self._write_audit(
                op="EMERGENCY_RESET_REJECTED",
                source=source,
                result="unsafe_match_state",
                from_value=from_state,
                to_value=from_state,
                user=actor,
                details={
                    "match_state": match_state,
                    "allowed_states": sorted(
                        self.ALLOWED_RESET_STATES
                    ),
                },
            )
            self.node.get_logger().warning(
                f"复位请求被拒绝：比赛状态为 {match_state}"
            )
            return False

        request_id = uuid.uuid4().hex

        # 先记录复位请求。审计失败时，不解除安全锁定。
        audit_ok = self._write_audit(
            op="EMERGENCY_RESET_REQUEST",
            source=source,
            result="requested",
            from_value="ESTOP_LATCHED",
            to_value="RESET_PENDING",
            user=actor,
            details={
                "request_id": request_id,
                "match_state": match_state,
                "manual_confirmation": True,
            },
        )

        if not audit_ok:
            self.node.get_logger().error(
                "复位请求被拒绝：无法可靠写入审计日志"
            )
            return False

        # 在发送请求前保存 ID，防止收到反馈时还无法匹配。
        with self._lock:
            self._pending_reset_id = request_id
            self._robot_state = "RESET_PENDING"

        try:
            self._publish_robot_command(
                action="RESET",
                request_id=request_id,
                source=source,
                user=actor,
                reason="manually confirmed reset",
                manual_confirmation=True,
                match_state=match_state,
            )
        except Exception as exc:
            self.node.get_logger().error(
                f"复位请求发布失败：{type(exc).__name__}: {exc}"
            )
            self._write_audit(
                op="EMERGENCY_RESET_REQUEST",
                source=source,
                result="publish_failed",
                from_value="ESTOP_LATCHED",
                to_value="ESTOP_LATCHED",
                user=actor,
                details={
                    "request_id": request_id,
                    "error": str(exc),
                },
            )
            return False

        # 注意：此时仍然保持 _latched=True。
        self._publish_host_status(
            state="RESET_PENDING",
            request_id=request_id,
            message="等待小车端确认复位",
        )

        return True

    def _on_robot_status(self, msg):
        """处理小车端反馈；过期或无关请求的反馈不能解除锁定。"""
        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.node.get_logger().warning(
                "忽略格式错误的小车端安全状态"
            )
            return

        if not isinstance(payload, dict):
            self.node.get_logger().warning("忽略非对象的小车端安全状态")
            return
        request_id = payload.get("request_id")
        if not isinstance(request_id, str) or not request_id.strip():
            self.node.get_logger().warning("忽略缺少请求 ID 的小车端安全状态")
            return

        state = str(payload.get("state", "UNKNOWN")).upper()
        request_id = payload.get("request_id")
        controller_result = str(
            payload.get("controller_result", "unknown")
        ).lower()
        message = str(payload.get("message", ""))

        audit_event = None

        with self._lock:
            self._robot_state = state
            self._controller_result = controller_result

            # 只处理与当前急停请求对应的反馈。
            if (
                request_id == self._pending_stop_id
                and state in {"ESTOP_LATCHED", "ESTOP_FAILED"}
            ):
                audit_event = {
                    "op": "EMERGENCY_STOP_CONTROLLER",
                    "source": "robot_side",
                    "result": (
                        "ok"
                        if (
                            state == "ESTOP_LATCHED"
                            and controller_result == "ok"
                        )
                        else "failed"
                    ),
                    "from_value": "ESTOP_REQUESTED",
                    "to_value": state,
                    "user": self.audit_user,
                    "details": {
                        "request_id": request_id,
                        "controller_result": controller_result,
                        "message": message,
                    },
                }

            elif self._pending_reset_id is not None and request_id == self._pending_reset_id:
                if (
                    state == "RESET_OK"
                    and controller_result == "ok"
                ):
                    # 只有匹配的成功反馈才解除主机侧锁存。
                    self._latched = False
                    self._pending_reset_id = None

                    audit_event = {
                        "op": "EMERGENCY_RESET",
                        "source": "robot_side",
                        "result": "ok",
                        "from_value": "ESTOP_LATCHED",
                        "to_value": "NORMAL",
                        "user": self.audit_user,
                        "details": {
                            "request_id": request_id,
                            "controller_result": controller_result,
                            "message": message,
                        },
                    }

                elif state in {"RESET_FAILED", "RESET_OK"}:
                    # 失败或缺少成功结果时，保持急停锁定。
                    audit_event = {
                        "op": "EMERGENCY_RESET",
                        "source": "robot_side",
                        "result": "failed",
                        "from_value": "ESTOP_LATCHED",
                        "to_value": "ESTOP_LATCHED",
                        "user": self.audit_user,
                        "details": {
                            "request_id": request_id,
                            "robot_state": state,
                            "controller_result": controller_result,
                            "message": message,
                        },
                    }

        if audit_event is not None:
            self._write_audit(**audit_event)

        self._publish_host_status(
            state=state,
            request_id=request_id,
            message=message,
        )

    def get_state(self):
        """提供给 Web 终端读取的线程安全状态快照。"""
        with self._lock:
            return {
                "latched": self._latched,
                "robot_state": self._robot_state,
                "controller_result": self._controller_result,
                "pending_stop_id": self._pending_stop_id,
                "pending_reset_id": self._pending_reset_id,
            }

    def close(self):
        """停止本组件的定时器；不销毁所属的 ROS 2 节点。"""
        if self.zero_timer is not None:
            self.zero_timer.cancel()
