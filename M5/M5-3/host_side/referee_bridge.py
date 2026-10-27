#!/usr/bin/env python3
"""M5-3 裁判协议适配器：状态同步、控制事件和审计记录。"""

import argparse
import json
import math
import sys
import tempfile
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

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

    # 防止非标准 JSON 数值等异常数据导致后续序列化失败。
    try:
        json.dumps(payload, ensure_ascii=False, allow_nan=False)
    except (TypeError, ValueError):
        return {}

    return payload


class RefereeBridge(Node):
    """裁判适配器。

    输入：
        /referee/cmd

    输出：
        /referee/status
        /task_state

    控制事件：
        /task_start
        /task_pause
        /task_resume
        /task_stop

    注意：控制事件需要后续任务控制模块消费。
    发布成功不代表任务已经实际执行。
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

        # 每个适配器实例使用统一审计写入器。
        self.audit = AuditLogger(
            log_dir=audit_dir,
            user=audit_user,
        )

        self.cmd_sub = self.create_subscription(
            String,
            "/referee/cmd",
            self._on_referee_command,
            10,
        )

        self.status_pub = self.create_publisher(
            String,
            "/referee/status",
            10,
        )

        # M5-2 的聚合器已有 /task_state 订阅配置。
        self.task_state_pub = self.create_publisher(
            String,
            "/task_state",
            10,
        )

        self.control_publishers = {
            topic: self.create_publisher(String, topic, 10)
            for topic in CONTROL_TOPICS.values()
        }

        self.timer = self.create_timer(
            1.0,
            self._on_timer,
        )

        self.get_logger().info(
            "裁判适配器已启动："
            "订阅 /referee/cmd，"
            "发布 /referee/status 和 /task_state"
        )

        self._publish_status()

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
                if self.state not in ("READY", "STOPPED"):
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

                    if (
                        not math.isfinite(duration)
                        or duration <= 0
                    ):
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
                if self.state != "PAUSED":
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

            if error is None and payload.get("phase") is not None:
                self.phase = str(payload["phase"])

            self.last_error = error
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
            # 审计失败时，不允许启动或恢复任务。
            # PAUSE 和 STOP 属于停止方向的控制，仍然发布。
            with self._lock:
                audit_error = "审计日志写入失败"

                if error:
                    self.last_error = f"{error}；{audit_error}"
                else:
                    self.last_error = audit_error

                if (
                    command in ("PAUSE", "STOP")
                    and error is None
                ):
                    # 保留安全方向的控制事件。
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
                    "reason": "COUNTDOWN_EXPIRED_WITHOUT_STOP"
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