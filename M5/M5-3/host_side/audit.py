
#!/usr/bin/env python3
"""M5-3 只追加审计日志与 SHA-256 哈希链。"""

import fcntl
import getpass
import hashlib
import hmac
import json
import os
import threading
from datetime import datetime, timezone
from pathlib import Path


ZERO_HASH = "0" * 64


def canonical_bytes(record):
    """将记录序列化为确定性的 JSON 字节。"""
    return json.dumps(
        record,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")


def calculate_hash(record):
    """计算记录哈希，不包含记录自身的 hash 字段。"""
    content = dict(record)
    content.pop("hash", None)
    return hashlib.sha256(canonical_bytes(content)).hexdigest()


def verify_existing_log(path):
    """验证现有日志的完整哈希链，返回末端哈希。

    空日志的起始哈希为 ZERO_HASH。
    如果日志损坏，拒绝继续追加。
    """
    previous_hash = ZERO_HASH

    if not path.exists():
        return previous_hash

    with path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            if not line.strip():
                raise ValueError(f"第 {line_number} 行为空")

            try:
                record = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(
                    f"第 {line_number} 行 JSON 格式错误"
                ) from exc

            if not isinstance(record, dict):
                raise ValueError(
                    f"第 {line_number} 行不是 JSON 对象"
                )

            if record.get("prev_hash") != previous_hash:
                raise ValueError(
                    f"第 {line_number} 行 prev_hash 不匹配"
                )

            claimed_hash = record.get("hash")
            actual_hash = calculate_hash(record)

            if not isinstance(claimed_hash, str):
                raise ValueError(
                    f"第 {line_number} 行缺少 hash 字段"
                )

            if not hmac.compare_digest(claimed_hash, actual_hash):
                raise ValueError(
                    f"第 {line_number} 行哈希校验失败"
                )

            previous_hash = claimed_hash

    return previous_hash


class AuditLogger:
    """M5-3 审计日志写入器。

    默认写入 M5-3 项目根目录下的 audit_log_YYYYMMDD.jsonl。

    操作人优先取 append() 显式传入的 user，
    其次取环境变量 M53_OPERATOR，
    最后使用当前操作系统账户名。
    """

    def __init__(self, log_dir=None, user=None):
        project_root = Path(__file__).resolve().parents[1]

        self.log_dir = Path(
            log_dir if log_dir is not None else project_root
        ).expanduser().resolve()

        self.log_dir.mkdir(parents=True, exist_ok=True)

        self.user = (
            user
            or os.environ.get("M53_OPERATOR")
            or getpass.getuser()
        )

        self._thread_lock = threading.Lock()

    def _today_log_path(self):
        """按 UTC 日期选择日志文件。"""
        date = datetime.now(timezone.utc).strftime("%Y%m%d")
        return self.log_dir / f"audit_log_{date}.jsonl"

    def append(
        self,
        op,
        source,
        result,
        from_value=None,
        to_value=None,
        user=None,
        details=None,
    ):
        """追加一条审计记录，返回写入的记录。

        op: 操作类型，如 EMERGENCY_STOP、MODEL_SWITCH
        source: 操作来源，如 ui、voice、physical_button、system
        result: 操作结果，如 ok、failed
        from_value / to_value: 操作前后的值或状态
        user: 操作人标识
        details: 可选的 JSON 兼容补充信息
        """
        if not op or not source or not result:
            raise ValueError("op、source、result 不能为空")

        record = {
            "ts": None,
            "op": str(op),
            "from": from_value,
            "to": to_value,
            "source": str(source),
            "user": str(user or self.user),
            "result": str(result),
            "prev_hash": ZERO_HASH,
        }

        if details is not None:
            record["details"] = details

        # 预先验证所有字段能否安全序列化。
        canonical_bytes(record)

        with self._thread_lock:
            path = self._today_log_path()

            fd = os.open(
                path,
                os.O_WRONLY | os.O_CREAT | os.O_APPEND,
                0o600,
            )

            try:
                # 协调遵守相同文件锁协议的其他写入进程。
                fcntl.flock(fd, fcntl.LOCK_EX)

                # 追加前检查历史链，损坏时拒绝继续写入。
                previous_hash = verify_existing_log(path)

                record["ts"] = datetime.now(
                    timezone.utc
                ).isoformat(timespec="milliseconds")

                record["prev_hash"] = previous_hash
                record["hash"] = calculate_hash(record)

                line = canonical_bytes(record) + b"\n"
                remaining = memoryview(line)

                while remaining:
                    written = os.write(fd, remaining)
                    if written <= 0:
                        raise OSError("审计日志写入失败")
                    remaining = remaining[written:]

                # 请求操作系统将记录同步到磁盘。
                os.fsync(fd)

            finally:
                try:
                    fcntl.flock(fd, fcntl.LOCK_UN)
                finally:
                    os.close(fd)

        return record
