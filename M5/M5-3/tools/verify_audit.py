
#!/usr/bin/env python3
"""验证 M5-3 JSONL 审计日志的 SHA-256 哈希链。"""

import argparse
import hashlib
import hmac
import json
import sys
from pathlib import Path


ZERO_HASH = "0" * 64


def canonical_bytes(record):
    return json.dumps(
        record,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")


def calculate_hash(record):
    content = dict(record)
    content.pop("hash", None)
    return hashlib.sha256(canonical_bytes(content)).hexdigest()


def verify_log(path, expected_final_hash=None):
    """验证每条记录的内容哈希及前序哈希。"""
    path = Path(path)

    if not path.is_file():
        raise ValueError(f"日志文件不存在：{path}")

    previous_hash = ZERO_HASH
    count = 0

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
                    f"第 {line_number} 行前序哈希不匹配："
                    "可能发生删除、重排或修改"
                )

            claimed_hash = record.get("hash")

            if not isinstance(claimed_hash, str):
                raise ValueError(
                    f"第 {line_number} 行缺少 hash 字段"
                )

            actual_hash = calculate_hash(record)

            if not hmac.compare_digest(claimed_hash, actual_hash):
                raise ValueError(
                    f"第 {line_number} 行内容哈希不匹配："
                    "记录可能被篡改"
                )

            previous_hash = claimed_hash
            count += 1

    if count == 0:
        raise ValueError("日志文件为空，没有可验证的记录")

    if expected_final_hash is not None:
        if not hmac.compare_digest(
            previous_hash, expected_final_hash.lower()
        ):
            raise ValueError(
                "最终哈希与可信检查点不一致："
                "日志可能被截断或替换"
            )

    return count, previous_hash


def main():
    parser = argparse.ArgumentParser(
        description="验证 M5-3 JSONL 审计日志哈希链"
    )

    parser.add_argument(
        "--log",
        required=True,
        help="需要验证的 JSONL 文件路径",
    )

    parser.add_argument(
        "--expected-final-hash",
        help="可选：外部可信渠道保存的末端哈希",
    )

    args = parser.parse_args()

    try:
        count, final_hash = verify_log(
            args.log,
            args.expected_final_hash,
        )
    except (OSError, ValueError, UnicodeError) as exc:
        print(f"FAIL: {exc}")
        return 1

    print("PASS: 哈希链验证成功")
    print(f"记录数：{count}")
    print(f"最终哈希：{final_hash}")

    if args.expected_final_hash is None:
        print(
            "WARNING: 未提供外部可信检查点，"
            "无法仅凭现存日志可靠检测尾部截断。"
        )

    return 0


if __name__ == "__main__":
    sys.exit(main())
