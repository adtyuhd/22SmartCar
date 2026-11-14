"""Read allowlisted dashboard messages from an indexed or unindexed MCAP."""
import ctypes
import ctypes.util
import io
import json
import struct
import zlib

MAGIC = b"\x89MCAP0\r\n"
TOPICS = frozenset({
    "/dashboard/metrics", "/dashboard/faults", "/dashboard/task_state",
    "/m5/review/metrics", "/m5/review/faults", "/m5/review/clock",
})


def unpack(fmt, stream):
    return struct.unpack("<" + fmt, stream.read(struct.calcsize("<" + fmt)))


def blob(stream, fmt="I"):
    size, = unpack(fmt, stream)
    value = stream.read(size)
    if len(value) != size:
        raise ValueError("记录长度错误")
    return value


def string(stream):
    return blob(stream).decode()


def decompress(name, data, size):
    if not name:
        return data
    library = ctypes.util.find_library(name) if name in {"zstd", "lz4"} else None
    if not library:
        raise ValueError(f"缺少 MCAP 解压库 lib{name}")
    lib, output = ctypes.CDLL(library), ctypes.create_string_buffer(size)
    if name == "zstd":
        lib.ZSTD_decompress.argtypes = [ctypes.c_void_p, ctypes.c_size_t,
                                        ctypes.c_char_p, ctypes.c_size_t]
        lib.ZSTD_decompress.restype = ctypes.c_size_t
        length = lib.ZSTD_decompress(output, size, data, len(data))
    else:
        context = ctypes.c_void_p()
        lib.LZ4F_createDecompressionContext(ctypes.byref(context), 100)
        source = ctypes.create_string_buffer(data)
        consumed = length = 0
        try:
            while consumed < len(data):
                available = ctypes.c_size_t(len(data) - consumed)
                capacity = ctypes.c_size_t(size - length)
                lib.LZ4F_decompress.restype = ctypes.c_size_t
                remaining = lib.LZ4F_decompress(
                    context, ctypes.byref(output, length), ctypes.byref(capacity),
                    ctypes.byref(source, consumed), ctypes.byref(available), None)
                if remaining > 2**63 or not available.value:
                    raise ValueError("LZ4 数据损坏")
                consumed += available.value
                length += capacity.value
        finally:
            lib.LZ4F_freeDecompressionContext(context)
        if remaining:
            raise ValueError("LZ4 帧未结束")
    if length != size:
        raise ValueError("解压长度不符")
    return output.raw


def records(stream, end):
    while stream.tell() < end:
        offset = stream.tell()
        opcode, size = unpack("BQ", stream)
        finish = stream.tell() + size
        if finish > end:
            raise ValueError("MCAP 记录越界")
        yield offset, opcode, finish
        stream.seek(finish)


def read_mcap(path, topics=TOPICS):
    messages, issues, channels, selected_chunks = [], [], {}, {}
    skipped = 0

    def channel(stream):
        channel_id, schema_id = unpack("HH", stream)
        topic, encoding = string(stream), string(stream)
        channels[channel_id] = (topic, encoding)

    def scan(stream, end):
        nonlocal skipped
        for offset, opcode, finish in records(stream, end):
            if opcode == 4:
                channel(stream)
            elif opcode == 5:
                channel_id, _, log_ns, _ = unpack("HIQQ", stream)
                topic, encoding = channels.get(channel_id, ("", ""))
                if topic not in topics:
                    continue
                try:
                    payload = stream.read(finish - stream.tell())
                    if encoding == "cdr":
                        if len(payload) < 9 or payload[:2] not in (b"\0\0", b"\0\1"):
                            raise ValueError("仅支持封装 JSON 的 ROS 2 String")
                        endian = "<" if payload[1] == 1 else ">"
                        length, = struct.unpack_from(endian + "I", payload, 4)
                        if length == 0 or length > len(payload) - 8 or payload[7 + length]:
                            raise ValueError("String CDR 长度错误")
                        payload = payload[8:7 + length]
                    elif encoding != "json":
                        raise ValueError(f"消息编码 {encoding} 无适配器")
                    value = json.loads(payload)
                    if not isinstance(value, dict):
                        raise ValueError("需要 JSON 对象")
                    messages.append((log_ns, topic, value))
                except (ValueError, UnicodeError) as exc:
                    warning = f"{topic} 消息无法读取：{exc}"
                    if warning not in issues and len(issues) < 20:
                        issues.append(warning)
            elif opcode == 6:
                if offset in selected_chunks and not selected_chunks[offset]:
                    skipped += 1
                    continue
                _, _, size, crc = unpack("QQQI", stream)
                compression = string(stream)
                if size > 512 * 1024 * 1024:
                    raise ValueError("单块解压大小超限")
                raw = decompress(compression, blob(stream, "Q"), size)
                if len(raw) != size or (crc and zlib.crc32(raw) != crc):
                    raise ValueError("MCAP 数据块长度或 CRC 错误")
                scan(io.BytesIO(raw), len(raw))

    try:
        with path.open("rb") as stream:
            if stream.read(8) != MAGIC:
                raise ValueError("文件头无效")
            end = stream.seek(0, 2)
            if end < 45:
                raise ValueError("文件截断")
            stream.seek(end - 8)
            if stream.read(8) != MAGIC:
                raise ValueError("文件尾缺失或截断")
            footer = end - 37
            stream.seek(footer)
            opcode, length, summary, _, _ = unpack("BQQQI", stream)
            if (opcode, length) != (2, 20) or (summary and not 8 <= summary <= footer):
                raise ValueError("Footer 无效")
            indexes = {}
            if summary:
                stream.seek(summary)
                for _, opcode, finish in records(stream, footer):
                    if opcode == 4:
                        channel(stream)
                    elif opcode == 8:
                        _, _, offset, _ = unpack("QQQQ", stream)
                        entries = blob(stream)
                        indexes[offset] = {item[0] for item in struct.iter_unpack("<HQ", entries)}
                for offset, ids in indexes.items():
                    if ids and ids <= channels.keys():
                        selected_chunks[offset] = any(channels[i][0] in topics for i in ids)
            stream.seek(8)
            scan(stream, summary or footer)
    except (OSError, ValueError, OverflowError, struct.error) as exc:
        issues.append(f"MCAP 读取失败（可用的日志与任务报告仍会复盘）：{exc}")
    messages.sort(key=lambda item: item[0])
    if not messages:
        issues.append("未读到白名单话题的有效消息")
    return messages, issues, skipped
