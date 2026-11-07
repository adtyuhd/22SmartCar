import json
from concurrent.futures import ThreadPoolExecutor
import pytest
from audit import AuditLogger
from verify_audit import verify_log

def test_concurrent_writers_and_tamper_detection(tmp_path):
    writers = [AuditLogger(tmp_path, user="test") for _ in range(3)]
    def append(i):
        return writers[i % 3].append(op="TEST", source="simulation", result="ok", details={"i": i})
    with ThreadPoolExecutor(max_workers=3) as executor:
        list(executor.map(append, range(15)))
    path = next(tmp_path.glob("audit_log_*.jsonl"))
    count, checkpoint = verify_log(path)
    assert count == 15
    lines = path.read_text().splitlines()
    path.write_text("\n".join(lines[:-1]) + "\n")
    with pytest.raises(ValueError, match="最终哈希"):
        verify_log(path, checkpoint)
    record = json.loads(lines[0]); record["result"] = "changed"
    path.write_text(json.dumps(record) + "\n")
    with pytest.raises(ValueError):
        verify_log(path)
    with pytest.raises(ValueError):
        writers[0].append(op="TEST", source="simulation", result="ok")
