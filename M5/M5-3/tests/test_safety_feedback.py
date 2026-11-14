import json
from types import SimpleNamespace
import pytest
from safety import SafetyController

class Node:
    def create_publisher(self, *args):
        return SimpleNamespace(publish=lambda msg: None)
    def create_subscription(self, *args):
        return None
    def create_timer(self, *args):
        return SimpleNamespace(cancel=lambda: None)
    def get_logger(self):
        return SimpleNamespace(info=lambda msg: None, warning=lambda msg: None, error=lambda msg: None)

@pytest.mark.parametrize("payload", [[], None, "invalid", {},
    {"state": "RESET_OK", "controller_result": "ok"},
    {"request_id": "stale", "state": "RESET_OK", "controller_result": "ok"}])
def test_unsolicited_or_malformed_feedback_cannot_unlock(tmp_path, payload):
    safety = SafetyController(Node(), audit_dir=tmp_path)
    safety._latched = True
    safety._on_robot_status(SimpleNamespace(data=json.dumps(payload)))
    assert safety.get_state()["latched"] is True
    safety.close()

def test_only_matching_success_unlocks(tmp_path):
    safety = SafetyController(Node(), audit_dir=tmp_path)
    safety._latched = True
    safety._pending_reset_id = "current"
    for request_id, result in [("old", "ok"), ("current", "failed")]:
        safety._on_robot_status(SimpleNamespace(data=json.dumps(dict(
            request_id=request_id, state="RESET_OK", controller_result=result))))
        assert safety.get_state()["latched"] is True
    safety._on_robot_status(SimpleNamespace(data=json.dumps(dict(
        request_id="current", state="RESET_OK", controller_result="ok"))))
    assert safety.get_state()["latched"] is False
    safety.close()


def test_publish_failure_does_not_claim_request_sent(tmp_path, monkeypatch):
    safety = SafetyController(Node(), audit_dir=tmp_path)
    def fail(**kwargs):
        raise RuntimeError("simulated publisher failure")
    monkeypatch.setattr(safety, "_publish_robot_command", fail)
    assert safety.trigger_emergency_stop() is False
    assert safety.get_state()["latched"] is True
    safety.close()


def test_concurrent_reset_and_stop_preserve_order_and_new_latch(tmp_path, monkeypatch):
    from concurrent.futures import ThreadPoolExecutor
    from threading import Event
    safety = SafetyController(Node(), audit_dir=tmp_path)
    safety._latched = True
    entered = Event(); release = Event(); stop_started = Event()
    published = []
    def publish(**payload):
        if payload["action"] == "RESET":
            entered.set()
            assert release.wait(3)
        published.append(payload)
    monkeypatch.setattr(safety, "_publish_robot_command", publish)
    def stop():
        stop_started.set()
        return safety.trigger_emergency_stop()
    with ThreadPoolExecutor(max_workers=2) as pool:
        reset = pool.submit(safety.request_reset, manual_confirmation=True, match_state="FAULT")
        assert entered.wait(3)
        stopping = pool.submit(stop)
        assert stop_started.wait(3)
        release.set()
        assert reset.result(timeout=3)
        assert stopping.result(timeout=3)
    assert [payload["action"] for payload in published] == ["RESET", "STOP"]
    old_reset = published[0]["request_id"]
    safety._on_robot_status(SimpleNamespace(data=json.dumps({"request_id": old_reset,
        "state": "RESET_OK", "controller_result": "ok"})))
    assert safety.get_state()["latched"] is True
    assert safety.get_state()["pending_reset_id"] is None
    safety.close()
