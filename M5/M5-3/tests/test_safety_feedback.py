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
