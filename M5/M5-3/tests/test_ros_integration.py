"""Real ROS 2 transport + mock parameter node; never robot acceptance."""
import os
import pytest

pytestmark = pytest.mark.skipif(os.environ.get("M53_RUN_ROS_TESTS") != "1",
                                reason="use tools/run_tests.sh --ros in isolated DDS domain")

def test_terminal_lifecycle_and_real_parameter_services(tmp_path, monkeypatch):
    import yaml
    from fastapi.testclient import TestClient
    from rclpy.node import Node
    import operator_terminal as terminal
    from verify_audit import verify_log

    config = yaml.safe_load(terminal.DEFAULT_CONFIG.read_text())
    config["hot_switch"]["parameter_targets"]["navigation"] = "/m53_mock_navigation"
    config["hot_switch"]["switch_log"] = str(tmp_path / "switch.csv")
    path = tmp_path / "terminal.yaml"
    path.write_text(yaml.safe_dump(config))
    monkeypatch.setenv("M53_CONFIG_PATH", str(path))
    monkeypatch.setenv("M53_AUDIT_DIR", str(tmp_path))
    with TestClient(terminal.app) as client:
        runtime = terminal.require_runtime()
        node = Node("m53_mock_navigation")
        node.declare_parameter("max_velocity", 0.2)
        runtime.executor.add_node(node)
        try:
            health = client.get("/health").json()
            assert health["ros_ok"] and health["executor_thread_alive"]
            assert client.get("/api/status").json()["referee"]["status"] == "READY"
            result = client.post("/api/parameters", json={"target": "navigation",
                "parameter": "max_velocity", "value": 0.4})
            assert result.status_code == 200, result.text
            assert node.get_parameter("max_velocity").value == 0.4
            assert client.post("/api/parameters", json={"target": "navigation",
                "parameter": "max_velocity", "value": 2}).status_code == 409
            assert node.get_parameter("max_velocity").value == 0.4
            assert client.post("/api/models", json={"model_id": "yolov8n_fp32"}).status_code == 409
            assert client.post("/api/referee/command", json={"command": "START"}).status_code == 200
            assert client.post("/api/referee/command", json={"command": "PAUSE"}).status_code == 200
            assert client.post("/api/referee/command", json={"command": "STOP"}).status_code == 200
            assert client.get("/api/tasks/capabilities").json()["skip_available"] is False
            assert verify_log(next(tmp_path.glob("audit_log_*.jsonl")))[0] > 0
        finally:
            runtime.executor.remove_node(node)
            node.destroy_node()
    assert terminal._runtime is None
    assert not runtime.spin_thread.is_alive()
