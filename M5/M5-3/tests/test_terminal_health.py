from types import SimpleNamespace
import asyncio
import httpx
import pytest
import operator_terminal as terminal

@pytest.mark.parametrize("ros_ok,alive", [(True, True), (False, True), (True, False), (False, False)])
def test_health_reflects_executor_and_ros(monkeypatch, ros_ok, alive):
    monkeypatch.setattr(terminal, "require_runtime", lambda: SimpleNamespace(spin_thread=SimpleNamespace(is_alive=lambda: alive)))
    monkeypatch.setattr(terminal.rclpy, "ok", lambda: ros_ok)
    response = terminal.health()
    assert response.status_code == (200 if ros_ok and alive else 503)

@pytest.mark.parametrize("value", ["NaN", "Infinity", "-Infinity"])
def test_nonfinite_referee_input_is_validation_error(value):
    async def check():
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=terminal.app), base_url="http://test") as client:
            for field in ("countdown_s", "penalty_time_s"):
                response = await client.post("/api/referee/command", json={"command": "START", field: value})
                assert response.status_code == 422
    asyncio.run(check())
