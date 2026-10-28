
"""M5-3 操作终端急停与复位 HTTP 接口测试。

测试真实 FastAPI 路由，但用 FakeSafety 和 FakeReferee 隔离
ROS 2 控制通道，避免自动化测试触发真实底盘动作。
"""

import asyncio
import sys
from pathlib import Path
from types import SimpleNamespace

import httpx
import pytest


PROJECT_ROOT = Path(__file__).resolve().parents[1]
HOST_SIDE = PROJECT_ROOT / "host_side"

if str(HOST_SIDE) not in sys.path:
    sys.path.insert(0, str(HOST_SIDE))

import operator_terminal as terminal


class FakeReferee:
    """模拟裁判状态，只提供复位接口需要的比赛状态。"""

    def __init__(self, status="STOPPED"):
        self.status = status


class FakeSafety:
    """模拟安全控制器，记录请求并执行基本安全约束。"""

    def __init__(self):
        self.latched = False
        self.pending_reset = False
        self.accept_stop = True

        self.stop_calls = []
        self.reset_calls = []

    def trigger_emergency_stop(self, source, user, reason):
        self.stop_calls.append({
            "source": source,
            "user": user,
            "reason": reason,
        })

        if not self.accept_stop:
            return False

        # 接受请求后锁存，但这不代表小车端已经确认停车。
        self.latched = True
        return True

    def request_reset(
        self,
        manual_confirmation,
        match_state,
        source,
        user,
    ):
        self.reset_calls.append({
            "manual_confirmation": manual_confirmation,
            "match_state": match_state,
            "source": source,
            "user": user,
        })

        if not self.latched:
            return False

        if manual_confirmation is not True:
            return False

        if match_state not in {"STOPPED", "FAULT"}:
            return False

        # 仅接受复位请求，继续保持急停锁存。
        # 实际解除锁存必须等待小车端的匹配成功反馈。
        self.pending_reset = True
        return True

    def get_state(self):
        return {
            "latched": self.latched,
            "pending_reset": self.pending_reset,
            "robot_state": "ESTOP_LATCHED" if self.latched else "UNKNOWN",
        }


@pytest.fixture
def api(monkeypatch):
    """构造不启动 ROS 2 生命周期的隔离 HTTP 测试环境。"""

    safety = FakeSafety()
    referee = FakeReferee()
    runtime = SimpleNamespace(
        safety=safety,
        referee=referee,
    )

    # 将路由依赖替换为测试运行时，避免初始化真实 ROS 2 节点。
    monkeypatch.setattr(
        terminal,
        "require_runtime",
        lambda: runtime,
    )

    # 复位接口只需要读取当前比赛状态。
    monkeypatch.setattr(
        terminal,
        "referee_snapshot",
        lambda node: {"status": node.status},
    )

    monkeypatch.setenv("M53_OPERATOR", "m53_test_operator")

    def post(path, payload):
        async def send_request():
            transport = httpx.ASGITransport(app=terminal.app)

            async with httpx.AsyncClient(
                transport=transport,
                base_url="http://m53-test",
            ) as client:
                return await client.post(path, json=payload)

        # 使用同步 pytest 测试，无需 pytest-asyncio。
        return asyncio.run(send_request())

    return SimpleNamespace(
        post=post,
        safety=safety,
        referee=referee,
    )


def test_emergency_stop_returns_request_accepted(api):
    """HTTP 202 表示请求已提交，不表示底盘已确认停车。"""

    response = api.post(
        "/api/emergency-stop",
        {"reason": "automated test"},
    )

    assert response.status_code == 202

    body = response.json()
    assert body["ok"] is True
    assert body["request_sent"] is True
    assert "等待小车端确认" in body["message"]

    assert api.safety.latched is True
    assert api.safety.stop_calls == [{
        "source": "ui",
        "user": "m53_test_operator",
        "reason": "automated test",
    }]


def test_emergency_stop_rejected_if_request_cannot_be_submitted(api):
    """安全控制器拒绝提交时，HTTP 接口必须返回 409。"""

    api.safety.accept_stop = False

    response = api.post(
        "/api/emergency-stop",
        {"reason": "automated rejection test"},
    )

    assert response.status_code == 409
    assert response.json()["detail"] == "急停请求未能提交"
    assert api.safety.latched is False
    assert len(api.safety.stop_calls) == 1


def test_reset_requires_manual_confirmation(api):
    """没有明确人工确认时，复位请求必须被拒绝。"""

    api.safety.latched = True
    api.referee.status = "STOPPED"

    response = api.post(
        "/api/emergency-reset",
        {"manual_confirmation": False},
    )

    assert response.status_code == 409
    assert api.safety.latched is True
    assert api.safety.pending_reset is False
    assert api.safety.reset_calls[-1]["manual_confirmation"] is False


def test_reset_rejected_while_match_is_running(api):
    """比赛运行期间，即使人工确认，也不能复位。"""

    api.safety.latched = True
    api.referee.status = "RUNNING"

    response = api.post(
        "/api/emergency-reset",
        {"manual_confirmation": True},
    )

    assert response.status_code == 409
    assert api.safety.latched is True
    assert api.safety.pending_reset is False
    assert api.safety.reset_calls[-1]["match_state"] == "RUNNING"


def test_accepted_reset_does_not_clear_estop_latch(api):
    """接受复位请求后仍锁存，等待小车端反馈。"""

    api.safety.latched = True
    api.referee.status = "STOPPED"

    response = api.post(
        "/api/emergency-reset",
        {"manual_confirmation": True},
    )

    assert response.status_code == 202

    body = response.json()
    assert body["ok"] is True
    assert body["request_sent"] is True
    assert "不会自动恢复" in body["message"]

    assert api.safety.pending_reset is True
    assert api.safety.latched is True
    assert api.safety.reset_calls[-1] == {
        "manual_confirmation": True,
        "match_state": "STOPPED",
        "source": "ui",
        "user": "m53_test_operator",
    }
