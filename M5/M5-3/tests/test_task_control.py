from types import SimpleNamespace
import asyncio
import httpx
import pytest
import operator_terminal as terminal
from task_control import TaskControl, SkipUnavailable, SkipRejected

class Audit:
    def __init__(self):
        self.records = []
    def append(self, **record):
        self.records.append(record)

class Backend:
    def __init__(self):
        self.calls = []
    def request_skip(self, **details):
        self.calls.append(details)
        return "simulated-request"

def arguments(**changes):
    return dict(task_id="t", node_id="n", reason="test", manual_confirmation=True,
                match_state="RUNNING", latched=False, **changes)

def test_default_boundary_rejects_and_audits():
    audit = Audit()
    control = TaskControl(audit)
    assert not control.capabilities()["skip_available"]
    with pytest.raises(SkipUnavailable):
        control.request_skip(**arguments())
    assert audit.records[-1]["result"] == "unavailable"

@pytest.mark.parametrize("field,value", [("manual_confirmation", False),
    ("match_state", "STOPPED"), ("latched", True)])
def test_guard_does_not_call_backend(field, value):
    backend = Backend()
    control = TaskControl(Audit(), backend)
    args = arguments()
    args[field] = value
    with pytest.raises(SkipRejected):
        control.request_skip(**args)
    assert backend.calls == []

def test_backend_acceptance_is_not_execution_confirmation():
    control = TaskControl(Audit(), Backend())
    result = control.request_skip(**arguments())
    assert result["request_sent"]
    assert result["execution_confirmed"] is False

def test_http_unavailable_validation_and_capabilities(monkeypatch):
    runtime = SimpleNamespace(task_control=TaskControl(Audit()), referee=None,
                              safety=SimpleNamespace(get_state=lambda: {"latched": False}))
    monkeypatch.setattr(terminal, "require_runtime", lambda: runtime)
    monkeypatch.setattr(terminal, "referee_snapshot", lambda _: {"status": "RUNNING"})
    async def check():
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=terminal.app),
                                    base_url="http://test") as client:
            assert (await client.get("/api/tasks/capabilities")).json()["skip_available"] is False
            body = dict(task_id="t", node_id="n", reason="test", manual_confirmation=True)
            assert (await client.post("/api/tasks/skip", json=body)).status_code == 501
            for change in ({"node_id": " "}, {"manual_confirmation": "true"}, {"reason": ""}):
                assert (await client.post("/api/tasks/skip", json={**body, **change})).status_code == 422
            page = await client.get("/")
            assert 'id="skipButton" disabled' in page.text
    asyncio.run(check())
