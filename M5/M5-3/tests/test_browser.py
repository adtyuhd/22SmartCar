"""Real Chromium -> real TCP HTTP -> ROS 2 -> simulated nodes/controller.

These tests never connect to an actual robot or M5-1 task scheduler.
"""
import json
import os
from pathlib import Path
import socket
import subprocess
import time
from types import SimpleNamespace

import httpx
import pytest

pytestmark = pytest.mark.skipif(os.environ.get("M53_RUN_BROWSER_TESTS") != "1",
                                reason="use tools/run_tests.sh --browser")
ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def demo(tmp_path, request):
    from playwright.sync_api import sync_playwright
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    options = getattr(request, "param", {})
    env = {**os.environ, "M53_DEMO_OUTPUT": str(tmp_path / "demo"),
           "TMPDIR": str(ROOT / "work"), "M53_OPERATOR": "browser_acceptance",
           "M53_DEMO_FAIL_SWITCH": "1" if options.get("fail_switch") else "0",
           "M53_DEMO_DOMAIN": "173"}
    url = f"http://127.0.0.1:{port}"
    with (tmp_path / "server.log").open("w") as log:
        process = subprocess.Popen(["bash", "tools/run_demo.sh", "--port", str(port)],
                                   cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 20
            while True:
                if process.poll() is not None:
                    pytest.fail((tmp_path / "server.log").read_text())
                try:
                    if httpx.get(url + "/health", timeout=0.5).status_code == 200:
                        break
                except httpx.HTTPError:
                    pass
                if time.monotonic() >= deadline:
                    pytest.fail("demo startup timeout: " + (tmp_path / "server.log").read_text())
                time.sleep(0.1)
            with sync_playwright() as playwright:
                browser = playwright.chromium.launch(headless=True)
                try:
                    page = browser.new_page(viewport={"width": 1366, "height": 960})
                    errors = []
                    page.on("pageerror", lambda error: errors.append(str(error)))
                    yield SimpleNamespace(page=page, url=url, errors=errors, output=tmp_path / "demo")
                    assert errors == []
                finally:
                    browser.close()
        finally:
            process.terminate()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait(timeout=5)


def ready(demo):
    from playwright.sync_api import expect
    demo.page.goto(demo.url)
    expect(demo.page.locator("#summary")).to_contain_text("READY", timeout=10000)
    expect(demo.page.locator("#mode")).to_contain_text("模拟演示")
    expect(demo.page.locator("#skipButton")).to_be_disabled()


def wait_api(demo, path, predicate):
    deadline = time.monotonic() + 8
    while time.monotonic() < deadline:
        body = demo.page.request.get(demo.url + path).json()
        if predicate(body):
            return body
        time.sleep(0.05)
    pytest.fail(f"timed out waiting for {path}: {body}")


def test_real_browser_controls_parameter_model_scene_and_audit(demo, tmp_path):
    from playwright.sync_api import expect
    from verify_audit import verify_log
    page = demo.page
    ready(demo)
    expect(page.locator("#model")).to_have_value("simulated_path_only")
    page.locator("#experiment").fill("browser-e2e")
    page.locator("#parameterValue").fill("0.42")
    page.locator("#parameterButton").click()
    expect(page.locator("#notice")).to_contain_text("参数更新成功")
    assert page.request.get(demo.url + "/api/simulation").json()["parameters"]["max_velocity"] == 0.42
    with page.expect_response("**/api/models") as response:
        page.locator("#modelButton").click()
    assert response.value.json()["model_load_confirmed"] is False
    expect(page.locator("#notice")).to_contain_text("尚未确认模型实际加载成功")
    page.locator("#scene").select_option("slow")
    page.locator("#sceneButton").click()
    expect(page.locator("#notice")).to_contain_text("场景切换完成")
    snapshot = page.request.get(demo.url + "/api/simulation").json()
    assert snapshot["parameters"]["max_velocity"] == 0.15
    assert snapshot["parameters"]["confidence_threshold"] == 0.8
    page.get_by_role("button", name="START", exact=True).click()
    expect(page.locator("#summary")).to_contain_text("RUNNING")
    page.get_by_role("button", name="PAUSE", exact=True).click()
    expect(page.locator("#summary")).to_contain_text("PAUSED")
    page.get_by_role("button", name="RESUME", exact=True).click()
    expect(page.locator("#summary")).to_contain_text("RUNNING")
    page.locator("#penalty").fill("2")
    page.locator("#penaltyReason").fill("browser simulation")
    page.get_by_role("button", name="提交加罚").click()
    expect(page.locator("#notice")).to_contain_text("PENALTY：指令已处理")
    page.get_by_role("button", name="查询分数").click()
    expect(page.locator("#notice")).to_contain_text("SCORE_QUERY：指令已处理")
    page.get_by_role("button", name="STOP", exact=True).click()
    expect(page.locator("#summary")).to_contain_text("STOPPED")
    records = []
    for path in demo.output.glob("audit_log_*.jsonl"):
        assert verify_log(path)[0] > 0
        records.extend(json.loads(line) for line in path.read_text().splitlines())
    assert any(record.get("details", {}).get("experiment") == "browser-e2e" for record in records)
    assert all(record["user"] == "browser_acceptance" for record in records)
    page.screenshot(path=str(tmp_path / "desktop.png"), full_page=True)
    page.set_viewport_size({"width": 390, "height": 844})
    assert page.evaluate("document.documentElement.scrollWidth <= window.innerWidth")
    page.screenshot(path=str(tmp_path / "mobile.png"), full_page=True)


def test_browser_emergency_roundtrip_over_dds_keeps_controller_inactive(demo):
    from playwright.sync_api import expect
    page = demo.page
    ready(demo)
    page.get_by_role("button", name="START", exact=True).click()
    expect(page.locator("#summary")).to_contain_text("RUNNING")
    page.get_by_role("button", name="立即急停").click()
    wait_api(demo, "/api/status", lambda body: body["referee"]["status"] == "FAULT" and body["safety"]["robot_state"] == "ESTOP_LATCHED")
    guard = wait_api(demo, "/api/simulation", lambda body: body["guard"]["controller_active"] is False)
    assert guard["controller_active"] is False
    with page.expect_response("**/api/referee/command") as response:
        page.get_by_role("button", name="START", exact=True).click()
    assert response.value.status == 409
    page.once("dialog", lambda dialog: dialog.accept())
    page.get_by_role("button", name="人工确认复位", exact=True).click()
    wait_api(demo, "/api/status", lambda body: not body["safety"]["latched"] and body["referee"]["status"] == "STOPPED")
    result = page.request.get(demo.url + "/api/simulation").json()
    assert result["guard"]["awaiting_enable"] is True
    assert result["controller_active"] is False


def test_browser_reconnects_after_startup_failure_and_disables_stale_controls(demo):
    from playwright.sync_api import expect
    page = demo.page
    calls = [0]
    def config_route(route):
        calls[0] += 1
        if calls[0] == 1:
            route.fulfill(status=503, json={"detail": "startup unavailable"})
        else:
            response = route.fetch()
            body = response.json()
            body["targets"]["navigation"] = False
            route.fulfill(response=response, json=body)
    page.route("**/api/config", config_route)
    page.goto(demo.url)
    expect(page.locator("#summary")).to_contain_text("连接中断")
    expect(page.locator("#parameterButton")).to_be_disabled()
    expect(page.locator("#summary")).to_contain_text("READY", timeout=10000)
    expect(page.locator("#target")).to_have_value("perception")
    expect(page.locator("#parameter")).to_have_value("confidence_threshold")
    page.route("**/api/status", lambda route: route.fulfill(status=503, json={"detail": "simulated disconnect"}))
    expect(page.locator("#summary")).to_contain_text("连接中断", timeout=10000)
    for selector in ("#parameterButton", "#modelButton", "#sceneButton", "#skipButton"):
        expect(page.locator(selector)).to_be_disabled()
    page.unroute("**/api/status")
    expect(page.locator("#summary")).to_contain_text("READY", timeout=10000)
    expect(page.locator("#parameterButton")).to_be_enabled()
    expect(page.locator("#notice")).to_have_text("连接已恢复")


def test_empty_or_out_of_range_numeric_input_never_sends_zero(demo):
    from playwright.sync_api import expect
    page = demo.page
    ready(demo)
    requests = []
    page.on("request", lambda request: requests.append(request.url) if "/api/parameters" in request.url else None)
    for value in ("", "5"):
        page.locator("#parameterValue").fill(value)
        page.locator("#parameterButton").click()
        expect(page.locator("#notice")).to_contain_text("请输入有效且在允许范围内的数值")
    assert requests == []
    assert page.request.get(demo.url + "/api/simulation").json()["parameters"]["max_velocity"] == 0.3


@pytest.mark.parametrize("demo", [{"fail_switch": True}], indirect=True)
def test_controller_service_failure_keeps_latches_and_rejects_reset(demo):
    from playwright.sync_api import expect
    page = demo.page
    ready(demo)
    page.get_by_role("button", name="立即急停").click()
    failed = wait_api(demo, "/api/status", lambda body:
        body["referee"]["status"] == "FAULT" and
        body["safety"]["robot_state"] == "ESTOP_FAILED")
    assert failed["safety"]["latched"] is True
    assert failed["safety"]["controller_result"] == "failed"
    before = page.request.get(demo.url + "/api/simulation").json()
    assert before["controller_active"] is True
    assert before["guard"]["controller_active"] is None
    page.once("dialog", lambda dialog: dialog.accept())
    page.get_by_role("button", name="人工确认复位", exact=True).click()
    rejected = wait_api(demo, "/api/status", lambda body:
        body["safety"]["robot_state"] == "RESET_FAILED")
    assert rejected["safety"]["latched"] is True
    assert rejected["referee"]["status"] == "FAULT"
    expect(page.locator("#skipButton")).to_be_disabled()
