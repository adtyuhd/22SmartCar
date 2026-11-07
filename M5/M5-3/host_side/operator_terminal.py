
#!/usr/bin/env python3
"""M5-3 一体化操作终端：FastAPI + ROS 2。"""

import argparse
import json
import os
import threading
from contextlib import asynccontextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal

import rclpy
import uvicorn
from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse
from pydantic import BaseModel, Field, StrictBool
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from audit import AuditLogger
from hot_switch import HotSwitchError, HotSwitchManager
from referee_bridge import RefereeBridge
from safety import SafetyController
from task_control import TaskControl, SkipUnavailable, SkipRejected


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = PROJECT_ROOT / "config" / "terminal.yaml"


@dataclass
class Runtime:
    host_node: Node
    referee: RefereeBridge
    safety: SafetyController
    hot_switch: HotSwitchManager
    executor: MultiThreadedExecutor
    spin_thread: threading.Thread
    ros_initialized_here: bool
    task_control: TaskControl


_runtime: Runtime | None = None
_runtime_lock = threading.RLock()


def build_runtime() -> Runtime:
    """初始化 ROS 2 节点、业务模块和 executor。"""
    ros_initialized_here = not rclpy.ok()

    if ros_initialized_here:
        rclpy.init(args=None)

    host_node = None
    referee = None
    executor = None
    safety = None

    try:
        user = (
            os.environ.get("M53_OPERATOR")
            or os.environ.get("USER")
            or "operator"
        )
        audit_dir = Path(
            os.environ.get("M53_AUDIT_DIR", str(PROJECT_ROOT))
        ).expanduser().resolve()
        config_path = Path(
            os.environ.get("M53_CONFIG_PATH", str(DEFAULT_CONFIG))
        ).expanduser().resolve()
        duration = float(
            os.environ.get("M53_MATCH_DURATION_S", "180")
        )

        host_node = Node("operator_terminal_host")

        referee = RefereeBridge(
            match_duration_s=duration,
            audit_dir=str(audit_dir),
            audit_user=user,
        )

        safety = SafetyController(
            node=host_node,
            audit_dir=str(audit_dir),
            audit_user=user,
        )

        hot_switch = HotSwitchManager(
            node=host_node,
            config_path=config_path,
            audit_logger=AuditLogger(
                log_dir=str(audit_dir),
                user=user,
            ),
            user=user,
        )

        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(host_node)
        executor.add_node(referee)

        spin_thread = threading.Thread(
            target=executor.spin,
            name="m53-ros-executor",
            daemon=True,
        )
        spin_thread.start()

        return Runtime(
            host_node=host_node,
            referee=referee,
            safety=safety,
            hot_switch=hot_switch,
            executor=executor,
            spin_thread=spin_thread,
            ros_initialized_here=ros_initialized_here,
            task_control=TaskControl(AuditLogger(log_dir=str(audit_dir), user=user)),
        )

    except Exception:
        if safety is not None:
            safety.close()

        if executor is not None:
            try:
                executor.shutdown(timeout_sec=2.0)
            except Exception:
                pass

        for node in (referee, host_node):
            if node is not None:
                try:
                    node.destroy_node()
                except Exception:
                    pass

        if ros_initialized_here and rclpy.ok():
            rclpy.shutdown()

        raise


def referee_snapshot(referee: RefereeBridge) -> dict:
    """在原有线程锁保护下获取比赛状态快照。"""
    with referee._lock:
        return referee._snapshot_locked()


def require_runtime() -> Runtime:
    with _runtime_lock:
        runtime = _runtime

    if runtime is None:
        raise HTTPException(
            status_code=503,
            detail="操作终端尚未完成初始化",
        )

    return runtime


def terminal_status(runtime: Runtime) -> dict:
    """汇总比赛、安全和热切配置状态。"""
    model_config = runtime.hot_switch.hot_config.get("models", {})

    model_paths_configured = {
        name: bool(str(item.get("path", "")).strip())
        for name, item in model_config.items()
    }

    return {
        "ok": True,
        "referee": referee_snapshot(runtime.referee),
        "safety": runtime.safety.get_state(),
        "hot_switch": runtime.hot_switch.get_config_summary(),
        "model_paths_configured": model_paths_configured,
        "task_control": runtime.task_control.capabilities(),
    }


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Web 服务启动时初始化，退出时清理 ROS 2 资源。"""
    global _runtime

    runtime = build_runtime()

    with _runtime_lock:
        _runtime = runtime

    try:
        yield
    finally:
        with _runtime_lock:
            _runtime = None

        runtime.safety.close()

        try:
            runtime.executor.shutdown(timeout_sec=3.0)
        finally:
            runtime.spin_thread.join(timeout=3.0)

            for node in (runtime.referee, runtime.host_node):
                try:
                    node.destroy_node()
                except Exception:
                    pass

            if runtime.ros_initialized_here and rclpy.ok():
                rclpy.shutdown()


app = FastAPI(
    title="M5-3 Operator Terminal",
    version="0.1.0",
    lifespan=lifespan,
)


class RefereeCommandRequest(BaseModel):
    command: Literal[
        "START",
        "PAUSE",
        "RESUME",
        "STOP",
        "PENALTY",
        "SCORE_QUERY",
    ]
    countdown_s: float | None = Field(default=None, gt=0)
    penalty_time_s: float = Field(default=0.0, ge=0)
    reason: str = Field(default="", max_length=300)
    score: Any = None


class ParameterRequest(BaseModel):
    target: Literal["navigation", "perception"]
    parameter: str = Field(min_length=1, max_length=100)
    value: Any
    experiment: str = Field(default="", max_length=100)


class ModelRequest(BaseModel):
    model_id: str = Field(min_length=1, max_length=100)
    experiment: str = Field(default="", max_length=100)


class SceneRequest(BaseModel):
    scene_name: str = Field(min_length=1, max_length=100)
    experiment: str = Field(default="", max_length=100)


class EmergencyStopRequest(BaseModel):
    reason: str = Field(
        default="operator emergency stop",
        max_length=300,
    )


class EmergencyResetRequest(BaseModel):
    manual_confirmation: StrictBool = False


class TaskSkipRequest(BaseModel):
    task_id: str = Field(min_length=1, max_length=100, pattern=r".*\S.*")
    node_id: str = Field(min_length=1, max_length=100, pattern=r".*\S.*")
    reason: str = Field(min_length=1, max_length=300, pattern=r".*\S.*")
    manual_confirmation: StrictBool = False


@app.get("/api/tasks/capabilities")
def task_capabilities():
    return require_runtime().task_control.capabilities()


@app.post("/api/tasks/skip", status_code=202)
def skip_task(request: TaskSkipRequest):
    runtime = require_runtime()
    try:
        return runtime.task_control.request_skip(
            **request.model_dump(),
            match_state=referee_snapshot(runtime.referee)["status"],
            latched=runtime.safety.get_state()["latched"],
        )
    except SkipUnavailable as exc:
        raise HTTPException(status_code=501, detail=str(exc)) from exc
    except SkipRejected as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.get("/health")
def health():
    """检查 Web 进程和 ROS 2 executor。"""
    runtime = require_runtime()

    return {
        "ok": True,
        "ros_ok": rclpy.ok(),
        "executor_thread_alive": runtime.spin_thread.is_alive(),
    }


@app.get("/api/status")
def get_status():
    return terminal_status(require_runtime())


@app.get("/api/config")
def get_config():
    runtime = require_runtime()
    manager = runtime.hot_switch

    return {
        **manager.get_config_summary(),
        "model_paths_configured": {
            name: bool(str(item.get("path", "")).strip())
            for name, item in manager.hot_config["models"].items()
        },
    }


@app.post("/api/referee/command")
def referee_command(request: RefereeCommandRequest):
    runtime = require_runtime()

    payload = {"command": request.command}

    if request.countdown_s is not None:
        payload["countdown_s"] = request.countdown_s

    if request.command == "PENALTY":
        payload["penalty_time_s"] = request.penalty_time_s
        payload["reason"] = request.reason

    if request.command == "SCORE_QUERY" and request.score is not None:
        payload["score"] = request.score

    try:
        runtime.referee.process_command(
            json.dumps(payload, ensure_ascii=False),
            source="ui",
        )
    except Exception as exc:
        raise HTTPException(
            status_code=500,
            detail=f"裁判指令处理异常：{exc}",
        ) from exc

    snapshot = referee_snapshot(runtime.referee)
    error = snapshot.get("error")

    # 急停锁存期间，普通 STOP 仍可停止任务，但不能解除 FAULT。
    safe_command_during_latch = (
        snapshot.get("emergency_latched")
        and request.command in {"STOP", "PENALTY", "SCORE_QUERY"}
    )

    if error and not safe_command_during_latch:
        raise HTTPException(
            status_code=409,
            detail={
                "message": error,
                "status": snapshot,
            },
        )

    return {
        "ok": True,
        "processed": True,
        "status": snapshot,
        "warning": error if safe_command_during_latch else None,
    }


@app.post("/api/parameters")
def set_parameter(request: ParameterRequest):
    runtime = require_runtime()

    try:
        result = runtime.hot_switch.set_parameter(
            alias=request.target,
            name=request.parameter,
            value=request.value,
            source="ui",
            experiment=request.experiment,
        )
        return {"ok": True, "result": result}

    except (HotSwitchError, ValueError) as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.post("/api/models")
def switch_model(request: ModelRequest):
    runtime = require_runtime()

    try:
        result = runtime.hot_switch.switch_model(
            model_id=request.model_id,
            source="ui",
            experiment=request.experiment,
        )

        return {
            "ok": True,
            "result": result,
            "model_load_confirmed": False,
            "message": "模型路径参数已更新；尚未确认模型实际加载成功",
        }

    except (HotSwitchError, ValueError) as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.post("/api/scenes")
def switch_scene(request: SceneRequest):
    runtime = require_runtime()

    try:
        result = runtime.hot_switch.switch_scene(
            scene_name=request.scene_name,
            source="ui",
            experiment=request.experiment,
        )
        return {"ok": True, "result": result}

    except (HotSwitchError, ValueError) as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.post("/api/emergency-stop", status_code=202)
def emergency_stop(request: EmergencyStopRequest):
    runtime = require_runtime()

    requested = runtime.safety.trigger_emergency_stop(
        source="ui",
        user=os.environ.get("M53_OPERATOR", "operator"),
        reason=request.reason,
    )

    if not requested:
        raise HTTPException(
            status_code=409,
            detail="急停请求未能提交",
        )

    return {
        "ok": True,
        "request_sent": True,
        "message": "急停请求已发出，等待小车端确认",
        "safety": runtime.safety.get_state(),
    }


@app.post("/api/emergency-reset", status_code=202)
def emergency_reset(request: EmergencyResetRequest):
    runtime = require_runtime()
    snapshot = referee_snapshot(runtime.referee)

    accepted = runtime.safety.request_reset(
        manual_confirmation=request.manual_confirmation,
        match_state=snapshot["status"],
        source="ui",
        user=os.environ.get("M53_OPERATOR", "operator"),
    )

    if not accepted:
        raise HTTPException(
            status_code=409,
            detail={
                "message": "复位请求被拒绝，请检查人工确认、比赛状态和急停锁存",
                "referee": snapshot,
                "safety": runtime.safety.get_state(),
            },
        )

    return {
        "ok": True,
        "request_sent": True,
        "message": "复位请求已发出，等待小车端反馈；比赛和底盘不会自动恢复",
        "safety": runtime.safety.get_state(),
    }


HTML_PAGE = r"""
<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>M5-3 一体化操作终端</title>
<style>
:root{color-scheme:dark;font-family:system-ui,sans-serif}
body{margin:0;background:#111827;color:#e5e7eb}
main{max-width:1100px;margin:auto;padding:24px}
h1{font-size:25px}
h2{font-size:18px;margin-top:0}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:16px}
.card{background:#1f2937;border:1px solid #374151;border-radius:12px;padding:18px}
button,select,input{font:inherit;border-radius:7px;padding:9px;margin:4px 2px}
button,select,input{border:1px solid #4b5563}
button{background:#374151;color:white;cursor:pointer}
button:disabled{opacity:.4;cursor:not-allowed}
input,select{background:#111827;color:white;max-width:100%}
.danger{background:#991b1b}
.warning{color:#fbbf24}
.muted{color:#9ca3af;font-size:13px}
pre{white-space:pre-wrap;overflow-wrap:anywhere;background:#111827;padding:12px;border-radius:8px}
label{display:block;margin-top:10px}
#notice{min-height:24px;margin:12px 0}
</style>
</head>
<body>
<main>
<h1>M5-3 一体化操作终端</h1>
<p class="muted">操作状态以业务模块和 ROS 2 实际反馈为准。</p>
<div id="notice" role="status" aria-live="polite"></div>

<div class="grid">
<section class="card">
<h2>比赛控制</h2>
<label>倒计时（秒，可留空使用默认值）
<input id="countdown" type="number" min="1" placeholder="使用默认值">
</label>
<div>
<button onclick="command('START')">START</button>
<button onclick="command('PAUSE')">PAUSE</button>
<button onclick="command('RESUME')">RESUME</button>
<button onclick="command('STOP')">STOP</button>
</div>
<label>加罚时间（秒）
<input id="penalty" type="number" min="0" value="0">
</label>
<label>加罚原因
<input id="penaltyReason" maxlength="300" placeholder="原因">
</label>
<button onclick="command('PENALTY')">提交加罚</button>
</section>

<section class="card">
<h2>安全控制</h2>
<p>急停会请求停止底盘并锁存安全状态。</p>
<button class="danger" onclick="emergencyStop()">立即急停</button>
<p class="muted">急停请求已发出不代表底盘已确认停止。</p>
<button onclick="emergencyReset()">人工确认复位</button>
<p class="muted">复位不会自动启动比赛，也不会自动重新激活控制器。</p>
</section>

<section class="card">
<h2>参数热切</h2>
<label>目标节点
<select id="target" onchange="updateParameterOptions()">
<option value="navigation">navigation</option>
<option value="perception">perception</option>
</select>
</label>
<label>参数名称
<select id="parameter"></select>
</label>
<label>参数值
<input id="parameterValue" type="number" step="any" value="0.3">
</label>
<button id="parameterButton" onclick="setParameter()">更新参数</button>
<p class="muted">实际节点未配置时，更新请求会被拒绝。</p>
</section>

<section class="card">
<h2>模型切换</h2>
<label>模型
<select id="model"></select>
</label>
<button id="modelButton" onclick="switchModel()">请求切换</button>
<p class="muted">修改模型路径参数不代表模型已经加载成功。</p>
</section>

<section class="card">
<h2>场景切换</h2>
<label>场景
<select id="scene"></select>
</label>
<button id="sceneButton" onclick="switchScene()">切换场景</button>
<p class="muted">跨节点切换失败时会尝试恢复之前的参数。</p>
</section>

<section class="card">
<h2>人工跳过任务节点</h2>
<p id="taskCapability" class="warning">正在检查调度器能力……</p>
<label>任务 ID <input id="skipTask" maxlength="100"></label>
<label>当前节点 ID <input id="skipNode" maxlength="100"></label>
<label>跳过原因 <input id="skipReason" maxlength="300"></label>
<button id="skipButton" disabled onclick="skipTaskNode()">人工确认并请求跳过</button>
</section>
<section class="card">
<h2>实时状态</h2>
<p id="summary">正在连接……</p>
<button onclick="refresh()">立即刷新</button>
<pre id="status">等待状态数据</pre>
</section>
</div>
</main>

<script>
let terminalConfig = null;

async function api(path, method="GET", body=null) {
  const options = {method, headers: {"Content-Type": "application/json"}};
  if (body !== null) options.body = JSON.stringify(body);

  const response = await fetch(path, options);
  const data = await response.json();

  if (!response.ok) {
    const detail = data.detail ?? data;
    throw new Error(
      typeof detail === "string" ? detail : JSON.stringify(detail)
    );
  }
  return data;
}

function notice(message, error=false) {
  const el = document.getElementById("notice");
  el.textContent = message;
  el.className = error ? "warning" : "";
}

function addOption(select, label, value, disabled=false) {
  const option = document.createElement("option");
  option.textContent = label;
  option.value = value;
  option.disabled = disabled;
  select.appendChild(option);
}

function updateParameterOptions() {
  if (!terminalConfig) return;

  const target = document.getElementById("target").value;
  const parameter = document.getElementById("parameter");
  parameter.replaceChildren();

  const ranges = terminalConfig.parameter_ranges || {};
  const names = Object.keys(ranges)
    .filter(key => key.startsWith(target + "."))
    .map(key => key.slice(target.length + 1));

  for (const name of names) addOption(parameter, name, name);

  const configured = !!terminalConfig.targets?.[target];
  document.getElementById("parameterButton").disabled =
    !configured || names.length === 0;
}

async function loadConfig() {
  terminalConfig = await api("/api/config");

  const target = document.getElementById("target");
  for (const option of target.options) {
    option.disabled = !terminalConfig.targets?.[option.value];
  }

  const model = document.getElementById("model");
  model.replaceChildren();
  for (const name of terminalConfig.models || []) {
    const ready = !!terminalConfig.model_paths_configured?.[name]
      && !!terminalConfig.targets?.perception;
    addOption(model, ready ? name : name + "（未配置）", name, !ready);
  }

  const scene = document.getElementById("scene");
  scene.replaceChildren();
  for (const name of terminalConfig.scenes || []) {
    addOption(scene, name, name);
  }
  if (!scene.options.length) {
    addOption(scene, "尚未配置场景", "", true);
  }

  document.getElementById("modelButton").disabled =
    !terminalConfig.targets?.perception
    || !(terminalConfig.models || []).some(
      name => terminalConfig.model_paths_configured?.[name]
    );

  document.getElementById("sceneButton").disabled =
    !(terminalConfig.scenes || []).length;

  updateParameterOptions();
}

async function refresh() {
  try {
    const data = await api("/api/status");
    const capability = data.task_control;
    document.getElementById("taskCapability").textContent = capability.message;
    document.getElementById("skipButton").disabled =
      !capability.skip_available || data.safety.latched || data.referee.status !== "RUNNING";
    const r = data.referee;
    const s = data.safety;

    document.getElementById("summary").textContent =
      `比赛：${r.status} / ${r.phase}；` +
      `倒计时：${r.countdown_remaining_s ?? "—"} 秒；` +
      `急停锁存：${s.latched ? "是" : "否"}`;

    document.getElementById("status").textContent =
      JSON.stringify(data, null, 2);
  } catch (error) {
    notice("状态读取失败：" + error.message, true);
  }
}

async function command(name) {
  try {
    const body = {command: name};
    const countdown = document.getElementById("countdown").value;

    if (name === "START" && countdown !== "") {
      body.countdown_s = Number(countdown);
    }
    if (name === "PENALTY") {
      body.penalty_time_s = Number(
        document.getElementById("penalty").value
      );
      body.reason = document.getElementById("penaltyReason").value;
    }

    const data = await api("/api/referee/command", "POST", body);
    notice(data.warning || (name + "：指令已处理"));
    await refresh();
  } catch (error) {
    notice(name + " 失败：" + error.message, true);
    await refresh();
  }
}

async function setParameter() {
  try {
    const data = await api("/api/parameters", "POST", {
      target: document.getElementById("target").value,
      parameter: document.getElementById("parameter").value,
      value: Number(document.getElementById("parameterValue").value)
    });
    notice("参数更新成功：" + JSON.stringify(data.result));
  } catch (error) {
    notice("参数更新失败：" + error.message, true);
  }
  await refresh();
}

async function switchModel() {
  const model = document.getElementById("model").value;
  if (!model) return;

  try {
    const data = await api("/api/models", "POST", {model_id: model});
    notice(data.message);
  } catch (error) {
    notice("模型切换失败：" + error.message, true);
  }
  await refresh();
}

async function switchScene() {
  const scene = document.getElementById("scene").value;
  if (!scene) return;

  try {
    await api("/api/scenes", "POST", {scene_name: scene});
    notice("场景切换完成");
  } catch (error) {
    notice("场景切换失败：" + error.message, true);
  }
  await refresh();
}

async function skipTaskNode() {
  if (!window.confirm("确认请求跳过指定的当前节点？请求接受不代表节点已跳过。")) return;
  try {
    const data = await api("/api/tasks/skip", "POST", {
      task_id: document.getElementById("skipTask").value.trim(),
      node_id: document.getElementById("skipNode").value.trim(),
      reason: document.getElementById("skipReason").value.trim(),
      manual_confirmation: true
    });
    notice(data.message);
  } catch (error) {
    notice("跳过请求失败：" + error.message, true);
  }
  await refresh();
}

async function emergencyStop() {
  try {
    const data = await api("/api/emergency-stop", "POST", {
      reason: "operator terminal emergency stop"
    });
    notice(data.message);
  } catch (error) {
    notice("急停请求错误：" + error.message, true);
  }
  await refresh();
}

async function emergencyReset() {
  if (!window.confirm(
    "确认已经检查现场安全，并请求人工复位？复位不会自动恢复比赛或底盘。"
  )) return;

  try {
    const data = await api("/api/emergency-reset", "POST", {
      manual_confirmation: true
    });
    notice(data.message);
  } catch (error) {
    notice("复位请求被拒绝：" + error.message, true);
  }
  await refresh();
}

(async function init() {
  try {
    await loadConfig();
    await refresh();
    setInterval(refresh, 1500);
  } catch (error) {
    notice("终端初始化失败：" + error.message, true);
  }
})();
</script>
</body>
</html>
"""


@app.get("/", response_class=HTMLResponse)
def index():
    return HTML_PAGE


def main():
    parser = argparse.ArgumentParser(description="M5-3 一体化操作终端")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument("--match-duration-s", type=float, default=180.0)
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    args = parser.parse_args()

    if args.port < 1 or args.port > 65535:
        parser.error("端口必须在 1 到 65535 之间")

    if args.match_duration_s <= 0:
        parser.error("比赛默认时长必须大于 0")

    os.environ["M53_MATCH_DURATION_S"] = str(args.match_duration_s)
    os.environ["M53_CONFIG_PATH"] = args.config

    uvicorn.run(
        app,
        host=args.host,
        port=args.port,
        log_level="info",
        workers=1,
    )


if __name__ == "__main__":
    main()
