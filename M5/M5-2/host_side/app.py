#!/usr/bin/env python3
import argparse
import asyncio
import csv
import io
import json
import threading
import time
import webbrowser
from collections import deque
from contextlib import asynccontextmanager
from datetime import datetime
from pathlib import Path

import yaml
import uvicorn
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, JSONResponse, Response

# ROS2 不可用时仍允许 Web 服务启动，并显示连接错误
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    ROS_IMPORT_ERROR = None
except ImportError as exc:
    rclpy = None
    Node = object
    String = None
    ROS_IMPORT_ERROR = str(exc)


ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = ROOT / "config" / "dashboard.yaml"


def load_config():
    try:
        with CONFIG_PATH.open(encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError) as exc:
        raise RuntimeError(f"配置文件读取失败：{exc}") from exc


CONFIG = load_config()
PUBLISH = CONFIG.get("publish", {})
HOST_CONFIG = CONFIG.get("host", {})

METRICS_TOPIC = PUBLISH.get("metrics_topic", "/dashboard/metrics")
FAULTS_TOPIC = PUBLISH.get("faults_topic", "/dashboard/faults")
STATE_TOPIC = PUBLISH.get("task_state_topic", "/dashboard/task_state")

# 控制指令话题必须显式配置，不擅自向未知话题发送指令
COMMAND_TOPIC = HOST_CONFIG.get("command_topic")

HISTORY_SECONDS = max(10, int(HOST_CONFIG.get("history_seconds", 180)))
METRICS_HZ = max(1, int(PUBLISH.get("metrics_hz", 10)))


class DashboardStore:
    """保存最近的指标、故障事件和状态机状态。"""

    def __init__(self):
        self.lock = threading.Lock()
        self.metrics = deque(maxlen=HISTORY_SECONDS * METRICS_HZ)
        self.faults = deque(maxlen=2000)
        self.states = deque(maxlen=1000)

        self.latest_metrics = None
        self.latest_state = None
        self.last_metric_time = None

        self.metric_seq = 0
        self.fault_seq = 0
        self.state_seq = 0
        self.ros_error = ROS_IMPORT_ERROR

    def set_ros_error(self, error):
        with self.lock:
            self.ros_error = error

    def add_metric(self, data):
        if not isinstance(data, dict):
            return

        now = time.time()
        item = {
            "seq": 0,
            "ts": data.get("timestamp", now),
            "data": data,
        }

        with self.lock:
            self.metric_seq += 1
            item["seq"] = self.metric_seq
            self.metrics.append(item)
            self.latest_metrics = item
            self.last_metric_time = time.monotonic()

    def add_fault(self, data):
        if not isinstance(data, dict):
            data = {"message": str(data)}

        with self.lock:
            self.fault_seq += 1
            self.faults.append({
                "seq": self.fault_seq,
                "ts": data.get("timestamp", time.time()),
                "data": data,
            })

    def add_state(self, data):
        if not isinstance(data, (dict, list, str)):
            data = {"value": str(data)}

        with self.lock:
            self.state_seq += 1
            item = {
                "seq": self.state_seq,
                "ts": time.time(),
                "data": data,
            }
            self.states.append(item)
            self.latest_state = item

    def snapshot(self):
        with self.lock:
            connected = (
                self.last_metric_time is not None
                and time.monotonic() - self.last_metric_time <= 3.0
            )

            return {
                "metrics_history": list(self.metrics),
                "faults": list(self.faults),
                "state_history": list(self.states),
                "latest_metrics": self.latest_metrics,
                "latest_task_state": self.latest_state,
                "fault_cursor": self.fault_seq,
                "state_cursor": self.state_seq,
                "connected": connected,
                "commands_enabled": bool(COMMAND_TOPIC),
                "ros_error": self.ros_error,
            }

    def delta(self, fault_cursor, state_cursor):
        with self.lock:
            connected = (
                self.last_metric_time is not None
                and time.monotonic() - self.last_metric_time <= 3.0
            )

            return {
                "latest_metrics": self.latest_metrics,
                "faults": [
                    x for x in self.faults
                    if x["seq"] > fault_cursor
                ],
                "states": [
                    x for x in self.states
                    if x["seq"] > state_cursor
                ],
                "fault_cursor": self.fault_seq,
                "state_cursor": self.state_seq,
                "connected": connected,
                "ros_error": self.ros_error,
                "commands_enabled": bool(COMMAND_TOPIC),
            }


store = DashboardStore()


class RosBridge:
    """通过 ROS2 DDS 接收聚合数据，不接收原始传感器流。"""

    def __init__(self):
        self.node = None
        self.command_pub = None
        self.thread = None
        self.stopping = False

    def start(self):
        if ROS_IMPORT_ERROR:
            return

        self.thread = threading.Thread(
            target=self._run,
            name="dashboard-ros-bridge",
            daemon=True,
        )
        self.thread.start()

    def _on_metrics(self, msg):
        try:
            store.add_metric(json.loads(msg.data))
        except (ValueError, TypeError):
            pass

    def _on_fault(self, msg):
        try:
            store.add_fault(json.loads(msg.data))
        except (ValueError, TypeError):
            store.add_fault({"message": msg.data})

    def _on_state(self, msg):
        try:
            store.add_state(json.loads(msg.data))
        except (ValueError, TypeError):
            store.add_state(msg.data)

    def _on_rtt_probe(self, msg):
        # 将探测消息原样回送，供小车端测量往返时间
        if self.rtt_pub is not None:
            self.rtt_pub.publish(msg)

    def _run(self):
        try:
            if not rclpy.ok():
                rclpy.init(args=None)

            self.node = Node("dashboard_web_bridge")

            self.node.create_subscription(
                String, METRICS_TOPIC, self._on_metrics, 10
            )
            self.node.create_subscription(
                String, FAULTS_TOPIC, self._on_fault, 10
            )
            self.node.create_subscription(
                String, STATE_TOPIC, self._on_state, 10
            )

            # RTT 探测使用 dashboard 的固定协议话题
            self.rtt_pub = self.node.create_publisher(
                String, "/dashboard/rtt_echo", 10
            )
            self.node.create_subscription(
                String, "/dashboard/rtt_probe", self._on_rtt_probe, 10
            )

            if COMMAND_TOPIC:
                self.command_pub = self.node.create_publisher(
                    String, COMMAND_TOPIC, 10
                )

            store.set_ros_error(None)
            self.node.get_logger().info("Web 监控桥接节点已启动")
            rclpy.spin(self.node)

        except Exception as exc:
            if not self.stopping:
                store.set_ros_error(str(exc))

        finally:
            if self.node is not None:
                self.node.destroy_node()
                self.node = None

            if rclpy is not None and rclpy.ok():
                rclpy.try_shutdown()

    def publish_command(self, data):
        if self.command_pub is None:
            return False

        self.command_pub.publish(
            String(data=json.dumps(data, ensure_ascii=False))
        )
        return True

    def stop(self):
        self.stopping = True

        if rclpy is not None and rclpy.ok():
            rclpy.try_shutdown()

        if self.thread is not None:
            self.thread.join(timeout=2)


bridge = RosBridge()


@asynccontextmanager
async def lifespan(app):
    bridge.start()
    yield
    bridge.stop()


app = FastAPI(title="M5-2 Dashboard", lifespan=lifespan)


PAGE = r"""
<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>M5-2 全栈健康监控</title>
<style>
:root {
  color-scheme: dark;
  --bg:#0b1120; --panel:#131d30; --border:#263650;
  --text:#e5edf9; --muted:#91a4c1; --green:#38d996;
  --red:#ff6174; --orange:#ffb454; --blue:#62a5ff;
}
* { box-sizing:border-box; }
body {
  margin:0; background:var(--bg); color:var(--text);
  font:14px/1.5 system-ui,sans-serif;
}
button,select {
  color:var(--text); background:#202f48; border:1px solid var(--border);
  padding:8px 11px; border-radius:7px; cursor:pointer;
}
button:disabled,select:disabled { opacity:.4; cursor:not-allowed; }
button:hover:not(:disabled) { filter:brightness(1.2); }
.topbar {
  display:flex; align-items:center; flex-wrap:wrap; gap:10px;
  padding:14px 18px; border-bottom:1px solid var(--border);
  background:#111a2b; position:sticky; top:0; z-index:5;
}
.brand { font-size:18px; font-weight:700; margin-right:auto; }
.status { display:flex; align-items:center; gap:7px; color:var(--muted); }
.lamp { width:11px;height:11px;border-radius:50%;background:#64748b; }
.lamp.green { background:var(--green);box-shadow:0 0 10px #38d99666; }
.lamp.red { background:var(--red); }
.danger { background:#9c263a; border-color:#f25e70; font-weight:800; }
main {
  padding:12px; display:grid; gap:12px;
  grid-template-columns:minmax(240px, .85fr) minmax(380px, 1.8fr) minmax(250px,.9fr);
  grid-template-rows:auto auto; align-items:stretch;
}
.panel {
  background:var(--panel); border:1px solid var(--border);
  border-radius:11px; padding:14px; min-width:0; overflow:hidden;
}
h2 { margin:0 0 12px; font-size:15px; }
h3 { font-size:13px; color:var(--muted); margin:12px 0 7px; }
.left { grid-column:1; grid-row:1 / 3; }
.metrics { grid-column:2; grid-row:1; }
.faults { grid-column:2; grid-row:2; }
.right { grid-column:3; grid-row:1 / 3; }
.muted { color:var(--muted); }
.small { font-size:12px; }
.node {
  padding:9px 10px; border-radius:7px; margin:7px 0;
  border:1px solid var(--border); background:#1b2940;
  border-left:4px solid #64748b;
}
.node.running { border-left-color:var(--green); }
.node.success { border-left-color:var(--blue); }
.node.failed { border-left-color:var(--red); }
.node.degraded { border-left-color:var(--orange); }
pre {
  white-space:pre-wrap; overflow-wrap:anywhere; max-height:250px;
  overflow:auto; background:#0d1525; padding:10px; border-radius:7px;
  color:#bdcce2; font-size:11px;
}
.chartgrid { display:grid; grid-template-columns:1fr 1fr; gap:10px; }
.chartbox { background:#0e1728; padding:9px; border-radius:8px; min-width:0; }
.charttitle { font-size:12px; color:var(--muted); margin-bottom:4px; }
canvas.chart { display:block; width:100%; height:140px; }
.legend { color:var(--muted); font-size:11px; }
.legend span { margin-right:8px; }
.metricstrip {
  display:flex; flex-wrap:wrap; gap:7px; margin-bottom:10px;
}
.pill { background:#1d2b42; padding:5px 9px; border-radius:6px; }
.pill b { color:#fff; margin-left:5px; }
.faulttools { display:flex; gap:8px; flex-wrap:wrap; margin-bottom:8px; }
.faultlist { max-height:390px; overflow:auto; }
.fault {
  border-left:3px solid var(--red); padding:9px 10px; margin:7px 0;
  background:#1a263a; border-radius:5px;
}
.fault.healed { border-left-color:var(--green); }
.fault strong { color:var(--orange); }
.resource { margin:14px 0; }
.resourcehead { display:flex; justify-content:space-between; gap:8px; }
.bar { height:7px; background:#27364d; border-radius:5px; margin-top:6px; overflow:hidden; }
.barfill { height:100%; width:0; background:var(--blue); transition:width .3s; }
#connection { font-size:12px; color:var(--muted); }
.notice { color:var(--orange); font-size:12px; margin-top:8px; }
@media(max-width:1100px) {
  main { grid-template-columns:minmax(220px,.8fr) minmax(340px,1.5fr); }
  .left { grid-column:1;grid-row:1; }
  .metrics { grid-column:2;grid-row:1; }
  .faults { grid-column:1;grid-row:2; }
  .right { grid-column:2;grid-row:2; }
}
@media(max-width:700px) {
  main { display:flex;flex-direction:column; }
  .chartgrid { grid-template-columns:1fr; }
  .topbar { position:static; }
}
</style>
</head>
<body>
<header class="topbar">
  <div class="brand">M5-2 · 全栈健康监控</div>
  <div class="status">
    <span id="readyLamp" class="lamp"></span>
    <span id="readyText">系统就绪状态未知</span>
  </div>
  <span id="connection">连接中…</span>
  <select id="scene" disabled title="需要配置 host.command_topic">
    <option value="default">默认场景</option>
    <option value="line_following">巡线</option>
    <option value="obstacle_avoidance">避障</option>
    <option value="race">模拟赛</option>
  </select>
  <button onclick="takeScreenshot()">一键截图</button>
  <button id="recordButton" onclick="toggleRecording()">录屏</button>
  <button onclick="exportSnapshot()">导出离线回放</button>
  <button onclick="exportFaults()">导出故障</button>
  <button id="stopButton" class="danger" disabled onclick="emergencyStop()">紧急停车</button>
</header>

<main>
  <section class="panel left">
    <h2>① 任务状态机</h2>
    <div class="muted small">数据来自 /dashboard/task_state</div>
    <div id="taskNodes"><p class="muted">等待状态机数据…</p></div>
    <h3>上下文 / 原始状态</h3>
    <pre id="taskContext">暂无数据</pre>
  </section>

  <section class="panel metrics">
    <h2>② 关键指标实时折线</h2>
    <div class="metricstrip">
      <span class="pill">感知 FPS <b id="fps">—</b></span>
      <span class="pill">置信度 <b id="confidence">—</b></span>
      <span class="pill">导航延迟 <b id="navDelay">—</b></span>
      <span class="pill">控制延迟 <b id="controlDelay">—</b></span>
    </div>
    <div class="chartgrid">
      <div class="chartbox">
        <div class="charttitle">感知 · FPS / 置信度</div>
        <canvas id="chart1" class="chart"></canvas>
        <div class="legend">FPS 与置信度分别归一化，观察趋势</div>
      </div>
      <div class="chartbox">
        <div class="charttitle">导航 · 延迟 / 路径偏离</div>
        <canvas id="chart2" class="chart"></canvas>
        <div class="legend">导航延迟与路径偏离</div>
      </div>
      <div class="chartbox">
        <div class="charttitle">控制 · 延迟 / 重规划</div>
        <canvas id="chart3" class="chart"></canvas>
        <div class="legend">控制延迟与重规划次数</div>
      </div>
      <div class="chartbox">
        <div class="charttitle">系统 · CPU / 温度 / 电压</div>
        <canvas id="chart4" class="chart"></canvas>
        <div class="legend">系统资源变化趋势</div>
      </div>
    </div>
    <div id="metricsNotice" class="notice">等待指标数据</div>
  </section>

  <section class="panel faults">
    <h2>③ 故障事件流</h2>
    <div class="faulttools">
      <select id="faultFilter" onchange="renderFaults()">
        <option value="all">全部事件</option>
        <option value="active">故障触发</option>
        <option value="healed">已自愈</option>
      </select>
      <span class="muted small" id="faultCount"></span>
    </div>
    <div id="faultList" class="faultlist">
      <p class="muted">等待故障事件…</p>
    </div>
  </section>

  <section class="panel right">
    <h2>④ 裁判状态</h2>
    <div id="referee"><p class="muted">等待裁判话题…</p></div>
    <h2 style="margin-top:22px">⑤ 系统资源仪表盘</h2>
    <div class="resource">
      <div class="resourcehead"><span>CPU</span><b id="cpuValue">—</b></div>
      <div class="bar"><div id="cpuBar" class="barfill"></div></div>
    </div>
    <div class="resource">
      <div class="resourcehead"><span>温度</span><b id="tempValue">—</b></div>
      <div class="bar"><div id="tempBar" class="barfill"></div></div>
    </div>
    <div class="resource">
      <div class="resourcehead"><span>电压</span><b id="voltageValue">—</b></div>
      <div class="bar"><div id="voltageBar" class="barfill"></div></div>
    </div>
    <div class="resource">
      <div class="resourcehead"><span>网络 RTT</span><b id="rttValue">—</b></div>
      <div class="bar"><div id="rttBar" class="barfill"></div></div>
    </div>
    <div class="resource">
      <div class="resourcehead"><span>重规划次数</span><b id="replanValue">—</b></div>
    </div>
    <div class="resource">
      <div class="resourcehead"><span>数据链路</span><b id="linkValue">等待数据</b></div>
    </div>
    <div id="rosError" class="notice"></div>
  </section>
</main>

<script>
let historyData = [];
let faults = [];
let states = [];
let latestMetrics = null;
let latestTaskState = null;
let lastMetricSeq = 0;
let lastFaultSeq = 0;
let lastStateSeq = 0;
let socket = null;
let recorder = null;
let recordingStream = null;
let recordingChunks = [];

const colors = ["#62a5ff", "#38d996", "#ffb454", "#d28cff"];

function esc(v) {
  return String(v ?? "").replace(/[&<>"']/g, c => ({
    "&":"&amp;", "<":"&lt;", ">":"&gt;", '"':"&quot;", "'":"&#39;"
  }[c]));
}

function fmt(v, suffix="", digits=2) {
  return typeof v === "number" && Number.isFinite(v)
    ? v.toFixed(digits) + suffix : "—";
}

function setBar(id, value, max) {
  const el = document.getElementById(id);
  if (typeof value !== "number" || !Number.isFinite(value)) {
    el.style.width = "0%";
    return;
  }
  el.style.width = Math.max(0, Math.min(100, value / max * 100)) + "%";
}

function boolValue(v) {
  if (v === true || v === 1) return true;
  if (v === false || v === 0) return false;
  if (typeof v === "string") {
    if (["true","ready","1","online"].includes(v.toLowerCase())) return true;
    if (["false","not_ready","0","offline"].includes(v.toLowerCase())) return false;
  }
  return null;
}

function renderReady(m) {
  const value = boolValue(m?.system_ready);
  const lamp = document.getElementById("readyLamp");
  lamp.className = "lamp" + (value === true ? " green" : value === false ? " red" : "");
  document.getElementById("readyText").textContent =
    value === true ? "系统已就绪" :
    value === false ? "系统未就绪" : "系统就绪状态未知";
}

function renderMetrics() {
  const m = latestMetrics;
  if (!m) return;

  document.getElementById("fps").textContent = fmt(m.perception_fps, " Hz", 1);
  document.getElementById("confidence").textContent = fmt(m.confidence_mean);
  document.getElementById("navDelay").textContent = fmt(m.navigation_delay_ms, " ms", 1);
  document.getElementById("controlDelay").textContent = fmt(m.control_delay_ms, " ms", 1);

  document.getElementById("cpuValue").textContent = fmt(m.cpu_percent, "%", 1);
  document.getElementById("tempValue").textContent = fmt(m.temperature_c, " °C", 1);
  document.getElementById("voltageValue").textContent = fmt(m.voltage_v, " V", 2);
  document.getElementById("rttValue").textContent = fmt(m.network_rtt_ms, " ms", 1);
  document.getElementById("replanValue").textContent =
    m.replanning_count == null ? "—" : String(m.replanning_count);

  setBar("cpuBar", m.cpu_percent, 100);
  setBar("tempBar", m.temperature_c, 100);
  setBar("voltageBar", m.voltage_v, 16);
  setBar("rttBar", m.network_rtt_ms, 100);

  renderReady(m);
  renderReferee(m);
  drawCharts();

  document.getElementById("metricsNotice").textContent =
    "指标更新时间：" + new Date((m.timestamp || Date.now()/1000) * 1000).toLocaleTimeString();
}

function renderReferee(m) {
  const box = document.getElementById("referee");
  const data = m.referee_state || m.referee || {};
  const topics = Object.entries(m.topics || {})
    .filter(([name]) => name.startsWith("/referee/"));

  let html = "";
  if (Object.keys(data).length) {
    html += `<pre>${esc(JSON.stringify(data, null, 2))}</pre>`;
  } else {
    html += `<div class="muted small">当前聚合消息未提供倒计时、阶段和犯规详情。</div>`;
  }

  if (topics.length) {
    html += topics.map(([name, info]) =>
      `<div class="node ${info.status === "online" ? "running" : "degraded"}">
        <b>${esc(name)}</b><br>
        <span class="muted">${esc(info.status)} · 频率 ${
          fmt(info.frequency_hz, " Hz", 1)
        }</span>
      </div>`
    ).join("");
  } else {
    html += `<p class="muted small">尚未发现裁判话题。</p>`;
  }

  box.innerHTML = html;
}

function statusClass(value) {
  const s = String(value || "").toLowerCase();
  if (/fail|error|abort/.test(s)) return "failed";
  if (/degrad|warning|fault|blocked/.test(s)) return "degraded";
  if (/success|complete|finish|done/.test(s)) return "success";
  if (/run|active|execut/.test(s)) return "running";
  return "";
}

function renderTaskState() {
  const box = document.getElementById("taskNodes");
  const context = document.getElementById("taskContext");
  const s = latestTaskState;

  if (s == null) {
    box.innerHTML = '<p class="muted">等待 /dashboard/task_state 数据…</p>';
    context.textContent = "暂无数据";
    return;
  }

  const obj = typeof s === "object" ? s : {state: String(s)};
  let nodes = obj.nodes || obj.tasks || obj.states || obj.machine;

  if (nodes && !Array.isArray(nodes) && typeof nodes === "object") {
    nodes = Object.entries(nodes).map(([name, value]) =>
      typeof value === "object" ? {name, ...value} : {name, status:value}
    );
  }

  if (!Array.isArray(nodes)) {
    const name = obj.current_node || obj.current || obj.node || obj.name;
    nodes = name ? [{
      name,
      status: obj.status || obj.state || "unknown",
      progress: obj.progress
    }] : [];
  }

  if (!nodes.length) {
    box.innerHTML = '<p class="muted">已收到状态数据，暂未识别节点列表。</p>';
  } else {
    box.innerHTML = nodes.map(n => {
      const name = n.name || n.id || n.node || n.label || "未命名节点";
      const status = n.status || n.state || "unknown";
      const progress = typeof n.progress === "number"
        ? ` · 进度 ${fmt(n.progress, "%", 1)}` : "";
      return `<div class="node ${statusClass(status)}">
        <b>${esc(name)}</b><br>
        <span class="muted">${esc(status)}${progress}</span>
      </div>`;
    }).join("");
  }

  context.textContent = JSON.stringify(obj.context || obj, null, 2);
}

function renderFaults() {
  const filter = document.getElementById("faultFilter").value;
  const selected = faults.filter(f => {
    const healed = Boolean(f.data?.self_healed);
    return filter === "all" ||
      (filter === "healed" && healed) ||
      (filter === "active" && !healed);
  }).slice().reverse();

  document.getElementById("faultCount").textContent =
    `${selected.length} 条事件`;

  const box = document.getElementById("faultList");
  if (!selected.length) {
    box.innerHTML = '<p class="muted">当前筛选条件下没有故障事件。</p>';
    return;
  }

  box.innerHTML = selected.map(f => {
    const d = f.data || {};
    const healed = Boolean(d.self_healed);
    const time = new Date((f.ts || Date.now()/1000) * 1000).toLocaleTimeString();
    return `<div class="fault ${healed ? "healed" : ""}">
      <div><strong>${healed ? "已自愈" : "故障触发"}</strong> · ${esc(time)}</div>
      <div>${esc(d.type || d.message || "未知事件")}</div>
      <div class="muted small">${esc(d.topic || "")}</div>
      ${d.duration_s != null
        ? `<div class="muted small">持续 ${esc(d.duration_s)} 秒</div>` : ""}
    </div>`;
  }).join("");
}

function drawChart(id, series) {
  const canvas = document.getElementById(id);
  const rect = canvas.getBoundingClientRect();
  if (!rect.width) return;

  const ratio = window.devicePixelRatio || 1;
  canvas.width = Math.round(rect.width * ratio);
  canvas.height = Math.round(140 * ratio);

  const ctx = canvas.getContext("2d");
  ctx.scale(ratio, ratio);

  const w = rect.width, h = 140;
  const left = 28, right = 8, top = 8, bottom = 17;
  const pw = w - left - right, ph = h - top - bottom;
  ctx.clearRect(0, 0, w, h);

  ctx.strokeStyle = "#29364b";
  ctx.lineWidth = 1;
  for (let i=0; i<=4; i++) {
    const y = top + ph * i / 4;
    ctx.beginPath(); ctx.moveTo(left,y); ctx.lineTo(w-right,y); ctx.stroke();
  }

  const points = historyData.slice(-120);
  if (points.length < 2) {
    ctx.fillStyle = "#91a4c1";
    ctx.fillText("等待历史数据…", left + 8, h/2);
    return;
  }

  series.forEach((s, index) => {
    const values = points.map(p => {
      const v = p.data?.[s.key];
      return typeof v === "number" && Number.isFinite(v) ? v : null;
    });
    const valid = values.filter(v => v !== null);
    if (!valid.length) return;

    let min = Math.min(...valid), max = Math.max(...valid);
    if (min === max) { min -= 1; max += 1; }

    ctx.strokeStyle = colors[index % colors.length];
    ctx.lineWidth = 1.8;
    ctx.beginPath();
    let started = false;

    values.forEach((v, i) => {
      if (v === null) { started = false; return; }
      const x = left + pw * i / (values.length - 1);
      const y = top + ph * (1 - (v-min)/(max-min));
      if (!started) { ctx.moveTo(x,y); started = true; }
      else ctx.lineTo(x,y);
    });

    ctx.stroke();
  });

  ctx.fillStyle = "#91a4c1";
  ctx.font = "10px system-ui";
  ctx.fillText(`${points.length} 个采样点`, left, h-3);
}

function drawCharts() {
  drawChart("chart1", [
    {key:"perception_fps"},
    {key:"confidence_mean"}
  ]);
  drawChart("chart2", [
    {key:"navigation_delay_ms"},
    {key:"path_deviation"}
  ]);
  drawChart("chart3", [
    {key:"control_delay_ms"},
    {key:"replanning_count"}
  ]);
  drawChart("chart4", [
    {key:"cpu_percent"},
    {key:"temperature_c"},
    {key:"voltage_v"}
  ]);
}

function updateConnection(data) {
  const connected = Boolean(data.connected);
  document.getElementById("linkValue").textContent =
    connected ? "在线" : "失联 / 等待数据";

  document.getElementById("connection").textContent =
    connected ? "DDS 指标在线" : "DDS 指标未连接";

  document.getElementById("rosError").textContent =
    data.ros_error ? "ROS2 连接错误：" + data.ros_error : "";

  document.getElementById("stopButton").disabled = !data.commands_enabled;
  document.getElementById("scene").disabled = !data.commands_enabled;
}

function applySnapshot(data) {
  historyData = data.metrics_history || [];
  faults = data.faults || [];
  states = data.state_history || [];

  latestMetrics = data.latest_metrics?.data || null;
  latestTaskState = data.latest_task_state?.data ?? null;

  lastMetricSeq = data.latest_metrics?.seq || 0;
  lastFaultSeq = data.fault_cursor || 0;
  lastStateSeq = data.state_cursor || 0;

  updateConnection(data);
  renderMetrics();
  renderTaskState();
  renderFaults();
}

function applyUpdate(data) {
  if (data.latest_metrics &&
      data.latest_metrics.seq > lastMetricSeq) {
    lastMetricSeq = data.latest_metrics.seq;
    historyData.push(data.latest_metrics);
    latestMetrics = data.latest_metrics.data;
    renderMetrics();
  }

  for (const f of data.faults || []) {
    if (f.seq > lastFaultSeq) {
      faults.push(f);
      lastFaultSeq = f.seq;
    }
  }

  for (const s of data.states || []) {
    if (s.seq > lastStateSeq) {
      states.push(s);
      lastStateSeq = s.seq;
      latestTaskState = s.data;
    }
  }

  if ((data.states || []).length) renderTaskState();
  if ((data.faults || []).length) renderFaults();
  updateConnection(data);
}

function connect() {
  const protocol = location.protocol === "https:" ? "wss:" : "ws:";
  socket = new WebSocket(`${protocol}//${location.host}/ws`);

  socket.onopen = () => {
    document.getElementById("connection").textContent = "WebSocket 已连接";
  };

  socket.onmessage = event => {
    try {
      const data = JSON.parse(event.data);
      if (data.type === "snapshot") applySnapshot(data);
      else if (data.type === "update") applyUpdate(data);
    } catch (err) {
      console.error("WebSocket 数据解析失败", err);
    }
  };

  socket.onclose = () => {
    document.getElementById("connection").textContent = "连接断开，正在重连…";
    setTimeout(connect, 1500);
  };

  socket.onerror = () => socket.close();
}

async function sendCommand(command) {
  const response = await fetch("/api/command", {
    method:"POST",
    headers:{"Content-Type":"application/json"},
    body:JSON.stringify(command)
  });
  const result = await response.json();

  if (!response.ok) throw new Error(result.error || "命令发送失败");
  return result;
}

async function emergencyStop() {
  if (!confirm("确认发送紧急停车指令？")) return;

  try {
    await sendCommand({action:"emergency_stop"});
    alert("停车指令已发送。请同时确认底盘控制器已收到指令。");
  } catch (err) {
    alert("指令发送失败：" + err.message);
  }
}

document.getElementById("scene").addEventListener("change", async event => {
  try {
    await sendCommand({action:"set_scene", scene:event.target.value});
  } catch (err) {
    alert("场景切换失败：" + err.message);
  }
});

function downloadBlob(blob, filename) {
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = filename;
  a.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

async function takeScreenshot() {
  let stream;

  try {
    stream = await navigator.mediaDevices.getDisplayMedia({video:true, audio:false});
    const video = document.createElement("video");
    video.srcObject = stream;
    await video.play();
    await new Promise(resolve => setTimeout(resolve, 250));

    const canvas = document.createElement("canvas");
    canvas.width = video.videoWidth;
    canvas.height = video.videoHeight;
    canvas.getContext("2d").drawImage(video, 0, 0);

    canvas.toBlob(blob => {
      if (blob) downloadBlob(blob, "dashboard_screenshot.png");
      stream.getTracks().forEach(t => t.stop());
    }, "image/png");

  } catch (err) {
    if (stream) stream.getTracks().forEach(t => t.stop());
    alert("截图失败：" + err.message);
  }
}

async function toggleRecording() {
  const button = document.getElementById("recordButton");

  if (recorder && recorder.state === "recording") {
    recorder.stop();
    return;
  }

  try {
    recordingStream = await navigator.mediaDevices.getDisplayMedia({
      video:true, audio:false
    });

    recordingChunks = [];
    recorder = new MediaRecorder(recordingStream);
    recorder.ondataavailable = e => {
      if (e.data.size) recordingChunks.push(e.data);
    };

    recorder.onstop = () => {
      downloadBlob(
        new Blob(recordingChunks, {type:"video/webm"}),
        "dashboard_recording.webm"
      );
      recordingStream.getTracks().forEach(t => t.stop());
      recordingStream = null;
      button.textContent = "录屏";
    };

    recorder.start();
    button.textContent = "停止录屏";

  } catch (err) {
    if (recordingStream) recordingStream.getTracks().forEach(t => t.stop());
    recordingStream = null;
    alert("录屏失败：" + err.message);
  }
}

function exportSnapshot() {
  location.href = "/api/export";
}

function exportFaults() {
  location.href = "/api/faults.csv";
}

window.addEventListener("resize", drawCharts);
connect();
</script>
</body>
</html>
"""


@app.get("/", response_class=HTMLResponse)
async def index():
    return HTMLResponse(PAGE)


@app.get("/api/health")
async def health():
    return store.snapshot()


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()

    snapshot = store.snapshot()
    await ws.send_json({"type": "snapshot", **snapshot})

    fault_cursor = snapshot["fault_cursor"]
    state_cursor = snapshot["state_cursor"]

    try:
        while True:
            data = store.delta(fault_cursor, state_cursor)
            await ws.send_json({"type": "update", **data})

            fault_cursor = data["fault_cursor"]
            state_cursor = data["state_cursor"]

            await asyncio.sleep(0.1)

    except WebSocketDisconnect:
        pass


@app.post("/api/command")
async def api_command(request: Request):
    try:
        body = await request.json()
    except (ValueError, TypeError):
        return JSONResponse(
            {"error": "无效的 JSON 请求"},
            status_code=400,
        )

    action = body.get("action")

    if action == "emergency_stop":
        command = {
            "type": "emergency_stop",
            "timestamp": time.time(),
            "source": "dashboard",
        }

    elif action == "set_scene":
        scene = body.get("scene")
        allowed = {
            "default", "line_following",
            "obstacle_avoidance", "race",
        }

        if scene not in allowed:
            return JSONResponse(
                {"error": "未知场景"},
                status_code=400,
            )

        command = {
            "type": "set_scene",
            "scene": scene,
            "timestamp": time.time(),
            "source": "dashboard",
        }

    else:
        return JSONResponse(
            {"error": "不支持的控制指令"},
            status_code=400,
        )

    if not bridge.publish_command(command):
        return JSONResponse(
            {
                "error": (
                    "控制指令尚未启用。请先在 dashboard.yaml 的 "
                    "host 部分配置 command_topic。"
                )
            },
            status_code=503,
        )

    return {"ok": True, "command": command}


@app.get("/api/export")
async def export_snapshot():
    data = store.snapshot()
    payload = json.dumps(
        data,
        ensure_ascii=False,
        allow_nan=False,
    ).replace("</", "<\\/")

    html = f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>M5-2 离线回放</title>
<style>
body{{font:15px system-ui;background:#0b1120;color:#e5edf9;padding:20px}}
button,input{{padding:8px;margin:5px}}
pre{{white-space:pre-wrap;overflow-wrap:anywhere;background:#172238;padding:14px}}
.panel{{background:#131d30;padding:14px;border-radius:10px;margin:12px 0}}
</style>
</head>
<body>
<h1>M5-2 赛后数据 · 离线回放</h1>
<div class="panel">
<p>历史指标：<span id="count"></span> 条</p>
<button onclick="play()">播放</button>
<button onclick="pause()">暂停</button>
<input id="slider" type="range" min="0" max="0" value="0"
 oninput="showPoint(+this.value)">
<span id="position"></span>
</div>
<div class="panel">
<h2>当前采样点</h2>
<pre id="point">暂无数据</pre>
</div>
<div class="panel">
<h2>故障事件</h2>
<pre id="faults"></pre>
</div>
<div class="panel">
<h2>状态机历史</h2>
<pre id="states"></pre>
</div>
<script>
const SNAPSHOT = {payload};
const points = SNAPSHOT.metrics_history || [];
const slider = document.getElementById("slider");
slider.max = Math.max(0, points.length - 1);
document.getElementById("count").textContent = points.length;
document.getElementById("faults").textContent =
 JSON.stringify(SNAPSHOT.faults || [], null, 2);
document.getElementById("states").textContent =
 JSON.stringify(SNAPSHOT.state_history || [], null, 2);
let timer = null;
function showPoint(i) {{
 i = Math.max(0, Math.min(i, points.length - 1));
 slider.value = i;
 document.getElementById("position").textContent =
  points.length ? (i+1) + " / " + points.length : "暂无数据";
 document.getElementById("point").textContent =
  JSON.stringify(points[i] || null, null, 2);
}}
function play() {{
 pause();
 timer = setInterval(() => {{
  const next = (+slider.value + 1) % points.length;
  showPoint(next);
 }}, 100);
}}
function pause() {{
 if (timer) clearInterval(timer);
 timer = null;
}}
showPoint(0);
</script>
</body>
</html>"""

    filename = "dashboard_snapshot_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".html"

    return HTMLResponse(
        html,
        headers={
            "Content-Disposition": f'attachment; filename="{filename}"'
        },
    )


@app.get("/api/faults.csv")
async def export_faults():
    output = io.StringIO()
    writer = csv.writer(output)

    writer.writerow([
        "timestamp", "type", "topic",
        "self_healed", "duration_s", "details",
    ])

    for item in store.snapshot()["faults"]:
        data = item.get("data", {})
        writer.writerow([
            item.get("ts"),
            data.get("type", ""),
            data.get("topic", ""),
            data.get("self_healed", False),
            data.get("duration_s", ""),
            json.dumps(data.get("details", {}), ensure_ascii=False),
        ])

    filename = "dashboard_faults_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv"

    return Response(
        content=output.getvalue(),
        media_type="text/csv; charset=utf-8",
        headers={
            "Content-Disposition": f'attachment; filename="{filename}"'
        },
    )


def main():
    parser = argparse.ArgumentParser(description="M5-2 Web 监控面板")
    parser.add_argument(
        "--host",
        default=HOST_CONFIG.get("address", "0.0.0.0"),
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8080,
    )
    parser.add_argument(
        "--no-browser",
        action="store_true",
        help="不自动打开浏览器",
    )
    args = parser.parse_args()

    url = f"http://127.0.0.1:{args.port}"
    print(f"监控面板地址：{url}")
    print(f"ROS2 指标话题：{METRICS_TOPIC}")
    print(f"ROS2 故障话题：{FAULTS_TOPIC}")
    print(f"ROS2 状态话题：{STATE_TOPIC}")

    if not args.no_browser:
        threading.Timer(1.5, lambda: webbrowser.open(url)).start()

    uvicorn.run(
        app,
        host=args.host,
        port=args.port,
        log_level="info",
    )


if __name__ == "__main__":
    main()