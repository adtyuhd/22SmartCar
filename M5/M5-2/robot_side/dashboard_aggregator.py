#!/usr/bin/env python3
import fnmatch, json, time
from collections import deque
from pathlib import Path
import psutil, yaml, rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict
from std_msgs.msg import String

class DashboardAggregator(Node):
    def __init__(self):
        super().__init__("dashboard_aggregator")
        path = Path(__file__).resolve().parents[1] / "config" / "dashboard.yaml"
        try: self.config = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        except (OSError, yaml.YAMLError) as exc:
            self.get_logger().warning(f"配置文件读取失败 {path}: {exc}; 使用默认值"); self.config = {}
        pub, rules = self.config.get("publish", {}), self.config.get("fault_rules", {})
        low = rules.get("freq_low", {})
        special = self.config.get("special_topics", {})
        self.task_source, self.ready_source = special.get("task_state", "/task_state"), special.get("system_ready", "/system_ready")
        self.declare_parameter("topic_patterns", self.config.get("topic_patterns", []))
        self.declare_parameter("metrics_hz", int(pub.get("metrics_hz", 10)))
        defaults = {"topic_timeout_s": rules.get("topic_timeout_s", 3.0), "freq_low_hz": low.get("threshold_hz", 10.0), "freq_low_duration_s": low.get("duration_s", 2.0), "cpu_high_pct": rules.get("cpu_high_pct", 85.0), "temp_high_c": rules.get("temp_high_c", 85.0), "voltage_low_v": rules.get("voltage_low_v", 10.5)}
        for name, value in defaults.items(): self.declare_parameter(name, float(value))
        self.patterns = list(self.get_parameter("topic_patterns").value)
        self.critical = rules.get("critical_topic_patterns", self.patterns)
        self.video_enabled = bool(self.config.get("video", {}).get("enable", False))
        self.metrics_pub = self.create_publisher(String, pub.get("metrics_topic", "/dashboard/metrics"), 10); self.faults_pub = self.create_publisher(String, pub.get("faults_topic", "/dashboard/faults"), 10)
        self.state_pub = self.create_publisher(String, pub.get("task_state_topic", "/dashboard/task_state"), 10)
        self.rtt_pub = self.create_publisher(String, "/dashboard/rtt_probe", 10); self.rtt_sub = self.create_subscription(String, "/dashboard/rtt_echo", self._on_rtt_echo, 10)
        self.rtt_pending, self.topic_subscriptions, self.stats, self.active_faults = {}, {}, {}, {}
        self.samples = {key: deque(maxlen=1000) for key in ("confidence", "path_deviation")}
        self.latest = {key: None for key in ("temperature_c", "voltage_v", "network_rtt_ms", "replanning_count")}
        self.system_ready = None; psutil.cpu_percent(None)
        self.add_on_set_parameters_callback(self._parameter_update)
        self.create_timer(1.0, self._discover); self.create_timer(1.0 / max(1.0, float(self.get_parameter("metrics_hz").value)), self._publish_metrics); self.create_timer(1.0, self._send_rtt_probe)
        self.get_logger().info(f"聚合节点已启动，模式数={len(self.patterns)}，指标频率={self.get_parameter('metrics_hz').value}Hz，视频默认关闭")

    def _parameter_update(self, params):
        for param in params:
            if param.name == "topic_patterns":
                if param.type_ != Parameter.Type.STRING_ARRAY:
                    return SetParametersResult(successful=False, reason="topic_patterns 必须是字符串列表")
                self.patterns = list(param.value)
        return SetParametersResult(successful=True)

    def _matches(self, topic, patterns=None):
        return any(fnmatch.fnmatchcase(topic, p) for p in (self.patterns if patterns is None else patterns))

    def _discover(self):
        special = {self.task_source, self.ready_source}
        for topic, types in self.get_topic_names_and_types():
            if topic in self.topic_subscriptions or not types or topic.startswith("/dashboard/") or (topic not in special and not self._matches(topic)) or "sensor_msgs/msg/Image" in types[0] or "sensor_msgs/msg/CompressedImage" in types[0]: continue
            try:
                cls = get_message(types[0])
                sub = self.create_subscription(cls, topic, lambda msg, name=topic: self._on_message(name, msg), qos_profile_sensor_data)
                self.topic_subscriptions[topic] = sub
                self.stats[topic] = {"type": types[0], "discovered": time.monotonic(), "times": deque(maxlen=200), "last": None, "delay_ms": None, "received": 0, "lost": 0, "seq": None, "low_since": None}
                self.get_logger().info(f"已订阅 {topic} [{types[0]}]")
            except Exception as exc: self.get_logger().warning(f"无法订阅 {topic} ({types[0]}): {exc}")
        for topic in list(self.topic_subscriptions):
            if topic not in special and not self._matches(topic):
                self.destroy_subscription(self.topic_subscriptions.pop(topic)); self.stats.pop(topic, None)

    def _fields(self, obj):
        if hasattr(obj, "get_fields_and_field_types"):
            for name in obj.get_fields_and_field_types():
                value = getattr(obj, name); yield name.lower(), value
                if hasattr(value, "get_fields_and_field_types") or (isinstance(value, (list, tuple)) and value and hasattr(value[0], "get_fields_and_field_types")): yield from self._fields(value)
        elif isinstance(obj, (list, tuple)):
            for item in obj: yield from self._fields(item)

    def _on_message(self, topic, msg):
        now, state = time.monotonic(), self.stats[topic]
        state["times"].append(now); state["last"] = now; state["received"] += 1
        stamp = getattr(getattr(msg, "header", None), "stamp", None)
        if stamp:
            delay = self.get_clock().now().nanoseconds / 1e6 - (stamp.sec * 1000 + stamp.nanosec / 1e6)
            state["delay_ms"] = delay if delay >= 0 else None
        for name, value in self._fields(msg):
            if not isinstance(value, (int, float)) or isinstance(value, bool): continue
            value = float(value)
            if name in ("seq", "sequence", "sequence_number"):
                if state["seq"] is not None and value > state["seq"] + 1: state["lost"] += int(value - state["seq"] - 1)
                state["seq"] = value
            if name in ("confidence", "detection_confidence"): self.samples["confidence"].append((now, value))
            if name in ("path_deviation", "cross_track_error", "lateral_error"): self.samples["path_deviation"].append((now, value))
            if name in ("temperature", "temperature_c", "temp_c"): self.latest["temperature_c"] = value
            if name in ("voltage", "voltage_v", "battery_voltage"): self.latest["voltage_v"] = value
            if name in ("rtt", "rtt_ms", "network_rtt_ms", "round_trip_time_ms"): self.latest["network_rtt_ms"] = value
            if name in ("replan_count", "replanning_count", "replans"): self.latest["replanning_count"] = int(value)
        if topic == self.task_source:
            data = msg.data if isinstance(msg, String) else message_to_ordereddict(msg)
            if isinstance(data, str):
                try: data = json.loads(data)
                except ValueError: data = {"data": data}
            self.state_pub.publish(String(data=json.dumps(data)))
        if topic == self.ready_source: self.system_ready = (values := dict(self._fields(msg))).get("ready", values.get("system_ready", getattr(msg, "data", None)))

    def _send_rtt_probe(self):
        try:
            vals = [x.current for group in psutil.sensors_temperatures().values() for x in group if x.current is not None]
            if vals and self.latest["temperature_c"] is None: self.latest["temperature_c"] = max(vals)
        except (AttributeError, OSError, NotImplementedError): pass
        token = str(time.monotonic_ns()); self.rtt_pending[token] = time.monotonic()
        if len(self.rtt_pending) > 10: self.rtt_pending.pop(next(iter(self.rtt_pending)))
        self.rtt_pub.publish(String(data=json.dumps({"token": token})))

    def _on_rtt_echo(self, msg):
        try: sent = self.rtt_pending.pop(json.loads(msg.data).get("token"), None)
        except (ValueError, TypeError): return
        if sent is not None: self.latest["network_rtt_ms"] = (time.monotonic() - sent) * 1000

    def _fault(self, key, active, topic, details=None):
        now = time.time()
        if active and key not in self.active_faults:
            self.active_faults[key] = time.monotonic(); event = {"type": key, "topic": topic, "timestamp": now, "details": details or {}, "self_healed": False}
        elif not active and key in self.active_faults:
            start = self.active_faults.pop(key); event = {"type": key, "topic": topic, "timestamp": now, "duration_s": round(time.monotonic() - start, 3), "self_healed": True}
        else: return
        self.faults_pub.publish(String(data=json.dumps(event)))

    def _publish_metrics(self):
        now, timeout = time.monotonic(), float(self.get_parameter("topic_timeout_s").value)
        topics, delays, control_delays = {}, [], []
        for topic, state in self.stats.items():
            age = now - (state["last"] if state["last"] is not None else state["discovered"])
            active, stale = any(self._matches(topic, [p]) for p in self.critical), age > timeout
            recent = [t for t in state["times"] if t >= now - 1.0]
            freq = len(recent) if state["last"] is not None and not stale else None
            status = "disconnected" if stale else ("no_data" if state["last"] is None else "online")
            loss = round(100 * state["lost"] / max(1, state["received"] + state["lost"]), 2) if state["seq"] is not None else None
            topics[topic] = {"type": state["type"], "status": status, "frequency_hz": freq, "age_s": round(age, 3) if state["last"] is not None else None, "delay_ms": state["delay_ms"], "packet_loss_pct": loss}
            if state["delay_ms"] is not None:
                if topic.startswith("/navigation/"): delays.append(state["delay_ms"])
                if topic.startswith("/control/"): control_delays.append(state["delay_ms"])
            self._fault(f"SENSOR_TIMEOUT:{topic}", active and stale, topic, {"age_s": round(age, 3)})
            low = active and state["last"] is not None and (freq or 0) < float(self.get_parameter("freq_low_hz").value)
            state["low_since"] = (state["low_since"] or now) if low else None
            low = low and now - state["low_since"] >= float(self.get_parameter("freq_low_duration_s").value)
            self._fault(f"SENSOR_LOW_FREQUENCY:{topic}", low, topic, {"frequency_hz": freq})
        cpu, temp, voltage = psutil.cpu_percent(None), self.latest["temperature_c"], self.latest["voltage_v"]
        self._fault("CPU_HIGH", cpu > float(self.get_parameter("cpu_high_pct").value), "system", {"cpu_percent": cpu})
        self._fault("TEMPERATURE_HIGH", temp is not None and temp > float(self.get_parameter("temp_high_c").value), "system", {"temperature_c": temp})
        self._fault("VOLTAGE_LOW", voltage is not None and voltage < float(self.get_parameter("voltage_low_v").value), "system", {"voltage_v": voltage})
        window = lambda key: [v for t, v in self.samples[key] if t >= now - 1.0]
        conf, deviation = window("confidence"), window("path_deviation")
        perception = [v["frequency_hz"] for k, v in topics.items() if k.startswith("/perception/") and v["frequency_hz"] is not None]
        metrics = {"timestamp": time.time(), "topics": topics, "perception_fps": max(perception) if perception else None, "confidence_mean": sum(conf) / len(conf) if conf else None, "navigation_delay_ms": sum(delays) / len(delays) if delays else None, "control_delay_ms": sum(control_delays) / len(control_delays) if control_delays else None, "replanning_count": self.latest["replanning_count"], "path_deviation": sum(deviation) / len(deviation) if deviation else None, "cpu_percent": cpu, "temperature_c": temp, "voltage_v": voltage, "network_rtt_ms": self.latest["network_rtt_ms"], "system_ready": self.system_ready, "video_enabled": self.video_enabled}
        self.metrics_pub.publish(String(data=json.dumps(metrics)))

def main(args=None):
    rclpy.init(args=args)
    node = DashboardAggregator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()