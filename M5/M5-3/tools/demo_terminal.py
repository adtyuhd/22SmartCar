#!/usr/bin/env python3
"""Explicit simulator: real DDS/HTTP, fake parameter nodes and controller service.

No physical motion, no model inference and no M5-1 scheduler integration.
"""
import argparse
from contextlib import asynccontextmanager
import os
from pathlib import Path
import sys
import threading

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "host_side"), str(ROOT / "robot_side")]

import yaml
import uvicorn
from controller_manager_msgs.srv import SwitchController
from rclpy.node import Node
from emergency_stop_node import EmergencyStopGuard
import operator_terminal as terminal


def prepare_config(output):
    output = output.resolve()
    if not output.is_relative_to(ROOT / "work"):
        raise ValueError("演示输出目录必须位于 M5-3/work 内")
    output.mkdir(parents=True, exist_ok=True)
    placeholder = output / "SIMULATION_NOT_A_MODEL.txt"
    placeholder.write_text("Simulation path-update fixture; not an inference model.\n")
    config = yaml.safe_load(terminal.DEFAULT_CONFIG.read_text())
    hot = config["hot_switch"]
    hot["parameter_targets"] = {"navigation": "/m53_demo_navigation", "perception": "/m53_demo_perception"}
    hot["models"] = {"missing_unconfigured": {"path": ""},
                     "simulated_path_only": {"path": str(placeholder)}}
    hot["scenes"] = {
        "normal": {"parameters": {"navigation": {"max_velocity": 0.4}, "perception": {"confidence_threshold": 0.5}}},
        "slow": {"parameters": {"navigation": {"max_velocity": 0.15}, "perception": {"confidence_threshold": 0.8}}},
    }
    hot["switch_log"] = str(output / "switch_log.csv")
    path = output / "terminal.yaml"
    path.write_text(yaml.safe_dump(config, allow_unicode=True))
    os.environ["M53_CONFIG_PATH"] = str(path)
    os.environ["M53_AUDIT_DIR"] = str(output)
    return config


class SimulatedController(Node):
    def __init__(self):
        super().__init__("m53_demo_controller")
        self._lock = threading.Lock()
        self.active = True
        self.fail_switch = os.environ.get("M53_DEMO_FAIL_SWITCH") == "1"
        self.service = self.create_service(SwitchController,
            "/controller_manager/switch_controller", self._switch)
        self.guard = EmergencyStopGuard(self)

    def _switch(self, request, response):
        with self._lock:
            response.ok = not self.fail_switch
            if response.ok:
                if self.guard.controller_name in request.deactivate_controllers:
                    self.active = False
                if self.guard.controller_name in request.activate_controllers:
                    self.active = True
        return response

    def snapshot(self):
        with self._lock:
            active = self.active
        return {"simulation": True, "physical_motion_confirmed": False,
                "controller_active": active, "guard": self.guard.get_state()}


_original_lifespan = terminal.lifespan


@asynccontextmanager
async def simulation_lifespan(app):
    if os.environ.get("ROS_DOMAIN_ID") not in {"173", "174"} or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError("模拟器必须运行在本机隔离 DDS 域 173 或 174")
    async with _original_lifespan(app):
        runtime = terminal.require_runtime()
        runtime.simulation = True
        nodes = []
        try:
            nav = Node("m53_demo_navigation"); nodes.append(nav)
            nav.declare_parameter("max_velocity", 0.3)
            perception = Node("m53_demo_perception"); nodes.append(perception)
            perception.declare_parameter("confidence_threshold", 0.5)
            perception.declare_parameter("model_path", "")
            controller = SimulatedController(); nodes.append(controller)
            app.state.simulator = controller
            app.state.simulation_nodes = (nav, perception)
            for node in nodes:
                runtime.executor.add_node(node)
            yield
        finally:
            app.state.simulator = None
            for node in reversed(nodes):
                runtime.executor.remove_node(node)
                node.destroy_node()


terminal.app.router.lifespan_context = simulation_lifespan


@terminal.app.get("/api/simulation")
def simulation_status():
    simulator = getattr(terminal.app.state, "simulator", None)
    if simulator is None:
        raise terminal.HTTPException(status_code=503, detail="模拟器未就绪")
    nav, perception = terminal.app.state.simulation_nodes
    return {**simulator.snapshot(), "parameters": {
        "max_velocity": nav.get_parameter("max_velocity").value,
        "confidence_threshold": perception.get_parameter("confidence_threshold").value,
        "model_path": perception.get_parameter("model_path").value,
    }}


def main():
    parser = argparse.ArgumentParser(description="M5-3 无实车模拟终端")
    parser.add_argument("--port", type=int, default=8082)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("端口必须在 1 到 65535 之间")
    prepare_config(Path(os.environ.get("M53_DEMO_OUTPUT", str(ROOT / "work/demo"))))
    uvicorn.run(terminal.app, host="127.0.0.1", port=args.port, workers=1)


if __name__ == "__main__":
    main()
