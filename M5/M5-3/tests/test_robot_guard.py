"""Robot guard state machine with a fake controller service, no DDS traffic."""
from concurrent.futures import Future
from types import SimpleNamespace
from emergency_stop_node import EmergencyStopGuard

class Node:
    def __init__(self):
        self.requests = []
        self.futures = []
    def create_publisher(self, *args):
        return SimpleNamespace(publish=lambda msg: None)
    def create_subscription(self, *args):
        return None
    def create_client(self, *args):
        def call(request):
            self.requests.append(request)
            future = Future()
            self.futures.append(future)
            return future
        return SimpleNamespace(service_is_ready=lambda: True, call_async=call)
    def get_logger(self):
        return SimpleNamespace(info=lambda msg: None, warning=lambda msg: None, error=lambda msg: None)

def test_stop_reset_enable_and_stale_enable_cannot_override_new_stop():
    node = Node()
    guard = EmergencyStopGuard(node)
    guard._handle_stop("stop1")
    assert guard.get_state()["latched"]
    assert node.requests[0].deactivate_controllers == [guard.controller_name]
    guard._handle_reset("early", {"manual_confirmation": True, "match_state": "FAULT"})
    assert guard.get_state()["latched"]
    node.futures[0].set_result(SimpleNamespace(ok=True))
    guard._handle_reset("reset", {"manual_confirmation": True, "match_state": "FAULT"})
    assert guard.get_state() == dict(latched=False, controller_active=False,
                                    awaiting_enable=True, transition_pending=None)
    guard._handle_enable("enable", {"manual_confirmation": True, "match_state": "RUNNING"})
    guard._handle_stop("stop2")
    node.futures[1].set_result(SimpleNamespace(ok=True))
    assert guard.get_state()["latched"]
    assert guard.get_state()["transition_pending"] == "STOP"
    node.futures[2].set_result(SimpleNamespace(ok=True))
    assert guard.get_state()["controller_active"] is False

def test_failed_controller_stop_never_allows_reset():
    node = Node()
    guard = EmergencyStopGuard(node)
    guard._handle_stop("stop")
    node.futures[0].set_result(SimpleNamespace(ok=False))
    guard._handle_reset("reset", {"manual_confirmation": True, "match_state": "STOPPED"})
    assert guard.get_state()["latched"]
    assert guard.get_state()["controller_active"] is None
