import threading
import pytest
from hot_switch import HotSwitchManager, HotSwitchError

def manager():
    m = object.__new__(HotSwitchManager)
    m._lock = threading.RLock()
    m.hot_config = {"scenes": {"test": {"parameters": {"navigation": {"v": 1}, "perception": {"c": 2}}}}}
    m._audit_request = lambda *a: None
    m._record_failure = lambda *a: None
    return m

def test_scene_failure_compensates_previous_node():
    m = manager(); calls = []
    def update(alias, values):
        calls.append((alias, values))
        if alias == "perception":
            raise HotSwitchError("simulated service failure")
        return {"v": 0}
    m._set_target_parameters = update
    with pytest.raises(HotSwitchError, match="simulated service failure"):
        m.switch_scene("test")
    assert calls == [("navigation", {"v": 1}), ("perception", {"c": 2}), ("navigation", {"v": 0})]

def test_readback_mismatch_restores_previous_value():
    m = manager(); calls = []
    m._get_parameters = lambda *a: {"v": 0}
    m._set_parameters_atomically = lambda alias, values: calls.append(values)
    with pytest.raises(HotSwitchError, match="校验"):
        m._set_target_parameters("navigation", {"v": 1})
    assert calls == [{"v": 1}, {"v": 0}]
