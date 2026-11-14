from pathlib import Path
import pytest
import yaml
from hot_switch import HotSwitchManager

ROOT = Path(__file__).resolve().parents[1]

def config():
    return yaml.safe_load((ROOT / "config/terminal.yaml").read_text())

@pytest.mark.parametrize("key,value", [
    ("timeout_s", 0), ("timeout_s", float("nan")), ("timeout_s", True),
    ("switch_log", ""), ("model_target", []), ("model_target", "unknown"),
    ("model_parameter", " "), ("parameter_targets", {"navigation": "relative"}),
    ("parameter_targets", {"perception": 123}),
    ("models", {"bad": None}), ("models", {"bad": {"path": None}}),
    ("parameter_ranges", {"navigation.v": {"min": 2, "max": 1}}),
    ("parameter_ranges", {"navigation.v": {"min": 0, "max": float("inf")}}),
    ("scenes", {"bad": {"parameters": {"unknown": {"v": 1}}}}),
    ("scenes", {"bad": {"parameters": {"navigation": {"v": [1]}}}}),
    ("scenes", {"bad": []}),
])
def test_malformed_config_rejected_before_service_calls(tmp_path, key, value):
    data = config(); data["hot_switch"][key] = value
    path = tmp_path / "bad.yaml"; path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        HotSwitchManager._load_config(path)

def test_valid_config_and_target_normalization(tmp_path):
    data = config(); data["hot_switch"]["parameter_targets"]["navigation"] = " /namespace/nav/ "
    path = tmp_path / "good.yaml"; path.write_text(yaml.safe_dump(data))
    loaded = HotSwitchManager._load_config(path)
    assert loaded["hot_switch"]["parameter_targets"]["navigation"] == "/namespace/nav"

def test_model_readiness_checks_file_instead_of_nonempty_string(tmp_path):
    manager = object.__new__(HotSwitchManager)
    manager.project_root = tmp_path
    (tmp_path / "present.bin").write_bytes(b"test fixture, not a model")
    manager.hot_config = {"models": {"blank": {"path": ""}, "missing": {"path": "no.bin"}, "present": {"path": "present.bin"}}}
    assert manager.get_model_readiness() == {"blank": False, "missing": False, "present": True}
