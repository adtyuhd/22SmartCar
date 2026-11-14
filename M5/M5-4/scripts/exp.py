"""Create, run, compare and promote M5-4 experiments."""
import argparse
from contextlib import contextmanager
from copy import deepcopy
from datetime import datetime, timezone
from hashlib import sha256
from html import escape
import json
from pathlib import Path
import re
import statistics
import subprocess
import tempfile
import sys
import threading

import yaml

from quick_review import METRICS, ROOT, cell, number, review

CONFIG = ROOT / "config"
EXPERIMENTS = CONFIG / "experiments"


def digest(path):
    with path.open("rb") as file:
        h = sha256()
        for block in iter(lambda: file.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def read_config(path):
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict):
        raise ValueError(f"{path} 须为 YAML 映射")
    return data


def write_config(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", dir=path.parent, delete=False) as file:
        temporary = Path(file.name)
        try:
            yaml.safe_dump(value, file, allow_unicode=True, sort_keys=False)
            file.close()
            temporary.replace(path)
        finally:
            temporary.unlink(missing_ok=True)


def scene_path(name):
    path = (CONFIG / name).resolve()
    if path.parent != CONFIG or path.suffix != ".yaml":
        raise ValueError("场景必须是 config 下的 .yaml 文件")
    return path


def experiment(name):
    if not re.fullmatch(r"[a-zA-Z0-9][a-zA-Z0-9_.-]*", name):
        raise ValueError("实验名只支持字母、数字、下划线、点和连字符")
    return EXPERIMENTS / (name + ".yaml")


def deep_merge(base, patch):
    result = deepcopy(base)
    for key, value in patch.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = deepcopy(value)
    return result


def git(*args):
    call = subprocess.run(["git", *args], cwd=ROOT, text=True, capture_output=True, check=True)
    return call.stdout.rstrip("\n")


def model_info(scene):
    models = scene.get("models", {})
    if not isinstance(models, dict):
        raise ValueError("models 须为映射")
    result = {}
    for name, model in models.items():
        path = (ROOT / model["path"]).resolve()
        result[name] = {**model, "sha256": digest(path)}
    return result


def resolved(name):
    meta = read_config(experiment(name))
    base = scene_path(meta["base"])
    if digest(base) != meta["base_sha256"]:
        raise ValueError(f"{name} 所依据的基线已变动，请用当前基线重新创建实验")
    return deep_merge(read_config(base), meta["overrides"])


def new(args):
    destination = experiment(args.name)
    if destination.exists():
        raise ValueError("实验名已存在")
    base = scene_path(args.base)
    overrides = {}
    for setting in args.set:
        key, sep, raw = setting.partition("=")
        parts = key.split(".")
        if not sep or not all(parts):
            raise ValueError(f"修改项须为 key=value：{setting}")
        patch = yaml.safe_load(raw)
        for part in reversed(parts):
            patch = {part: patch}
        overrides = deep_merge(overrides, patch)

    def differences(original, changes):
        result = {}
        for key, value in changes.items():
            if isinstance(value, dict) and isinstance(original.get(key), dict):
                diff = differences(original[key], value)
                if diff:
                    result[key] = diff
            elif key not in original or type(original[key]) is not type(value) or original[key] != value:
                result[key] = value
        return result

    write_config(destination, {
        "base": base.name, "base_sha256": digest(base), "base_git": git("rev-parse", "HEAD"),
        "created_at": datetime.now(timezone.utc).isoformat(), "description": args.desc,
        "expected": args.expect, "overrides": differences(read_config(base), overrides),
    })
    print(destination)


def hot_load_ros(scene, settings, directory, name):
    """Apply the resolved configuration through M5-3's audited scene switch."""
    upstream = ROOT.parent / "M5-3"
    terminal = read_config((ROOT / settings.get("terminal", "../M5-3/config/terminal.yaml")).resolve())
    hot = terminal["hot_switch"]
    parameters = {}
    for alias, bindings in settings["bindings"].items():
        values = {}
        for parameter, key in bindings.items():
            value = scene
            for part in key.split("."):
                value = value[part]
            if key.startswith("models.") and key.endswith(".path"):
                value = str((ROOT / value).resolve())
            values[parameter] = value
        parameters[alias] = values
    if not parameters:
        raise ValueError("runner.ros_hot_load.bindings 不能为空")
    hot["scenes"] = {"m54_experiment": {"parameters": parameters}}
    hot["switch_log"] = str(directory / "switch_log.csv")
    terminal_path = directory / "m53_terminal.yaml"
    write_config(terminal_path, terminal)

    sys.path.insert(0, str(upstream / "host_side"))
    try:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from audit import AuditLogger
        from hot_switch import HotSwitchManager

        with _ros_context(rclpy):
            node = rclpy.create_node("m54_experiment")
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            thread = threading.Thread(target=executor.spin, daemon=True)
            thread.start()
            try:
                manager = HotSwitchManager(node, terminal_path, AuditLogger(log_dir=directory / "logs"))
                manager.switch_scene("m54_experiment", source="cli", experiment=name)
            finally:
                executor.shutdown()
                thread.join()
                node.destroy_node()
    except ImportError as exc:
        raise ValueError("ROS 热加载需要 source ROS 2 setup.bash 并使用 ROS 对应的 Python") from exc
    finally:
        sys.path.remove(str(upstream / "host_side"))


@contextmanager
def _ros_context(rclpy):
    rclpy.init()
    try:
        yield
    finally:
        rclpy.shutdown()


def run(args):
    if args.runs < 1:
        raise ValueError("--runs 至少为 1")
    scene = resolved(args.scene or args.name)
    experiment(args.name)
    scene["models"] = model_info(scene)
    commands = scene.get("runner", {})
    ros_settings = commands.get("ros_hot_load")
    if not args.replay_dir:
        if ros_settings is not None and commands.get("hot_load"):
            raise ValueError("runner.hot_load 和 runner.ros_hot_load 只能配置一个")
        for key in (("run",) if ros_settings is not None else ("hot_load", "run")):
            if not isinstance(commands.get(key), list) or not commands[key] or not all(isinstance(v, str) for v in commands[key]):
                raise ValueError(f"现场运行需要配置 runner.{key} 命令数组")
    reports = ROOT / "outputs"
    reports.mkdir(exist_ok=True)
    batch = Path(tempfile.mkdtemp(prefix=f"{args.name}_", dir=reports))
    effective = batch / "effective.yaml"
    write_config(effective, scene)
    config_hash = digest(effective)
    commit = git("rev-parse", "HEAD")
    dirty = git("status", "--porcelain", "--untracked-files=no", "--", ":!outputs")
    results = []
    for i in range(1, args.runs + 1):
        if (git("rev-parse", "HEAD") != commit or model_info(scene) != scene["models"]
                or digest(effective) != config_hash):
            raise ValueError("批次中 Git、模型或配置发生变化")
        directory = (args.replay_dir / f"run_{i:02d}" if args.replay_dir else batch / f"run_{i:02d}").resolve()
        if not args.replay_dir:
            directory.mkdir()
            values = {"config": str(effective), "run_dir": str(directory), "run_index": str(i)}
            if ros_settings is not None:
                hot_load_ros(scene, ros_settings, directory, args.name)
            else:
                subprocess.run([arg.format_map(values) for arg in commands["hot_load"]], cwd=ROOT, check=True)
            subprocess.run([arg.format_map(values) for arg in commands["run"]], cwd=ROOT, check=True)
            missing = [key for key in ("blackbox.mcap", "logs", "switch_log.csv", "task_report.json")
                       if not (directory / key).exists()]
            if missing:
                raise ValueError(f"第 {i} 把运行器未交齐数据：{', '.join(missing)}")
        if (git("rev-parse", "HEAD") != commit or model_info(scene) != scene["models"]
                or digest(effective) != config_hash):
            raise ValueError("批次中 Git、模型或配置发生变化")
        item = review(directory / "blackbox.mcap", directory / "logs",
                      directory / "switch_log.csv", directory / "task_report.json", reports)
        results.append(item)
    statistics_by_metric = {}
    for key in METRICS:
        values = [r["metrics"][key] for r in results if number(r["metrics"][key])]
        statistics_by_metric[key] = {
            "mean": statistics.mean(values) if values else None,
            "variance": statistics.pvariance(values) if values else None,
            "n": len(values),
        }
    successes = sum(r["status"] == "SUCCESS" for r in results)
    write_config(batch / "summary.yaml", {
        "name": args.name, "scene": args.scene or args.name,
        "experiment_sha256": digest(experiment(args.scene or args.name)),
        "git": commit, "tracked_worktree_changes": dirty, "effective_sha256": config_hash,
        "mode": "replay" if args.replay_dir else "live", "runs": args.runs,
        "successes": successes, "success_rate": 100 * successes / args.runs,
        "statistics": statistics_by_metric, "results": results,
    })
    print(f"{successes}/{args.runs} SUCCESS · {batch / 'summary.yaml'}")


def latest(name):
    experiment(name)
    summaries = []
    for path in (ROOT / "outputs").glob(f"{name}_*/summary.yaml"):
        data = read_config(path)
        if data["name"] == name:
            summaries.append((path.stat().st_mtime_ns, path, data))
    if not summaries:
        raise ValueError(f"{name} 没有完成的实验批次")
    _, path, data = max(summaries)
    return path, data


def compare(args):
    names = [name.strip() for name in args.names.split(",")]
    metrics = [key.strip() for key in args.metric.split(",")]
    if set(metrics) - (METRICS.keys() | {"success_rate"}):
        raise ValueError("存在未知对比指标")
    report = ["# 实验对比", "", "| 实验 | 模式 | 成功/总把数 | " + " | ".join(
        f"{key} ({METRICS.get(key, '%')})" for key in metrics) + " |",
        "|---|---|---|" + "---:|" * len(metrics)]
    graph = {key: [] for key in metrics}
    for name in names:
        _, data = latest(name)
        cells = []
        for key in metrics:
            if key == "success_rate":
                value = data[key]
                cells.append(cell(value))
            else:
                stat = data["statistics"][key]
                value = stat["mean"]
                cells.append(f"{cell(value)} / 方差 {cell(stat['variance'])} (n={stat['n']})")
            graph[key].append((name, value))
        report.append(f"| {name} | {data['mode']} | {data['successes']}/{data['runs']} | "
                      + " | ".join(cells) + " |")
    stem = ROOT / "outputs" / f"compare_{datetime.now(timezone.utc):%Y%m%dT%H%M%S_%fZ}"
    options = "".join(f"<option>{escape(key)}</option>" for key in metrics)
    chart = """<!doctype html><html lang="zh"><meta charset="utf-8"><title>实验对比</title>
<style>body{font:16px system-ui;max-width:850px;margin:3rem auto;padding:0 1rem;color:#172235}
label{display:block;margin:1rem 0}meter{width:65%;height:1.4rem;vertical-align:middle}</style>
<h1>实验对比</h1><label>指标 <select id="metric">OPTIONS</select></label><div id="chart"></div>
<script>const data=DATA, selector=document.querySelector('#metric'), chart=document.querySelector('#chart');
function render(){const rows=data[selector.value], max=Math.max(1,...rows.map(r=>Math.abs(r[1]??0)));
chart.replaceChildren(); for(const [name,value] of rows){const row=document.createElement('p');
row.append(name+' '); if(value!==null){const meter=document.createElement('meter');
meter.min=-max;meter.max=max;meter.value=value;meter.title=String(value);row.append(meter);}
row.append(value===null?' 缺数据':' '+value.toFixed(3));chart.append(row);}}
selector.addEventListener('change',render);render();</script></html>"""
    stem.with_suffix(".html").write_text(chart.replace("OPTIONS", options).replace("DATA", json.dumps(graph)))
    report.extend(["", f"[交互图]({stem.name}.html)", ""])
    stem.with_suffix(".md").write_text("\n".join(report))
    print(stem.with_suffix(".md"), stem.with_suffix(".html"))


def tested_scene(name):
    path, summary = latest(name)
    effective = path.with_name("effective.yaml")
    if (summary["scene"] != name or summary["experiment_sha256"] != digest(experiment(name))
            or digest(effective) != summary["effective_sha256"]):
        raise ValueError("实验定义与已运行配置不一致")
    scene = read_config(effective)
    if model_info(scene) != scene["models"]:
        raise ValueError("模型文件与运行快照不一致")
    return scene, summary


def promote(args):
    scene, summary = tested_scene(args.name)
    target = scene_path(args.to)
    filename = target.relative_to(ROOT).as_posix()
    if git("status", "--porcelain", "--", filename):
        raise ValueError("目标场景有本地修改，请先处理")
    if summary["mode"] == "replay" and not args.allow_replay:
        raise ValueError("回放批次验收需显式加 --allow-replay")
    if not summary["successes"]:
        raise ValueError("没有成功运行")
    backup = target.read_bytes() if target.exists() else None
    write_config(target, scene)
    try:
        # --only scopes the commit even when the user already staged unrelated files.
        git("add", "--", filename)
        commit = git("commit", "--only", "-m",
                     f"promote(M5-4): {args.name} ({summary['successes']}/{summary['runs']} successful, {summary['mode']})",
                     "--", filename)
    except subprocess.CalledProcessError:
        git("reset", "-q", "HEAD", "--", filename)
        if backup is None:
            target.unlink()
        else:
            target.write_bytes(backup)
        raise
    print(commit)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    create = sub.add_parser("new")
    create.add_argument("--name", required=True)
    create.add_argument("--base", default="scene_race.yaml")
    create.add_argument("--set", action="append", required=True)
    create.add_argument("--desc", default="")
    create.add_argument("--expect", action="append", default=[])
    execute = sub.add_parser("run")
    execute.add_argument("--name", required=True)
    execute.add_argument("--scene")
    execute.add_argument("--runs", type=int, default=1)
    execute.add_argument("--replay-dir", type=Path)
    comparison = sub.add_parser("compare")
    comparison.add_argument("--names", required=True)
    comparison.add_argument("--metric", required=True)
    promotion = sub.add_parser("promote")
    promotion.add_argument("--name", required=True)
    promotion.add_argument("--to", default="scene_race.yaml")
    promotion.add_argument("--allow-replay", action="store_true")
    args = parser.parse_args()
    try:
        {"new": new, "run": run, "compare": compare, "promote": promote}[args.command](args)
    except (ValueError, OSError, KeyError, TypeError, RuntimeError, subprocess.CalledProcessError, yaml.YAMLError) as exc:
        detail = exc.stderr.strip() if isinstance(exc, subprocess.CalledProcessError) and exc.stderr else str(exc)
        parser.exit(1, f"实验操作失败：{detail}\n")


if __name__ == "__main__":
    main()
