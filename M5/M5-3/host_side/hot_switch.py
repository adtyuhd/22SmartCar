
#!/usr/bin/env python3
"""M5-3 热切管理器：参数更新、模型切换、场景切换与审计。"""

import csv
import fcntl
import json
import math
import os
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import yaml
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.parameter import Parameter

from audit import AuditLogger


class HotSwitchError(RuntimeError):
    """热切操作失败。"""


class ParameterUpdateRejected(HotSwitchError):
    """目标节点拒绝了原子参数更新。"""


class HotSwitchManager:
    """通过 ROS 2 参数服务管理运行中节点的参数。

    前提：
    - 传入的 node 必须属于一个正在运行的 ROS 2 context。
    - ROS 2 executor 必须在其他线程持续处理服务响应。
    - 跨节点场景切换不是全局原子事务，失败时采用补偿性回滚。
    """

    def __init__(
        self,
        node,
        config_path=None,
        audit_logger=None,
        user=None,
    ):
        self.node = node
        self.project_root = Path(__file__).resolve().parents[1]

        self.config_path = Path(
            config_path
            if config_path is not None
            else self.project_root / "config" / "terminal.yaml"
        ).expanduser().resolve()

        self.config = self._load_config(self.config_path)
        self.hot_config = self.config["hot_switch"]

        operator_config = self.config.get("operator", {})
        user_env = operator_config.get("user_env", "M53_OPERATOR")

        self.user = (
            user
            or os.environ.get(user_env)
            or os.environ.get("USER", "operator")
        )

        self.timeout_s = float(self.hot_config["timeout_s"])

        if not math.isfinite(self.timeout_s) or self.timeout_s <= 0:
            raise ValueError("timeout_s 必须是有限的正数")

        self.audit = audit_logger or AuditLogger(
            log_dir=self.project_root,
            user=self.user,
        )

        log_path = Path(
            self.hot_config.get("switch_log", "switch_log.csv")
        ).expanduser()

        if not log_path.is_absolute():
            log_path = self.project_root / log_path

        self.switch_log_path = log_path.resolve()
        self.switch_log_path.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        self._clients = {}
        self._lock = threading.RLock()

    @staticmethod
    def _load_config(path):
        """读取并检查热切配置。"""
        if not path.is_file():
            raise FileNotFoundError(f"配置文件不存在：{path}")

        with path.open("r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream)

        if not isinstance(config, dict):
            raise ValueError("配置文件根节点必须是字典")

        if config.get("version") != 1:
            raise ValueError("不支持的终端配置版本")

        hot = config.get("hot_switch")

        if not isinstance(hot, dict):
            raise ValueError("缺少 hot_switch 配置")

        required = (
            "timeout_s",
            "switch_log",
            "parameter_targets",
            "models",
            "scenes",
            "parameter_ranges",
        )

        for key in required:
            if key not in hot:
                raise ValueError(f"热切配置缺少字段：{key}")

        for key in (
            "parameter_targets",
            "models",
            "scenes",
            "parameter_ranges",
        ):
            if not isinstance(hot[key], dict):
                raise ValueError(f"{key} 必须是字典")

        return config

    def get_config_summary(self):
        """返回适合 Web 界面展示的配置摘要。"""
        return {
            "models": sorted(self.hot_config["models"]),
            "scenes": sorted(self.hot_config["scenes"]),
            "targets": {
                alias: bool(name)
                for alias, name in
                self.hot_config["parameter_targets"].items()
            },
            "parameter_ranges": self.hot_config["parameter_ranges"],
            "timeout_s": self.timeout_s,
        }

    def _target_name(self, alias):
        """获取已配置的 ROS 2 节点绝对名称。"""
        target = self.hot_config["parameter_targets"].get(alias)

        if not isinstance(target, str) or not target.strip():
            raise HotSwitchError(
                f"目标 {alias!r} 尚未配置实际 ROS 2 节点名称"
            )

        target = target.strip()

        if not target.startswith("/") or target == "/":
            raise HotSwitchError(
                f"目标节点必须配置有效的绝对名称：{target!r}"
            )

        return target.rstrip("/")

    def _client(self, alias, service_kind):
        """按目标节点和服务类型缓存客户端。"""
        target = self._target_name(alias)
        key = (target, service_kind)

        if key not in self._clients:
            if service_kind == "get":
                service_type = GetParameters
                service_name = f"{target}/get_parameters"
            elif service_kind == "set_atomic":
                service_type = SetParametersAtomically
                service_name = f"{target}/set_parameters_atomically"
            else:
                raise ValueError(f"未知服务类型：{service_kind}")

            self._clients[key] = self.node.create_client(
                service_type,
                service_name,
            )

        return self._clients[key]

    def _await_future(self, future):
        """等待异步服务结果；executor 必须在其他线程运行。"""
        deadline = time.monotonic() + self.timeout_s

        while not future.done():
            if time.monotonic() >= deadline:
                raise HotSwitchError("ROS 2 参数服务调用超时")

            time.sleep(0.01)

        try:
            result = future.result()
        except Exception as exc:
            raise HotSwitchError(
                f"ROS 2 参数服务调用异常：{exc}"
            ) from exc

        if result is None:
            raise HotSwitchError("ROS 2 参数服务没有返回结果")

        return result

    def _call_service(self, client, request):
        """等待服务可用，然后发送请求。"""
        deadline = time.monotonic() + self.timeout_s

        while not client.wait_for_service(timeout_sec=0.1):
            if time.monotonic() >= deadline:
                raise HotSwitchError(
                    f"参数服务不可用：{client.srv_name}"
                )

        return self._await_future(client.call_async(request))

    @staticmethod
    def _decode_scalar(value):
        """将 ROS 2 ParameterValue 解码为 Python 标量。

        注意：value.type 是整数，而 Parameter.Type 是枚举。
        必须先将整数转换成枚举成员，再进行类型比较。
        """
        try:
            parameter_type = Parameter.Type(value.type)
        except (ValueError, TypeError) as exc:
            raise HotSwitchError(
                f"未知的 ROS 2 参数类型：{value.type}"
            ) from exc

        if parameter_type == Parameter.Type.BOOL:
            return value.bool_value

        if parameter_type == Parameter.Type.INTEGER:
            return value.integer_value

        if parameter_type == Parameter.Type.DOUBLE:
            return value.double_value

        if parameter_type == Parameter.Type.STRING:
            return value.string_value

        if parameter_type == Parameter.Type.NOT_SET:
            raise HotSwitchError("参数未声明或尚未设置")

        raise HotSwitchError(
            f"暂不支持此参数类型：{parameter_type.name}"
        )

    def _get_parameters(self, alias, names):
        """读取指定节点的一组参数。"""
        client = self._client(alias, "get")

        request = GetParameters.Request()
        request.names = list(names)

        response = self._call_service(client, request)

        if len(response.values) != len(names):
            raise HotSwitchError("参数读取返回数量不匹配")

        return {
            name: self._decode_scalar(value)
            for name, value in zip(names, response.values)
        }

    def _set_parameters_atomically(self, alias, values):
        """通过标准原子参数服务提交一组参数。"""
        if not values:
            raise ValueError("参数更新列表不能为空")

        client = self._client(alias, "set_atomic")
        request = SetParametersAtomically.Request()

        request.parameters = [
            Parameter(name=name, value=value).to_parameter_msg()
            for name, value in values.items()
        ]

        response = self._call_service(client, request)

        if not response.result.successful:
            raise ParameterUpdateRejected(
                response.result.reason or "目标节点拒绝参数更新"
            )

    @staticmethod
    def _same_value(expected, actual):
        """比较数值参数，避免浮点数比较误差。"""
        if (
            isinstance(expected, (int, float))
            and not isinstance(expected, bool)
            and isinstance(actual, (int, float))
            and not isinstance(actual, bool)
        ):
            return math.isclose(
                float(expected),
                float(actual),
                rel_tol=1e-9,
                abs_tol=1e-9,
            )

        return expected == actual

    def _set_target_parameters(self, alias, new_values):
        """读取旧值、更新参数、读取验证；异常时尝试回滚。"""
        if not new_values:
            raise ValueError("参数更新列表不能为空")

        names = list(new_values)
        old_values = self._get_parameters(alias, names)

        try:
            self._set_parameters_atomically(alias, new_values)

        except ParameterUpdateRejected:
            # 原子服务明确拒绝时，不应假定参数已发生变化。
            raise

        except Exception as exc:
            # 超时或通信异常时，实际更新结果可能不确定。
            try:
                self._set_parameters_atomically(alias, old_values)
            except Exception as rollback_exc:
                raise HotSwitchError(
                    f"参数更新异常：{exc}；回滚也失败：{rollback_exc}"
                ) from exc

            raise HotSwitchError(
                f"参数更新异常，已尝试恢复原值：{exc}"
            ) from exc

        try:
            actual_values = self._get_parameters(alias, names)

            mismatches = [
                name
                for name in names
                if not self._same_value(
                    new_values[name],
                    actual_values[name],
                )
            ]

            if mismatches:
                raise HotSwitchError(
                    f"更新后参数校验不一致：{mismatches}"
                )

        except Exception as exc:
            try:
                self._set_parameters_atomically(alias, old_values)
            except Exception as rollback_exc:
                raise HotSwitchError(
                    f"参数校验失败：{exc}；回滚也失败：{rollback_exc}"
                ) from exc

            raise HotSwitchError(
                f"参数校验失败，已尝试恢复原值：{exc}"
            ) from exc

        return old_values

    def _validate_range(self, alias, name, value):
        """验证参数是否属于允许手动调整的清单及范围。"""
        key = f"{alias}.{name}"
        bounds = self.hot_config["parameter_ranges"].get(key)

        if not isinstance(bounds, dict):
            raise HotSwitchError(
                f"参数不在允许的手动调整清单中：{key}"
            )

        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError("滑块参数必须是数值")

        number = float(value)

        if not math.isfinite(number):
            raise ValueError("参数必须是有限数值")

        minimum = float(bounds["min"])
        maximum = float(bounds["max"])

        if (
            not math.isfinite(minimum)
            or not math.isfinite(maximum)
            or minimum > maximum
        ):
            raise ValueError(f"参数范围配置无效：{key}")

        if not minimum <= number <= maximum:
            raise ValueError(
                f"{key} 超出允许范围 [{minimum}, {maximum}]"
            )

        return number

    @staticmethod
    def _json_value(value):
        return json.dumps(
            value,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        )

    def _audit_request(self, op, source, details):
        """先记录请求，再执行操作；审计失败时不执行。"""
        self.audit.append(
            op=op,
            source=source,
            result="requested",
            user=self.user,
            details=details,
        )

    def _write_switch_log(
        self,
        op,
        source,
        target,
        old_value,
        new_value,
        result,
        details,
    ):
        """向 CSV 追加一条操作结果。"""
        columns = [
            "ts",
            "op",
            "source",
            "user",
            "target",
            "from_value",
            "to_value",
            "result",
            "details",
        ]

        record = {
            "ts": datetime.now(timezone.utc).isoformat(),
            "op": op,
            "source": source,
            "user": self.user,
            "target": target,
            "from_value": self._json_value(old_value),
            "to_value": self._json_value(new_value),
            "result": result,
            "details": self._json_value(details),
        }

        with self._lock:
            with self.switch_log_path.open(
                "a+",
                encoding="utf-8",
                newline="",
            ) as stream:
                fcntl.flock(stream.fileno(), fcntl.LOCK_EX)

                try:
                    stream.seek(0, os.SEEK_END)
                    empty = stream.tell() == 0

                    writer = csv.DictWriter(
                        stream,
                        fieldnames=columns,
                    )

                    if empty:
                        writer.writeheader()

                    writer.writerow(record)
                    stream.flush()
                    os.fsync(stream.fileno())

                finally:
                    fcntl.flock(stream.fileno(), fcntl.LOCK_UN)

    def _finish(
        self,
        op,
        source,
        target,
        old_value,
        new_value,
        result,
        details,
    ):
        """记录审计结果与 CSV 结果。"""
        self.audit.append(
            op=op,
            source=source,
            result=result,
            from_value=old_value,
            to_value=new_value,
            user=self.user,
            details=details,
        )

        self._write_switch_log(
            op,
            source,
            target,
            old_value,
            new_value,
            result,
            details,
        )

    def _record_failure(
        self,
        op,
        source,
        target,
        old_value,
        new_value,
        details,
        error,
    ):
        """记录失败结果；记录本身失败时也明确报告。"""
        try:
            self._finish(
                op,
                source,
                target,
                old_value,
                new_value,
                "failed",
                {**details, "error": str(error)},
            )
        except Exception as log_error:
            raise HotSwitchError(
                f"操作失败：{error}；记录失败结果时也发生错误：{log_error}"
            ) from error

    def set_parameter(
        self,
        alias,
        name,
        value,
        source="ui",
        experiment="",
    ):
        """修改单个经过白名单和范围校验的参数。"""
        op = "PARAMETER_SET"

        details = {
            "target_alias": alias,
            "parameter": name,
            "requested_value": value,
            "experiment": experiment,
        }

        with self._lock:
            self._audit_request(op, source, details)

            try:
                new_value = self._validate_range(alias, name, value)

                old_values = self._set_target_parameters(
                    alias,
                    {name: new_value},
                )

            except Exception as exc:
                self._record_failure(
                    op, source, alias, None, value, details, exc
                )
                raise

            self._finish(
                op,
                source,
                alias,
                old_values,
                {name: new_value},
                "ok",
                details,
            )

            return {
                "ok": True,
                "target": alias,
                "parameter": name,
                "old_value": old_values[name],
                "new_value": new_value,
            }

    def _resolve_model_path(self, model_id):
        """解析模型清单，并拒绝空路径或不存在的文件。"""
        models = self.hot_config["models"]

        if model_id not in models:
            raise HotSwitchError(f"未配置模型：{model_id}")

        model_config = models[model_id]

        if not isinstance(model_config, dict):
            raise HotSwitchError(f"模型配置格式无效：{model_id}")

        raw_path = model_config.get("path")

        if not isinstance(raw_path, str) or not raw_path.strip():
            raise HotSwitchError(
                f"模型 {model_id} 尚未配置真实文件路径"
            )

        path = Path(raw_path).expanduser()

        if not path.is_absolute():
            path = self.project_root / path

        path = path.resolve()

        if not path.is_file():
            raise HotSwitchError(
                f"模型文件不存在或不是普通文件：{path}"
            )

        return str(path)

    def switch_model(
        self,
        model_id,
        source="ui",
        experiment="",
    ):
        """通过配置的模型路径参数请求模型切换。

        注意：参数更新成功并不自动证明模型已加载成功。
        目标感知节点必须实现实际加载逻辑和失败保护。
        """
        op = "MODEL_SWITCH"
        alias = self.hot_config.get("model_target", "perception")
        parameter = self.hot_config.get("model_parameter", "model_path")

        details = {
            "model_id": model_id,
            "target_alias": alias,
            "parameter": parameter,
            "experiment": experiment,
        }

        with self._lock:
            self._audit_request(op, source, details)

            try:
                new_path = self._resolve_model_path(model_id)

                old_values = self._set_target_parameters(
                    alias,
                    {parameter: new_path},
                )

            except Exception as exc:
                self._record_failure(
                    op, source, alias, None, model_id, details, exc
                )
                raise

            self._finish(
                op,
                source,
                alias,
                old_values,
                {parameter: new_path},
                "ok",
                details,
            )

            return {
                "ok": True,
                "model_id": model_id,
                "path": new_path,
                "previous_path": old_values[parameter],
            }

    def switch_scene(
        self,
        scene_name,
        source="ui",
        experiment="",
    ):
        """跨节点切换场景；失败时逆序补偿已成功的更新。"""
        op = "SCENE_SWITCH"

        details = {
            "scene": scene_name,
            "experiment": experiment,
        }

        applied = []
        old_values = {}
        new_values = {}

        with self._lock:
            self._audit_request(op, source, details)

            try:
                scenes = self.hot_config["scenes"]

                if scene_name not in scenes:
                    raise HotSwitchError(
                        f"未配置场景：{scene_name}"
                    )

                scene = scenes[scene_name]
                parameters = scene.get("parameters")

                if not isinstance(parameters, dict) or not parameters:
                    raise HotSwitchError(
                        f"场景 {scene_name} 没有有效参数映射"
                    )

                for alias, values in parameters.items():
                    if not isinstance(values, dict) or not values:
                        raise HotSwitchError(
                            f"场景目标 {alias} 的参数映射无效"
                        )

                    previous = self._set_target_parameters(
                        alias,
                        values,
                    )

                    applied.append((alias, previous))
                    old_values[alias] = previous
                    new_values[alias] = values

            except Exception as exc:
                rollback_errors = []

                for alias, previous in reversed(applied):
                    try:
                        self._set_target_parameters(alias, previous)
                    except Exception as rollback_exc:
                        rollback_errors.append(
                            f"{alias}: {rollback_exc}"
                        )

                failure_details = {
                    **details,
                    "error": str(exc),
                    "rollback_errors": rollback_errors,
                }

                error = HotSwitchError(
                    f"场景切换失败：{exc}"
                    + (
                        f"；回滚失败：{rollback_errors}"
                        if rollback_errors
                        else "；已尝试恢复之前的参数"
                    )
                )

                self._record_failure(
                    op,
                    source,
                    scene_name,
                    old_values,
                    new_values,
                    failure_details,
                    error,
                )

                raise error from exc

            self._finish(
                op,
                source,
                scene_name,
                old_values,
                new_values,
                "ok",
                details,
            )

            return {
                "ok": True,
                "scene": scene_name,
                "old_values": old_values,
                "new_values": new_values,
            }
