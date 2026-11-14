# M5-3 一体化操作终端

本目录独立提供 FastAPI 网页、裁判状态适配、ROS 2 参数/模型/场景热切、主机急停、机器人侧急停组件与哈希链审计。所有开发与提交仅涉及 `M5/M5-3/`，不导入或修改 M5-1 调度器。

## 环境与启动

ROS 2 Humble 使用系统 Python 3.10，不能直接使用当前 Conda Python 3.14。ROS 系统需已有 `rclpy`、`std_msgs`、`geometry_msgs`、`rcl_interfaces`；机器人侧组件另需 `controller_manager_msgs`。

在本目录执行：

```bash
unset PYTHONHOME PYTHONPATH
source /opt/ros/humble/setup.bash
/usr/bin/python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install --no-cache-dir -r requirements.txt -r requirements-test.txt
mkdir -p work/ros_logs
export ROS_LOG_DIR="$PWD/work/ros_logs"
export M53_OPERATOR=operator_name
.venv/bin/python -B host_side/operator_terminal.py --host 127.0.0.1 --port 8081
```

打开 `http://127.0.0.1:8081/`，接口说明在 `/docs`。默认仅监听本机；远程访问请通过受控网络或 SSH 转发。服务没有账户认证，不宜直接暴露至公共网络。

`--config` 指定 YAML，`--match-duration-s` 设置默认比赛时长。`M53_AUDIT_DIR` 指定审计目录，默认本项目根目录；开发验收时请保持所有输出位于本目录。启动单 worker，避免生成多个控制运行时。

## 无实车演示

已有上述环境后，在本目录执行：

```bash
bash tools/run_demo.sh
```

打开 `http://127.0.0.1:8082/`。页面会显示“模拟演示”标识。演示启动真实 HTTP 与 ROS 2 通信，附带模拟导航/感知参数节点、模拟控制器管理服务，并将本目录真实 `EmergencyStopGuard` 附着于模拟节点。可验证比赛控制、热切与急停/人工复位链路，无需修改或启动其他项目。

模型项使用名称明确的占位文本文件，只验证路径更新和回读，不做推理；接口继续返回 `model_load_confirmed=false`。模拟控制器“停用”只是内存状态变化，没有物理运动效果。演示不接入任务调度器，人工跳过仍禁用。

演示默认使用仅本机 DDS 域 174，浏览器自动化使用域 173；同一域同名演示节点请勿同时启动多份，也不要用于真实机器人。演示默认监听本机端口 8082，可用 `--port` 更改。所有生成配置、占位文件和审计写在 `work/demo/`；`M53_DEMO_OUTPUT` 可改到本项目 `work/` 内的其他目录，不能覆盖生产配置。不会改动 `config/terminal.yaml`。

`GET /api/simulation` 仅在演示进程存在，可查看模拟控制器、Guard 状态和参数值；正常终端没有这个接口。要演示控制器服务拒绝停用：

```bash
M53_DEMO_FAIL_SWITCH=1 bash tools/run_demo.sh
```

点击急停后应显示失败且保持锁存，人工复位也不能解除锁存。这证明软件错误路径已处理，不代表实际底盘已停车。

## 配置与行为

编辑 `config/terminal.yaml`：

- `parameter_targets`：实际 ROS 节点绝对名称；默认空值会拒绝操作。
- `parameter_ranges`：允许人工修改的数值参数及范围；目标节点须已声明这些参数。
- `models`：实际模型文件路径；空路径或文件不存在会拒绝切换。路径参数更新、回读成功不证明模型加载成功。
- `scenes`：按目标别名配置参数组，例如 `scenes.slow.parameters.navigation.max_velocity: 0.2`。跨节点失败会逆序补偿，补偿失败会明确报告；不保证跨节点全局原子性。
- `timeout_s`：每次服务等待/响应的超时；整个场景切换可能涉及多次等待。通信超时后的远端执行结果可能不确定，需现场复核。
- `switch_log`：切换 CSV 路径；相对路径以本目录为基准。

网页提供比赛控制、加罚、查询分数、参数修改、模型/场景切换、急停/人工复位和状态轮询。实验标识会随热切操作写入审计。配置加载或状态连接失败后持续重试；断线时禁用热切和跳过按钮，连接恢复后更新提示。急停入口保留，以便仍可尝试发出请求。

配置会在启动时校验节点名称、数值范围、模型字段、场景映射及有限正超时；无效配置会拒绝启动。界面根据配置设置输入范围、选择首个可用目标和模型，空数值不会被转换为 0。模型可用状态还检查文件是否存在，但不保证模型格式正确或实际加载完成。浏览器请求超时会提示执行结果未知，不自动重试操作。

## HTTP 接口

| 接口 | 含义 |
|---|---|
| `GET /health` | ROS context 与执行线程均正常时为 200；否则 503 且 ok=false |
| `GET /api/status`、`/api/config` | 裁判、安全、配置与任务控制能力 |
| `POST /api/referee/command` | START/PAUSE/RESUME/STOP/PENALTY/SCORE_QUERY |
| `POST /api/parameters` | target、parameter、value、可选 experiment |
| `POST /api/models` | model_id、可选 experiment；不承诺实际加载完成 |
| `POST /api/scenes` | scene_name、可选 experiment |
| `POST /api/emergency-stop` | reason；202 仅表示请求提交 |
| `POST /api/emergency-reset` | manual_confirmation 必须为 JSON 布尔 true；等待匹配反馈 |
| `GET /api/tasks/capabilities` | 当前是否存在跳过适配器 |
| `POST /api/tasks/skip` | task_id、node_id、reason、manual_confirmation；默认 501 |

比赛状态冲突和业务拒绝通常返回 409；输入校验失败返回 422；运行时未就绪返回 503。人工确认字段拒绝字符串 `"true"` 和数值 1。

### 人工跳过的实现边界

当前运行时没有接入跳过后端：`skip_available=false`，页面按钮禁用，POST 返回 501 并记录 unavailable 审计。不会发布虚构的跳过话题，也不会调用 M5-1 私有字段或假装实际节点已跳过。

`host_side/task_control.py` 定义独立 `SkipBackend` 协议，供将来有安全取消能力的调度器集成。后端必须原子核对 task_id/node_id，处理节点退出、资源释放、并发重复请求及状态推进，并返回真实请求 ID；过期身份或不可中断任务须拒绝。适配器要求人工确认、RUNNING 且无急停；这些本地检查不能代替后端的原子校验。后端接受也只返回 `execution_confirmed=false`。当前没有执行结果反馈协议，也没有通过配置启用真实后端的入口。测试替身仅验证此协议，不能证明能跳过 M5-1 正在运行的节点。

### 急停与实车边界

主机发布零速度、`/emergency_stop` 和 `/safety/fault`，等待 `/safety/status`。仅匹配当前请求 ID 的成功复位反馈可解除主机锁存，复位不会自动恢复比赛或底盘。

`robot_side/emergency_stop_node.py` 是附着于已有 ROS 节点的 `EmergencyStopGuard`，不是可直接运行的节点。当前范围内没有修改其他项目来安装该组件，因此仅启动网页不能保证机器人端已接入。已有机器人节点需由对应项目负责人实例化组件，并确认实际控制器名、切换服务、速度命令仲裁和独立物理急停链路。组件已有 ENABLE 协议，但本次不添加自动激活或主机恢复入口。

## 测试与审计

```bash
bash tools/run_tests.sh           # 单元和隔离 HTTP 测试；跳过 DDS/浏览器测试
bash tools/run_tests.sh --ros     # 加上真实 ROS 2 服务与应用生命周期集成测试
bash tools/run_tests.sh --browser # 加上 Chromium→TCP HTTP→DDS→模拟控制器完整链路
.venv/bin/python -B tools/verify_audit.py --log audit_log_YYYYMMDD.jsonl
```

浏览器测试需额外安装一次依赖和 Chromium，安装位置限定在本目录：

```bash
.venv/bin/python -m pip install --no-cache-dir -r requirements-browser.txt
PLAYWRIGHT_BROWSERS_PATH="$PWD/work/browsers" .venv/bin/python -B -m playwright install chromium
```

浏览器包可选，不影响普通单元/HTTP 测试。`--browser` 会启动并清理独立演示进程和无头 Chromium，生成桌面/手机截图并检查页面脚本无异常、手机宽度无横向溢出。截图和服务器日志在 `work/pytest/`，下次测试会重建该目录。

测试脚本固定 DDS 域 173、仅本机通信，使用模拟参数节点；运行前确认该域不用于机器人，也不要同时在此域运行其他演示进程。临时文件和 ROS 日志在本目录 `work/`。运行 `--ros` 不会运行急停硬件指令。SHA-256 链能发现现存记录修改/删除/重排；检测尾部截断需另外保留可信末端哈希，再传入 `--expected-final-hash`。哈希链不是数字签名，不能抵抗能重写整条链的攻击者。

详细验收范围和实车检查见 [docs/acceptance.md](docs/acceptance.md)。
