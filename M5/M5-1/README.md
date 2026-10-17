# M5-1 任务状态机

基于 Python 实现的轻量级任务调度框架。使用 YAML 编排任务，通过统一的节点接口完成顺序执行、失败重试、超时控制、fallback 降级和并行任务处理，并生成可用于调试与复盘的执行报告。

项目不依赖现成的行为树库，也不要求在 ROS 2 环境中运行。导航、感知、抓取与语音等业务能力通过可替换的客户端或事件接口接入，便于使用 Mock 进行单元测试。

## 1. 主要功能

- **YAML 任务编排**：任务类型、执行顺序及失败策略由配置文件决定，无需修改调度代码。
- **统一节点接口**：节点实现 `setup()`、`execute()`、`cancel()` 和 `teardown()`，支持初始化、执行、协作式取消与资源清理。
- **状态管理**：使用 `IDLE`、`RUNNING`、`SUCCESS`、`FAILED`、`SKIPPED`、`RETRYING` 描述调度与步骤状态。
- **重试与降级**：支持 `retry`、`retry_interval`、`fallback`、`on_fail: skip/abort`，并限制 fallback 跳转次数以避免无限循环。
- **超时控制**：支持节点级 `timeout` 和任务级 `deadline`。
- **并行分支**：支持 `any_fail`、`all_must_succeed` 两种失败策略。
- **运行记录**：终端输出状态转移、重试和降级日志，任务结束后生成 `task_report.json`。

## 2. 项目结构

```text
M5-1/
├── task_fsm/
│   ├── core.py           # 调度器、状态转移、重试、超时与降级
│   ├── context.py        # TaskContext 共享上下文
│   ├── nodes.py          # 五种标准节点及 TaskNode 基类
│   ├── registry.py       # 节点类型注册表
│   ├── parallel.py       # 并行分支执行器
│   └── logger.py         # 终端日志与颜色映射
├── run_task.py           # 命令行入口、YAML 加载与报告生成
├── tasks/
│   └── task_demo.yaml    # 含并行分支的示例配置
├── tests/
│   ├── test_nodes.py     # 节点单元测试
│   └── test_fsm.py       # 调度、并行及配置校验测试
├── docs/
│   ├── design.md         # 框架设计、状态转移与实现取舍
│   └── dsl.md            # YAML 配置字段及执行语义
└── README.md
```

## 3. 环境与依赖

- Python 3.10 或更新版本
- `PyYAML`：解析任务 YAML
- `pytest`：运行单元测试

安装依赖：

```bash
python3 -m pip install PyYAML pytest
```

框架核心逻辑使用 Python 标准库实现，无须安装 ROS 2 或行为树框架。

## 4. 快速开始

在 `M5-1` 目录下执行：

```bash
python3 run_task.py tasks/task_demo.yaml
```

运行时输出状态转移和节点结果，结束后在当前工作目录生成 `task_report.json`。

退出码为 `0` 表示整体任务结果为 `SUCCESS`；其他结果返回非零退出码。

**示例运行说明：**

`tasks/task_demo.yaml` 的首个节点是 `WaitForTrigger`，等待时间为 30 秒。

当前命令行入口不会自动产生触发事件，也不会自动注入 ROS 2 客户端，因此直接运行该示例时，首节点通常会超时并使任务返回 `FAILED`。

这是可预期的业务执行结果，不代表 YAML 解析或调度器异常。

如需验证实际业务成功路径，可在测试或应用程序中注入 Mock 客户端、触发事件和检测结果。

## 5. YAML 任务配置

顶层使用 `tasks` 列表定义执行顺序，可选 `task_id` 和任务级 `deadline`（秒）。

普通节点使用 `name` 和 `type` 声明；并行任务使用 `parallel` 声明。

下面是一个不依赖外部客户端、用于验证重试和并行降级的简短配置示例：

```yaml
task_id: control_flow_demo

tasks:
  - name: wait_start
    type: WaitForTrigger
    timeout: 0.05
    retry: 1
    retry_interval: 0
    fallback: parallel_checks

  - name: parallel_checks
    parallel:
      fail_policy: any_fail
      branches:
        - name: check_battery
          type: SpeakTTS
          text: "检查电量"
        - name: check_lidar
          type: SpeakTTS
          text: "检查雷达"
    on_fail: skip
```

该示例在没有触发事件及 TTS 客户端时，会先重试等待节点，再跳转到并行组，最后按 `on_fail: skip` 完成降级流程。

此时整体结果可为 `SUCCESS`，但仅表示调度流程完成，不表示具体业务操作成功。

完整字段、节点参数及失败处理规则见 [`docs/dsl.md`](docs/dsl.md)。

### 5.1 标准节点

| 节点类型 | 作用 | 关键参数或输入 |
|---|---|---|
| `WaitForTrigger` | 等待触发事件 | `timeout`，运行时 `trigger()` |
| `NavigateTo` | 导航到指定位姿 | `pose` / `pose_from_context`，运行时客户端 |
| `DetectTarget` | 等待满足条件的检测结果 | `target_class`、`min_confidence`、`timeout`，运行时 `on_detection()` |
| `ExecuteGrasp` | 执行抓取 | `grasp_pose`、`grasp_type`，运行时客户端 |
| `SpeakTTS` | 执行语音播报 | `text`，运行时客户端 |

各节点共享 `TaskContext.data`，可传递导航、检测、抓取等结果。

节点类型由 `task_fsm/registry.py` 注册和创建。

## 6. 执行结果与报告

任务运行结束后，生成 `task_report.json`。

报告包含以下字段：

| 字段 | 内容 |
|---|---|
| `task_id` | 本次任务标识 |
| `result` | 整体任务执行结果 |
| `total_elapsed_s` | 任务执行总耗时（秒） |
| `steps` | 各步骤的名称、状态、耗时、重试次数和降级标记 |
| `final_context` | 最终共享上下文数据 |

### 6.1 任务结果

| 结果 | 含义 |
|---|---|
| `SUCCESS` | 所有顶层任务已处理完成 |
| `FAILED` | 存在未处理的失败，任务终止 |
| `TIMEOUT` | 任务级截止时间到达 |
| `ABORTED` | 任务执行过程中受到键盘中断 |

需要区分两种超时机制：

- **步骤级超时**：某个节点超时，按 `FAILED` 处理，可以根据配置进行重试或降级。
- **任务级超时**：整个任务超过 `deadline`，任务直接返回 `TIMEOUT`。

程序运行时出现异常，也会尽可能记录失败报告；配置加载阶段失败时不生成新的执行报告。

### 6.2 日志

终端日志记录状态转移、节点执行结果、重试、降级、上下文变化和超时事件。

不同状态使用不同颜色进行区分。彩色日志仅在支持 ANSI 颜色的交互式终端中启用。

## 7. 运行测试

运行全部单元测试：

```bash
python3 -m pytest tests/ -q
```

如果环境中安装的 ROS 2 pytest 插件与当前 Python 环境冲突，可关闭插件自动加载：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/ -q
```

已验证的项目测试结果：

```text
41 passed
```

测试覆盖以下场景：

- 五种标准节点的正常执行、失败和取消
- 调度器状态转移
- 重试成功及重试耗尽
- fallback 跳转及循环保护
- 步骤级超时与任务级截止时间
- `on_fail: skip` 降级处理
- 并行分支成功、失败及取消
- 节点生命周期管理
- YAML 配置校验
- 任务报告生成

## 8. 设计说明与限制

本项目采用顺序任务调度器与并行执行器相结合的结构。

调度器通过节点注册表创建任务，通过 `TaskContext` 共享数据，并使用统一的节点接口完成任务执行。

当前实现存在以下限制：

1. 框架主要面向软件调度与 Mock 测试，命令行入口尚未自动连接 ROS 2 Action、Service 或话题。
2. 节点取消采用协作式机制。若外部客户端阻塞且不响应取消，请求取消后仍可能等待较长时间。
3. 并行组在报告中作为整体步骤记录，尚未展开记录各分支的独立耗时及状态。
4. `TaskContext` 提供 `retry_budget`、`degraded`、`metadata` 字段，但当前主要使用节点局部重试计数及步骤报告中的降级标记。
5. 当前不支持 YAML `!include`；任务位姿可直接配置，或通过上下文字段传递。

详细的状态转移、模块设计和设计取舍见 [`docs/design.md`](docs/design.md)。