# YAML 任务配置 DSL 规范

## 1. 适用范围

本规范定义 M5-1 任务配置文件的结构、字段含义、节点参数、失败处理规则及运行报告格式。任务执行顺序由 YAML 文件中的 `tasks` 列表决定。

运行任务配置的命令：

```bash
python3 run_task.py tasks/task_demo.yaml
```

配置加载成功后，程序按顺序构建节点并交由 `TaskScheduler` 执行。运行结束后，报告写入当前工作目录下的 `task_report.json`。

## 2. 顶层配置

YAML 根节点必须是字典，并至少包含一个任务。

| 字段 | 必填 | 类型 | 默认值 | 说明 |
|---|---|---|---|---|
| `tasks` | 是 | 列表 | 无 | 按顺序执行的顶层任务列表，不能为空。 |
| `task_id` | 否 | 字符串 | `demo` | 本次任务的标识符。 |
| `deadline` | 否 | 正数 | 无 | 整个任务的时间预算，单位为秒；不是绝对时间点。 |

`deadline` 在任务开始时转换为单调时钟截止时间。到达任务级截止时间后，调度器返回 `TIMEOUT`。`deadline` 必须是正数。

当前实现不支持 YAML `!include` 标签，也不自动加载外部位姿文件。静态位姿写在同一 YAML 文件中；运行期间产生的数据通过 `pose_from_context` 从共享上下文读取。

## 3. 任务结构与通用字段

普通节点的基本结构如下：

```yaml
- name: navigate_to_scan
  type: NavigateTo
  pose: {x: 2.0, y: 0.5, yaw: 0.0}
  timeout: 10
  retry: 2
  retry_interval: 1.0
  fallback: wait_start
  on_fail: abort
```

| 字段 | 必填 | 类型/有效值 | 默认值 | 说明 |
|---|---|---|---|---|
| `name` | 是 | 非空字符串 | 无 | 节点名称。顶层节点名称应唯一，因为 fallback 按名称定位顶层节点。 |
| `type` | 普通节点必填 | 已注册的节点类型 | 无 | 指定要构建的节点类。并行任务使用 `parallel`，不需要 `type`。 |
| `timeout` | 否 | 秒数 | 由节点决定 | 单次节点执行的超时时间。步骤级超时按 `FAILED` 处理，并进入重试或失败处理流程；任务级 `deadline` 除外。 |
| `retry` | 否 | 非负整数 | `0` | 初次执行失败后允许的额外尝试次数。`retry: 2` 表示最多执行 3 次。 |
| `retry_interval` | 否 | 非负数，单位秒 | `0` | 每次重试前等待的时间。等待期间若达到任务级 `deadline`，任务返回 `TIMEOUT`。 |
| `fallback` | 否 | 顶层节点名称 | 无 | 重试耗尽后跳转到指定的顶层节点。配置后优先于 `on_fail`。 |
| `on_fail` | 否 | `skip` / `abort` | `abort` | 没有配置 `fallback` 且重试耗尽时使用的失败策略。`skip` 将该步骤记为 `SKIPPED` 并继续；其他值按终止任务处理。 |

`name`、`type`、`parallel` 由配置加载器单独处理；其他任务字段会放入节点的 `params` 字典。节点只读取自身需要的参数。

当前实现没有单独校验节点名称是否重复。重复名称会覆盖调度器内部名称映射，可能导致 fallback 跳转错误，因此配置中的顶层节点名称必须保持唯一。Fallback 目标必须是顶层节点名称，不能指向并行分支内部的节点。

## 4. 标准节点及专属字段

当前注册表支持以下五种节点类型：`WaitForTrigger`、`NavigateTo`、`DetectTarget`、`ExecuteGrasp` 和 `SpeakTTS`。

### 4.1 `WaitForTrigger`

```yaml
- name: wait_start
  type: WaitForTrigger
  timeout: 30
```

| 字段 | 默认值 | 说明 |
|---|---|---|
| `timeout` | `30.0` 秒 | 等待触发事件的最长时间。 |

收到 `trigger()` 事件后返回 `SUCCESS`；超时后返回 `FAILED`；取消后返回 `SKIPPED`。当前命令行入口不会自动生成触发事件，也未连接 ROS 话题或键盘输入；若没有运行时触发源，该节点会超时。

### 4.2 `NavigateTo`

```yaml
- name: navigate_to_scan
  type: NavigateTo
  pose: {x: 2.0, y: 0.5, yaw: 0.0}
```

| 字段 | 默认值 | 说明 |
|---|---|---|
| `pose` | 无 | 直接提供导航目标位姿。 |
| `pose_from_context` | 无 | 从 `TaskContext.data` 中读取目标位姿的键名。 |

`pose` 和 `pose_from_context` 至少应提供一个可用目标。节点还需要运行时注入的可调用 `client`，该客户端接收位姿并返回导航结果。当前 `run_task.py` 不会自动创建或注入 ROS 2 Action 客户端；未注入客户端时，节点返回 `FAILED`。成功时，返回值写入 `TaskContext.data["last_nav_result"]`。

### 4.3 `DetectTarget`

```yaml
- name: detect_qr
  type: DetectTarget
  target_class: "qrcode"
  min_confidence: 0.7
  timeout: 15
```

| 字段 | 默认值 | 说明 |
|---|---|---|
| `target_class` | 无 | 目标类别。有效配置应明确指定。 |
| `min_confidence` | `0` | 最低检测置信度，包含阈值本身。 |
| `timeout` | `15` 秒 | 等待符合条件的检测结果的最长时间。 |

检测结果必须包含 `class`、`confidence`、`id` 和 `pose`；类别必须与 `target_class` 一致，置信度必须大于或等于 `min_confidence`。检测成功后，`pose` 和 `id` 分别写入 `TaskContext.data["target_pose"]` 与 `TaskContext.data["target_id"]`。

检测结果通过运行时调用 `on_detection()` 传入。当前命令行入口不会自动订阅 ROS 2 检测话题；没有符合条件的检测结果时，节点返回 `FAILED`。

### 4.4 `ExecuteGrasp`

```yaml
- name: grasp
  type: ExecuteGrasp
  grasp_pose: [1.0, 2.0, 3.0]
  grasp_type: "top_down"
  retry: 2
  on_fail: skip
```

| 字段 | 默认值 | 说明 |
|---|---|---|
| `grasp_pose` | 无 | 抓取位姿；若未指定，则从 `TaskContext.data["grasp_pose"]` 读取。 |
| `grasp_type` | 无 | 抓取类型，有效配置必须提供。 |

节点还需要运行时注入可调用的 `client`。客户端接收 `pose` 和 `grasp_type`，并应返回包含 `success` 与 `object_pose` 的字典。成功后，完整返回字典写入 `TaskContext.data["grasp_result"]`，物体位姿写入 `TaskContext.data["object_pose"]`。缺少位姿、抓取类型或客户端，或者客户端返回失败结果时，节点返回 `FAILED`。

### 4.5 `SpeakTTS`

```yaml
- name: announce
  type: SpeakTTS
  text: "任务开始"
```

| 字段 | 默认值 | 说明 |
|---|---|---|
| `text` | 无 | 要播报的非空文本。 |

节点还需要运行时注入可调用的 `client`，并将 `text` 传给客户端。缺少文本或客户端，或者客户端抛出异常时，节点返回 `FAILED`。当前命令行入口不会自动创建或注入 ROS 2 Service 客户端。

`client` 是运行时可调用对象，不是可由普通 YAML 文本直接构造的配置字段。客户端应由应用程序或测试代码注入；把普通字符串写入 YAML 的 `client` 字段不能创建可调用客户端。

## 5. 共享上下文与运行时接口

所有节点接收同一个 `TaskContext` 实例。该对象包含以下属性：

| 属性 | 含义 | 当前实现中的用途 |
|---|---|---|
| `task_id` | 任务标识符 | 从顶层 `task_id` 初始化。 |
| `deadline` | 单调时钟截止时间 | 由顶层 `deadline` 转换得到；调度器用它控制任务级超时。 |
| `data` | 节点间共享数据字典 | `NavigateTo`、`DetectTarget` 和 `ExecuteGrasp` 通过该字典传递结果。 |
| `retry_budget` | 重试预算字典 | 当前调度器使用每个节点的局部计数处理 `retry`，尚未使用此字典。 |
| `degraded` | 上下文降级标记 | 当前调度器将降级情况写入每步报告的 `degraded` 字段，尚未更新此属性。 |
| `metadata` | 附加元数据字典 | 当前标准节点和调度器未主动使用。 |

运行时客户端、触发事件和检测事件通过应用程序或测试代码注入，不由节点读取 YAML 或依赖 ROS 2 全局状态。

## 6. 重试、Fallback 与失败处理

调度器对每个节点单独计数重试次数。执行返回 `FAILED` 时，若 `retry` 预算尚未耗尽，则先增加重试计数，等待 `retry_interval` 后重新执行。

重试耗尽后的处理顺序如下：

1. 如果配置了 `fallback`，先检查目标是否存在于顶层节点名称映射中；存在则记录当前步骤为 `FAILED`、`degraded: true`，并跳转到目标节点。
2. 如果未配置 `fallback` 且 `on_fail: skip`，当前步骤记为 `SKIPPED`、`degraded: true`，调度器继续下一个节点。
3. 其他情况记录 `FAILED` 并终止任务。

因此，`fallback` 优先于 `on_fail`。同时配置两者时，只要 fallback 目标有效，就会先跳转；`on_fail: skip` 不会覆盖 fallback。

Fallback 可以回跳到先前节点。为避免无限循环，调度器限制整个任务的 fallback 跳转总次数，当前上限为顶层节点数量；超过上限时任务返回 `FAILED`。不存在的 fallback 目标也会使任务返回 `FAILED`。

节点本身返回 `SKIPPED` 时，调度器直接继续下一个节点。此情形与节点返回 `FAILED` 后由 `on_fail: skip` 触发的降级不同：后者会记录 `degraded: true`。

## 7. 步骤超时与任务截止时间

| 机制 | 配置 | 超时后的行为 |
|---|---|---|
| 步骤级超时 | 节点字段 `timeout` | 调用节点的 `cancel()`，将步骤超时按 `FAILED` 处理；可按重试、fallback 或 `on_fail` 继续处理。 |
| 任务级截止时间 | 顶层字段 `deadline` | 整个任务返回 `TIMEOUT`，不再执行后续节点。 |

当步骤超时与任务级截止时间同时适用时，调度器使用剩余任务时间与步骤超时中的较小值。任务级截止时间到达后，整体结果为 `TIMEOUT`，不是普通的可降级步骤失败。

取消采用协作式机制：调度器调用节点的 `cancel()`，节点实现应尽快响应并退出。Python 线程不会被强制杀死；如果运行时客户端阻塞且不响应取消，超时后的实际退出时间可能延长。

## 8. 并行任务

并行任务使用 `parallel` 字段定义，且不需要 `type`：

```yaml
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

| 字段 | 必填 | 默认值 | 说明 |
|---|---|---|---|
| `parallel` | 是 | 无 | 标记该任务为并行任务组。 |
| `parallel.branches` | 是 | 无 | 非空分支列表；每个分支使用普通任务格式。 |
| `parallel.fail_policy` | 否 | `all_must_succeed` | 并行组的失败策略。只接受 `any_fail` 或 `all_must_succeed`。 |

策略语义：

- `any_fail`：任一分支返回 `FAILED` 时，请求取消其他分支，并将并行组结果设为 `FAILED`。
- `all_must_succeed`：所有分支均返回 `SUCCESS` 时并行组才返回 `SUCCESS`；否则返回 `FAILED`。

并行组也可以在 `parallel` 同级设置 `timeout`、`retry`、`retry_interval`、`fallback` 和 `on_fail`。这些字段由调度器按普通节点的规则处理。分支的取消同样是协作式的。

## 9. 必填字段、解析错误与类型错误

配置加载器当前提供以下错误检查：

| 情况 | 当前行为 |
|---|---|
| YAML 语法错误 | 输出包含行号的 `YAML syntax error` 信息（若解析器提供行号）。 |
| YAML 根节点不是字典 | 报错 `YAML root must be a dictionary`。 |
| `tasks` 缺失、不是列表或为空 | 报错 `Missing or empty tasks list`。 |
| 任务缺少 `name` | 报错 `Missing required field: name`。 |
| 普通任务缺少 `type` | 报错并指出对应节点名称缺少 `type`。 |
| 节点类型未知 | `ValueError` 报错，并列出已注册的节点类型。 |
| 并行任务缺少非空 `branches` | 报错并指出并行任务名称。 |
| `fail_policy` 不受支持 | 报错并指出无效的策略值。 |
| `deadline` 不是正数 | 报错 `deadline must be a positive number`。 |
| 节点缺少运行时必需输入 | 通常在执行时返回 `FAILED`，而不是 YAML 加载错误。 |

任务列表中的每个元素应为字典，且顶层 `name` 应唯一。当前加载器没有对这两项进行完整的独立校验；不符合结构的任务元素可能引发未格式化的运行时异常。

## 10. 运行报告与退出码

运行完成后，`task_report.json` 包含以下字段：

| 字段 | 含义 |
|---|---|
| `task_id` | 顶层配置中的任务标识，默认 `demo`。 |
| `result` | 整体任务结果。 |
| `total_elapsed_s` | 整体执行耗时，单位为秒。 |
| `steps` | 节点执行记录列表。 |
| `final_context` | 最终 `TaskContext.data` 内容。 |

每条步骤记录包含 `name`、`status`、`elapsed_s`、`retries` 和 `degraded`。步骤状态可能为 `SUCCESS`、`SKIPPED`、`FAILED` 或 `TIMEOUT`。

当前命令行入口的整体结果包括：

| 结果 | 含义 |
|---|---|
| `SUCCESS` | 调度器执行完任务列表，包括已按配置跳过的步骤。它不保证每个节点的业务操作都成功。 |
| `FAILED` | 存在未能通过重试或降级处理的失败，任务提前终止。 |
| `TIMEOUT` | 任务级 `deadline` 到期。 |

当前实现未实现外部中断结果 `ABORTED`。命令行入口仅在整体结果为 `SUCCESS` 时返回退出码 `0`；其他整体结果返回非零退出码。配置加载失败时会向标准错误输出简短错误信息并返回非零退出码。

## 11. 完整可运行示例

以下示例用于验证配置解析、步骤超时、重试耗尽、fallback 链、并行分支失败和 `on_fail: skip`。它不依赖 ROS 2，但由于命令行入口没有注入导航、抓取或 TTS 客户端，也不会自动生成触发或检测事件，示例中的业务节点会按预期失败并走配置的降级路径。

保存为 `tasks/task_dsl_example.yaml`：

```yaml
task_id: "dsl_control_flow_demo"

tasks:
  - name: wait_start
    type: WaitForTrigger
    timeout: 0.02
    retry: 1
    retry_interval: 0
    fallback: recovery

  - name: recovery
    type: SpeakTTS
    text: "备用播报"
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

  - name: finish
    type: SpeakTTS
    text: "任务结束"
    on_fail: skip
```

执行命令：

```bash
python3 run_task.py tasks/task_dsl_example.yaml
```

预期控制流：

1. `wait_start` 等待触发超时，重试一次后仍失败，跳转到 `recovery`。
2. `recovery` 因缺少 TTS 客户端失败，重试一次后仍失败，跳转到 `parallel_checks`。
3. 并行分支因缺少 TTS 客户端而失败，`any_fail` 使并行组返回 `FAILED`；同级 `on_fail: skip` 将并行组降级为 `SKIPPED` 并继续。
4. `finish` 因缺少 TTS 客户端失败，并按 `on_fail: skip` 跳过。
5. 调度器执行完任务列表，整体结果为 `SUCCESS`，退出码为 `0`。此结果表示控制流完成，不表示语音播报或触发业务成功。

若需要验证节点的业务成功路径，应通过应用程序或单元测试注入 Mock 客户端，并为 `WaitForTrigger`、`DetectTarget` 提供对应事件。