# M0-4 Mini Project：Task Scheduler

这是一个使用 Python 实现的任务调度器模拟器。

程序从 YAML 或 JSON 配置文件中读取任务，根据任务之间的依赖关系进行拓扑排序，并按照依赖顺序执行任务。

每个任务支持：

* 执行时长模拟
* 随机成功或失败
* 最多 3 次尝试
* 失败后的依赖跳过
* 全局超时
* 彩色状态输出
* JSON 执行报告
* 固定随机种子，方便复现实验结果

---

## 1. 环境要求

推荐使用 Python 3.10 或更高版本。

创建虚拟环境：

```bash
python3 -m venv .venv
```

激活虚拟环境：

```bash
source .venv/bin/activate
```

安装依赖：

```bash
pip install -r requirements.txt
```

`requirements.txt` 内容：

```text
PyYAML
```

---

## 2. 运行方式

基本格式：

```bash
python3 scheduler.py \
  --config tasks.yaml \
  --timeout 15 \
  --report report.json \
  --seed 42
```

---

## 3. 命令行参数

### `--config`

必填参数。

指定任务配置文件。

支持：

* `.yaml`
* `.yml`
* `.json`

示例：

```bash
python3 scheduler.py --config tasks_demo.yaml
```

---

### `--timeout`

可选参数。

设置整个任务调度器的最大运行时间，单位为秒。

如果命令行中提供了 `--timeout`，它会覆盖配置文件中的 `timeout`。

示例：

```bash
python3 scheduler.py \
  --config tasks_demo.yaml \
  --timeout 15
```

---

### `--report`

可选参数。

指定执行报告的输出文件。

默认值：

```text
report.json
```

示例：

```bash
python3 scheduler.py \
  --config tasks_demo.yaml \
  --report result.json
```

---

### `--seed`

可选参数。

设置随机种子，使任务成功或失败的随机结果可以重复复现。

示例：

```bash
python3 scheduler.py \
  --config tasks_demo.yaml \
  --seed 42
```

---

## 4. YAML 配置格式

示例：

```yaml
timeout: 15

tasks:
  - name: "Init"
    duration: 0.5
    success_rate: 1.0
    dependencies: []

  - name: "Navigate"
    duration: 1.0
    success_rate: 0.9
    dependencies: ["Init"]

  - name: "Vision"
    duration: 0.8
    success_rate: 0.8
    dependencies: ["Init"]

  - name: "Avoid"
    duration: 1.0
    success_rate: 0.9
    dependencies: ["Navigate", "Vision"]

  - name: "Park"
    duration: 0.5
    success_rate: 0.9
    dependencies: ["Avoid"]
```

---

## 5. JSON 配置格式

程序也支持 JSON 配置文件。

示例：

```json
{
  "timeout": 15,
  "tasks": [
    {
      "name": "Init",
      "duration": 0.5,
      "success_rate": 1.0,
      "dependencies": []
    },
    {
      "name": "Navigate",
      "duration": 1.0,
      "success_rate": 0.9,
      "dependencies": ["Init"]
    }
  ]
}
```

---

## 6. 配置字段说明

### 全局 `timeout`

可选。

表示调度器允许的最大运行时间，单位为秒。

必须是大于 0 的数字。

命令行中的 `--timeout` 优先级更高。

---

### `name`

任务名称。

要求：

* 必须是非空字符串
* 每个任务名称必须唯一

---

### `duration`

任务每次尝试的模拟执行时间，单位为秒。

要求：

```text
duration >= 0
```

程序使用：

```python
time.sleep(duration)
```

模拟真实任务运行。

---

### `success_rate`

任务每次尝试成功的概率。

范围：

```text
0 <= success_rate <= 1
```

例如：

```yaml
success_rate: 1.0
```

表示一定成功。

```yaml
success_rate: 0.0
```

表示一定失败。

---

### `dependencies`

当前任务依赖的任务列表。

例如：

```yaml
dependencies: ["Navigate", "Vision"]
```

表示当前任务只有在 `Navigate` 和 `Vision` 都成功后才能运行。

如果没有依赖：

```yaml
dependencies: []
```

---

## 7. 调度规则

程序首先根据所有任务的依赖关系进行拓扑排序。

例如：

```text
Init
├── Navigate
└── Vision
      ↓
    Avoid
      ↓
     Park
```

只有当前任务的所有依赖任务都成功后，当前任务才会执行。

如果检测到依赖环，例如：

```text
A -> B -> C -> A
```

程序会停止，并输出：

```text
Error: dependency cycle detected
```

同时返回非 0 退出码。

---

## 8. 重试规则

每个任务最多尝试：

```text
3 次
```

也就是：

```text
第一次执行
失败 -> RETRY

第二次执行
失败 -> RETRY

第三次执行
失败 -> 最终 SKIPPED
```

如果任意一次成功，则任务最终状态为：

```text
SUCCESS
```

---

## 9. 依赖失败传播

如果一个任务最终失败并成为：

```text
SKIPPED
```

所有依赖它的下游任务也会：

```text
SKIPPED
```

但是不依赖它的其他独立任务仍然会继续执行。

例如：

```text
Start
├── BadTask -> Downstream
└── Independent
```

如果：

```text
BadTask = SKIPPED
```

那么：

```text
Downstream = SKIPPED
```

但：

```text
Independent
```

仍然正常执行。

---

## 10. 全局超时

调度器使用真实经过时间检查全局 timeout。

程序会在以下位置检查超时：

* 每个任务开始之前
* 每次任务尝试开始之前
* `time.sleep()` 执行之后

如果总时间超过 timeout：

```text
Global scheduler status: TIMEOUT
```

当前正在执行的任务记录为：

```text
TIMEOUT
```

超时之后还没有开始运行的任务也会记录在 report 中：

```text
status: TIMEOUT
attempts: 0
started_at: null
ended_at: null
```

这样报告中始终包含配置文件里的所有任务。

---

## 11. 状态颜色

在支持 ANSI 颜色的终端中：

* `SUCCESS`：绿色
* `FAILED`：红色
* `RETRY`：黄色
* `SKIPPED`：蓝色
* `TIMEOUT`：红色

如果标准输出被重定向到文件：

```bash
python3 scheduler.py \
  --config tasks_demo.yaml \
  > output.txt
```

程序会自动关闭颜色控制字符，输出普通文本。

---

## 12. 执行报告

默认生成：

```text
report.json
```

示例：

```json
{
  "timeout": false,
  "total_duration": 1.5,
  "tasks": [
    {
      "name": "Init",
      "status": "SUCCESS",
      "attempts": 1,
      "duration": 0.5,
      "started_at": 1699999999.12,
      "ended_at": 1699999999.62
    }
  ]
}
```

每个任务包含：

* `name`
* `status`
* `attempts`
* `duration`
* `started_at`
* `ended_at`

可能的状态：

```text
SUCCESS
FAILED
SKIPPED
TIMEOUT
```

其中 `FAILED` 主要用于运行过程中的失败尝试输出；任务连续失败 3 次后，最终状态记录为 `SKIPPED`。

---

## 13. 运行示例

### 示例 1：基本运行

```bash
python3 scheduler.py \
  --config tasks_demo.yaml
```

默认生成：

```text
report.json
```

---

### 示例 2：指定 timeout 和随机种子

```bash
python3 scheduler.py \
  --config tasks_demo.yaml \
  --timeout 15 \
  --seed 42
```

`--seed 42` 可以保证随机执行结果可以重复复现。

---

### 示例 3：指定报告文件

```bash
python3 scheduler.py \
  --config tasks_demo.yaml \
  --timeout 15 \
  --report my_report.json \
  --seed 42
```

---

### 示例 4：使用 JSON 配置

```bash
python3 scheduler.py \
  --config tasks_demo.json \
  --report json_report.json \
  --seed 42
```

---

### 示例 5：测试全局 timeout

```bash
python3 scheduler.py \
  --config test_timeout.yaml \
  --timeout 0.2 \
  --report timeout_report.json \
  --seed 42
```

如果任务执行时间超过 0.2 秒，调度器会进入：

```text
TIMEOUT
```

---

## 14. 输入错误处理

程序会检查常见配置错误，包括：

* 配置文件不存在
* YAML 格式错误
* JSON 格式错误
* `tasks` 不存在
* `tasks` 为空
* 任务缺少必要字段
* 任务名重复
* 任务名为空
* `duration` 非法
* `success_rate` 超出范围
* `dependencies` 不是列表
* dependency 不是字符串
* dependency 不存在
* 重复 dependency
* 任务依赖自己
* 依赖关系存在环
* timeout 非法

发生错误时程序会输出可读错误信息，并返回非 0 退出码，不显示 Python traceback。

---

## 15. 项目文件

项目主要文件：

```text
scheduler.py
requirements.txt
README.md
tasks_demo.yaml
tasks_demo.json
report.json
```

其中：

* `scheduler.py`：任务调度器主程序
* `requirements.txt`：Python 第三方依赖
* `README.md`：项目说明
* `tasks_demo.yaml`：YAML 示例配置
* `tasks_demo.json`：JSON 示例配置
* `report.json`：示例执行报告
