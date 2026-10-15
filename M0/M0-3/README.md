# M0-3 代码排查问题报告

## 1. 运行

```bash
cd ~/22SmartCar/M0/M0-3
python3 sensor_analyzer.py
```

---

## 2. Error 1

### 发现报错

```text
File "/home/adtyuhd/22SmartCar/M0/M0-3/sensor_analyzer.py", line 37, in <module>
    v = float(row["Value"])
KeyError: 'Value'
```

### 问题

问题是 `V` 大小写错误，应为 `value`。

---

## 3. Error 2

### 现象

可以读取数据，但是无法输出。

发现报错：

```text
File "/home/adtyuhd/22SmartCar/M0/M0-3/sensor_analyzer.py", line 62, in <module>
    f = open(output_path, "w")
FileNotFoundError: [Errno 2] No such file or directory: '/out/cleaned_data.csv'
```

### 问题

问题是 path 错误和 out 目录可能不存在。

### 代码修正为

```python
os.makedirs(OUTPUT_DIR, exist_ok=True)
output_path = os.path.join(OUTPUT_DIR, OUTPUT_FILE)
```

---

## 4. Error 3

### 输出

```text
=== 传感器数据分析 ===
共读取 60 条数据
均值 mean = 50.4315
标准差 std = -0.0000
清洗后剩余 0 条
已保存到 out/cleaned_data.csv
```

### 问题

`std` 的结果存在明显错误，推测代码逻辑存在错误。

### 代码修正为

```python
for v in data:
    acc += (v - mean) * (v - mean)

std = math.sqrt(acc / len(data))
```

---

## 5. Error 4

### 输出

```text
=== 传感器数据分析 ===
共读取 60 条数据
均值 mean = 50.4315
标准差 std = 18.5033
清洗后剩余 0 条
已保存到 out/cleaned_data.csv
```

### 问题

代码逻辑和“是超过 2 倍标准差的绝对值判定”不符。

### 代码修正为

```python
if abs(v - mean) > 2 * std:
        data.remove(v)
```

---

## 6. Error 5

### 输出

```text
=== 传感器数据分析 ===
共读取 60 条数据
均值 mean = 50.4315
标准差 std = 18.5033
清洗后剩余 57 条
已保存到 out/cleaned_data.csv
```

### 问题

但是查看 `cleaned_data.csv` 发现只有一列，数据输出格式错误。

同时 `remove` 的用法存在问题。

### 代码修正为

```python
for t, v in zip(times, data):
    if abs(v - mean) <= 2 * std:
        cleaned.append((t, v))
```

输出部分：

```python
for t, v in cleaned:
    writer.writerow([t, v])
```

---

## 7. Error 6

### 问题

发现代码写死绝对路径。

### 代码修正

`cleaned_data.csv` 默认写到当前工作目录，同时处理 `input` 和 `output`。

---

## 8. Error 7

### 问题

数据读取报错处理，不出现 Traceback。

### 代码修正处理

```python
try:
    f = open(input_path, "r")
except FileNotFoundError:
    print("Error: 输入文件不存在:", input_path)
    sys.exit(1)

reader = csv.DictReader(f)

# 空文件：连表头都没有
if reader.fieldnames is None:
    print("Error: 输入文件为空")
    f.close()
    sys.exit(1)

# 检查列名
if "time" not in reader.fieldnames or "value" not in reader.fieldnames:
    print("Error: CSV 必须包含 time 和 value 两列")
    f.close()
    sys.exit(1)

for line_number, row in enumerate(reader, start=2):
    try:
        t = float(row["time"])
        v = float(row["value"])
    except (ValueError, TypeError):
        print("Error: 第 %d 行包含非数值内容" % line_number)
        f.close()
        sys.exit(1)

    times.append(t)
    data.append(v)

f.close()

# 有表头，但没有任何数据行
if len(data) == 0:
    print("Error: CSV 没有数据")
    sys.exit(1)
```

---

## 9. Error 8

### 代码缺陷

缺失：

```python
f.close()
```

同时多了一行：

```python
f = open(output_path, "w")
```

### 修正

删除多余的：

```python
f = open(output_path, "w")
```

保留：

```python
f = open(output_path, "w", newline="")
```

并在写入完成后增加：

```python
f.close()
```
---

## 10. AI 使用说明

本次任务使用 ChatGPT 辅助检查代码、分析部分错误原因，
并对 README 问题报告提供修改建议。

具体使用范围：

 辅助写异常处理和路径处理的代码。


代码修复过程按照报错信息逐步排查，
并通过多次运行验证修改结果。

提交前对代码进行逐行检查，确保理解各部分功能及修复原理。
