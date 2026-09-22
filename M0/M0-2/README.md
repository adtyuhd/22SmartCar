# M0-2 Pearson Correlation Tool

## 运行方法

进入 `M0/M0-2` 目录后运行：

```bash
python3 legacy_corr.py --config config.yaml
```

`--config` 用于指定 YAML 配置文件。

正常输出示例：

```text
n = 40
mean_x = 49.43782499999999
mean_y = 43.310725
r = 0.5634747254524842
```

## 输入文件格式

### YAML

配置文件需要指定输入 CSV 文件以及参与计算的两列：

```yaml
input_csv: "sample_data.csv"

columns:
  x: "sensor_a"
  y: "sensor_b"
```

字段说明：

- `input_csv`：输入 CSV 文件的路径。
- `columns.x`：第一列数据的列名。
- `columns.y`：第二列数据的列名。

当 `input_csv` 为相对路径时，程序相对于 YAML 配置文件所在目录查找 CSV 文件。

### CSV

CSV 第一行必须包含列名，并且必须存在 YAML 中 `columns.x` 和 `columns.y` 指定的两列。

例如：

```csv
timestamp,sensor_a,sensor_b,note
2025-09-20T10:00:00,48.2,41.5,normal
2025-09-20T10:01:00,49.1,42.3,normal
2025-09-20T10:02:00,50.0,43.1,normal
```

如果 YAML 中配置：

```yaml
columns:
  x: "sensor_a"
  y: "sensor_b"
```

则 CSV 中必须存在 `sensor_a` 和 `sensor_b` 两列。

参与计算的两列数据必须为可转换成浮点数的数值。

## 输出结果说明

程序输出：

- `n`：参与计算的数据点数量。
- `mean_x`：x 列数据的平均值。
- `mean_y`：y 列数据的平均值。
- `r`：x、y 两列数据的 Pearson 相关系数。

Pearson 相关系数 `r` 的范围为 `[-1, 1]`：

- `r` 接近 `1`：较强的正线性相关。
- `r` 接近 `-1`：较强的负线性相关。
- `r` 接近 `0`：线性相关较弱或没有明显的线性相关。

其中 `r = 1` 表示完全正线性相关，`r = -1` 表示完全负线性相关。
