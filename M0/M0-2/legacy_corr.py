# 导入需要的模块
import csv
import yaml
import math
import sys
import argparse
from pathlib import Path


# 解析命令行参数
def parse_args():
    parser = argparse.ArgumentParser(
        description="Calculate the Pearson correlation of two CSV columns."
    )
    parser.add_argument(
        "--config",
        default="config.yaml",  # 默认配置文件
        help="path to the YAML configuration file",
    )
    return parser.parse_args()


# 读取 YAML 配置文件
def load_config(config_path):
    try:
        with open(config_path) as f:
            config = yaml.safe_load(f)  # 将 YAML 转换为 Python 对象
    except FileNotFoundError:
        raise ValueError(f"config file not found: {config_path}")

    # 检查配置是否为空
    if not config:
        raise ValueError("config file is empty")

    return config


# 读取 CSV 中指定的两列数据
def load_csv_data(csv_path, col_x, col_y):
    xs = []  # 存储 X 列数据
    ys = []  # 存储 Y 列数据

    try:
        with open(csv_path) as f:
            reader = csv.DictReader(f)  # 按列名读取 CSV

            # 检查 CSV 是否为空
            if reader.fieldnames is None:
                raise ValueError("CSV file is empty")

            # 检查 X 列是否存在
            if col_x not in reader.fieldnames:
                raise ValueError(f"column not found: {col_x}")

            # 检查 Y 列是否存在
            if col_y not in reader.fieldnames:
                raise ValueError(f"column not found: {col_y}")

            # 逐行读取数据
            for line_number, row in enumerate(reader, start=2):
                try:
                    xs.append(float(row[col_x]))  # 转换 X 为浮点数
                    ys.append(float(row[col_y]))  # 转换 Y 为浮点数
                except (TypeError, ValueError):
                    raise ValueError(
                        f"non-numeric value in row {line_number}"
                    )

    except FileNotFoundError:
        raise ValueError(f"CSV file not found: {csv_path}")

    # 检查是否有数据行
    if not xs:
        raise ValueError("CSV contains no data rows")

    return xs, ys  


# 计算 Pearson 相关系数
def calculate_correlation(xs, ys):
    # 检查两列长度是否一致
    if len(xs) != len(ys):
        raise ValueError("input columns must have the same length")

    n = len(xs)  # 数据数量

    # 检查数据是否为空
    if n == 0:
        raise ValueError("input data is empty")

    sum_x = 0.0  # X 总和
    sum_y = 0.0  # Y 总和

    # 计算两列数据的总和
    for i in range(n):
        sum_x += xs[i]
        sum_y += ys[i]

    # 计算平均值
    mean_x = sum_x / n
    mean_y = sum_y / n

    dx = 0.0    # X 的离差平方和
    dy = 0.0    # Y 的离差平方和
    prod = 0.0  # X 和 Y 的离差乘积和

    # 计算离差平方和与乘积和
    for i in range(n):
        a = xs[i] - mean_x  # X 与平均值的差
        b = ys[i] - mean_y  # Y 与平均值的差

        dx += a * a
        dy += b * b
        prod += a * b

    # 常数列无法计算相关系数
    if dx == 0 or dy == 0:
        raise ValueError("correlation is undefined for a constant column")

    denom = math.sqrt(dx * dy)  # 计算分母
    r = prod / denom           # 计算 Pearson 相关系数

    return n, mean_x, mean_y, r  


# 主函数
def main():
    args = parse_args()  # 获取命令行参数

    config_path = Path(args.config)  # 配置文件路径
    cfg = load_config(config_path)   # 读取配置

    csv_path = Path(cfg["input_csv"])  # 获取 CSV 路径

    # 如果是相对路径，则与配置文件目录拼接
    if not csv_path.is_absolute():
        csv_path = config_path.parent / csv_path

    # 获取需要计算的列名
    col_x = cfg["columns"]["x"]
    col_y = cfg["columns"]["y"]

    # 读取 CSV 数据
    xs, ys = load_csv_data(csv_path, col_x, col_y)

    # 计算相关系数
    n, mean_x, mean_y, r = calculate_correlation(xs, ys)

    # 输出计算结果
    print("n =", n)
    print("mean_x =", mean_x)
    print("mean_y =", mean_y)
    print("r =", r)


# 程序入口
if __name__ == "__main__":
    try:
        main()  # 执行主函数
    except (ValueError, KeyError) as exc:
        # 输出错误信息
        print(f"Error: {exc}", file=sys.stderr)
        sys.exit(1)  # 出错时退出程序
