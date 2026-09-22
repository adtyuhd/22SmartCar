#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sensor_analyzer.py  —— 上一届学长留下的"能用"的脚本

注释（学长原话）：
    "处理一下传感器数据就能用"

原本意图：
    1. 读取 sensor_data.csv（列：time, value）
    2. 计算 value 的平均值、标准差
    3. 剔除离群值（|value - mean| > 2 * std）
    4. 把清洗后的数据保存为 cleaned_data.csv
    5. 打印一份统计摘要

现状：跑不通 / 跑出来数不对。就交给你了。
"""

import csv
import os
import math
import argparse
import sys

DEFAULT_INPUT = "sensor_data.csv"
DEFAULT_OUTPUT = "cleaned_data.csv"
# --- 路径处理 ---
parser = argparse.ArgumentParser(
    description="Analyze and clean sensor data"
)

parser.add_argument(
    "--input",
    default=DEFAULT_INPUT,
    help="input CSV file"
)

parser.add_argument(
    "--output",
    default=DEFAULT_OUTPUT,
    help="output CSV file"
)

args = parser.parse_args()

input_path = args.input
output_path = args.output

data = []
times = []
cleaned = []

print("=== 传感器数据分析 ===")

# --- 读取数据 ---
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

print("共读取 %d 条数据" % len(data))

# --- 计算平均值 ---
total = 0
for v in data:
    total += v
mean = total / len(data)

# --- 计算标准差 ---
acc = 0
for v in data:
    acc += (v - mean)*(v - mean)
std = math.sqrt(acc / len(data))

# --- 剔除离群值 ---
for t, v in zip(times, data):
    if abs(v - mean) <= 2 * std:
        cleaned.append((t, v))

# --- 输出清洗后的数据 ---
f = open(output_path, "w", newline="")
writer = csv.writer(f)
writer.writerow(["time", "value"])
for t,v in cleaned:
    writer.writerow([t,v])
f.close()
print("均值 mean = %.4f" % mean)
print("标准差 std = %.4f" % std)
print("清洗后剩余 %d 条" % len(cleaned))
print("已保存到 %s" % output_path)
