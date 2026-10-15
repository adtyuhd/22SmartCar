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
import math
import argparse
import sys

DEFAULT_INPUT = "sensor_data.csv"
DEFAULT_OUTPUT = "cleaned_data.csv"


# --- 读取数据 ---
def read_data(input_path):
    times = []
    data = []

    try:
        f = open(input_path, "r", newline="", encoding="utf-8")
    except FileNotFoundError:
        raise FileNotFoundError("输入文件不存在: " + input_path) from None

    with f:
        reader = csv.DictReader(f)

        if reader.fieldnames is None:
            raise ValueError("输入文件为空")

        if "time" not in reader.fieldnames or "value" not in reader.fieldnames:
            raise ValueError("CSV 必须包含 time 和 value 两列")

        for line_number, row in enumerate(reader, start=2):
            try:
                t = float(row["time"])
                v = float(row["value"])
            except (ValueError, TypeError):
                raise ValueError("第 %d 行包含非数值内容" % line_number) from None

            times.append(t)
            data.append(v)

    if len(data) == 0:
        raise ValueError("CSV 没有数据")

    return times, data


# --- 计算均值和标准差 ---
def calculate_stats(data):
    total = 0
    for v in data:
        total += v
    mean = total / len(data)

    acc = 0
    for v in data:
        acc += (v - mean) * (v - mean)
    std = math.sqrt(acc / len(data))

    return mean, std


# --- 剔除离群值 ---
def remove_outliers(times, data, mean, std):
    cleaned = []

    for t, v in zip(times, data):
        if abs(v - mean) <= 2 * std:
            cleaned.append((t, v))

    return cleaned


# --- 保存清洗后的数据 ---
def save_data(output_path, cleaned):
    with open(output_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["time", "value"])

        for t, v in cleaned:
            writer.writerow([t, v])


# --- 主函数 ---
def main():
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

    try:
        # 读取数据
        times, data = read_data(args.input)

        # 计算统计量
        mean, std = calculate_stats(data)

        # 剔除离群值
        cleaned = remove_outliers(times, data, mean, std)

        # 保存数据
        save_data(args.output, cleaned)

    except (OSError, ValueError, csv.Error) as e:
        print("Error:", e)
        return 1

    # 统一打印统计摘要
    print("=== 传感器数据分析 ===")
    print("共读取 %d 条数据" % len(data))
    print("均值 mean = %.4f" % mean)
    print("标准差 std = %.4f" % std)
    print("清洗后剩余 %d 条" % len(cleaned))
    print("已保存到 %s" % args.output)

    return 0


if __name__ == "__main__":
    sys.exit(main())
