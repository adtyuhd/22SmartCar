import csv
import yaml
import math
import sys
import argparse
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(
        description="Calculate the Pearson correlation of two CSV columns."
    )
    parser.add_argument(
        "--config",
        default="config.yaml",
        help="path to the YAML configuration file",
    )
    return parser.parse_args()
    
def load_config(config_path):
    try:
        with open(config_path) as f:
            config = yaml.safe_load(f)
    except FileNotFoundError:
        raise ValueError(f"config file not found: {config_path}")

    if not config:
        raise ValueError("config file is empty")

    return config
    
def load_csv_data(csv_path, col_x, col_y):
    xs = []
    ys = []

    try:
        with open(csv_path) as f:
            reader = csv.DictReader(f)

            if reader.fieldnames is None:
                raise ValueError("CSV file is empty")

            if col_x not in reader.fieldnames:
                raise ValueError(f"column not found: {col_x}")

            if col_y not in reader.fieldnames:
                raise ValueError(f"column not found: {col_y}")

            for line_number, row in enumerate(reader, start=2):
                try:
                    xs.append(float(row[col_x]))
                    ys.append(float(row[col_y]))
                except (TypeError, ValueError):
                    raise ValueError(
                        f"non-numeric value in row {line_number}"
                    )

    except FileNotFoundError:
        raise ValueError(f"CSV file not found: {csv_path}")

    if not xs:
        raise ValueError("CSV contains no data rows")

    return xs, ys
    
def calculate_correlation(xs, ys):
    if len(xs) != len(ys):
       raise ValueError("input columns must have the same length")

    n = len(xs)
    
    if n == 0:
       raise ValueError("input data is empty")
    
    sum_x = 0.0
    sum_y = 0.0

    for i in range(n):
        sum_x += xs[i]
        sum_y += ys[i]

    mean_x = sum_x / n
    mean_y = sum_y / n

    dx = 0.0
    dy = 0.0
    prod = 0.0

    for i in range(n):
        a = xs[i] - mean_x
        b = ys[i] - mean_y

        dx += a * a
        dy += b * b
        prod += a * b
    if dx == 0 or dy == 0:
       raise ValueError("correlation is undefined for a constant column")
    denom = math.sqrt(dx * dy)
    r = prod / denom

    return n, mean_x, mean_y, r


def main():
    args = parse_args()

    config_path = Path(args.config)
    cfg = load_config(config_path)

    csv_path = Path(cfg["input_csv"])

    if not csv_path.is_absolute():
        csv_path = config_path.parent / csv_path

    col_x = cfg["columns"]["x"]
    col_y = cfg["columns"]["y"]

    xs, ys = load_csv_data(csv_path, col_x, col_y)
    n, mean_x, mean_y, r = calculate_correlation(xs, ys)

    print("n =", n)
    print("mean_x =", mean_x)
    print("mean_y =", mean_y)
    print("r =", r)

if __name__ == "__main__":
    try:
        main()
    except (ValueError, KeyError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        sys.exit(1)
