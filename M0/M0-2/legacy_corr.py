import csv
import yaml


CONFIG_PATH = "config.yaml"


def load_config(config_path):
    with open(config_path) as f:
        return yaml.safe_load(f)


def load_csv_data(csv_path, col_x, col_y):
    xs = []
    ys = []

    with open(csv_path) as f:
        reader = csv.DictReader(f)

        for row in reader:
            xs.append(float(row[col_x]))
            ys.append(float(row[col_y]))

    return xs, ys


def calculate_correlation(xs, ys):
    n = len(xs)

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
    denom = dx * dy
    r = prod / denom

    return n, mean_x, mean_y, r


def main():
    cfg = load_config(CONFIG_PATH)

    csv_path = cfg["input_csv"]
    col_x = cfg["columns"]["x"]
    col_y = cfg["columns"]["y"]

    xs, ys = load_csv_data(csv_path, col_x, col_y)

    n, mean_x, mean_y, r = calculate_correlation(xs, ys)

    print("n =", n)
    print("mean_x =", mean_x)
    print("mean_y =", mean_y)
    print("r =", r)


if __name__ == "__main__":
    main()
