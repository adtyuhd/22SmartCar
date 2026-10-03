#!/usr/bin/env python3

import argparse
import csv
import math


def read_first_last(csv_path):
    """
    读取 CSV 的第一条和最后一条数据。
    CSV 格式：
        t,x,y,theta
    """

    with open(csv_path, 'r', newline='') as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    if len(rows) < 2:
        raise RuntimeError(
            f'Not enough data in {csv_path}'
        )

    first = rows[0]
    last = rows[-1]

    start = {
        't': float(first['t']),
        'x': float(first['x']),
        'y': float(first['y']),
        'theta': float(first['theta']),
    }

    end = {
        't': float(last['t']),
        'x': float(last['x']),
        'y': float(last['y']),
        'theta': float(last['theta']),
    }

    return start, end


def normalize_angle(angle):
    """
    将角度归一化到 [-pi, pi]。
    """

    return math.atan2(
        math.sin(angle),
        math.cos(angle)
    )


def calculate_error(start, end):
    """
    计算闭环误差。
    """

    dx = end['x'] - start['x']
    dy = end['y'] - start['y']

    dtheta = normalize_angle(
        end['theta'] - start['theta']
    )

    position_error = math.hypot(
        dx,
        dy
    )

    return {
        'dx': dx,
        'dy': dy,
        'dtheta': dtheta,
        'position_error': position_error,
    }


def print_result(name, start, end, error):
    print(
        f'========== {name} =========='
    )

    print(
        f'Start: '
        f'x={start["x"]:.4f} m, '
        f'y={start["y"]:.4f} m, '
        f'theta={math.degrees(start["theta"]):.2f} deg'
    )

    print(
        f'End:   '
        f'x={end["x"]:.4f} m, '
        f'y={end["y"]:.4f} m, '
        f'theta={math.degrees(end["theta"]):.2f} deg'
    )

    print(
        f'dx = {error["dx"]:.4f} m'
    )

    print(
        f'dy = {error["dy"]:.4f} m'
    )

    print(
        f'dtheta = '
        f'{math.degrees(error["dtheta"]):.2f} deg'
    )

    print(
        f'position error = '
        f'{error["position_error"]:.4f} m'
    )

    print()


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        '--odom',
        required=True,
        help='odom CSV file'
    )

    parser.add_argument(
        '--truth',
        required=True,
        help='Gazebo truth CSV file'
    )

    args = parser.parse_args()

    odom_start, odom_end = read_first_last(
        args.odom
    )

    truth_start, truth_end = read_first_last(
        args.truth
    )

    odom_error = calculate_error(
        odom_start,
        odom_end
    )

    truth_error = calculate_error(
        truth_start,
        truth_end
    )

    print_result(
        'ODOM',
        odom_start,
        odom_end,
        odom_error
    )

    print_result(
        'GAZEBO TRUTH',
        truth_start,
        truth_end,
        truth_error
    )


if __name__ == '__main__':
    main()