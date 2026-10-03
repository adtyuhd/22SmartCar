#!/usr/bin/env python3

import argparse
import csv
import glob
import math
import os
import statistics


def normalize_angle(angle):
    """
    将角度归一化到 [-pi, pi]。
    """
    return math.atan2(
        math.sin(angle),
        math.cos(angle)
    )


def read_first_last(csv_path):
    """
    读取一份轨迹 CSV 的第一条和最后一条数据。
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
        'x': float(first['x']),
        'y': float(first['y']),
        'theta': float(first['theta']),
    }

    end = {
        'x': float(last['x']),
        'y': float(last['y']),
        'theta': float(last['theta']),
    }

    return start, end


def calculate_error(start, end):
    """
    计算一圈的闭环误差。
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


def mean_std(values):
    """
    返回平均值和样本标准差。
    """

    mean_value = statistics.mean(values)

    if len(values) >= 2:
        std_value = statistics.stdev(values)
    else:
        std_value = 0.0

    return mean_value, std_value


def analyze_group(files):
    """
    分析一组 odom 或 truth CSV。
    """

    results = []

    for csv_path in files:

        start, end = read_first_last(
            csv_path
        )

        error = calculate_error(
            start,
            end
        )

        results.append({
            'file': os.path.basename(csv_path),
            **error
        })

    return results


def print_results(name, results):
    print(
        f'========== {name} =========='
    )

    for index, result in enumerate(
        results,
        start=1
    ):

        heading_error_deg = math.degrees(
            result['dtheta']
        )

        print(
            f'Run {index:02d}: '
            f'position_error='
            f'{result["position_error"]:.4f} m, '
            f'dtheta='
            f'{heading_error_deg:.2f} deg'
        )

    position_errors = [
        result['position_error']
        for result in results
    ]

    angle_errors_deg = [
        math.degrees(result['dtheta'])
        for result in results
    ]

    pos_mean, pos_std = mean_std(
        position_errors
    )

    angle_mean, angle_std = mean_std(
        angle_errors_deg
    )

    print()

    print(
        f'Position error: '
        f'{pos_mean:.4f} ± '
        f'{pos_std:.4f} m'
    )

    print(
        f'Heading error: '
        f'{angle_mean:.2f} ± '
        f'{angle_std:.2f} deg'
    )

    print()

    return {
        'position_mean': pos_mean,
        'position_std': pos_std,
        'angle_mean_deg': angle_mean,
        'angle_std_deg': angle_std,
    }


def write_summary(
    output_path,
    odom_results,
    truth_results
):
    """
    保存每一圈的闭环误差。
    """

    with open(
        output_path,
        'w',
        newline=''
    ) as f:

        writer = csv.writer(f)

        writer.writerow([
            'run',
            'odom_position_error',
            'odom_heading_error_deg',
            'truth_position_error',
            'truth_heading_error_deg'
        ])

        for i in range(
            len(odom_results)
        ):

            odom_heading_deg = math.degrees(
                odom_results[i]['dtheta']
            )

            truth_heading_deg = math.degrees(
                truth_results[i]['dtheta']
            )

            writer.writerow([
                i + 1,
                f'{odom_results[i]["position_error"]:.9f}',
                f'{odom_heading_deg:.9f}',
                f'{truth_results[i]["position_error"]:.9f}',
                f'{truth_heading_deg:.9f}',
            ])


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        '--dir',
        required=True,
        help='directory containing run_XX CSV files'
    )

    args = parser.parse_args()

    odom_files = sorted(
        glob.glob(
            os.path.join(
                args.dir,
                'run_*_odom.csv'
            )
        )
    )

    truth_files = sorted(
        glob.glob(
            os.path.join(
                args.dir,
                'run_*_truth.csv'
            )
        )
    )

    if not odom_files:
        raise RuntimeError(
            'No odom CSV files found.'
        )

    if not truth_files:
        raise RuntimeError(
            'No truth CSV files found.'
        )

    if len(odom_files) != len(truth_files):
        raise RuntimeError(
            'Number of odom and truth '
            'files does not match.'
        )

    print(
        f'Found {len(odom_files)} runs.'
    )

    print()

    odom_results = analyze_group(
        odom_files
    )

    truth_results = analyze_group(
        truth_files
    )

    print_results(
        'ODOM',
        odom_results
    )

    print_results(
        'GAZEBO TRUTH',
        truth_results
    )

    summary_path = os.path.join(
        args.dir,
        'summary.csv'
    )

    write_summary(
        summary_path,
        odom_results,
        truth_results
    )

    print(
        f'Summary saved to: '
        f'{summary_path}'
    )


if __name__ == '__main__':
    main()