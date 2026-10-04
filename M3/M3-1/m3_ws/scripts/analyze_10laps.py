#!/usr/bin/env python3

import argparse
import csv
import math
import os
import statistics


def normalize_angle(angle):
    """
    Normalize angle to (-pi, pi].
    """
    return math.atan2(
        math.sin(angle),
        math.cos(angle)
    )


def read_first_last(csv_path):
    """
    Read the first and last data rows from:
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

    return {
        'start_x': float(first['x']),
        'start_y': float(first['y']),
        'start_theta': float(first['theta']),
        'end_x': float(last['x']),
        'end_y': float(last['y']),
        'end_theta': float(last['theta']),
    }


def calculate_error(csv_path):
    data = read_first_last(csv_path)

    dx = (
        data['end_x']
        - data['start_x']
    )

    dy = (
        data['end_y']
        - data['start_y']
    )

    dtheta = normalize_angle(
        data['end_theta']
        - data['start_theta']
    )

    position_error = math.hypot(
        dx,
        dy
    )

    # 题目要求角度“偏差”，因此取绝对值
    heading_error_deg = abs(
        math.degrees(dtheta)
    )

    return {
        'position_error': position_error,
        'heading_error_deg': heading_error_deg,
    }


def mean_std(values):
    mean_value = statistics.mean(values)

    if len(values) >= 2:
        std_value = statistics.stdev(values)
    else:
        std_value = 0.0

    return mean_value, std_value


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        '--dir',
        type=str,
        required=True,
        help='directory containing run_XX_odom.csv and run_XX_truth.csv'
    )

    args = parser.parse_args()

    data_dir = args.dir

    odom_files = sorted([
        name
        for name in os.listdir(data_dir)
        if name.startswith('run_')
        and name.endswith('_odom.csv')
    ])

    truth_files = sorted([
        name
        for name in os.listdir(data_dir)
        if name.startswith('run_')
        and name.endswith('_truth.csv')
    ])

    if len(odom_files) != len(truth_files):
        raise RuntimeError(
            'Number of odom files and truth files does not match.'
        )

    if not odom_files:
        raise RuntimeError(
            f'No run files found in {data_dir}'
        )

    print(
        f'Found {len(odom_files)} runs.'
    )

    odom_results = []
    truth_results = []

    for index, (
        odom_name,
        truth_name
    ) in enumerate(
        zip(
            odom_files,
            truth_files
        ),
        start=1
    ):
        odom_path = os.path.join(
            data_dir,
            odom_name
        )

        truth_path = os.path.join(
            data_dir,
            truth_name
        )

        odom_error = calculate_error(
            odom_path
        )

        truth_error = calculate_error(
            truth_path
        )

        odom_results.append(
            odom_error
        )

        truth_results.append(
            truth_error
        )

        print()
        print(
            f'Run {index:02d}'
        )

        print(
            '  ODOM:  '
            f'{odom_error["position_error"]:.4f} m, '
            f'{odom_error["heading_error_deg"]:.2f} deg'
        )

        print(
            '  TRUTH: '
            f'{truth_error["position_error"]:.4f} m, '
            f'{truth_error["heading_error_deg"]:.2f} deg'
        )

    odom_position_values = [
        item['position_error']
        for item in odom_results
    ]

    odom_heading_values = [
        item['heading_error_deg']
        for item in odom_results
    ]

    truth_position_values = [
        item['position_error']
        for item in truth_results
    ]

    truth_heading_values = [
        item['heading_error_deg']
        for item in truth_results
    ]

    (
        odom_position_mean,
        odom_position_std
    ) = mean_std(
        odom_position_values
    )

    (
        odom_heading_mean,
        odom_heading_std
    ) = mean_std(
        odom_heading_values
    )

    (
        truth_position_mean,
        truth_position_std
    ) = mean_std(
        truth_position_values
    )

    (
        truth_heading_mean,
        truth_heading_std
    ) = mean_std(
        truth_heading_values
    )

    print()
    print(
        '========== ODOM =========='
    )

    print(
        'Position error: '
        f'{odom_position_mean:.4f} '
        f'± {odom_position_std:.4f} m'
    )

    print(
        'Heading error:  '
        f'{odom_heading_mean:.2f} '
        f'± {odom_heading_std:.2f} deg'
    )

    print()
    print(
        '========== TRUTH =========='
    )

    print(
        'Position error: '
        f'{truth_position_mean:.4f} '
        f'± {truth_position_std:.4f} m'
    )

    print(
        'Heading error:  '
        f'{truth_heading_mean:.2f} '
        f'± {truth_heading_std:.2f} deg'
    )

    summary_path = os.path.join(
        data_dir,
        'summary.csv'
    )

    with open(
        summary_path,
        'w',
        newline=''
    ) as f:
        writer = csv.writer(f)

        writer.writerow([
            'run',
            'odom_position_error',
            'odom_heading_error_deg',
            'truth_position_error',
            'truth_heading_error_deg',
        ])

        for index, (
            odom_result,
            truth_result
        ) in enumerate(
            zip(
                odom_results,
                truth_results
            ),
            start=1
        ):
            writer.writerow([
                index,
                f'{odom_result["position_error"]:.9f}',
                f'{odom_result["heading_error_deg"]:.9f}',
                f'{truth_result["position_error"]:.9f}',
                f'{truth_result["heading_error_deg"]:.9f}',
            ])

    print()
    print(
        f'Saved summary to: '
        f'{summary_path}'
    )


if __name__ == '__main__':
    main()