#!/usr/bin/env python3

import argparse
import csv
import math
import os

import matplotlib.pyplot as plt


def read_trajectory(csv_path):
    """
    读取 t,x,y,theta CSV。
    """

    t_values = []
    x_values = []
    y_values = []
    theta_values = []

    with open(csv_path, 'r', newline='') as f:

        reader = csv.DictReader(f)

        for row in reader:

            t_values.append(
                float(row['t'])
            )

            x_values.append(
                float(row['x'])
            )

            y_values.append(
                float(row['y'])
            )

            theta_values.append(
                float(row['theta'])
            )

    if not x_values:
        raise RuntimeError(
            f'No trajectory data in {csv_path}'
        )

    return (
        t_values,
        x_values,
        y_values,
        theta_values
    )


def transform_to_start_frame(
    x_values,
    y_values,
    theta_values
):
    """
    把轨迹转换到自己的起始坐标系：

        起点 -> (0, 0)
        初始朝向 -> 0 rad
    """

    x0 = x_values[0]
    y0 = y_values[0]
    theta0 = theta_values[0]

    cos_theta = math.cos(
        theta0
    )

    sin_theta = math.sin(
        theta0
    )

    local_x = []
    local_y = []

    for x, y in zip(
        x_values,
        y_values
    ):

        dx = x - x0
        dy = y - y0

        # 旋转 -theta0
        x_local = (
            cos_theta * dx
            + sin_theta * dy
        )

        y_local = (
            -sin_theta * dx
            + cos_theta * dy
        )

        local_x.append(
            x_local
        )

        local_y.append(
            y_local
        )

    return local_x, local_y


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

    parser.add_argument(
        '--out',
        default='plots/trajectory.png',
        help='output image path'
    )

    args = parser.parse_args()

    (
        odom_t,
        odom_x,
        odom_y,
        odom_theta
    ) = read_trajectory(
        args.odom
    )

    (
        truth_t,
        truth_x,
        truth_y,
        truth_theta
    ) = read_trajectory(
        args.truth
    )

    odom_local_x, odom_local_y = (
        transform_to_start_frame(
            odom_x,
            odom_y,
            odom_theta
        )
    )

    truth_local_x, truth_local_y = (
        transform_to_start_frame(
            truth_x,
            truth_y,
            truth_theta
        )
    )

    output_dir = os.path.dirname(
        args.out
    )

    if output_dir:
        os.makedirs(
            output_dir,
            exist_ok=True
        )

    plt.figure(
        figsize=(8, 8)
    )

    plt.plot(
        odom_local_x,
        odom_local_y,
        label='Odometry',
        linewidth=2
    )

    plt.plot(
        truth_local_x,
        truth_local_y,
        label='Gazebo Ground Truth',
        linewidth=2
    )

    # 起点
    plt.scatter(
        [0.0],
        [0.0],
        marker='o',
        s=80,
        label='Start'
    )

    # 两条轨迹各自终点
    plt.scatter(
        [odom_local_x[-1]],
        [odom_local_y[-1]],
        marker='x',
        s=100,
        label='Odom End'
    )

    plt.scatter(
        [truth_local_x[-1]],
        [truth_local_y[-1]],
        marker='+',
        s=120,
        label='Truth End'
    )

    plt.xlabel(
        'x [m]'
    )

    plt.ylabel(
        'y [m]'
    )

    plt.title(
        'Ackermann Square Trajectory'
    )

    plt.axis(
        'equal'
    )

    plt.grid(
        True
    )

    plt.legend()

    plt.tight_layout()

    plt.savefig(
        args.out,
        dpi=200
    )

    print(
        f'Trajectory plot saved to: '
        f'{args.out}'
    )


if __name__ == '__main__':
    main()