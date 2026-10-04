#!/usr/bin/env python3

import argparse
import csv
import math
import os

import matplotlib.pyplot as plt


def read_trajectory(csv_path):
    """
    Read trajectory CSV with columns:
    t,x,y,theta
    """
    t_values = []
    x_values = []
    y_values = []
    theta_values = []

    with open(csv_path, 'r', newline='') as f:
        reader = csv.DictReader(f)

        for row in reader:
            t_values.append(float(row['t']))
            x_values.append(float(row['x']))
            y_values.append(float(row['y']))
            theta_values.append(float(row['theta']))

    if len(x_values) < 2:
        raise RuntimeError(
            f'Not enough trajectory data in {csv_path}'
        )

    return {
        't': t_values,
        'x': x_values,
        'y': y_values,
        'theta': theta_values,
    }


def normalize_angle(angle):
    """
    Normalize angle to (-pi, pi].
    """
    return math.atan2(
        math.sin(angle),
        math.cos(angle)
    )


def transform_to_start_frame(
    x_values,
    y_values,
    theta0
):
    """
    Translate the start point to (0, 0),
    then rotate the trajectory so the initial heading is 0 rad.
    """
    x0 = x_values[0]
    y0 = y_values[0]

    cos_theta = math.cos(theta0)
    sin_theta = math.sin(theta0)

    x_local = []
    y_local = []

    for x, y in zip(
        x_values,
        y_values
    ):
        dx = x - x0
        dy = y - y0

        x_rotated = (
            cos_theta * dx
            + sin_theta * dy
        )

        y_rotated = (
            -sin_theta * dx
            + cos_theta * dy
        )

        x_local.append(
            x_rotated
        )

        y_local.append(
            y_rotated
        )

    return x_local, y_local


def calculate_closed_loop_error(
    trajectory
):
    """
    Calculate closed-loop position error and
    absolute normalized heading error.
    """
    dx = (
        trajectory['x'][-1]
        - trajectory['x'][0]
    )

    dy = (
        trajectory['y'][-1]
        - trajectory['y'][0]
    )

    position_error = math.hypot(
        dx,
        dy
    )

    dtheta = normalize_angle(
        trajectory['theta'][-1]
        - trajectory['theta'][0]
    )

    heading_error_deg = abs(
        math.degrees(dtheta)
    )

    return (
        position_error,
        heading_error_deg
    )


def detect_turn_direction(
    truth_y_local
):
    """
    Detect whether the current rounded-square path
    is mainly above or below the starting x-axis.

    +1: counter-clockwise / left-turn path
    -1: clockwise / right-turn path
    """
    max_y = max(truth_y_local)
    min_y = min(truth_y_local)

    if abs(max_y) >= abs(min_y):
        return 1

    return -1


def draw_reference_square(
    ax,
    side,
    turn_radius,
    turn_direction
):
    """
    Draw the 2 m outer square envelope.

    The current path starts on one side of the square,
    travels in +x initially, then uses rounded corners.

    For side = 2.0 m and R = 0.6 m:
        straight length = side - 2R = 0.8 m
    """
    x_min = -turn_radius
    x_max = side - turn_radius

    if turn_direction > 0:
        y_min = 0.0
        y_max = side
        horizontal_dimension_y = (
            y_max + 0.12
        )
    else:
        y_min = -side
        y_max = 0.0
        horizontal_dimension_y = (
            y_min - 0.12
        )

    square_x = [
        x_min,
        x_max,
        x_max,
        x_min,
        x_min,
    ]

    square_y = [
        y_min,
        y_min,
        y_max,
        y_max,
        y_min,
    ]

    ax.plot(
        square_x,
        square_y,
        linestyle='--',
        linewidth=1.2,
        color='gray',
        alpha=0.7,
        label='2.0 m outer boundary'
    )

    # Horizontal 2 m dimension
    ax.annotate(
        '',
        xy=(
            x_max,
            horizontal_dimension_y
        ),
        xytext=(
            x_min,
            horizontal_dimension_y
        ),
        arrowprops={
            'arrowstyle': '<->',
            'linewidth': 1.2,
            'color': 'black',
        }
    )

    ax.text(
        (x_min + x_max) / 2.0,
        horizontal_dimension_y,
        f'{side:.1f} m',
        horizontalalignment='center',
        verticalalignment='bottom'
        if turn_direction > 0
        else 'top',
        fontsize=10
    )

    # Vertical 2 m dimension
    vertical_dimension_x = (
        x_max + 0.12
    )

    ax.annotate(
        '',
        xy=(
            vertical_dimension_x,
            y_max
        ),
        xytext=(
            vertical_dimension_x,
            y_min
        ),
        arrowprops={
            'arrowstyle': '<->',
            'linewidth': 1.2,
            'color': 'black',
        }
    )

    ax.text(
        vertical_dimension_x + 0.04,
        (y_min + y_max) / 2.0,
        f'{side:.1f} m',
        rotation=90,
        horizontalalignment='left',
        verticalalignment='center',
        fontsize=10
    )


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        '--odom',
        type=str,
        required=True,
        help='odom CSV file'
    )

    parser.add_argument(
        '--truth',
        type=str,
        required=True,
        help='Gazebo ground-truth CSV file'
    )

    parser.add_argument(
        '--out',
        type=str,
        required=True,
        help='output PNG path'
    )

    parser.add_argument(
        '--side',
        type=float,
        default=2.0,
        help='outer square side length in meters'
    )

    parser.add_argument(
        '--turn-radius',
        type=float,
        default=0.6,
        help='rounded-corner turning radius in meters'
    )

    args = parser.parse_args()

    odom = read_trajectory(
        args.odom
    )

    truth = read_trajectory(
        args.truth
    )

    odom_x_local, odom_y_local = (
        transform_to_start_frame(
            odom['x'],
            odom['y'],
            odom['theta'][0]
        )
    )

    truth_x_local, truth_y_local = (
        transform_to_start_frame(
            truth['x'],
            truth['y'],
            truth['theta'][0]
        )
    )

    (
        odom_position_error,
        odom_heading_error
    ) = calculate_closed_loop_error(
        odom
    )

    (
        truth_position_error,
        truth_heading_error
    ) = calculate_closed_loop_error(
        truth
    )

    turn_direction = detect_turn_direction(
        truth_y_local
    )

    output_dir = os.path.dirname(
        args.out
    )

    if output_dir:
        os.makedirs(
            output_dir,
            exist_ok=True
        )

    fig, ax = plt.subplots(
        figsize=(9, 8)
    )

    # Required colors:
    # odom = red
    # truth = green
    ax.plot(
        odom_x_local,
        odom_y_local,
        color='red',
        linewidth=2.0,
        label='/odom'
    )

    ax.plot(
        truth_x_local,
        truth_y_local,
        color='green',
        linewidth=2.0,
        label='Gazebo Ground Truth'
    )

    draw_reference_square(
        ax,
        side=args.side,
        turn_radius=args.turn_radius,
        turn_direction=turn_direction
    )

    # Start point
    ax.scatter(
        [0.0],
        [0.0],
        marker='*',
        s=160,
        color='black',
        zorder=5,
        label='Start'
    )

    # Odom end point
    ax.scatter(
        [odom_x_local[-1]],
        [odom_y_local[-1]],
        marker='o',
        s=75,
        color='red',
        edgecolors='black',
        zorder=5,
        label='Odom End'
    )

    # Truth end point
    ax.scatter(
        [truth_x_local[-1]],
        [truth_y_local[-1]],
        marker='o',
        s=75,
        color='green',
        edgecolors='black',
        zorder=5,
        label='Truth End'
    )

    error_text = (
        'Closed-loop error\n'
        f'Odom:  {odom_position_error:.3f} m, '
        f'{odom_heading_error:.2f} deg\n'
        f'Truth: {truth_position_error:.3f} m, '
        f'{truth_heading_error:.2f} deg'
    )

    ax.text(
        0.02,
        0.98,
        error_text,
        transform=ax.transAxes,
        verticalalignment='top',
        horizontalalignment='left',
        fontsize=10,
        bbox={
            'boxstyle': 'round',
            'facecolor': 'white',
            'alpha': 0.85,
        }
    )

    straight_length = (
        args.side
        - 2.0 * args.turn_radius
    )

    ax.set_title(
        'Ackermann Square Trajectory\n'
        f'Outer side = {args.side:.1f} m, '
        f'R = {args.turn_radius:.1f} m, '
        f'straight = {straight_length:.1f} m'
    )

    ax.set_xlabel(
        'x (m)'
    )

    ax.set_ylabel(
        'y (m)'
    )

    # Important requirement:
    # equal x/y scale
    ax.set_aspect(
        'equal',
        adjustable='box'
    )

    ax.grid(
        True,
        linestyle=':',
        alpha=0.6
    )

    ax.legend(
        loc='best'
    )

    fig.tight_layout()

    fig.savefig(
        args.out,
        dpi=200,
        bbox_inches='tight'
    )

    print(
        f'Saved trajectory plot to: '
        f'{args.out}'
    )

    print()
    print(
        'ODOM closed-loop error: '
        f'{odom_position_error:.4f} m, '
        f'{odom_heading_error:.2f} deg'
    )

    print(
        'TRUTH closed-loop error: '
        f'{truth_position_error:.4f} m, '
        f'{truth_heading_error:.2f} deg'
    )


if __name__ == '__main__':
    main()