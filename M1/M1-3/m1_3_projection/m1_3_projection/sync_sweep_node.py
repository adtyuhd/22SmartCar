#!/usr/bin/env python3

import argparse
import csv
import os
import statistics

import rosbag2_py

from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan, Image

from message_filters import (
    SimpleFilter,
    ApproximateTimeSynchronizer,
)


TOPIC_SCAN = '/scan'
TOPIC_IMAGE = '/camera/image_raw'


class SlopSweep:
    """
    离线 slop 扫描实验。

    核心思想：

        rosbag
          |
          | 按原始记录顺序读取
          |
          +---- scan -----> ATS 20 ms
          |             -> ATS 30 ms
          |             -> ATS 40 ms
          |             -> ...
          |
          +---- image ----> ATS 150 ms

    这里不经过 DDS，不实时播放 bag。

    因此实验只研究：
        ApproximateTimeSynchronizer 本身的匹配行为。
    """

    def __init__(
        self,
        slop_values_ms,
        queue_size,
    ):
        self.slop_values_ms = slop_values_ms
        self.queue_size = queue_size

        # 每个 slop 都有自己独立的输入 filter。
        self.scan_filters = {}
        self.image_filters = {}

        # 必须保存 synchronizer 的引用，
        # 防止被 Python 垃圾回收。
        self.synchronizers = {}

        # 保存实验结果。
        #
        # results[50] =
        # {
        #     'count': 40,
        #     'diffs_ms': [...]
        # }
        self.results = {}

        for slop_ms in self.slop_values_ms:

            scan_filter = SimpleFilter()
            image_filter = SimpleFilter()

            synchronizer = ApproximateTimeSynchronizer(
                [
                    scan_filter,
                    image_filter,
                ],
                queue_size=self.queue_size,
                slop=slop_ms / 1000.0,
            )

            # slop_ms=slop_ms 很重要。
            #
            # 它把当前循环的 slop 值固定下来，
            # 避免 lambda 最后全部引用同一个 slop。
            synchronizer.registerCallback(
                lambda scan_msg,
                       image_msg,
                       slop_ms=slop_ms:
                self.synchronized_callback(
                    slop_ms,
                    scan_msg,
                    image_msg,
                )
            )

            self.scan_filters[slop_ms] = scan_filter
            self.image_filters[slop_ms] = image_filter

            self.synchronizers[slop_ms] = synchronizer

            self.results[slop_ms] = {
                'count': 0,
                'diffs_ms': [],
            }

    @staticmethod
    def stamp_to_nanoseconds(stamp):
        """
        ROS 时间：

            sec
            nanosec

        转换成整数纳秒。

        使用整数而不是浮点秒，
        可以避免不必要的浮点误差。
        """

        return (
            int(stamp.sec) * 1_000_000_000
            + int(stamp.nanosec)
        )

    def synchronized_callback(
        self,
        slop_ms,
        scan_msg,
        image_msg,
    ):
        """
        某一个 ATS 成功配出一对 scan + image 时调用。
        """

        scan_ns = self.stamp_to_nanoseconds(
            scan_msg.header.stamp
        )

        image_ns = self.stamp_to_nanoseconds(
            image_msg.header.stamp
        )

        diff_ns = abs(
            scan_ns - image_ns
        )

        diff_ms = diff_ns / 1_000_000.0

        self.results[slop_ms]['count'] += 1

        self.results[slop_ms]['diffs_ms'].append(
            diff_ms
        )

    def feed_scan(self, msg):
        """
        把同一条 LaserScan 喂给所有 slop 实验。
        """

        for slop_ms in self.slop_values_ms:
            self.scan_filters[slop_ms].signalMessage(
                msg
            )

    def feed_image(self, msg):
        """
        把同一条 Image 喂给所有 slop 实验。
        """

        for slop_ms in self.slop_values_ms:
            self.image_filters[slop_ms].signalMessage(
                msg
            )

    def print_results(self):
        """
        打印题目最关心的两个指标：

            matched_pairs
            median_dt_ms
        """

        print()
        print('=' * 54)
        print('ApproximateTimeSynchronizer slop sweep')
        print('=' * 54)

        print(
            f'{"slop_ms":>10}'
            f'{"pairs":>12}'
            f'{"median_dt_ms":>20}'
        )

        print('-' * 54)

        for slop_ms in self.slop_values_ms:

            result = self.results[slop_ms]

            count = result['count']
            diffs = result['diffs_ms']

            if diffs:
                median_ms = statistics.median(
                    diffs
                )

                median_text = f'{median_ms:.3f}'
            else:
                median_text = 'N/A'

            print(
                f'{slop_ms:>10}'
                f'{count:>12}'
                f'{median_text:>20}'
            )

        print('=' * 54)
        print()

    def save_csv(self, output_file):
        """
        保存实验结果，方便后面写 README。
        """

        output_path = os.path.abspath(
            output_file
        )

        with open(
            output_path,
            'w',
            newline='',
            encoding='utf-8',
        ) as f:

            writer = csv.writer(f)

            writer.writerow(
                [
                    'slop_ms',
                    'matched_pairs',
                    'median_dt_ms',
                ]
            )

            for slop_ms in self.slop_values_ms:

                result = self.results[slop_ms]

                count = result['count']
                diffs = result['diffs_ms']

                if diffs:
                    median_ms = statistics.median(
                        diffs
                    )
                else:
                    median_ms = ''

                writer.writerow(
                    [
                        slop_ms,
                        count,
                        median_ms,
                    ]
                )

        print(
            f'Results saved to: {output_path}'
        )


def read_bag(
    bag_path,
    sweep,
):
    """
    使用 rosbag2_py 顺序读取 bag。

    这里只处理：
        /scan
        /camera/image_raw

    /camera/camera_info 暂时跳过。
    """

    reader = rosbag2_py.SequentialReader()

    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id='sqlite3',
    )

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )

    reader.open(
        storage_options,
        converter_options,
    )

    scan_count = 0
    image_count = 0

    while reader.has_next():

        topic, data, bag_timestamp = reader.read_next()

        if topic == TOPIC_SCAN:

            msg = deserialize_message(
                data,
                LaserScan,
            )

            sweep.feed_scan(msg)

            scan_count += 1

        elif topic == TOPIC_IMAGE:

            msg = deserialize_message(
                data,
                Image,
            )

            sweep.feed_image(msg)

            image_count += 1

    return scan_count, image_count


def main():
    parser = argparse.ArgumentParser(
        description=(
            'Offline ApproximateTimeSynchronizer '
            'slop sweep for M1-3'
        )
    )

    parser.add_argument(
        '--bag',
        default='sample_bag',
        help='rosbag directory',
    )

    parser.add_argument(
        '--queue-size',
        type=int,
        default=50,
        help='ApproximateTimeSynchronizer queue size',
    )

    parser.add_argument(
        '--start-ms',
        type=int,
        default=20,
        help='first slop value in milliseconds',
    )

    parser.add_argument(
        '--end-ms',
        type=int,
        default=150,
        help='last slop value in milliseconds',
    )

    parser.add_argument(
        '--step-ms',
        type=int,
        default=10,
        help='slop step in milliseconds',
    )

    parser.add_argument(
        '--output',
        default='sync_slop_results.csv',
        help='CSV output file',
    )

    args = parser.parse_args()

    if args.start_ms <= 0:
        raise ValueError(
            '--start-ms must be > 0'
        )

    if args.end_ms < args.start_ms:
        raise ValueError(
            '--end-ms must be >= --start-ms'
        )

    if args.step_ms <= 0:
        raise ValueError(
            '--step-ms must be > 0'
        )

    if args.queue_size <= 0:
        raise ValueError(
            '--queue-size must be > 0'
        )

    bag_path = os.path.abspath(
        args.bag
    )

    if not os.path.isdir(bag_path):
        raise FileNotFoundError(
            f'Bag directory does not exist: {bag_path}'
        )

    slop_values_ms = list(
        range(
            args.start_ms,
            args.end_ms + 1,
            args.step_ms,
        )
    )

    print(
        f'Reading bag: {bag_path}'
    )

    print(
        'Slop values: '
        + ', '.join(
            f'{value} ms'
            for value in slop_values_ms
        )
    )

    print(
        f'ATS queue size: {args.queue_size}'
    )

    sweep = SlopSweep(
        slop_values_ms=slop_values_ms,
        queue_size=args.queue_size,
    )

    scan_count, image_count = read_bag(
        bag_path,
        sweep,
    )

    print()
    print(
        f'Bag messages read: '
        f'{scan_count} scans, '
        f'{image_count} images'
    )

    sweep.print_results()

    sweep.save_csv(
        args.output
    )


if __name__ == '__main__':
    main()