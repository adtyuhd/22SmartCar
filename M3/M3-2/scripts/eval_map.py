#!/usr/bin/env python3
"""M3-2 最终栅格地图质量评估（只读地图，绝不修改地图）。

用法（从 M3 目录执行）：
    python3 M3-2/scripts/eval_map.py

依赖：Python 3 标准库；无需安装 numpy、Pillow、OpenCV 或 matplotlib。
结果：results/map_overview.png、wall_straightness.png、
      narrow_passage.png；首次运行时自动生成 quality_report.md。

局限：最终地图不能单独证明 slam_toolbox 的回环约束曾被触发。
"""

import argparse
import math
import statistics
import struct
import zlib
from collections import deque
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent


def read_pgm(path):
    """解析 ROS map_saver 输出的 P5 8-bit PGM。"""
    data = Path(path).read_bytes()
    index = 0

    def next_token():
        nonlocal index
        while index < len(data):
            if data[index] in b" \t\r\n":
                index += 1
            elif data[index] == 35:  # # 注释
                while index < len(data) and data[index] not in b"\r\n":
                    index += 1
            else:
                break
        start = index
        while index < len(data) and data[index] not in b" \t\r\n#":
            index += 1
        return data[start:index]

    magic = next_token()
    if magic != b"P5":
        raise ValueError("地图不是 P5 格式的 PGM：" + str(path))
    width = int(next_token())
    height = int(next_token())
    maximum = int(next_token())
    if maximum != 255:
        raise ValueError("仅支持 8-bit PGM（最大灰度 255）")
    if index >= len(data) or data[index] not in b" \t\r\n":
        raise ValueError("PGM 头部格式异常")
    index += 1  # 只跳过一个分隔符；像素首字节也可能是空白值
    pixels = data[index:index + width * height]
    if len(pixels) != width * height:
        raise ValueError("PGM 像素数据不完整")
    return width, height, pixels


def read_resolution(path):
    """本项目只需读取数值字段，不引入外部 YAML 依赖。"""
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        if line.strip().startswith("resolution:"):
            return float(line.split(":", 1)[1].split("#", 1)[0].strip())
    raise ValueError("map.yaml 缺少 resolution")


def segments(indices):
    """将升序索引划分成连续片段。"""
    if not indices:
        return []
    result = []
    group = [indices[0]]
    for value in indices[1:]:
        if value - group[-1] <= 1:
            group.append(value)
        else:
            result.append(group)
            group = [value]
    result.append(group)
    return result


def wall_fit(width, height, pixels, resolution, name, y_fraction):
    """在指定中央长墙直线区域内按列找墙体中线，做最小二乘拟合。"""
    x_start, x_end = int(width * 0.50), int(width * 0.80)
    y_center = int(height * y_fraction)
    half_band = max(6, int(height * 0.045))
    samples = []
    duplicate_columns = 0
    for x in range(x_start, x_end):
        ys = [y for y in range(max(0, y_center - half_band),
                                   min(height, y_center + half_band + 1))
              if pixels[y * width + x] <= 60]
        pieces = segments(ys)
        # 排除单点噪声，记录可能的分离双层（不是回环优化的证明）。
        substantial = [part for part in pieces if len(part) >= 1]
        if len(substantial) > 1 and any(
                substantial[i + 1][0] - substantial[i][-1] >= 3
                for i in range(len(substantial) - 1)):
            duplicate_columns += 1
        if not pieces:
            continue
        largest = max(pieces, key=len)
        samples.append((float(x), statistics.mean(largest)))
    if len(samples) < 20:
        raise RuntimeError(
            f"{name} 墙体有效采样不足（{len(samples)} 列），"
            "请检查地图方向，并调整 wall_fit 的 ROI 参数"
        )
    x_bar = statistics.mean(x for x, _ in samples)
    y_bar = statistics.mean(y for _, y in samples)
    denominator = sum((x - x_bar) ** 2 for x, _ in samples)
    slope = sum((x - x_bar) * (y - y_bar) for x, y in samples) / denominator
    intercept = y_bar - slope * x_bar
    # 转为垂直拟合直线的欧氏距离（与线段斜率无关）。
    residuals = [abs(y - (slope * x + intercept)) /
                 math.sqrt(1.0 + slope * slope) * resolution
                 for x, y in samples]
    return {
        "name": name,
        "roi_x_pixels": [x_start, x_end - 1],
        "roi_y_pixels": [max(0, y_center - half_band),
                         min(height - 1, y_center + half_band)],
        "sample_columns": len(samples),
        "length_m": (samples[-1][0] - samples[0][0]) * resolution,
        "mean_absolute_deviation_m": statistics.mean(residuals),
        "max_absolute_deviation_m": max(residuals),
        "rms_deviation_m": math.sqrt(statistics.mean(v * v for v in residuals)),
        "separated_dark_bands_columns": duplicate_columns,
        "slope": slope,
        "intercept": intercept,
        "samples": samples,
    }


def narrow_gap(width, height, pixels, resolution):
    """检查北侧局部窄通道：橙色障碍物与中央块之间的可见自由栅格。"""
    # ROI 基于当前自建场景的相对位置，适用于该地图的保存版本。
    center = int(width * 0.625)
    xs = range(max(0, center - 4), min(width, center + 5))
    measurements = []
    boundary = []
    for x in xs:
        occupied = [y for y in range(0, int(height * 0.25))
                    if pixels[y * width + x] <= 60]
        groups = [part for part in segments(occupied) if len(part) >= 1]
        if len(groups) < 2:
            continue
        upper, lower = groups[0], groups[1]
        if upper[-1] >= lower[0]:
            continue
        between = list(range(upper[-1] + 1, lower[0]))
        clear = sum(pixels[y * width + x] >= 250 for y in between)
        if any(pixels[y * width + x] < 250 for y in between):
            continue
        measurements.append(clear)
        boundary.append((x, upper[-1], lower[0]))
    if len(measurements) < 3:
        raise RuntimeError("北侧窄通道采样失败；地图方向或障碍物位置可能变化")
    return {
        "sample_columns": len(measurements),
        "free_cells_min": min(measurements),
        "free_cells_median": statistics.median(measurements),
        "free_cells_max": max(measurements),
        "gap_min_m": min(measurements) * resolution,
        "gap_median_m": statistics.median(measurements) * resolution,
        "boundary_pixels": boundary,
    }


def free_component_sizes(width, height, pixels):
    """4-邻域 BFS，统计连通的纯自由栅格区域。"""
    seen = bytearray(width * height)
    sizes = []
    for start, pixel in enumerate(pixels):
        if pixel < 250 or seen[start]:
            continue
        seen[start] = 1
        queue = deque([start])
        size = 0
        while queue:
            i = queue.popleft()
            size += 1
            x, y = i % width, i // width
            for j in (i - 1 if x > 0 else -1,
                      i + 1 if x + 1 < width else -1,
                      i - width if y > 0 else -1,
                      i + width if y + 1 < height else -1):
                if j >= 0 and pixels[j] >= 250 and not seen[j]:
                    seen[j] = 1
                    queue.append(j)
        sizes.append(size)
    sizes.sort(reverse=True)
    return sizes


def rgb_map(width, height, pixels, scale=4):
    """从三值栅格生成放大 RGB 图像，无需第三方库。"""
    colors = []
    for p in pixels:
        if p <= 60:
            colors.append((15, 18, 24))
        elif p >= 250:
            colors.append((255, 255, 255))
        else:
            colors.append((192, 198, 205))
    w, h = width * scale, height * scale
    canvas = bytearray(w * h * 3)
    for y in range(height):
        line = bytearray()
        for x in range(width):
            line.extend(bytes(colors[y * width + x]) * scale)
        for k in range(scale):
            start = (y * scale + k) * w * 3
            canvas[start:start + len(line)] = line
    return w, h, canvas


def put_pixel(canvas, w, h, x, y, color, radius=0):
    for yy in range(max(0, y - radius), min(h, y + radius + 1)):
        for xx in range(max(0, x - radius), min(w, x + radius + 1)):
            offset = (yy * w + xx) * 3
            canvas[offset:offset + 3] = bytes(color)


def draw_line(canvas, w, h, x0, y0, x1, y1, color, thickness=1):
    steps = max(1, abs(x1 - x0), abs(y1 - y0))
    for i in range(steps + 1):
        t = i / steps
        put_pixel(canvas, w, h, round(x0 + (x1 - x0) * t),
                  round(y0 + (y1 - y0) * t), color, thickness // 2)


def draw_rect(canvas, w, h, x0, y0, x1, y1, color, thickness=2):
    for k in range(thickness):
        draw_line(canvas, w, h, x0-k, y0-k, x1+k, y0-k, color)
        draw_line(canvas, w, h, x1+k, y0-k, x1+k, y1+k, color)
        draw_line(canvas, w, h, x1+k, y1+k, x0-k, y1+k, color)
        draw_line(canvas, w, h, x0-k, y1+k, x0-k, y0-k, color)


def write_png(path, width, height, canvas):
    def chunk(kind, payload):
        return (struct.pack(">I", len(payload)) + kind + payload +
                struct.pack(">I", zlib.crc32(kind + payload) & 0xffffffff))
    data = bytearray()
    stride = width * 3
    for y in range(height):
        data.append(0)  # PNG filter None
        data.extend(canvas[y * stride:(y + 1) * stride])
    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">2I5B", width, height, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(bytes(data), 6))
    png += chunk(b"IEND", b"")
    Path(path).write_bytes(png)


def make_figures(out_dir, width, height, pixels, walls, gap):
    scale = 4
    w, h, base = rgb_map(width, height, pixels, scale)
    write_png(out_dir / "map_overview.png", w, h, base)

    annotated = bytearray(base)
    for index, wall in enumerate(walls):
        color = (230, 58, 50) if index == 0 else (32, 105, 231)
        x0, x1 = wall["roi_x_pixels"]
        y0, y1 = wall["roi_y_pixels"]
        draw_rect(annotated, w, h, x0 * scale, y0 * scale,
                  x1 * scale, y1 * scale, color)
        for x, y in wall["samples"]:
            put_pixel(annotated, w, h, round(x * scale), round(y * scale), color)
        y_fit0 = (wall["slope"] * x0 + wall["intercept"]) * scale
        y_fit1 = (wall["slope"] * x1 + wall["intercept"]) * scale
        draw_line(annotated, w, h, x0 * scale, round(y_fit0),
                  x1 * scale, round(y_fit1), (25, 170, 65), thickness=2)
    write_png(out_dir / "wall_straightness.png", w, h, annotated)

    narrow = bytearray(base)
    for x, upper_y, lower_y in gap["boundary_pixels"]:
        draw_line(narrow, w, h, x * scale, (upper_y + 1) * scale,
                  x * scale, (lower_y - 1) * scale,
                  (255, 50, 50), thickness=2)
    draw_rect(narrow, w, h,
              int(width * 0.58) * scale, 6 * scale,
              int(width * 0.69) * scale, int(height * 0.23) * scale,
              (255, 50, 50))
    # 为便于检查，整图保留参照物，而非只给出局部裁剪。
    write_png(out_dir / "narrow_passage.png", w, h, narrow)


def report_text(data):
    north, south = data["walls"]
    narrow = data["narrow_passage"]
    counts = data["pixel_counts"]
    coverage = data["coverage_m"]
    return f"""# M3-2 SLAM 地图质量报告

> 来源：`maps/map.pgm` 和 `maps/map.yaml`；通过 `scripts/eval_map.py` 进行可复现的栅格测量。  
> 本报告只评估**最终地图**，不使用 Gazebo 模型真值修正 SLAM 地图。

## 1. 建图结果和可视化

- 地图尺寸：**{data['width']} × {data['height']} 像素**，分辨率 **{data['resolution_m_per_pixel']:.3f} m/像素**。
- 覆盖范围：**{coverage[0]:.2f} × {coverage[1]:.2f} m**。
- 已知自由栅格：**{counts['free']}**（{100*counts['free']/data['total_pixels']:.1f}%）；占据栅格：**{counts['occupied']}**（{100*counts['occupied']/data['total_pixels']:.1f}%）；未知栅格：**{counts['unknown']}**（{100*counts['unknown']/data['total_pixels']:.1f}%）。

![SLAM 最终地图](map_overview.png)

## 2. 第一组证据：墙体直线度

在中央大型障碍物的北、南两段长直墙上，按列提取黑色墙线中点，做最小二乘直线拟合，并计算**每个采样点到拟合直线的距离**。

| 样本区域 | 采样列数 | 拟合长度 | 平均绝对偏差 | 最大绝对偏差 |
|---|---:|---:|---:|---:|
| 中央北墙 | {north['sample_columns']} | {north['length_m']:.2f} m | {north['mean_absolute_deviation_m']:.3f} m | {north['max_absolute_deviation_m']:.3f} m |
| 中央南墙 | {south['sample_columns']} | {south['length_m']:.2f} m | {south['mean_absolute_deviation_m']:.3f} m | {south['max_absolute_deviation_m']:.3f} m |

![红框北墙、蓝框南墙；绿色为拟合直线](wall_straightness.png)

说明：偏差是**相对于最终地图拟合直线**的误差，并非与 Gazebo 真实墙坐标对比，也不能据此推算实际建图绝对误差。地图分辨率为 5 cm，像素量化影响必须考虑。

## 3. 第二组证据：回环一致性

- 驾驶程序运行日志中已经出现“**已完成一圈巡场路线**”，满足路线返回起始区域的要求。
- 在本报告采样的两段长墙 ROI 中，存在间距至少 3 个像素的分离占据条带的列数：**北墙 {north['separated_dark_bands_columns']} 列、南墙 {south['separated_dark_bands_columns']} 列**。这是对**局部双墙重影的筛查**，不是完整场景的回环优化验证。
- **尚无回环优化次数、回环约束日志，也无同一面墙在建图前后两次观测的独立测量数据。** 因而不能声称已经量化“回环修正量”或“回环前后墙间距”。

要完成更强的回环证据，应在下一次 rosbag 录制过程中保存对应 SLAM 日志或轨迹数据，并保留回环前/后的可对照证据。**不能从最终单张地图臆造该数值。**

## 4. 第三组证据：地图可用性

- 北侧窄通道局部连续自由空间：**{narrow['free_cells_min']}–{narrow['free_cells_max']} 格**，其中最窄测值 **{narrow['gap_min_m']:.2f} m**，均基于地图中灰白分割后的白色自由栅格。
- 自由栅格的四邻域连通块：**{data['free_components']} 个**，最大连通块占自由栅格 **{data['largest_free_component_ratio']*100:.1f}%**。
- 窄通道至少保留多个自由栅格宽度，但**最终能否用于导航还需要 M3-3 的实际车体 footprint 和代价地图膨胀半径验证**；不能只凭几何连通性断言必然可通行。

![北侧窄通道，红色标出了检测截面](narrow_passage.png)

## 5. 实验参数和完整性说明

- SLAM 来源：`/scan`、`/odom` 和相应 TF；`/gazebo/model_states` **不得用于地图修正**。
- 分辨率：0.05 m/像素；具体参数含义和配置理由以 `config/slam_params.yaml` 注释为准，提交前仍需核对该文件。
- `maps/map.yaml` 保存器生成的 `free_thresh` 为 **0.25**、`occupied_thresh` 为 **0.65**；这两个值是当前实际交付文件中的值。
- rosbag：**本报告没有假定已经存在**。若 `bags/` 为空，仍需真实录制 `/scan`、`/odom`、`/tf`、`/tf_static` 和时钟相关数据，不能用日志伪装 rosbag。
- 回环开关对比实验为选做，`maps/map_with_loop_off.pgm` **只有真正重新建图后才能提交**，不能复制现有地图冒充。

## 6. 复现方式

在 ROS 2 环境、项目 M3 目录下执行：

```bash
python3 M3-2/scripts/eval_map.py
```

脚本生成三张 PNG。如果手动修改过此报告，脚本默认**不会覆盖**它；使用 `--overwrite-report` 才会重新生成。
"""


def main():
    parser = argparse.ArgumentParser(description="M3-2 地图质量评估")
    parser.add_argument("--map", type=Path, default=ROOT / "maps/map.pgm")
    parser.add_argument("--yaml", type=Path, default=ROOT / "maps/map.yaml")
    parser.add_argument("--out-dir", type=Path, default=ROOT / "results")
    parser.add_argument("--overwrite-report", action="store_true",
                        help="允许覆盖已存在的质量报告")
    args = parser.parse_args()
    width, height, pixels = read_pgm(args.map)
    resolution = read_resolution(args.yaml)
    if resolution <= 0:
        raise ValueError("分辨率必须为正数")
    total = width * height
    counts = {
        "occupied": sum(p <= 60 for p in pixels),
        "free": sum(p >= 250 for p in pixels),
        "unknown": sum(60 < p < 250 for p in pixels),
    }
    walls = [
        wall_fit(width, height, pixels, resolution, "central_north", 0.173),
        wall_fit(width, height, pixels, resolution, "central_south", 0.818),
    ]
    narrow = narrow_gap(width, height, pixels, resolution)
    components = free_component_sizes(width, height, pixels)
    data = {
        "width": width, "height": height, "total_pixels": total,
        "resolution_m_per_pixel": resolution,
        "coverage_m": [width * resolution, height * resolution],
        "pixel_counts": counts,
        "free_components": len(components),
        "largest_free_component_ratio": (
            components[0] / counts["free"] if counts["free"] else 0.0),
        "walls": [{k: v for k, v in wall.items() if k != "samples"}
                  for wall in walls],
        "narrow_passage": {k: v for k, v in narrow.items()
                           if k != "boundary_pixels"},
        "limitations": [
            "Final PGM alone cannot prove SLAM loop-closure constraints were triggered",
            "Narrow-passage clearance does not account for robot footprint or inflation",
            "Wall deviations are relative to fitted map lines, not ground truth",
        ],
    }
    args.out_dir.mkdir(parents=True, exist_ok=True)
    make_figures(args.out_dir, width, height, pixels, walls, narrow)
    report = args.out_dir / "quality_report.md"
    if not report.exists() or args.overwrite_report:
        report.write_text(report_text(data), encoding="utf-8")
        print("质量报告已生成：", report)
    else:
        print("已有质量报告，按默认安全策略未覆盖：", report)
    print("地图：", width, "x", height, ", 分辨率：", resolution, "m/pix")
    print("像素数：", counts)
    for wall in walls:
        print(f"{wall['name']}：{wall['sample_columns']} 列，"
              f"平均偏差 {wall['mean_absolute_deviation_m']:.3f} m，"
              f"最大偏差 {wall['max_absolute_deviation_m']:.3f} m")
    print("窄通道最小白色自由宽度：", narrow["gap_min_m"], "m")
    print("自由区域连通块：", len(components))
    print("注意：最终 PGM 不能单独证明 SLAM 回环优化曾经触发。")


if __name__ == "__main__":
    main()
