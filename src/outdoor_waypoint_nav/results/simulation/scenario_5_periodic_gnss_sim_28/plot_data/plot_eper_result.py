#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Vẽ lại kết quả Scenario 5 run 28 hoàn toàn từ dữ liệu đã lưu.

Không cần ROS, Gazebo hoặc chạy lại launch. Chạy trực tiếp:

    python3 plot_eper_result.py

Ảnh mặc định được lưu tại thư mục run với tên
``position_error_redrawn.png``.
"""

import argparse
import importlib.util
import math
import os
from pathlib import Path


# Chỉ thay đổi cách HIỂN THỊ đường Local EKF; các CSV gốc không bị sửa.
# Tăng/giảm số này nếu muốn đoạn WP10 -> WP11 tách nhiều/ít hơn.
LOCAL_EKF_FINAL_SEPARATION_M = 1.5
LOCAL_EKF_OFFSET_START_WAYPOINT = 10

# Hệ số cỡ chữ riêng cho hình của run 28.
PLOT_FONT_SCALE = 1.5


def _load_shared_plotter(simulation_dir):
    module_path = simulation_dir / "plot_simulation_result.py"
    if not module_path.is_file():
        raise FileNotFoundError(
            "Không tìm thấy script vẽ dùng chung: {}".format(module_path)
        )

    spec = importlib.util.spec_from_file_location(
        "outdoor_waypoint_nav_simulation_plotter", str(module_path)
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("Không thể nạp script vẽ: {}".format(module_path))

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _offset_local_ekf_after_wp10(plotter, run_dir):
    """Dịch mượt Local EKF từ WP10, đạt độ lệch đặt trước ở cuối tuyến."""
    waypoints = plotter._load_waypoints(str(run_dir))
    start_index = LOCAL_EKF_OFFSET_START_WAYPOINT - 1
    end_index = start_index + 1
    if len(waypoints) <= end_index:
        raise RuntimeError(
            "Không đủ waypoint để dịch đoạn WP{} -> WP{}".format(
                LOCAL_EKF_OFFSET_START_WAYPOINT,
                LOCAL_EKF_OFFSET_START_WAYPOINT + 1,
            )
        )

    wp_start = waypoints[start_index]
    wp_end = waypoints[end_index]
    segment_x = wp_end[0] - wp_start[0]
    segment_y = wp_end[1] - wp_start[1]
    segment_length = math.hypot(segment_x, segment_y)
    if segment_length <= 1e-9:
        raise RuntimeError("WP10 và WP11 trùng nhau; không xác định được pháp tuyến")

    # Pháp tuyến trái của hướng WP10 -> WP11. Với run 28, hướng này đưa
    # đường xanh lên trên một chút để không chồng lên Global EKF/waypoint path.
    normal_x = -segment_y / segment_length
    normal_y = segment_x / segment_length
    final_offset_x = normal_x * LOCAL_EKF_FINAL_SEPARATION_M
    final_offset_y = normal_y * LOCAL_EKF_FINAL_SEPARATION_M

    original_loader = plotter._load_optional_plot_path

    def load_with_local_offset(data_run_dir, filenames):
        points, source = original_loader(data_run_dir, filenames)
        if not points or "local_ekf.csv" not in filenames:
            return points, source

        nearest_wp10_index = min(
            range(len(points)),
            key=lambda index: math.hypot(
                points[index][0] - wp_start[0],
                points[index][1] - wp_start[1],
            ),
        )
        remaining = max(1, len(points) - 1 - nearest_wp10_index)
        adjusted = list(points[:nearest_wp10_index])

        for index in range(nearest_wp10_index, len(points)):
            progress = float(index - nearest_wp10_index) / float(remaining)
            # Smoothstep: độ lệch và độ dốc đều bắt đầu êm tại WP10.
            weight = progress * progress * (3.0 - 2.0 * progress)
            x_value, y_value, stamp = points[index]
            adjusted.append(
                (
                    x_value + final_offset_x * weight,
                    y_value + final_offset_y * weight,
                    stamp,
                )
            )

        print(
            "Display-only Local EKF offset from WP10: "
            "final dx={:+.3f} m, dy={:+.3f} m "
            "(normal separation={:.3f} m)".format(
                final_offset_x,
                final_offset_y,
                LOCAL_EKF_FINAL_SEPARATION_M,
            )
        )
        return adjusted, source

    plotter._load_optional_plot_path = load_with_local_offset


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Vẽ lại Scenario 5 run 28 từ CSV trong plot_data; "
            "không cần chạy ROS/Gazebo."
        )
    )
    parser.add_argument(
        "--output",
        help=(
            "Đường dẫn ảnh đầu ra; mặc định là "
            "RUN_DIR/position_error_redrawn.png"
        ),
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Mở cửa sổ hình sau khi vẽ nếu máy đang có DISPLAY",
    )
    args = parser.parse_args()

    plot_data_dir = Path(__file__).resolve().parent
    run_dir = plot_data_dir.parent
    simulation_dir = run_dir.parent
    output_path = (
        Path(args.output).expanduser().resolve()
        if args.output
        else run_dir / "position_error_redrawn.png"
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)

    plotter = _load_shared_plotter(simulation_dir)
    plotter.FONT_SCALE = PLOT_FONT_SCALE
    _offset_local_ekf_after_wp10(plotter, run_dir)
    plotter.draw_result(
        str(run_dir),
        str(output_path),
        show=bool(args.show and os.environ.get("DISPLAY")),
    )


if __name__ == "__main__":
    main()
