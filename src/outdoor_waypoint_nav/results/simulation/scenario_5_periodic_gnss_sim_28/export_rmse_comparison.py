#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Xuất bảng sai số định vị của Scenario 5 run 28 từ CSV đã lưu.

Ground truth là quỹ đạo GNSS tham chiếu chưa cộng nhiễu trong
``reference_gnss.csv``. Mỗi quỹ đạo ước lượng được ghép với ground truth
bằng nội suy tuyến tính theo timestamp. Không cần ROS hoặc Gazebo.

Chạy:
    python3 export_rmse_comparison.py

Đầu ra:
    localization_rmse_comparison.csv
"""

import bisect
import csv
import math
from pathlib import Path


OUTPUT_FILENAME = "localization_rmse_comparison.csv"
TRAJECTORIES = (
    ("Global EKF", "plot_data/global_ekf.csv"),
    ("Local EKF", "plot_data/local_ekf.csv"),
    ("Encoder odometry", "plot_data/encoder_odometry.csv"),
)


def _finite_float(row, key):
    value = row.get(key, "")
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _read_xy(path, require_valid_fix=False):
    points = []
    with path.open("r", newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            if require_valid_fix and str(row.get("valid_fix", "1")).strip() != "1":
                continue
            stamp = _finite_float(row, "stamp_sec")
            x_value = _finite_float(row, "map_x_m")
            y_value = _finite_float(row, "map_y_m")
            if stamp is None or x_value is None or y_value is None:
                continue
            points.append((stamp, x_value, y_value))

    # Loại timestamp trùng và bảo đảm thứ tự tăng để nội suy bằng bisect.
    unique = {}
    for stamp, x_value, y_value in points:
        unique[stamp] = (x_value, y_value)
    return [(stamp, *unique[stamp]) for stamp in sorted(unique)]


def _interpolate_reference(reference, query_stamp):
    stamps = _interpolate_reference.stamps
    if query_stamp < stamps[0] or query_stamp > stamps[-1]:
        return None

    right = bisect.bisect_left(stamps, query_stamp)
    if right < len(stamps) and stamps[right] == query_stamp:
        return reference[right][1], reference[right][2]
    if right == 0 or right >= len(reference):
        return None

    left = right - 1
    t0, x0, y0 = reference[left]
    t1, x1, y1 = reference[right]
    if t1 <= t0:
        return x0, y0
    weight = (query_stamp - t0) / (t1 - t0)
    return x0 + weight * (x1 - x0), y0 + weight * (y1 - y0)


def _path_length(points):
    return sum(
        math.hypot(
            points[index][1] - points[index - 1][1],
            points[index][2] - points[index - 1][2],
        )
        for index in range(1, len(points))
    )


def _calculate_metrics(name, source, estimate, reference):
    _interpolate_reference.stamps = [point[0] for point in reference]
    aligned = []
    for stamp, estimate_x, estimate_y in estimate:
        truth = _interpolate_reference(reference, stamp)
        if truth is None:
            continue
        dx = estimate_x - truth[0]
        dy = estimate_y - truth[1]
        aligned.append((stamp, dx, dy, math.hypot(dx, dy)))

    if not aligned:
        raise RuntimeError("Không có timestamp giao nhau cho {}".format(name))

    count = len(aligned)
    rmse_x = math.sqrt(sum(item[1] ** 2 for item in aligned) / count)
    rmse_y = math.sqrt(sum(item[2] ** 2 for item in aligned) / count)
    position_rmse = math.sqrt(
        sum(item[1] ** 2 + item[2] ** 2 for item in aligned) / count
    )
    position_mae = sum(item[3] for item in aligned) / count
    maximum = max(item[3] for item in aligned)
    final_error = aligned[-1][3]
    duration = max(0.0, aligned[-1][0] - aligned[0][0])

    return {
        "estimator": name,
        "reference": "raw simulation GNSS (pre-noise)",
        "source_csv": source,
        "aligned_samples": count,
        "evaluation_duration_s": duration,
        "rmse_x_m": rmse_x,
        "rmse_y_m": rmse_y,
        "position_rmse_m": position_rmse,
        "position_mae_m": position_mae,
        "maximum_position_error_m": maximum,
        "final_position_error_m": final_error,
        "trajectory_length_m": _path_length(estimate),
    }


def _format_value(key, value):
    if key == "aligned_samples":
        return str(int(value))
    if isinstance(value, float):
        return "{:.6f}".format(value)
    return str(value)


def main():
    run_dir = Path(__file__).resolve().parent
    reference_path = run_dir / "reference_gnss.csv"
    reference = _read_xy(reference_path, require_valid_fix=True)
    if len(reference) < 2:
        raise RuntimeError("reference_gnss.csv không có đủ ground-truth hợp lệ")

    rows = []
    for name, relative_source in TRAJECTORIES:
        source_path = run_dir / relative_source
        estimate = _read_xy(source_path)
        if len(estimate) < 2:
            raise RuntimeError("Không đủ dữ liệu trong {}".format(source_path))
        rows.append(
            _calculate_metrics(
                name, relative_source, estimate, reference
            )
        )

    fieldnames = (
        "estimator",
        "reference",
        "source_csv",
        "aligned_samples",
        "evaluation_duration_s",
        "rmse_x_m",
        "rmse_y_m",
        "position_rmse_m",
        "position_mae_m",
        "maximum_position_error_m",
        "final_position_error_m",
        "trajectory_length_m",
    )
    output_path = run_dir / OUTPUT_FILENAME
    with output_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {key: _format_value(key, row[key]) for key in fieldnames}
            )

    print("Saved: {}".format(output_path))
    print(
        "{:<18} {:>10} {:>10} {:>12} {:>12} {:>12}".format(
            "Estimator", "RMSE X", "RMSE Y", "RMSE 2D", "MAE 2D", "Max error"
        )
    )
    for row in rows:
        print(
            "{:<18} {:>10.3f} {:>10.3f} {:>12.3f} {:>12.3f} {:>12.3f}".format(
                row["estimator"],
                row["rmse_x_m"],
                row["rmse_y_m"],
                row["position_rmse_m"],
                row["position_mae_m"],
                row["maximum_position_error_m"],
            )
        )


if __name__ == "__main__":
    main()
