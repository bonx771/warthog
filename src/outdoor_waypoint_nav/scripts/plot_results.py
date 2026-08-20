#!/usr/bin/env python3
"""
plot_results.py — Vẽ đồ thị kết quả đánh giá EKF loop closure

Cách dùng:
  rosrun outdoor_waypoint_nav plot_results.py /tmp/ekf_eval/result_sim_1234567.yaml
  python3 plot_results.py /tmp/ekf_eval/result_sim_1234567.yaml

Đầu ra:
  - Figure 1: Đường đi EKF vs GT (nếu có) + hình vuông lý tưởng
  - Figure 2: Sai số vị trí theo thời gian (EKF vs GT)
  - Figure 3: Biểu đồ tóm tắt chỉ số
  Lưu ảnh PNG cùng thư mục với file YAML
"""
import sys
import os
import math
import csv
import re
import yaml
import numpy as np
import matplotlib
matplotlib.use("TkAgg")   # dùng TkAgg cho môi trường desktop
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.gridspec import GridSpec


# ── Màu sắc ───────────────────────────────────────────────────────────────────
C_EKF  = "#00CC44"   # xanh lá — EKF path
C_GT   = "#FF4444"   # đỏ      — Ground truth
C_IDEAL= "#4488FF"   # xanh dương — Hình vuông lý tưởng
C_START= "#FFD700"   # vàng    — điểm xuất phát
C_END  = "#FF8C00"   # cam     — điểm kết thúc


def load_yaml(yaml_path):
    with open(yaml_path) as f:
        return yaml.safe_load(f)


def load_csv(csv_path):
    xs, ys, ts = [], [], []
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            xs.append(float(row["x"]))
            ys.append(float(row["y"]))
            ts.append(float(row["t"]))
    return np.array(xs), np.array(ys), np.array(ts)


def find_companion_csv(yaml_path, prefix):
    """Tìm file CSV cùng thư mục với yaml, cùng timestamp."""
    dirpath = os.path.dirname(yaml_path)
    basename = os.path.basename(yaml_path)         # result_sim_1234567.yaml
    ts_part = basename.replace("result_", "").replace(".yaml", "")  # sim_1234567
    mode_ts = ts_part                              # sim_1234567
    candidate = os.path.join(dirpath, f"{prefix}_{mode_ts}.csv")
    return candidate if os.path.exists(candidate) else None


def extract_result_timestamp(yaml_path):
    """Lấy timestamp từ tên file result_<mode>_<timestamp>.yaml."""
    basename = os.path.basename(yaml_path)
    match = re.search(r"_(\d+)\.yaml$", basename)
    if match:
        return int(match.group(1))
    return int(os.path.getmtime(yaml_path))


def ideal_square(start_x, start_y, start_yaw, side, turn_direction=1):
    """Tạo tọa độ hình vuông lý tưởng từ điểm xuất phát."""
    pts = [(start_x, start_y)]
    yaw = start_yaw
    for _ in range(4):
        x = pts[-1][0] + side * math.cos(yaw)
        y = pts[-1][1] + side * math.sin(yaw)
        pts.append((x, y))
        yaw += turn_direction * math.pi / 2
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return xs, ys


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def cumulative_distance(xs, ys):
    dist = [0.0]
    for i in range(1, len(xs)):
        dist.append(dist[-1] + math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]))
    return np.array(dist)


def index_at_distance(dist, target):
    return int(np.argmin(np.abs(dist - target)))


def estimate_heading_between_dist(xs, ys, dist, start_dist, end_dist):
    if len(xs) < 2:
        return None
    i0 = index_at_distance(dist, start_dist)
    i1 = index_at_distance(dist, end_dist)
    if i0 == i1:
        return None
    dx = xs[i1] - xs[i0]
    dy = ys[i1] - ys[i0]
    if math.hypot(dx, dy) < 0.1:
        return None
    return math.atan2(dy, dx)


def estimate_square_heading(xs, ys, side):
    """Ước lượng heading cạnh đầu và chiều quay từ chính EKF path."""
    dist = cumulative_distance(xs, ys)
    if len(dist) < 2:
        return 0.0, 1

    edge_sample = min(2.0, max(0.5, side * 0.75))
    h1 = estimate_heading_between_dist(xs, ys, dist, 0.5, edge_sample)
    if h1 is None:
        h1 = 0.0

    h2 = estimate_heading_between_dist(
        xs, ys, dist, side + 0.5, side + edge_sample
    )
    if h2 is None:
        return h1, 1

    turn_direction = 1 if normalize_angle(h2 - h1) > 0.0 else -1
    return h1, turn_direction


def compute_error_over_time(ekf_xs, ekf_ys, ekf_ts,
                             gt_xs,  gt_ys,  gt_ts):
    """Tính error EKF vs GT theo thời gian bằng nội suy."""
    errors = []
    t_out  = []
    for i, et in enumerate(ekf_ts):
        if et < gt_ts[0] or et > gt_ts[-1]:
            continue
        # Nội suy GT
        idx = np.searchsorted(gt_ts, et)
        idx = min(max(idx, 1), len(gt_ts)-1)
        alpha = (et - gt_ts[idx-1]) / (gt_ts[idx] - gt_ts[idx-1] + 1e-9)
        gx = gt_xs[idx-1] + alpha * (gt_xs[idx] - gt_xs[idx-1])
        gy = gt_ys[idx-1] + alpha * (gt_ys[idx] - gt_ys[idx-1])
        err = math.sqrt((ekf_xs[i]-gx)**2 + (ekf_ys[i]-gy)**2)
        errors.append(err)
        t_out.append(et - ekf_ts[0])
    return np.array(t_out), np.array(errors)


def plot_all(yaml_path):
    result = load_yaml(yaml_path)
    mode   = result["mode"]
    side   = result["side_length_m"]
    total  = result["total_path_m"]
    lce    = result["loop_closure_error_m"]
    lce_pct= result["loop_closure_error_pct"]
    hdg    = result["heading_error_deg"]
    grade  = result.get("grade", "N/A")
    verdict= result.get("verdict", "")

    sx = result["start_pos"]["x"]
    sy = result["start_pos"]["y"]
    ex = result["end_pos_ekf"]["x"]
    ey = result["end_pos_ekf"]["y"]

    # ── Load CSV ──────────────────────────────────────────────────────────────
    ekf_csv = find_companion_csv(yaml_path, "ekf_path")
    gt_csv  = find_companion_csv(yaml_path, "gt_path")

    has_ekf = ekf_csv is not None
    has_gt  = (mode == "sim") and (gt_csv is not None)

    print(f"[plot_results] YAML đang dùng: {yaml_path}")
    print(f"[plot_results] EKF CSV: {ekf_csv if has_ekf else 'không tìm thấy'}")
    if mode == "sim":
        print(f"[plot_results] GT CSV : {gt_csv if has_gt else 'không tìm thấy'}")

    ekf_xs = ekf_ys = ekf_ts = None
    gt_xs  = gt_ys  = gt_ts  = None

    if has_ekf:
        ekf_xs, ekf_ys, ekf_ts = load_csv(ekf_csv)
    if has_gt:
        gt_xs, gt_ys, gt_ts = load_csv(gt_csv)

    # ── Figure layout ─────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(16, 6))
    fig.suptitle(
        f"Đánh giá EKF — Loop Closure Test ({mode.upper()})  |  "
        f"{side}m × 4 = {total:.0f}m  |  LCE = {lce:.4f}m ({lce_pct:.2f}%)",
        fontsize=13, fontweight="bold"
    )
    gs = GridSpec(1, 2, figure=fig, hspace=0.4, wspace=0.35)

    # ── Plot 1: Đường đi ──────────────────────────────────────────────────────
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.set_title("Quỹ đạo di chuyển", fontsize=11)
    ax1.set_xlabel("X (m)"); ax1.set_ylabel("Y (m)")
    ax1.set_aspect("equal")
    ax1.grid(True, alpha=0.4)

    # Hình vuông lý tưởng bám theo hướng và chiều quay thật của EKF path.
    # turn_direction trong YAML là dấu cmd_vel, có thể ngược với chiều trên plot.
    if has_ekf:
        start_yaw, turn_direction = estimate_square_heading(
            ekf_xs - sx, ekf_ys - sy, side
        )
    else:
        start_yaw = result.get("start_yaw_rad", 0.0)
        turn_direction = result.get("turn_direction", 1)
    ix, iy = ideal_square(0.0, 0.0, start_yaw, side, turn_direction)
    ax1.plot(ix, iy, "--", color=C_IDEAL, lw=1.5,
             label=f"Lý tưởng ({side}m×4)", alpha=0.7)

    # Normalize về gốc (0,0) = đúng thời điểm bắt đầu chạy hình vuông
    # EKF: dùng start_pos đã lưu trong YAML
    # GT:  dùng gt_start_pos đã lưu trong YAML (không dùng gt_xs[0] vì
    #       GT ghi từ trước khi xe chạy — bao gồm cả lúc robot rơi/settling)
    ekf_x0 = result["start_pos"]["x"]
    ekf_y0 = result["start_pos"]["y"]
    if has_ekf:
        ekf_xs_n = ekf_xs - ekf_x0
        ekf_ys_n = ekf_ys - ekf_y0

    if has_gt:
        gt_start = result.get("gt_start_pos")
        if gt_start:
            gt_x0, gt_y0 = gt_start["x"], gt_start["y"]
        else:
            gt_x0, gt_y0 = gt_xs[0], gt_ys[0]
        gt_xs_n = gt_xs - gt_x0
        gt_ys_n = gt_ys - gt_y0

    # EKF path (đã normalize)
    if has_ekf:
        ax1.plot(ekf_xs_n, ekf_ys_n, "-", color=C_EKF, lw=2,
                 label="EKF (odometry/filtered)")

    # Điểm xuất phát (0, 0) và kết thúc (đã normalize)
    ex_n = ex - ekf_x0
    ey_n = ey - ekf_y0
    ax1.scatter([0.0], [0.0], c=C_START, s=120, zorder=5, label="Xuất phát (0,0)")
    ax1.scatter([ex_n], [ey_n], c=C_END, s=120, zorder=5, marker="X", label="Kết thúc (EKF)")
    # Mũi tên loop closure error
    ax1.annotate("", xy=(ex_n, ey_n), xytext=(0.0, 0.0),
                 arrowprops=dict(arrowstyle="<->", color="black", lw=1.5))
    ax1.text((ex_n)/2 + 0.1, (ey_n)/2 + 0.1,
             f"LCE={lce:.3f}m", fontsize=9, color="black")

    ax1.legend(fontsize=8, loc="best")

    # ── Plot 2: Tóm tắt chỉ số ────────────────────────────────────────────────
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.set_title("Chỉ số đánh giá", fontsize=11)
    ax2.axis("off")

    rmse_str = f"{result.get('ekf_rmse_vs_gt_m', 'N/A'):.4f} m" \
               if isinstance(result.get("ekf_rmse_vs_gt_m"), float) else "N/A"
    gt_lce_str = f"{result.get('gt_loop_closure_error_m', 'N/A'):.4f} m" \
                 if isinstance(result.get("gt_loop_closure_error_m"), float) else "N/A"

    metrics = [
        ("Chế độ",                 mode.upper()),
        ("Cạnh hình vuông",        f"{side} m"),
        ("Tổng quãng đường",       f"{total:.0f} m"),
        ("Vận tốc",                f"{result['linear_vel_ms']} m/s"),
        ("Loop Closure Error",     f"{lce:.4f} m"),
        ("LCE %",                  f"{lce_pct:.2f}%"),
        ("Heading Error",          f"{hdg:.2f}°"),
    ]
    if mode == "sim":
        metrics += [
            ("GT Loop Closure",    gt_lce_str),
            ("RMSE (EKF vs GT)",   rmse_str),
        ]
    metrics += [
        ("",                       ""),
        ("Đánh giá",               grade),
        ("Kết luận",               verdict),
    ]

    # Màu theo grade
    grade_colors = {"XUẤT SẮC": "#00CC44", "TỐT": "#88CC00",
                    "CHẤP NHẬN": "#FFAA00", "YẾU": "#FF6600", "KÉM": "#FF0000"}
    grade_color = grade_colors.get(grade, "black")

    y_pos = 0.95
    for label, val in metrics:
        if label == "":
            y_pos -= 0.03
            continue
        color = grade_color if label == "Đánh giá" else "black"
        weight = "bold" if label in ("Đánh giá", "Kết luận") else "normal"
        ax2.text(0.0, y_pos, f"{label}:", transform=ax2.transAxes,
                 fontsize=9, va="top", color="gray")
        ax2.text(0.45, y_pos, val, transform=ax2.transAxes,
                 fontsize=9, va="top", color=color, fontweight=weight)
        y_pos -= 0.08

    # ── Lưu và hiển thị ───────────────────────────────────────────────────────
    out_png = yaml_path.replace(".yaml", "_plot.png")
    plt.savefig(out_png, dpi=150, bbox_inches="tight")
    print(f"[plot_results] Đã lưu đồ thị: {out_png}")
    plt.show()


if __name__ == "__main__":
    import glob, subprocess
    if len(sys.argv) < 2:
        try:
            pkg_path = subprocess.check_output(
                ["rospack", "find", "outdoor_waypoint_nav"], text=True).strip()
            results_dir = os.path.join(pkg_path, "results")
        except Exception:
            results_dir = "/tmp/ekf_eval"
        files = glob.glob(os.path.join(results_dir, "result_*.yaml"))
        if not files:
            print(f"Không có file kết quả trong: {results_dir}")
            print("Dùng: rosrun outdoor_waypoint_nav plot_results.py <path/to/result.yaml>")
            sys.exit(1)
        yaml_path = max(files, key=lambda path: (extract_result_timestamp(path), os.path.getmtime(path)))
        print(f"[plot_results] Dùng file mới nhất: {yaml_path}")
    else:
        yaml_path = sys.argv[1]

    if not os.path.exists(yaml_path):
        print(f"Không tìm thấy file: {yaml_path}")
        sys.exit(1)

    plot_all(yaml_path)
