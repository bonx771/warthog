#!/usr/bin/env python3
"""Export a publication-ready CTE error table from saved experiment CSVs.

The calculation follows ``experiment_trajectory_csv_logger.py``: only rows
with ``rmse_used == 1`` and a finite ``cross_track_error_m`` are included.
The approach from the recording start to WP1 is therefore excluded.
"""

import argparse
import csv
import math
import os


SCENARIOS = (
    (1, "GNSS noise", "Noise run 1", "scenario_2_gnss_noise_1"),
    (2, "GNSS noise", "Noise run 3", "scenario_2_gnss_noise_3"),
    (3, "GNSS noise", "Noise run 5", "scenario_2_gnss_noise_5"),
    (4, "GNSS outage", "Outage run 1", "scenario_3_gnss_outage_1"),
    (5, "GNSS outage", "Outage run 2", "scenario_3_gnss_outage_2"),
    (6, "Obstacle avoidance", "Obstacle run 1", "scenario_obstacle_1"),
    (7, "Obstacle avoidance", "Obstacle run 2", "scenario_obstacle_2"),
)

OUTPUT_COLUMNS = (
    "order",
    "experiment_group",
    "condition",
    "scenario",
    "cte_samples",
    "max_cte_m",
    "cte_rmse_m",
    "cte_mae_m",
    "metric_frame",
    "source_csv",
)


def _parse_float(value):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _is_enabled(value):
    number = _parse_float(value)
    return number is not None and int(number) == 1


def calculate_metrics(reference_csv):
    """Return Max CTE, RMSE and MAE from logger-approved CTE samples."""
    errors = []
    metric_frames = set()

    with open(reference_csv, "r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        required = {"rmse_used", "cross_track_error_m"}
        missing = required.difference(reader.fieldnames or ())
        if missing:
            raise ValueError(
                "{} is missing columns: {}".format(
                    reference_csv, ", ".join(sorted(missing))
                )
            )

        for row in reader:
            if not _is_enabled(row.get("rmse_used")):
                continue
            error = _parse_float(row.get("cross_track_error_m"))
            if error is None:
                continue
            errors.append(abs(error))
            frame = (row.get("metric_frame") or "").strip()
            if frame:
                metric_frames.add(frame)

    if not errors:
        raise ValueError("No valid CTE samples in {}".format(reference_csv))

    count = len(errors)
    return {
        "cte_samples": count,
        "max_cte_m": max(errors),
        "cte_rmse_m": math.sqrt(sum(value * value for value in errors) / count),
        "cte_mae_m": sum(errors) / count,
        "metric_frame": "+".join(sorted(metric_frames)) or "unknown",
    }


def build_rows(root_dir):
    rows = []
    for order, group, condition, scenario in SCENARIOS:
        source = os.path.join(root_dir, scenario, "reference_gnss.csv")
        if not os.path.isfile(source):
            raise FileNotFoundError("Missing experiment CSV: {}".format(source))
        metrics = calculate_metrics(source)
        rows.append(
            {
                "order": order,
                "experiment_group": group,
                "condition": condition,
                "scenario": scenario,
                "cte_samples": metrics["cte_samples"],
                "max_cte_m": "{:.6f}".format(metrics["max_cte_m"]),
                "cte_rmse_m": "{:.6f}".format(metrics["cte_rmse_m"]),
                "cte_mae_m": "{:.6f}".format(metrics["cte_mae_m"]),
                "metric_frame": metrics["metric_frame"],
                "source_csv": os.path.relpath(source, root_dir),
            }
        )
    return rows


def write_csv(rows, output_path):
    output_dir = os.path.dirname(os.path.abspath(output_path))
    os.makedirs(output_dir, exist_ok=True)
    with open(output_path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=OUTPUT_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)


def print_table(rows):
    headers = ("No.", "Scenario", "N", "Max CTE (m)", "CTE RMSE (m)", "CTE MAE (m)")
    data = [
        (
            str(row["order"]),
            row["scenario"],
            str(row["cte_samples"]),
            row["max_cte_m"],
            row["cte_rmse_m"],
            row["cte_mae_m"],
        )
        for row in rows
    ]
    widths = [
        max(len(headers[index]), *(len(row[index]) for row in data))
        for index in range(len(headers))
    ]
    template = "  ".join("{{:<{}}}".format(width) for width in widths)
    print(template.format(*headers))
    print(template.format(*("-" * width for width in widths)))
    for row in data:
        print(template.format(*row))


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(
        description="Export Max CTE, CTE RMSE and CTE MAE for seven experiments."
    )
    parser.add_argument(
        "--output",
        default=os.path.join(script_dir, "experiment_error_metrics.csv"),
        help="Output CSV path (default: experiment_error_metrics.csv beside this script)",
    )
    args = parser.parse_args()

    rows = build_rows(script_dir)
    write_csv(rows, args.output)
    print_table(rows)
    print("\nSaved: {}".format(os.path.abspath(args.output)))


if __name__ == "__main__":
    main()
