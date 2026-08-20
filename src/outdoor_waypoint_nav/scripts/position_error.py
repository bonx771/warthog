#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Real-world localization final-position error evaluator.

Run this node before waypoint navigation:
  rosrun outdoor_waypoint_nav position_error.py

The node records the odometry trajectory, loads the waypoint file, transforms
waypoints into the map frame, and saves a PNG report when waypoint navigation
finishes or when the user stops the node with Ctrl-C.
"""

import math
import os

import roslib.packages
import rospy
import tf
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

try:
    import utm
except ImportError:
    utm = None

try:
    from pyproj import Transformer
except ImportError:
    Transformer = None


class FinalPositionErrorEvaluator:
    def __init__(self):
        rospy.init_node("real_world_localization_error_eval")

        self.package_dir = roslib.packages.get_pkg_dir("outdoor_waypoint_nav")
        default_waypoint_file = rospy.get_param(
            "/outdoor_waypoint_nav/coordinates_file",
            "/waypoint_files/points_outdoor.txt",
        )

        self.odom_topic = rospy.get_param(
            "~odom_topic",
            "/outdoor_waypoint_nav/odometry/filtered_map",
        )
        self.finish_topic = rospy.get_param(
            "~finish_topic",
            "/outdoor_waypoint_nav/waypoint_following_status",
        )
        configured_waypoint_file = rospy.get_param("~waypoint_file", "")
        if not configured_waypoint_file:
            configured_waypoint_file = default_waypoint_file
        self.waypoint_file = self._resolve_package_path(configured_waypoint_file)
        self.goal_frame = rospy.get_param("~goal_frame", "map")
        self.utm_frame = rospy.get_param("~utm_frame", "utm")
        self.output_dir = self._resolve_package_path(
            rospy.get_param("~output_dir", "/results")
        )
        self.show_plot = rospy.get_param("~show_plot", True)
        self.auto_finish_on_status = rospy.get_param("~auto_finish_on_status", True)
        self.finish_delay_sec = rospy.get_param("~finish_delay_sec", 0.5)
        self.sample_min_distance = rospy.get_param("~sample_min_distance", 0.03)
        self.sample_max_period = rospy.get_param("~sample_max_period", 0.5)
        self.tf_retry_period = rospy.get_param("~tf_retry_period", 0.5)
        self.tf_wait_timeout = rospy.get_param("~tf_wait_timeout", 0.2)
        self.waypoint_file_type = rospy.get_param("~waypoint_file_type", "auto")
        # Scenario 4 creates its route interactively after this evaluator has
        # started.  These optional controls keep the normal K1-K3 behavior
        # unchanged while allowing K4 to load the route only when r starts the
        # actual waypoint run.
        self.defer_waypoint_load = rospy.get_param("~defer_waypoint_load", False)
        self.start_on_waypoint_index_zero = rospy.get_param(
            "~start_on_waypoint_index_zero", False
        )
        self.reached_waypoint_topic = rospy.get_param(
            "~reached_waypoint_topic",
            "/outdoor_waypoint_nav/waypoint_reached_index",
        )
        self.start_topic = str(rospy.get_param("~start_topic", "")).strip()
        self.allow_multiple_runs = rospy.get_param("~allow_multiple_runs", False)
        self.run_directory_topic = str(
            rospy.get_param(
                "~run_directory_topic",
                "/outdoor_waypoint_nav/experiment_logger/run_directory",
            )
        ).strip()
        self.current_run_directory = ""

        os.makedirs(self.output_dir, exist_ok=True)

        self.tf_listener = tf.TransformListener()
        self._utm_transformers = {}
        self.raw_waypoints = []
        self.map_waypoints = []
        self.waypoints_loaded = False
        self.last_tf_attempt = rospy.Time(0)

        self.path = []
        self.latest_pose = None
        self.start_time = None
        self.finish_requested = False
        self.finish_request_time = None
        self.report_written = False
        self.recording_active = not (
            self.start_on_waypoint_index_zero or bool(self.start_topic)
        )

        if not self.defer_waypoint_load:
            self._load_configured_waypoints()

        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=200)
        if self.auto_finish_on_status:
            rospy.Subscriber(self.finish_topic, Bool, self._finish_cb, queue_size=5)
        if self.start_topic:
            rospy.Subscriber(self.start_topic, Bool, self._start_cb, queue_size=5)
        elif self.start_on_waypoint_index_zero:
            from std_msgs.msg import Int32

            rospy.Subscriber(
                self.reached_waypoint_topic,
                Int32,
                self._reached_waypoint_cb,
                queue_size=10,
            )
        if self.run_directory_topic:
            rospy.Subscriber(
                self.run_directory_topic,
                String,
                self._run_directory_cb,
                queue_size=1,
            )
        rospy.on_shutdown(self._on_shutdown)

        rospy.loginfo(
            "[final_position_error] Recording odometry: %s | waypoint file: %s",
            self.odom_topic,
            self.waypoint_file,
        )
        if self.defer_waypoint_load:
            rospy.loginfo(
                "[final_position_error] Waypoint file loading is deferred until the run starts."
            )
        if self.start_topic:
            rospy.loginfo(
                "[final_position_error] Waiting for %s = true before recording.",
                self.start_topic,
            )
        elif self.start_on_waypoint_index_zero:
            rospy.loginfo(
                "[final_position_error] Waiting for %s = 0 before recording.",
                self.reached_waypoint_topic,
            )
        rospy.loginfo(
            "[final_position_error] Start this node before waypoint navigation. "
            "The report image will be generated automatically when navigation finishes."
        )

    def _resolve_package_path(self, value):
        if os.path.isabs(value) and os.path.exists(value):
            return value
        package_relative = value[1:] if value.startswith("/") else value
        return os.path.join(self.package_dir, package_relative)

    def _load_waypoint_pairs(self, path):
        if not os.path.exists(path):
            raise RuntimeError("Waypoint file not found: {}".format(path))

        values = []
        with open(path, "r", encoding="utf-8") as waypoint_file:
            for line in waypoint_file:
                stripped = line.strip()
                if not stripped or stripped.startswith("#"):
                    continue
                for token in stripped.replace(",", " ").split():
                    values.append(float(token))

        if len(values) < 2 or len(values) % 2 != 0:
            raise RuntimeError(
                "Waypoint file must contain lat/lon or x/y pairs. "
                "Value count: {}".format(len(values))
            )

        pairs = [(values[i], values[i + 1]) for i in range(0, len(values), 2)]
        rospy.loginfo("[final_position_error] Loaded %d waypoint(s).", len(pairs))
        return pairs

    def _load_configured_waypoints(self):
        self.raw_waypoints = self._load_waypoint_pairs(self.waypoint_file)
        self.map_waypoints = []
        self.waypoints_loaded = True

    def _coordinates_are_gps(self):
        if self.waypoint_file_type == "gps":
            return True
        if self.waypoint_file_type == "map":
            return False

        for first, second in self.raw_waypoints:
            if abs(first) > 90.0 or abs(second) > 180.0:
                return False
        return True

    def _latlon_to_utm(self, latitude_deg, longitude_deg):
        """Convert WGS-84 lat/lon to UTM without requiring the optional utm module."""
        if utm is not None:
            easting, northing, _, _ = utm.from_latlon(latitude_deg, longitude_deg)
            return easting, northing

        if Transformer is None:
            raise RuntimeError(
                "Missing UTM converter: install python3-utm or python3-pyproj"
            )

        zone = max(1, min(60, int((longitude_deg + 180.0) / 6.0) + 1))
        epsg = (32600 if latitude_deg >= 0.0 else 32700) + zone
        transformer = self._utm_transformers.get(epsg)
        if transformer is None:
            transformer = Transformer.from_crs("EPSG:4326", "EPSG:{}".format(epsg), always_xy=True)
            self._utm_transformers[epsg] = transformer
        return transformer.transform(longitude_deg, latitude_deg)

    def _try_update_map_waypoints(self, blocking_timeout=0.0):
        if not self.waypoints_loaded:
            if self.defer_waypoint_load and not self.recording_active:
                return False
            try:
                self._load_configured_waypoints()
            except Exception as exc:
                rospy.logwarn_throttle(
                    2.0,
                    "[final_position_error] Waiting for waypoint file %s: %s",
                    self.waypoint_file,
                    exc,
                )
                return False

        if self.map_waypoints:
            return True

        now = rospy.Time.now()
        if (
            blocking_timeout <= 0.0
            and self.last_tf_attempt.to_sec() > 0.0
            and (now - self.last_tf_attempt).to_sec() < self.tf_retry_period
        ):
            return False
        self.last_tf_attempt = now

        use_gps = self._coordinates_are_gps()
        converted_points = []
        source_frame = self.utm_frame if use_gps else self.goal_frame

        try:
            for first, second in self.raw_waypoints:
                if use_gps:
                    easting, northing = self._latlon_to_utm(first, second)
                    converted_points.append((easting, northing))
                else:
                    converted_points.append((first, second))

            if source_frame != self.goal_frame:
                timeout = max(self.tf_wait_timeout, blocking_timeout)
                self.tf_listener.waitForTransform(
                    self.goal_frame,
                    source_frame,
                    rospy.Time(0),
                    rospy.Duration(timeout),
                )

            transformed = []
            for x_value, y_value in converted_points:
                point = PointStamped()
                point.header.frame_id = source_frame
                point.header.stamp = rospy.Time(0)
                point.point.x = x_value
                point.point.y = y_value
                point.point.z = 0.0

                if source_frame == self.goal_frame:
                    transformed.append((x_value, y_value))
                else:
                    map_point = self.tf_listener.transformPoint(self.goal_frame, point)
                    transformed.append((map_point.point.x, map_point.point.y))

            self.map_waypoints = transformed
            rospy.loginfo(
                "[final_position_error] Transformed waypoints into %s frame.",
                self.goal_frame,
            )
            return True

        except Exception as exc:
            rospy.logwarn_throttle(
                2.0,
                "[final_position_error] Waiting for waypoint transform %s->%s: %s",
                source_frame,
                self.goal_frame,
                exc,
            )
            return False

    def _odom_cb(self, msg):
        if not self.recording_active:
            return

        stamp = msg.header.stamp.to_sec()
        if stamp <= 0.0:
            stamp = rospy.get_time()

        position = msg.pose.pose.position
        pose = (position.x, position.y, stamp)
        self.latest_pose = pose

        if self.start_time is None:
            self.start_time = stamp

        if not self.path:
            self.path.append(pose)
            return

        last_x, last_y, last_t = self.path[-1]
        moved = math.hypot(position.x - last_x, position.y - last_y)
        elapsed = stamp - last_t
        if moved >= self.sample_min_distance or elapsed >= self.sample_max_period:
            self.path.append(pose)

    def _finish_cb(self, msg):
        if not msg.data or self.finish_requested:
            return
        if not self.recording_active:
            rospy.logwarn(
                "[final_position_error] Ignoring completion received before the run started."
            )
            return
        self.finish_requested = True
        self.finish_request_time = rospy.Time.now()
        rospy.loginfo(
            "[final_position_error] Received waypoint completion signal. "
            "Waiting %.1fs before writing the report.",
            self.finish_delay_sec,
        )

    def _reached_waypoint_cb(self, msg):
        if msg.data != 0 or self.recording_active:
            return
        self._start_recording("waypoint_reached_index=0")

    def _start_cb(self, msg):
        if not msg.data or self.recording_active:
            return
        self._start_recording("start topic")

    def _run_directory_cb(self, msg):
        candidate = str(msg.data).strip()
        if not candidate:
            return
        candidate = os.path.normpath(candidate)
        if not os.path.isabs(candidate):
            candidate = os.path.join(self.package_dir, candidate)
        if not os.path.isdir(candidate):
            rospy.logwarn_throttle(
                5.0,
                "[final_position_error] CSV run directory does not exist yet: %s",
                candidate,
            )
            return
        self.current_run_directory = candidate
        rospy.loginfo(
            "[final_position_error] PNG report directory: %s",
            self.current_run_directory,
        )

    def _start_recording(self, reason):
        self.recording_active = True
        self.path = []
        self.latest_pose = None
        self.start_time = None
        self.finish_requested = False
        self.finish_request_time = None
        rospy.loginfo(
            "[final_position_error] Received %s; recording started.", reason
        )
        self._try_update_map_waypoints(blocking_timeout=0.0)

    def _reset_for_next_run(self):
        """Re-arm a K4 evaluator after a complete PNG report is written."""
        self.path = []
        self.latest_pose = None
        self.start_time = None
        self.finish_requested = False
        self.finish_request_time = None
        self.report_written = False
        self.current_run_directory = ""
        self.recording_active = not (
            self.start_on_waypoint_index_zero or bool(self.start_topic)
        )
        rospy.loginfo(
            "[final_position_error] Re-armed for the next run on the same route."
        )

    def _path_length(self):
        total = 0.0
        for index in range(1, len(self.path)):
            total += math.hypot(
                self.path[index][0] - self.path[index - 1][0],
                self.path[index][1] - self.path[index - 1][1],
            )
        return total

    def _load_pyplot(self):
        import matplotlib

        if self.show_plot and os.environ.get("DISPLAY"):
            try:
                matplotlib.use("TkAgg", force=True)
                import matplotlib.pyplot as plt

                return plt
            except Exception:
                rospy.logwarn(
                    "[final_position_error] GUI backend is unavailable; saving PNG only."
                )
                self.show_plot = False

        matplotlib.use("Agg", force=True)

        import matplotlib.pyplot as plt

        return plt

    def _write_report(self, reason):
        if self.report_written:
            return
        self.report_written = True

        if self.latest_pose is None or len(self.path) < 2:
            rospy.logwarn(
                "[final_position_error] Not enough odometry samples to plot the trajectory. "
                "Start this node before waypoint navigation."
            )
            return

        if not self._try_update_map_waypoints(blocking_timeout=3.0):
            rospy.logerr(
                "[final_position_error] Could not transform waypoints into map frame; "
                "cannot compute final-position error."
            )
            return

        goal_x, goal_y = self.map_waypoints[-1]
        actual_x, actual_y, final_time = self.latest_pose
        error_x = actual_x - goal_x
        error_y = actual_y - goal_y
        final_error = math.hypot(error_x, error_y)
        duration = max(0.0, final_time - self.start_time) if self.start_time else 0.0
        path_length = self._path_length()

        report_dir = self.current_run_directory
        if not report_dir or not os.path.isdir(report_dir):
            report_dir = self.output_dir
            if self.run_directory_topic:
                rospy.logwarn(
                    "[final_position_error] No valid CSV run directory was received; "
                    "saving the PNG to fallback directory: %s",
                    report_dir,
                )
        os.makedirs(report_dir, exist_ok=True)

        output_path = os.path.join(report_dir, "position_error.png")

        self._plot_report(
            output_path=output_path,
            reason=reason,
            goal=(goal_x, goal_y),
            actual=(actual_x, actual_y),
            error=(error_x, error_y, final_error),
            duration=duration,
            path_length=path_length,
        )

        rospy.loginfo("[final_position_error] Final-position error: %.3f m", final_error)
        rospy.loginfo("[final_position_error] Saved report image: %s", output_path)

    def _plot_report(self, output_path, reason, goal, actual, error, duration, path_length):
        plt = self._load_pyplot()

        path_x = [pose[0] for pose in self.path]
        path_y = [pose[1] for pose in self.path]
        waypoint_x = [point[0] for point in self.map_waypoints]
        waypoint_y = [point[1] for point in self.map_waypoints]

        fig, ax = plt.subplots(figsize=(11, 8))
        fig.subplots_adjust(right=0.72)
        ax.set_title("Real-World Localization Evaluation - Final Position Error")
        ax.set_xlabel("Map X (m)")
        ax.set_ylabel("Map Y (m)")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, alpha=0.35)

        ax.plot(
            path_x,
            path_y,
            color="#2563eb",
            linewidth=2.0,
            label="Recorded EKF trajectory",
        )
        ax.plot(
            waypoint_x,
            waypoint_y,
            "--",
            color="#64748b",
            linewidth=1.4,
            label="Waypoint path",
        )
        ax.scatter(
            waypoint_x,
            waypoint_y,
            s=70,
            color="#f59e0b",
            edgecolors="#111827",
            linewidths=0.8,
            zorder=5,
            label="Waypoints",
        )

        for index, (x_value, y_value) in enumerate(self.map_waypoints, start=1):
            ax.text(
                x_value,
                y_value,
                "  WP{}".format(index),
                fontsize=9,
                weight="bold",
                color="#111827",
            )

        start_x, start_y, _ = self.path[0]
        ax.scatter(
            [start_x],
            [start_y],
            s=90,
            color="#16a34a",
            edgecolors="#111827",
            zorder=6,
            label="Recording start",
        )
        ax.scatter(
            [goal[0]],
            [goal[1]],
            s=160,
            marker="*",
            color="#9333ea",
            edgecolors="#111827",
            zorder=7,
            label="Final waypoint",
        )
        ax.scatter(
            [actual[0]],
            [actual[1]],
            s=120,
            marker="X",
            color="#dc2626",
            edgecolors="#111827",
            zorder=7,
            label="Final vehicle position",
        )
        ax.plot(
            [goal[0], actual[0]],
            [goal[1], actual[1]],
            ":",
            color="#dc2626",
            linewidth=2.0,
            label="Final position error",
        )

        mid_x = (goal[0] + actual[0]) * 0.5
        mid_y = (goal[1] + actual[1]) * 0.5
        ax.text(
            mid_x,
            mid_y,
            "{:.3f} m".format(error[2]),
            fontsize=10,
            color="#dc2626",
            weight="bold",
            bbox=dict(boxstyle="round,pad=0.25", facecolor="white", edgecolor="#dc2626"),
        )

        stats = [
            "EVALUATION SUMMARY",
            "Metric: final-position error only",
            "",
            "Final waypoint: WP{}".format(len(self.map_waypoints)),
            "Target (map):      x={:.3f}, y={:.3f}".format(goal[0], goal[1]),
            "Actual final pose: x={:.3f}, y={:.3f}".format(actual[0], actual[1]),
            "",
            "dx = {:+.3f} m".format(error[0]),
            "dy = {:+.3f} m".format(error[1]),
            "Position error = {:.3f} m".format(error[2]),
            "",
            "Waypoints = {}".format(len(self.map_waypoints)),
            "Trajectory samples = {}".format(len(self.path)),
            "Trajectory length = {:.2f} m".format(path_length),
            "Recording duration = {:.1f} s".format(duration),
            "Stop reason = {}".format(reason),
        ]
        ax.text(
            1.03,
            0.98,
            "\n".join(stats),
            transform=ax.transAxes,
            va="top",
            ha="left",
            fontsize=9.5,
            family="monospace",
            bbox=dict(
                boxstyle="round,pad=0.6",
                facecolor="#f8fafc",
                edgecolor="#334155",
                alpha=0.97,
            ),
        )

        ax.legend(loc="best", fontsize=8)
        fig.savefig(output_path, dpi=160, bbox_inches="tight")

        if self.show_plot and os.environ.get("DISPLAY"):
            plt.show(block=True)
        else:
            plt.close(fig)

    def _on_shutdown(self):
        if not self.report_written:
            self._write_report("manual shutdown")

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.report_written:
                if not self.allow_multiple_runs:
                    break
                self._reset_for_next_run()

            if self.waypoints_loaded or self.recording_active:
                self._try_update_map_waypoints()

            if self.finish_requested and self.finish_request_time is not None:
                elapsed = (rospy.Time.now() - self.finish_request_time).to_sec()
                if elapsed >= self.finish_delay_sec:
                    self._write_report("waypoint completion signal")
                    if not self.allow_multiple_runs:
                        rospy.signal_shutdown("final position error report written")
                        break

            rate.sleep()


if __name__ == "__main__":
    try:
        evaluator = FinalPositionErrorEvaluator()
        evaluator.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("[final_position_error] Error: %s", exc)
        raise
