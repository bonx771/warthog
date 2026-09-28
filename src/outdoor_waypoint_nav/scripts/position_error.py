#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Real-world localization position-error evaluator.

Run this node before waypoint navigation:
  rosrun outdoor_waypoint_nav position_error.py

The node records the odometry trajectory, loads the waypoint file, transforms
waypoints into the map frame, and saves a PNG report when waypoint navigation
finishes or when the user stops the node with Ctrl-C.
"""

import csv
import json
import math
import os

import roslib.packages
import rospy
import tf
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, String
from tf.transformations import euler_from_quaternion

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
        self.encoder_odom_topic = str(
            rospy.get_param("~encoder_odom_topic", "/odometry/encoder")
        ).strip()
        self.gps_topic = str(
            rospy.get_param(
                "~gps_topic", "/outdoor_waypoint_nav/gps/fix_selected"
            )
        ).strip()
        self.plot_encoder_odometry = bool(
            rospy.get_param("~plot_encoder_odometry", True)
        )
        self.plot_gps_fixes = bool(rospy.get_param("~plot_gps_fixes", True))
        self.show_summary_panel = bool(
            rospy.get_param("~show_summary_panel", True)
        )
        self.show_title = bool(rospy.get_param("~show_title", True))
        self.legend_below = bool(rospy.get_param("~legend_below", False))
        self.legend_inside_right = bool(
            rospy.get_param("~legend_inside_right", False)
        )
        self.legend_rows = max(1, int(rospy.get_param("~legend_rows", 3)))
        self.global_ekf_color = str(
            rospy.get_param("~global_ekf_color", "#dc2626")
        )
        self.plot_local_ekf = bool(rospy.get_param("~plot_local_ekf", False))
        self.local_ekf_topic = str(
            rospy.get_param(
                "~local_ekf_topic", "/outdoor_waypoint_nav/odometry/filtered"
            )
        ).strip()
        self.local_ekf_color = str(
            rospy.get_param("~local_ekf_color", "#2563eb")
        )
        self.plot_line_width_scale = max(
            0.1, float(rospy.get_param("~plot_line_width_scale", 1.0))
        )
        self.save_plot_data = bool(rospy.get_param("~save_plot_data", False))
        self.finish_topic = rospy.get_param(
            "~finish_topic",
            "/outdoor_waypoint_nav/waypoint_following_status",
        )
        self.home_finish_topic = str(
            rospy.get_param(
                "~home_finish_topic",
                "/outdoor_waypoint_nav/home_navigation_status",
            )
        ).strip()
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
        self.gps_sample_min_period = max(
            0.0, float(rospy.get_param("~gps_sample_min_period", 0.2))
        )
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
        self.shutdown_after_report = rospy.get_param("~shutdown_after_report", True)
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
        self.local_ekf_path = []
        self.encoder_path = []
        self.gps_samples = []
        self.encoder_to_map_transform = None
        self.local_ekf_to_map_transform = None
        self.last_gps_sample_time = None
        self.latest_pose = None
        self.start_time = None
        self.finish_requested = False
        self.finish_request_time = None
        self.finish_reason = ""
        self.finish_target_index = -1
        self.finish_target_description = "final waypoint"
        self.report_written = False
        self.recording_active = not (
            self.start_on_waypoint_index_zero or bool(self.start_topic)
        )

        if not self.defer_waypoint_load:
            self._load_configured_waypoints()

        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=200)
        if self.plot_local_ekf and self.local_ekf_topic:
            rospy.Subscriber(
                self.local_ekf_topic,
                Odometry,
                self._local_ekf_cb,
                queue_size=200,
                tcp_nodelay=True,
            )
        if self.plot_encoder_odometry and self.encoder_odom_topic:
            rospy.Subscriber(
                self.encoder_odom_topic,
                Odometry,
                self._encoder_odom_cb,
                queue_size=200,
                tcp_nodelay=True,
            )
        if self.plot_gps_fixes and self.gps_topic:
            rospy.Subscriber(
                self.gps_topic,
                NavSatFix,
                self._gps_cb,
                queue_size=100,
                tcp_nodelay=True,
            )
        if self.auto_finish_on_status:
            rospy.Subscriber(self.finish_topic, Bool, self._finish_cb, queue_size=5)
            if self.home_finish_topic:
                rospy.Subscriber(
                    self.home_finish_topic,
                    Bool,
                    self._home_finish_cb,
                    queue_size=5,
                )
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
            "[final_position_error] Recording global EKF: %s | local EKF: %s "
            "| encoder: %s | GPS: %s | waypoint file: %s",
            self.odom_topic,
            self.local_ekf_topic if self.plot_local_ekf else "disabled",
            self.encoder_odom_topic if self.plot_encoder_odometry else "disabled",
            self.gps_topic if self.plot_gps_fixes else "disabled",
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
            "The report image will be generated automatically when navigation or home finishes."
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

    @staticmethod
    def _apply_planar_transform(x_value, y_value, transform):
        translation_x, translation_y, yaw = transform
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        return (
            translation_x + cosine * x_value - sine * y_value,
            translation_y + sine * x_value + cosine * y_value,
        )

    def _lookup_planar_transform(self, source_frame):
        source_frame = str(source_frame).strip()
        if not source_frame:
            source_frame = "odom"
        if source_frame.lstrip("/") == self.goal_frame.lstrip("/"):
            return 0.0, 0.0, 0.0

        translation, rotation = self.tf_listener.lookupTransform(
            self.goal_frame,
            source_frame,
            rospy.Time(0),
        )
        yaw = euler_from_quaternion(rotation)[2]
        return float(translation[0]), float(translation[1]), float(yaw)

    def _encoder_odom_cb(self, msg):
        if not self.recording_active:
            return

        if self.encoder_to_map_transform is None:
            try:
                # Freeze this transform. A time-varying map<-odom transform
                # contains global EKF/GPS corrections and would hide encoder
                # dead-reckoning drift in the comparison plot.
                self.encoder_to_map_transform = self._lookup_planar_transform(
                    msg.header.frame_id
                )
                rospy.loginfo(
                    "[final_position_error] Fixed encoder transform %s -> %s "
                    "captured for this run.",
                    msg.header.frame_id or "odom",
                    self.goal_frame,
                )
            except Exception as exc:
                rospy.logwarn_throttle(
                    2.0,
                    "[final_position_error] Waiting for encoder transform %s->%s: %s",
                    msg.header.frame_id or "odom",
                    self.goal_frame,
                    exc,
                )
                return

        position = msg.pose.pose.position
        map_x, map_y = self._apply_planar_transform(
            position.x,
            position.y,
            self.encoder_to_map_transform,
        )
        stamp = msg.header.stamp.to_sec()
        if stamp <= 0.0:
            stamp = rospy.get_time()
        pose = (map_x, map_y, stamp)

        if not self.encoder_path:
            self.encoder_path.append(pose)
            return

        last_x, last_y, last_t = self.encoder_path[-1]
        moved = math.hypot(map_x - last_x, map_y - last_y)
        elapsed = stamp - last_t
        if moved >= self.sample_min_distance or elapsed >= self.sample_max_period:
            self.encoder_path.append(pose)

    def _local_ekf_cb(self, msg):
        if not self.recording_active:
            return

        if self.local_ekf_to_map_transform is None:
            try:
                # Freeze map<-odom at the start of the run so this line shows
                # the drift of the local IMU+encoder EKF without GPS feedback.
                self.local_ekf_to_map_transform = self._lookup_planar_transform(
                    msg.header.frame_id
                )
                rospy.loginfo(
                    "[final_position_error] Fixed local-EKF transform %s -> %s "
                    "captured for this run.",
                    msg.header.frame_id or "odom",
                    self.goal_frame,
                )
            except Exception as exc:
                rospy.logwarn_throttle(
                    2.0,
                    "[final_position_error] Waiting for local-EKF transform %s->%s: %s",
                    msg.header.frame_id or "odom",
                    self.goal_frame,
                    exc,
                )
                return

        position = msg.pose.pose.position
        map_x, map_y = self._apply_planar_transform(
            position.x,
            position.y,
            self.local_ekf_to_map_transform,
        )
        stamp = msg.header.stamp.to_sec()
        if stamp <= 0.0:
            stamp = rospy.get_time()
        pose = (map_x, map_y, stamp)

        if not self.local_ekf_path:
            self.local_ekf_path.append(pose)
            return

        last_x, last_y, last_t = self.local_ekf_path[-1]
        moved = math.hypot(map_x - last_x, map_y - last_y)
        elapsed = stamp - last_t
        if moved >= self.sample_min_distance or elapsed >= self.sample_max_period:
            self.local_ekf_path.append(pose)

    def _gps_cb(self, msg):
        if not self.recording_active:
            return
        if msg.status.status < NavSatStatus.STATUS_FIX:
            return
        if not (
            math.isfinite(msg.latitude)
            and math.isfinite(msg.longitude)
            and -90.0 <= msg.latitude <= 90.0
            and -180.0 <= msg.longitude <= 180.0
        ):
            return

        stamp = msg.header.stamp.to_sec()
        if stamp <= 0.0:
            stamp = rospy.get_time()
        if (
            self.last_gps_sample_time is not None
            and stamp >= self.last_gps_sample_time
            and (stamp - self.last_gps_sample_time) < self.gps_sample_min_period
        ):
            return

        self.gps_samples.append((float(msg.latitude), float(msg.longitude), stamp))
        self.last_gps_sample_time = stamp

    def _gps_samples_in_map(self):
        if not self.gps_samples:
            return []
        try:
            utm_to_map = self._lookup_planar_transform(self.utm_frame)
            result = []
            for latitude, longitude, stamp in self.gps_samples:
                easting, northing = self._latlon_to_utm(latitude, longitude)
                map_x, map_y = self._apply_planar_transform(
                    easting, northing, utm_to_map
                )
                result.append((map_x, map_y, stamp))
            return result
        except Exception as exc:
            rospy.logwarn(
                "[final_position_error] GPS samples could not be transformed "
                "from %s to %s: %s",
                self.utm_frame,
                self.goal_frame,
                exc,
            )
            return []

    def _finish_cb(self, msg):
        if not msg.data:
            return
        self._request_finish("waypoint completion signal", -1, "final waypoint")

    def _home_finish_cb(self, msg):
        if not msg.data:
            return
        self._request_finish("home completion signal", 0, "initial waypoint")

    def _request_finish(self, reason, target_index, target_description):
        if self.finish_requested:
            return
        if not self.recording_active:
            rospy.logwarn(
                "[final_position_error] Ignoring %s received before the run started.",
                reason,
            )
            return
        self.finish_requested = True
        self.finish_request_time = rospy.Time.now()
        self.finish_reason = reason
        self.finish_target_index = int(target_index)
        self.finish_target_description = str(target_description)
        rospy.loginfo(
            "[final_position_error] Received %s. Target is %s. "
            "Waiting %.1fs before writing the report.",
            reason,
            self.finish_target_description,
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
        self.local_ekf_path = []
        self.encoder_path = []
        self.gps_samples = []
        self.encoder_to_map_transform = None
        self.local_ekf_to_map_transform = None
        self.last_gps_sample_time = None
        self.latest_pose = None
        self.start_time = None
        self.finish_requested = False
        self.finish_request_time = None
        self.finish_reason = ""
        self.finish_target_index = -1
        self.finish_target_description = "final waypoint"
        rospy.loginfo(
            "[final_position_error] Received %s; recording started.", reason
        )
        self._try_update_map_waypoints(blocking_timeout=0.0)

    def _reset_for_next_run(self):
        """Re-arm a K4 evaluator after a complete PNG report is written."""
        self.path = []
        self.local_ekf_path = []
        self.encoder_path = []
        self.gps_samples = []
        self.encoder_to_map_transform = None
        self.local_ekf_to_map_transform = None
        self.last_gps_sample_time = None
        self.latest_pose = None
        self.start_time = None
        self.finish_requested = False
        self.finish_request_time = None
        self.finish_reason = ""
        self.finish_target_index = -1
        self.finish_target_description = "final waypoint"
        self.report_written = False
        self.current_run_directory = ""
        self.recording_active = not (
            self.start_on_waypoint_index_zero or bool(self.start_topic)
        )
        rospy.loginfo(
            "[final_position_error] Re-armed for the next run on the same route."
        )

    def _path_length(self, path=None):
        path = self.path if path is None else path
        total = 0.0
        for index in range(1, len(path)):
            total += math.hypot(
                path[index][0] - path[index - 1][0],
                path[index][1] - path[index - 1][1],
            )
        return total

    def _report_target(self):
        if not self.map_waypoints:
            raise RuntimeError("No transformed waypoint is available for report target")

        target_index = int(self.finish_target_index)
        if target_index < 0:
            target_index = len(self.map_waypoints) + target_index
        target_index = max(0, min(len(self.map_waypoints) - 1, target_index))
        return target_index, self.map_waypoints[target_index]

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

    @staticmethod
    def _write_xy_csv(path, points, start_time):
        """Write an already map-aligned trajectory without resampling it."""
        with open(path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(("stamp_sec", "elapsed_sec", "map_x_m", "map_y_m"))
            for x_value, y_value, stamp in points:
                elapsed = max(0.0, stamp - start_time) if start_time else 0.0
                writer.writerow(
                    (
                        "{:.9f}".format(stamp),
                        "{:.9f}".format(elapsed),
                        "{:.9f}".format(x_value),
                        "{:.9f}".format(y_value),
                    )
                )

    def _write_plot_data(
        self,
        report_dir,
        gps_path,
        target_index,
        target_name,
        target_description,
        goal,
        actual,
        final_error,
        duration,
        path_length,
        reason,
    ):
        """Save every series needed to reproduce the result figure offline."""
        if not self.save_plot_data:
            return

        plot_data_dir = os.path.join(report_dir, "plot_data")
        os.makedirs(plot_data_dir, exist_ok=True)
        self._write_xy_csv(
            os.path.join(plot_data_dir, "global_ekf.csv"),
            self.path,
            self.start_time,
        )
        self._write_xy_csv(
            os.path.join(plot_data_dir, "local_ekf.csv"),
            self.local_ekf_path,
            self.start_time,
        )
        self._write_xy_csv(
            os.path.join(plot_data_dir, "encoder_odometry.csv"),
            self.encoder_path,
            self.start_time,
        )
        self._write_xy_csv(
            os.path.join(plot_data_dir, "gps_fixes.csv"),
            gps_path,
            self.start_time,
        )

        metadata = {
            "target_index": int(target_index),
            "target_name": str(target_name),
            "target_description": str(target_description),
            "goal": [float(goal[0]), float(goal[1])],
            "actual": [float(actual[0]), float(actual[1])],
            "final_error_m": float(final_error),
            "trajectory_length_m": float(path_length),
            "duration_sec": float(duration),
            "stop_reason": str(reason),
            "line_width_scale": float(self.plot_line_width_scale),
        }
        with open(
            os.path.join(plot_data_dir, "metadata.json"),
            "w",
            encoding="utf-8",
        ) as handle:
            json.dump(metadata, handle, indent=2, sort_keys=True)
            handle.write("\n")

        rospy.loginfo(
            "[final_position_error] Saved offline plot data: %s",
            plot_data_dir,
        )

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
                "cannot compute position error."
            )
            return

        target_index, target = self._report_target()
        goal_x, goal_y = target
        target_name = "WP{}".format(target_index + 1)
        target_description = self.finish_target_description or "waypoint"
        actual_x, actual_y, final_time = self.latest_pose
        error_x = actual_x - goal_x
        error_y = actual_y - goal_y
        final_error = math.hypot(error_x, error_y)
        duration = max(0.0, final_time - self.start_time) if self.start_time else 0.0
        path_length = self._path_length()
        gps_path = self._gps_samples_in_map()

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

        try:
            self._write_plot_data(
                report_dir=report_dir,
                gps_path=gps_path,
                target_index=target_index,
                target_name=target_name,
                target_description=target_description,
                goal=(goal_x, goal_y),
                actual=(actual_x, actual_y),
                final_error=final_error,
                duration=duration,
                path_length=path_length,
                reason=reason,
            )
        except Exception as exc:
            rospy.logerr(
                "[final_position_error] Could not save offline plot data: %s", exc
            )

        self._plot_report(
            output_path=output_path,
            reason=reason,
            goal=(goal_x, goal_y),
            actual=(actual_x, actual_y),
            error=(error_x, error_y, final_error),
            target_name=target_name,
            target_description=target_description,
            duration=duration,
            path_length=path_length,
            gps_path=gps_path,
        )

        rospy.loginfo(
            "[final_position_error] Position error to %s (%s): %.3f m",
            target_name,
            target_description,
            final_error,
        )
        rospy.loginfo("[final_position_error] Saved report image: %s", output_path)

    def _plot_report(
        self,
        output_path,
        reason,
        goal,
        actual,
        error,
        target_name,
        target_description,
        duration,
        path_length,
        gps_path,
    ):
        plt = self._load_pyplot()

        path_x = [pose[0] for pose in self.path]
        path_y = [pose[1] for pose in self.path]
        waypoint_x = [point[0] for point in self.map_waypoints]
        waypoint_y = [point[1] for point in self.map_waypoints]
        local_ekf_x = [pose[0] for pose in self.local_ekf_path]
        local_ekf_y = [pose[1] for pose in self.local_ekf_path]
        encoder_x = [pose[0] for pose in self.encoder_path]
        encoder_y = [pose[1] for pose in self.encoder_path]
        gps_x = [pose[0] for pose in gps_path]
        gps_y = [pose[1] for pose in gps_path]

        # A narrower canvas avoids unused space on both sides when the legend
        # is placed below the equal-aspect trajectory axes.
        figure_size = (9, 8) if self.legend_below else (11, 8)
        fig, ax = plt.subplots(figsize=figure_size)
        if self.show_summary_panel:
            fig.subplots_adjust(right=0.72)
        if self.show_title:
            ax.set_title("Real-World Localization Evaluation - Position Error")
        ax.set_xlabel("Map X (m)")
        ax.set_ylabel("Map Y (m)")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, alpha=0.35)

        ax.plot(
            path_x,
            path_y,
            color=self.global_ekf_color,
            linewidth=2.0 * self.plot_line_width_scale,
            label="Global EKF",
        )
        if local_ekf_x:
            ax.plot(
                local_ekf_x,
                local_ekf_y,
                color=self.local_ekf_color,
                linewidth=1.8 * self.plot_line_width_scale,
                linestyle="-.",
                alpha=0.95,
                label="Local EKF",
            )
        if encoder_x:
            ax.plot(
                encoder_x,
                encoder_y,
                color="#16a34a",
                linewidth=1.8 * self.plot_line_width_scale,
                linestyle="-",
                alpha=0.95,
                label="Encoder odometry",
            )
        if gps_x:
            ax.scatter(
                gps_x,
                gps_y,
                s=14,
                color="#d4b000",
                edgecolors="none",
                alpha=0.7,
                zorder=3,
                label="GPS fixes (available)",
            )
        ax.plot(
            waypoint_x,
            waypoint_y,
            "--",
            color="#64748b",
            linewidth=1.4 * self.plot_line_width_scale,
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
                fontsize=10,
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
            label="Target {}".format(target_name),
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
            linewidth=2.0 * self.plot_line_width_scale,
            label="Position error to {}".format(target_name),
        )

        mid_x = (goal[0] + actual[0]) * 0.5
        mid_y = (goal[1] + actual[1]) * 0.5
        error_annotation = ax.text(
            mid_x,
            mid_y,
            "{:.3f} m".format(error[2]),
            fontsize=10,
            color="#dc2626",
            weight="bold",
            bbox=dict(boxstyle="round,pad=0.25", facecolor="white", edgecolor="#dc2626"),
        )

        encoder_final_separation = None
        if self.encoder_path:
            encoder_final_separation = math.hypot(
                self.encoder_path[-1][0] - actual[0],
                self.encoder_path[-1][1] - actual[1],
            )

        stats = [
            "EVALUATION SUMMARY",
            "Metric: position error to {}".format(target_name),
            "",
            "Target waypoint: {} ({})".format(target_name, target_description),
            "Target (map):      x={:.3f}, y={:.3f}".format(goal[0], goal[1]),
            "Actual pose:       x={:.3f}, y={:.3f}".format(actual[0], actual[1]),
            "",
            "dx = {:+.3f} m".format(error[0]),
            "dy = {:+.3f} m".format(error[1]),
            "Position error = {:.3f} m".format(error[2]),
            "",
            "Waypoints = {}".format(len(self.map_waypoints)),
            "Trajectory samples = {}".format(len(self.path)),
            "Trajectory length = {:.2f} m".format(path_length),
            "Encoder samples = {}".format(len(self.encoder_path)),
            "Encoder length = {:.2f} m".format(self._path_length(self.encoder_path)),
            "Encoder/EKF final gap = {}".format(
                "{:.3f} m".format(encoder_final_separation)
                if encoder_final_separation is not None
                else "not available"
            ),
            "Valid GPS fixes plotted = {}".format(len(gps_path)),
            "Recording duration = {:.1f} s".format(duration),
            "Stop reason = {}".format(reason),
        ]
        if self.show_summary_panel:
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

        # Keep the travelled distance with the plotted-series legend so it
        # remains visible even when the separate summary panel is disabled.
        ax.plot(
            [], [], linestyle="none", marker="",
            label="Trajectory length = {:.2f} m".format(path_length),
        )
        if self.legend_inside_right:
            # Reserve a clear band to the right of every plotted data point.
            # This keeps the legend inside the axes without covering the
            # target marker or its final-position error annotation.
            x_left, x_right = ax.get_xlim()
            x_span = max(1e-6, x_right - x_left)
            ax.set_xlim(x_left, x_right + 0.15 * x_span)
            legend = ax.legend(
                loc="lower right",
                bbox_to_anchor=(0.995, 0.015),
                ncol=1,
                fontsize=10,
                frameon=True,
                framealpha=0.92,
            )
            fig.tight_layout()
            # Check the rendered geometry as well. If a wide legend still
            # touches the target, final pose, or error text, grow only the
            # right side until those artists are clear.
            for _ in range(6):
                fig.canvas.draw()
                renderer = fig.canvas.get_renderer()
                legend_box = legend.get_window_extent(renderer=renderer)
                error_box = error_annotation.get_window_extent(renderer=renderer)
                target_px = ax.transData.transform(goal)
                actual_px = ax.transData.transform(actual)

                def point_overlaps(box, point, padding=18.0):
                    return (
                        box.x0 < point[0] + padding
                        and box.x1 > point[0] - padding
                        and box.y0 < point[1] + padding
                        and box.y1 > point[1] - padding
                    )

                boxes_overlap = (
                    legend_box.x0 < error_box.x1
                    and legend_box.x1 > error_box.x0
                    and legend_box.y0 < error_box.y1
                    and legend_box.y1 > error_box.y0
                )
                if not (
                    boxes_overlap
                    or point_overlaps(legend_box, target_px)
                    or point_overlaps(legend_box, actual_px)
                ):
                    break

                x_left, x_right = ax.get_xlim()
                ax.set_xlim(x_left, x_right + 0.12 * (x_right - x_left))
                fig.tight_layout()
        elif self.legend_below:
            handles, labels = ax.get_legend_handles_labels()
            legend_columns = max(
                1, int(math.ceil(len(handles) / float(self.legend_rows)))
            )
            ax.legend(
                handles,
                labels,
                loc="upper center",
                bbox_to_anchor=(0.5, -0.13),
                ncol=legend_columns,
                fontsize=7.5,
                columnspacing=0.8,
                handletextpad=0.5,
                frameon=True,
            )
            # Anchoring to the axes keeps the four-row legend close to the
            # x-label even when equal aspect makes the plot vertically short.
            fig.tight_layout()
        else:
            ax.legend(loc="best", fontsize=8)
            if not self.show_summary_panel:
                fig.tight_layout()
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
                    if self.shutdown_after_report:
                        break
                    rate.sleep()
                    continue
                self._reset_for_next_run()

            if self.waypoints_loaded or self.recording_active:
                self._try_update_map_waypoints()

            if self.finish_requested and self.finish_request_time is not None:
                elapsed = (rospy.Time.now() - self.finish_request_time).to_sec()
                if elapsed >= self.finish_delay_sec:
                    self._write_report(self.finish_reason or "completion signal")
                    if not self.allow_multiple_runs and self.shutdown_after_report:
                        rospy.signal_shutdown("position error report written")
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
