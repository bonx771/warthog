#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Record one outdoor direct-pursuit experiment as analysis-ready CSV files.

The logger is deliberately started with Terminal 2 but does not record while
the operator is preparing the robot.  It starts only when ``gps_waypoint``
publishes waypoint-reached index 0 after RB/``r`` starts a run, and it closes
the files when waypoint following reports completion or the launch is stopped.

Files written for every run:

* ``waypoints.csv``: requested route in latitude/longitude and local ENU.
* ``reference_gnss.csv``: raw /gps/fix reference trajectory and path CTE.
* ``controller_trajectory.csv``: EKF map pose used by direct pursuit.

The primary RMSE uses clean raw GNSS /gps/fix, not the selected GNSS stream
that is deliberately degraded or gated in Scenarios 2 and 3.  It is therefore
a GNSS-reference metric; call it ground truth only when that receiver/source
is independently validated (for example RTK Fixed).
"""

import csv
import math
import os
import re
import threading
import time
from datetime import datetime

import roslib.packages
import rospy
import tf
from geometry_msgs.msg import PointStamped, Twist, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Float64, Int32, String

try:
    from pyproj import Transformer
except ImportError:
    Transformer = None


WGS84_A_M = 6378137.0
WGS84_E2 = 6.6943799901413165e-3


def _finite(value):
    return math.isfinite(value)


def _safe_float(value):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if _finite(result) else None


def _stamp_or_now(stamp):
    value = stamp.to_sec() if stamp is not None else 0.0
    if value > 0.0:
        return value
    now = rospy.get_time()
    return now if now > 0.0 else time.time()


class ExperimentTrajectoryCsvLogger:
    """Stream direct-pursuit experiment data into one CSV directory per run."""

    def __init__(self):
        self.package_dir = roslib.packages.get_pkg_dir("outdoor_waypoint_nav")
        self.scenario_name = str(rospy.get_param("~scenario_name", "unspecified"))
        self.scenario_metadata = str(rospy.get_param("~scenario_metadata", ""))
        self.run_naming_mode = str(
            rospy.get_param("~run_naming_mode", "timestamp")
        ).strip().lower()
        if self.run_naming_mode not in ("timestamp", "sequential"):
            raise rospy.ROSException(
                "~run_naming_mode must be 'timestamp' or 'sequential', got {!r}".format(
                    self.run_naming_mode
                )
            )
        self.output_root = self._resolve_output_path(
            rospy.get_param("~output_dir", "/results/experiment_runs")
        )
        self.default_waypoint_file = rospy.get_param("~waypoint_file", "")
        if not self.default_waypoint_file:
            self.default_waypoint_file = rospy.get_param(
                "/outdoor_waypoint_nav/coordinates_file",
                "/waypoint_files/points_outdoor.txt",
            )
        self.waypoint_file_type = str(rospy.get_param("~waypoint_file_type", "auto")).lower()
        self.goal_frame = str(rospy.get_param("~goal_frame", "map"))
        self.utm_frame = str(rospy.get_param("~utm_frame", "utm"))

        self.reference_gps_topic = rospy.get_param("~reference_gps_topic", "/gps/fix")
        self.controller_odom_topic = rospy.get_param(
            "~controller_odom_topic", "/outdoor_waypoint_nav/odometry/filtered_map"
        )
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.reached_waypoint_topic = rospy.get_param(
            "~reached_waypoint_topic", "/outdoor_waypoint_nav/waypoint_reached_index"
        )
        self.start_topic = str(rospy.get_param("~start_topic", "")).strip()
        self.allow_multiple_runs = bool(rospy.get_param("~allow_multiple_runs", False))
        self.finish_topic = rospy.get_param(
            "~finish_topic", "/outdoor_waypoint_nav/waypoint_following_status"
        )
        self.finish_delay_sec = max(0.0, float(rospy.get_param("~finish_delay_sec", 0.5)))
        self.rmse_topic = rospy.get_param(
            "~rmse_topic", "/outdoor_waypoint_nav/experiment_logger/cross_track_rmse_m"
        )
        self.run_directory_topic = rospy.get_param(
            "~run_directory_topic",
            "/outdoor_waypoint_nav/experiment_logger/run_directory",
        )

        self._lock = threading.RLock()
        self._run_active = False
        self._finalized = False
        self._finish_timer = None
        self._run_start_time = None
        self._run_start_wall = None
        self._run_id = ""
        self._run_dir = ""
        self._files = {}
        self._writers = {}

        self._waypoint_file = ""
        self._waypoints = []
        self._waypoints_are_gps = False
        self._last_reached_index = 0
        self._latest_cmd_linear = 0.0
        self._latest_cmd_angular = 0.0
        self._noise_active = False
        self._noise_east_m = 0.0
        self._noise_north_m = 0.0
        self._outage_active = False
        self._outage_distance_m = 0.0
        self._last_reference_enu = None
        self._utm_transformers = {}
        self._map_transform_warning_emitted = False
        self.tf_listener = tf.TransformListener()

        self._reference_sample_count = 0
        self._rmse_sample_count = 0
        self._rmse_sum_squared = 0.0
        self._rmse_sum_absolute = 0.0
        self._rmse_max = 0.0
        self._affected_rmse_sample_count = 0
        self._affected_rmse_sum_squared = 0.0

        self.rmse_publisher = rospy.Publisher(self.rmse_topic, Float64, queue_size=1, latch=True)
        self.rmse_publisher.publish(Float64(data=float("nan")))
        self.run_directory_publisher = rospy.Publisher(
            self.run_directory_topic, String, queue_size=1, latch=True
        )
        self.run_directory_publisher.publish(String(data=""))

        rospy.Subscriber(
            self.reference_gps_topic, NavSatFix, self._reference_gps_cb, queue_size=50, tcp_nodelay=True
        )
        rospy.Subscriber(
            self.controller_odom_topic, Odometry, self._controller_odom_cb, queue_size=100, tcp_nodelay=True
        )
        rospy.Subscriber(self.cmd_vel_topic, Twist, self._cmd_vel_cb, queue_size=100, tcp_nodelay=True)
        rospy.Subscriber(
            self.reached_waypoint_topic, Int32, self._reached_waypoint_cb, queue_size=20, tcp_nodelay=True
        )
        if self.start_topic:
            rospy.Subscriber(self.start_topic, Bool, self._start_cb, queue_size=5, tcp_nodelay=True)
        rospy.Subscriber(self.finish_topic, Bool, self._finish_cb, queue_size=10, tcp_nodelay=True)
        rospy.Subscriber(
            "/outdoor_waypoint_nav/gnss_noise/active", Bool, self._noise_active_cb, queue_size=10
        )
        rospy.Subscriber(
            "/outdoor_waypoint_nav/gnss_noise/offset_enu", Vector3Stamped, self._noise_offset_cb, queue_size=20
        )
        rospy.Subscriber(
            "/outdoor_waypoint_nav/gnss_outage/active", Bool, self._outage_active_cb, queue_size=10
        )
        rospy.Subscriber(
            "/outdoor_waypoint_nav/gnss_outage/distance_m", Float64, self._outage_distance_cb, queue_size=20
        )
        rospy.on_shutdown(self._on_shutdown)

        rospy.loginfo(
            "[experiment_csv] Armed for %s. Recording begins when %s.",
            self.scenario_name,
            "the start topic {} publishes true".format(self.start_topic)
            if self.start_topic
            else "{} publishes 0".format(self.reached_waypoint_topic),
        )

    def _resolve_output_path(self, value):
        value = str(value)
        # The package convention uses /results/... as a package-relative path,
        # while an explicit arbitrary absolute path (for example /tmp/foo) is
        # respected even before that directory exists.
        if value == "/results" or value.startswith("/results/"):
            return os.path.join(self.package_dir, value[1:])
        if os.path.isabs(value):
            return value
        return os.path.join(self.package_dir, value)

    def _resolve_waypoint_path(self, value):
        value = str(value)
        if os.path.exists(value):
            return value
        package_relative = value[1:] if value.startswith("/") else value
        return os.path.join(self.package_dir, package_relative)

    @staticmethod
    def _coordinates_are_gps(pairs, requested_type):
        if requested_type == "gps":
            return True
        if requested_type == "map":
            return False
        return all(abs(first) <= 90.0 and abs(second) <= 180.0 for first, second in pairs)

    @staticmethod
    def _local_enu(latitude_deg, longitude_deg, origin_latitude_deg, origin_longitude_deg):
        """Small-area WGS-84 ENU approximation; precise for this ~300 m route."""
        latitude_rad = math.radians(latitude_deg)
        origin_latitude_rad = math.radians(origin_latitude_deg)
        longitude_delta_rad = math.radians(longitude_deg - origin_longitude_deg)
        latitude_delta_rad = latitude_rad - origin_latitude_rad
        sin_origin = math.sin(origin_latitude_rad)
        denominator = math.sqrt(1.0 - WGS84_E2 * sin_origin * sin_origin)
        transverse_radius = WGS84_A_M / denominator
        meridional_radius = (
            WGS84_A_M * (1.0 - WGS84_E2) / (denominator * denominator * denominator)
        )
        east = longitude_delta_rad * transverse_radius * math.cos(origin_latitude_rad)
        north = latitude_delta_rad * meridional_radius
        return east, north

    def _latlon_to_utm(self, latitude_deg, longitude_deg):
        """Return WGS-84 UTM easting/northing, or ``(None, None)`` if unavailable."""
        if Transformer is None:
            return None, None
        zone = max(1, min(60, int((longitude_deg + 180.0) / 6.0) + 1))
        epsg = (32600 if latitude_deg >= 0.0 else 32700) + zone
        transformer = self._utm_transformers.get(epsg)
        if transformer is None:
            transformer = Transformer.from_crs("EPSG:4326", "EPSG:{}".format(epsg), always_xy=True)
            self._utm_transformers[epsg] = transformer
        return transformer.transform(longitude_deg, latitude_deg)

    def _utm_to_map(self, easting, northing):
        """Transform a UTM point into the controller map frame when TF is ready."""
        if easting is None or northing is None:
            return None
        try:
            point = PointStamped()
            point.header.frame_id = self.utm_frame
            point.header.stamp = rospy.Time(0)
            point.point.x = easting
            point.point.y = northing
            point.point.z = 0.0
            transformed = self.tf_listener.transformPoint(self.goal_frame, point)
            return transformed.point.x, transformed.point.y
        except Exception as exc:
            if not self._map_transform_warning_emitted:
                rospy.logwarn(
                    "[experiment_csv] UTM->%s TF is not ready; CSV still contains local ENU "
                    "reference/path columns. Map columns will be blank: %s",
                    self.goal_frame,
                    exc,
                )
                self._map_transform_warning_emitted = True
            return None

    @staticmethod
    def _project_to_segment(point_east, point_north, start, end):
        segment_east = end[0] - start[0]
        segment_north = end[1] - start[1]
        length_sq = segment_east * segment_east + segment_north * segment_north
        if length_sq <= 1e-12:
            return None

        offset_east = point_east - start[0]
        offset_north = point_north - start[1]
        projection_raw = (
            offset_east * segment_east + offset_north * segment_north
        ) / length_sq
        projection = max(0.0, min(1.0, projection_raw))
        closest_east = start[0] + projection * segment_east
        closest_north = start[1] + projection * segment_north
        error_east = point_east - closest_east
        error_north = point_north - closest_north
        error = math.hypot(error_east, error_north)
        segment_length = math.sqrt(length_sq)
        signed_error = (
            (segment_east * offset_north - segment_north * offset_east) / segment_length
        )
        return {
            "error": error,
            "signed_error": signed_error,
            "along_track": projection_raw * segment_length,
            "projection": projection_raw,
            "projection_clamped": projection,
            "closest_east": closest_east,
            "closest_north": closest_north,
            "segment_length": segment_length,
        }

    def _load_waypoints(self):
        self.goal_frame = str(
            rospy.get_param("/outdoor_waypoint_nav/goal_frame", self.goal_frame)
        )
        configured_path = rospy.get_param(
            "/outdoor_waypoint_nav/coordinates_file", self.default_waypoint_file
        )
        waypoint_path = self._resolve_waypoint_path(configured_path)
        if not os.path.isfile(waypoint_path):
            raise RuntimeError("Waypoint file not found: {}".format(waypoint_path))

        values = []
        with open(waypoint_path, "r", encoding="utf-8") as waypoint_file:
            for line in waypoint_file:
                stripped = line.strip()
                if not stripped or stripped.startswith("#"):
                    continue
                values.extend(float(token) for token in stripped.replace(",", " ").split())

        if len(values) < 2 or len(values) % 2 != 0:
            raise RuntimeError("Waypoint file must contain complete coordinate pairs")

        pairs = [(values[index], values[index + 1]) for index in range(0, len(values), 2)]
        are_gps = self._coordinates_are_gps(pairs, self.waypoint_file_type)
        records = []
        if are_gps:
            origin_latitude, origin_longitude = pairs[0]
            for index, (latitude, longitude) in enumerate(pairs, start=1):
                east, north = self._local_enu(
                    latitude, longitude, origin_latitude, origin_longitude
                )
                utm_east, utm_north = self._latlon_to_utm(latitude, longitude)
                map_point = self._utm_to_map(utm_east, utm_north)
                records.append(
                    {
                        "index": index,
                        "latitude": latitude,
                        "longitude": longitude,
                        "east": east,
                        "north": north,
                        "utm_east": utm_east,
                        "utm_north": utm_north,
                        "map_x": map_point[0] if map_point is not None else None,
                        "map_y": map_point[1] if map_point is not None else None,
                    }
                )
        else:
            for index, (east, north) in enumerate(pairs, start=1):
                records.append(
                    {
                        "index": index,
                        "latitude": None,
                        "longitude": None,
                        "east": east,
                        "north": north,
                        "utm_east": None,
                        "utm_north": None,
                        "map_x": east,
                        "map_y": north,
                    }
                )

        self._waypoint_file = waypoint_path
        self._waypoints_are_gps = are_gps
        self._waypoints = records

    def _open_csv(self, key, filename, fieldnames):
        path = os.path.join(self._run_dir, filename)
        handle = open(path, "w", newline="", encoding="utf-8", buffering=1)
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        self._files[key] = handle
        self._writers[key] = writer

    def _write_row(self, key, row):
        writer = self._writers.get(key)
        if writer is not None:
            writer.writerow(row)

    def _write_waypoints_csv(self):
        path = os.path.join(self._run_dir, "waypoints.csv")
        with open(path, "w", newline="", encoding="utf-8") as handle:
            fieldnames = [
                "waypoint_index",
                "latitude_deg",
                "longitude_deg",
                "local_east_m",
                "local_north_m",
                "utm_easting_m",
                "utm_northing_m",
                "map_x_m",
                "map_y_m",
            ]
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            for waypoint in self._waypoints:
                writer.writerow(
                    {
                        "waypoint_index": waypoint["index"],
                        "latitude_deg": waypoint["latitude"],
                        "longitude_deg": waypoint["longitude"],
                        "local_east_m": waypoint["east"],
                        "local_north_m": waypoint["north"],
                        "utm_easting_m": waypoint["utm_east"],
                        "utm_northing_m": waypoint["utm_north"],
                        "map_x_m": waypoint["map_x"],
                        "map_y_m": waypoint["map_y"],
                    }
                )

    def _create_run_files(self):
        safe_scenario = re.sub(r"[^A-Za-z0-9_.-]+", "_", self.scenario_name).strip("_")
        safe_scenario = safe_scenario or "scenario"
        if self.run_naming_mode == "sequential":
            os.makedirs(self.output_root, exist_ok=True)
            pattern = re.compile(r"^{}_(\d+)$".format(re.escape(safe_scenario)))
            existing_indices = []
            for entry in os.listdir(self.output_root):
                match = pattern.match(entry)
                if match and os.path.isdir(os.path.join(self.output_root, entry)):
                    existing_indices.append(int(match.group(1)))
            next_index = max(existing_indices, default=0) + 1
            while True:
                self._run_id = "{}_{}".format(safe_scenario, next_index)
                self._run_dir = os.path.join(self.output_root, self._run_id)
                try:
                    os.makedirs(self._run_dir, exist_ok=False)
                    break
                except FileExistsError:
                    next_index += 1
        else:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            base_run_id = "{}_{}".format(safe_scenario, timestamp)
            suffix = 1
            while True:
                self._run_id = base_run_id if suffix == 1 else "{}_{}".format(base_run_id, suffix)
                self._run_dir = os.path.join(self.output_root, self._run_id)
                try:
                    os.makedirs(self._run_dir, exist_ok=False)
                    break
                except FileExistsError:
                    suffix += 1

        self._open_csv(
            "reference",
            "reference_gnss.csv",
            [
                "stamp_sec", "elapsed_sec", "latitude_deg", "longitude_deg", "altitude_m",
                "local_east_m", "local_north_m", "utm_easting_m", "utm_northing_m",
                "map_x_m", "map_y_m", "navsat_status", "valid_fix",
                "covariance_xx_m2", "covariance_yy_m2", "last_reached_wp",
                "active_target_wp", "segment_start_wp", "segment_end_wp",
                "segment_label", "rmse_used", "cross_track_error_m",
                "signed_cross_track_error_m", "along_track_m", "projection_unclamped", "metric_frame",
                "noise_active", "noise_east_m", "noise_north_m", "outage_active",
                "outage_distance_m",
            ],
        )
        self._open_csv(
            "controller",
            "controller_trajectory.csv",
            [
                "stamp_sec", "elapsed_sec", "map_x_m", "map_y_m", "yaw_rad",
                "linear_x_mps", "angular_z_rps", "last_reached_wp", "active_target_wp",
                "target_local_east_m", "target_local_north_m", "target_map_x_m", "target_map_y_m",
                "latest_cmd_linear_x_mps",
                "latest_cmd_angular_z_radps", "noise_active", "outage_active",
            ],
        )
        self._write_waypoints_csv()

    def _elapsed(self, stamp):
        return stamp - self._run_start_time if self._run_start_time is not None else 0.0

    def _target_waypoint_index(self):
        waypoint_count = len(self._waypoints)
        if waypoint_count == 0 or self._last_reached_index >= waypoint_count:
            return 0
        return self._last_reached_index + 1

    def _target_waypoint(self):
        target_index = self._target_waypoint_index()
        if target_index <= 0:
            return target_index, None
        return target_index, self._waypoints[target_index - 1]

    def _current_route_segment(self):
        target_index = self._target_waypoint_index()
        if target_index < 2 or target_index > len(self._waypoints):
            return None
        start = self._waypoints[target_index - 2]
        end = self._waypoints[target_index - 1]
        use_utm = all(
            value is not None
            for value in (start["utm_east"], start["utm_north"], end["utm_east"], end["utm_north"])
        )
        return {
            "start_wp": target_index - 1,
            "end_wp": target_index,
            "start": (
                (start["utm_east"], start["utm_north"])
                if use_utm
                else (start["east"], start["north"])
            ),
            "end": (
                (end["utm_east"], end["utm_north"])
                if use_utm
                else (end["east"], end["north"])
            ),
            "metric_frame": "utm" if use_utm else "local_enu",
        }

    def _begin_run(self, stamp):
        try:
            self._load_waypoints()
            if not self._waypoints:
                raise RuntimeError("No waypoints were loaded")
            self._create_run_files()
        except Exception as exc:
            rospy.logerr("[experiment_csv] Cannot start CSV logging: %s", exc)
            return False

        self._run_active = True
        self._finalized = False
        self._run_start_time = stamp
        self._run_start_wall = datetime.now().isoformat(timespec="seconds")
        # Share the exact run directory with the PNG evaluator.  Publishing it
        # latched lets the evaluator receive it even if its subscriber connects
        # after the CSV files have already been opened.
        self.run_directory_publisher.publish(String(data=self._run_dir))
        self._last_reached_index = 0
        self._last_reference_enu = None
        self._reference_sample_count = 0
        self._rmse_sample_count = 0
        self._rmse_sum_squared = 0.0
        self._rmse_sum_absolute = 0.0
        self._rmse_max = 0.0
        self._affected_rmse_sample_count = 0
        self._affected_rmse_sum_squared = 0.0
        self.rmse_publisher.publish(Float64(data=float("nan")))
        rospy.loginfo(
            "[experiment_csv] Recording %d waypoint(s) to %s",
            len(self._waypoints),
            self._run_dir,
        )
        if len(self._waypoints) < 4:
            rospy.logwarn(
                "[experiment_csv] The active route has only %d waypoint(s). "
                "Check Scenario 2 if it is configured to stop noise at WP4.",
                len(self._waypoints),
            )
        return True

    def _reached_waypoint_cb(self, msg):
        stamp = _stamp_or_now(None)
        index = int(msg.data)
        if index < 0:
            return
        with self._lock:
            if not self._run_active:
                if self.start_topic:
                    rospy.logwarn_throttle(
                        5.0,
                        "[experiment_csv] Ignoring waypoint index %d until %s starts a run.",
                        index,
                        self.start_topic,
                    )
                    return
                if index != 0:
                    rospy.logwarn_throttle(
                        5.0,
                        "[experiment_csv] Ignoring waypoint index %d until a fresh index 0 starts a run.",
                        index,
                    )
                    return
                if not self._begin_run(stamp):
                    return
            if self._finalized:
                return
            self._last_reached_index = index

    def _start_cb(self, msg):
        if not msg.data:
            return
        stamp = _stamp_or_now(None)
        with self._lock:
            if self._run_active:
                return
            if self._finalized:
                if not self.allow_multiple_runs:
                    rospy.logwarn(
                        "[experiment_csv] This logger is already finalized; restart Terminal 2 for another run."
                    )
                    return
                self._finalized = False
                self._finish_timer = None
            self._begin_run(stamp)

    def _reference_gps_cb(self, msg):
        stamp = _stamp_or_now(msg.header.stamp)
        with self._lock:
            if not self._run_active or self._finalized:
                return

            covariance = list(msg.position_covariance)
            covariance_xx = covariance[0] if len(covariance) >= 1 else ""
            covariance_yy = covariance[4] if len(covariance) >= 5 else ""
            valid_fix = (
                msg.status.status != NavSatStatus.STATUS_NO_FIX
                and _finite(msg.latitude)
                and _finite(msg.longitude)
                and -90.0 <= msg.latitude <= 90.0
                and -180.0 <= msg.longitude <= 180.0
            )
            target_index = self._target_waypoint_index()
            row = {
                "stamp_sec": stamp,
                "elapsed_sec": self._elapsed(stamp),
                "latitude_deg": msg.latitude,
                "longitude_deg": msg.longitude,
                "altitude_m": msg.altitude,
                "local_east_m": "",
                "local_north_m": "",
                "utm_easting_m": "",
                "utm_northing_m": "",
                "map_x_m": "",
                "map_y_m": "",
                "navsat_status": msg.status.status,
                "valid_fix": int(valid_fix),
                "covariance_xx_m2": covariance_xx,
                "covariance_yy_m2": covariance_yy,
                "last_reached_wp": self._last_reached_index,
                "active_target_wp": target_index,
                "segment_start_wp": "",
                "segment_end_wp": "",
                "segment_label": "approach_to_wp1" if target_index == 1 else "completed_or_unknown",
                "rmse_used": 0,
                "cross_track_error_m": "",
                "signed_cross_track_error_m": "",
                "along_track_m": "",
                "projection_unclamped": "",
                "metric_frame": "",
                "noise_active": int(self._noise_active),
                "noise_east_m": self._noise_east_m,
                "noise_north_m": self._noise_north_m,
                "outage_active": int(self._outage_active),
                "outage_distance_m": self._outage_distance_m,
            }

            if valid_fix and self._waypoints_are_gps:
                origin = self._waypoints[0]
                east, north = self._local_enu(
                    msg.latitude, msg.longitude, origin["latitude"], origin["longitude"]
                )
                self._last_reference_enu = (east, north)
                self._reference_sample_count += 1
                row["local_east_m"] = east
                row["local_north_m"] = north
                utm_east, utm_north = self._latlon_to_utm(msg.latitude, msg.longitude)
                map_point = self._utm_to_map(utm_east, utm_north)
                row["utm_easting_m"] = utm_east if utm_east is not None else ""
                row["utm_northing_m"] = utm_north if utm_north is not None else ""
                if map_point is not None:
                    row["map_x_m"] = map_point[0]
                    row["map_y_m"] = map_point[1]

                segment = self._current_route_segment()
                if segment is not None:
                    metric_east, metric_north = (
                        (utm_east, utm_north)
                        if segment["metric_frame"] == "utm" and utm_east is not None and utm_north is not None
                        else (east, north)
                    )
                    metric = self._project_to_segment(
                        metric_east, metric_north, segment["start"], segment["end"]
                    )
                    if metric is not None:
                        row.update(
                            {
                                "segment_start_wp": segment["start_wp"],
                                "segment_end_wp": segment["end_wp"],
                                "segment_label": "WP{}->WP{}".format(
                                    segment["start_wp"], segment["end_wp"]
                                ),
                                "rmse_used": 1,
                                "cross_track_error_m": metric["error"],
                                "signed_cross_track_error_m": metric["signed_error"],
                                "along_track_m": metric["along_track"],
                                "projection_unclamped": metric["projection"],
                                "metric_frame": segment["metric_frame"],
                            }
                        )
                        self._rmse_sample_count += 1
                        self._rmse_sum_squared += metric["error"] ** 2
                        self._rmse_sum_absolute += metric["error"]
                        self._rmse_max = max(self._rmse_max, metric["error"])
                        if self._noise_active or self._outage_active:
                            self._affected_rmse_sample_count += 1
                            self._affected_rmse_sum_squared += metric["error"] ** 2

            self._write_row("reference", row)

    def _controller_odom_cb(self, msg):
        stamp = _stamp_or_now(msg.header.stamp)
        with self._lock:
            if not self._run_active or self._finalized:
                return
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
            try:
                _, _, yaw = tf.transformations.euler_from_quaternion(
                    [orientation.x, orientation.y, orientation.z, orientation.w]
                )
            except Exception:
                yaw = ""
            target_index, target = self._target_waypoint()
            self._write_row(
                "controller",
                {
                    "stamp_sec": stamp,
                    "elapsed_sec": self._elapsed(stamp),
                    "map_x_m": position.x,
                    "map_y_m": position.y,
                    "yaw_rad": yaw,
                    "linear_x_mps": msg.twist.twist.linear.x,
                    "angular_z_rps": msg.twist.twist.angular.z,
                    "last_reached_wp": self._last_reached_index,
                    "active_target_wp": target_index,
                    "target_local_east_m": target["east"] if target else "",
                    "target_local_north_m": target["north"] if target else "",
                    "target_map_x_m": target["map_x"] if target else "",
                    "target_map_y_m": target["map_y"] if target else "",
                    "latest_cmd_linear_x_mps": self._latest_cmd_linear,
                    "latest_cmd_angular_z_radps": self._latest_cmd_angular,
                    "noise_active": int(self._noise_active),
                    "outage_active": int(self._outage_active),
                },
            )

    def _cmd_vel_cb(self, msg):
        with self._lock:
            self._latest_cmd_linear = msg.linear.x
            self._latest_cmd_angular = msg.angular.z

    def _noise_active_cb(self, msg):
        with self._lock:
            self._noise_active = bool(msg.data)

    def _noise_offset_cb(self, msg):
        with self._lock:
            self._noise_east_m = msg.vector.x
            self._noise_north_m = msg.vector.y

    def _outage_active_cb(self, msg):
        with self._lock:
            self._outage_active = bool(msg.data)

    def _outage_distance_cb(self, msg):
        with self._lock:
            self._outage_distance_m = msg.data

    def _finish_cb(self, msg):
        if not msg.data:
            return
        with self._lock:
            if not self._run_active or self._finalized or self._finish_timer is not None:
                return
            self._finish_timer = rospy.Timer(
                rospy.Duration(self.finish_delay_sec), self._finish_timer_cb, oneshot=True
            )

    def _finish_timer_cb(self, _event):
        with self._lock:
            self._finalize("finish_signal")

    def _finalize(self, status):
        if not self._run_active or self._finalized:
            return
        self._finalized = True

        rmse = ""
        if self._rmse_sample_count > 0:
            rmse = math.sqrt(self._rmse_sum_squared / self._rmse_sample_count)

        for handle in self._files.values():
            try:
                handle.flush()
                handle.close()
            except Exception:
                pass
        self._files.clear()
        self._writers.clear()
        self._run_active = False
        self.rmse_publisher.publish(Float64(data=rmse if rmse != "" else float("nan")))
        rospy.loginfo(
            "[experiment_csv] Saved run to %s | CTE RMSE: %s m",
            self._run_dir,
            "{:.3f}".format(rmse) if rmse != "" else "N/A",
        )

    def _on_shutdown(self):
        with self._lock:
            self._finalize("ros_shutdown")


if __name__ == "__main__":
    rospy.init_node("experiment_trajectory_csv_logger")
    ExperimentTrajectoryCsvLogger()
    rospy.spin()
