#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Interactive waypoint generator and runner for Scenario 4.

Scenario 4 must keep the number of waypoints and total route length fixed
while changing only the zig-zag angle.  This node creates the five GPS
waypoints from one stationary WP1 measurement and the robot's heading in the
UTM frame; no manual collection of WP2..WP5 is needed.

Keyboard / joystick state machine:

    l -> wait for WP1
    y -> sample stationary WP1 and heading
    c -> calculate and preview the five-waypoint route
    k -> validate and lock the route
    r -> start gps_waypoint with the locked route

The standard joy_launch_control.py is deliberately not used here because its
LB/RB handlers start the legacy collector/sender and would overwrite the
normal outdoor waypoint file.
"""

import json
import math
import os
import signal
import subprocess
import sys
import threading

import rospy
import rospkg
import tf
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import Joy, NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Int32, String
from visualization_msgs.msg import Marker, MarkerArray

try:
    from pyproj import Transformer
except ImportError:
    Transformer = None


STATE_IDLE = "IDLE"
STATE_ARMED = "WAITING_FOR_WP1"
STATE_SAMPLING = "SAMPLING_WP1"
STATE_CAPTURED = "WP1_CAPTURED"
STATE_GENERATED = "ROUTE_GENERATED"
STATE_CONFIRMED = "ROUTE_READY"
STATE_STARTING = "STARTING"
STATE_RUNNING = "RUNNING"
STATE_FINISHED = "FINISHED"
STATE_ABORTED = "ABORTED"


def _is_valid_fix(msg):
    return (
        msg.status.status != NavSatStatus.STATUS_NO_FIX
        and math.isfinite(msg.latitude)
        and math.isfinite(msg.longitude)
        and -90.0 <= msg.latitude <= 90.0
        and -180.0 <= msg.longitude <= 180.0
    )


def _normalize_angle(angle_rad):
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad <= -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


class Scenario4WaypointManager:
    """Own the Scenario-4 command sequence and its child gps_waypoint launch."""

    def __init__(self):
        if Transformer is None:
            raise RuntimeError(
                "Scenario 4 needs pyproj. Install python3-pyproj before running it."
            )

        self.package_dir = rospkg.RosPack().get_path("outdoor_waypoint_nav")

        self.gps_topic = rospy.get_param("~gps_topic", "/gps/fix")
        self.utm_frame = rospy.get_param("~utm_frame", "utm")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        # Gazebo's navsat_transform may publish map<->utm with an older
        # simulation stamp. In that case tf cannot always compose
        # utm->base_link directly even though both transforms through map are
        # available. This frame supplies a mathematically equivalent fallback.
        self.heading_fallback_frame = rospy.get_param(
            "~heading_fallback_frame", "map"
        )
        self.keyboard_joy_topic = rospy.get_param(
            "~keyboard_joy_topic", "/outdoor_waypoint_nav/scenario_4/keyboard_joy"
        )
        self.command_topic = rospy.get_param(
            "~command_topic", "/outdoor_waypoint_nav/scenario_4/command"
        )
        self.status_topic = rospy.get_param(
            "~status_topic", "/outdoor_waypoint_nav/scenario_4/status"
        )
        self.route_ready_topic = rospy.get_param(
            "~route_ready_topic", "/outdoor_waypoint_nav/scenario_4/route_ready"
        )
        self.run_started_topic = rospy.get_param(
            "~run_started_topic", "/outdoor_waypoint_nav/scenario_4/run_started"
        )
        self.marker_topic = rospy.get_param(
            "~marker_topic", "/outdoor_waypoint_nav/scenario_4/generated_waypoints"
        )
        self.finish_topic = rospy.get_param(
            "~finish_topic", "/outdoor_waypoint_nav/waypoint_following_status"
        )
        self.reached_waypoint_topic = rospy.get_param(
            "~reached_waypoint_topic",
            "/outdoor_waypoint_nav/waypoint_reached_index",
        )

        self.sample_duration_sec = max(
            1.0, float(rospy.get_param("~sample_duration_sec", 5.0))
        )
        self.min_gps_samples = max(1, int(rospy.get_param("~min_gps_samples", 5)))
        self.min_heading_samples = max(
            1, int(rospy.get_param("~min_heading_samples", 10))
        )
        self.max_heading_std_deg = max(
            0.1, float(rospy.get_param("~max_heading_std_deg", 5.0))
        )
        self.theta_deg = float(rospy.get_param("~theta_deg", 15.0))
        self.route_length_m = float(rospy.get_param("~route_length_m", 300.0))
        self.first_side = str(rospy.get_param("~first_side", "right")).strip().lower()
        self.marker_scale = max(0.05, float(rospy.get_param("~marker_scale", 0.7)))

        allowed_angles = rospy.get_param("~allowed_theta_deg", [15.0, 30.0, 45.0])
        if isinstance(allowed_angles, str):
            allowed_angles = [
                float(item.strip()) for item in allowed_angles.split(",") if item.strip()
            ]
        self.allowed_theta_deg = [float(value) for value in allowed_angles]

        if self.route_length_m <= 0.0:
            raise RuntimeError("route_length_m must be greater than zero")
        if self.first_side not in ("right", "left"):
            raise RuntimeError("first_side must be either 'right' or 'left'")
        if not any(
            abs(self.theta_deg - allowed) <= 1e-6 for allowed in self.allowed_theta_deg
        ):
            raise RuntimeError(
                "theta_deg={} is not an allowed Scenario-4 angle: {}".format(
                    self.theta_deg, self.allowed_theta_deg
                )
            )

        route_file_value = rospy.get_param(
            "~waypoint_file", "/waypoint_files/points_scenario_4.txt"
        )
        self.route_file_path, self.route_file_param = self._resolve_package_file(
            route_file_value
        )
        metadata_value = rospy.get_param("~metadata_file", "")
        if metadata_value:
            self.metadata_path, _ = self._resolve_package_file(metadata_value)
        else:
            base, _ = os.path.splitext(self.route_file_path)
            self.metadata_path = base + "_metadata.json"

        send_launch_value = rospy.get_param(
            "~send_launch_file", "/launch/include/send_goals_scenario_4.launch"
        )
        self.send_launch_path = self._resolve_package_launch_file(send_launch_value)
        self.heading_calibration_launch_path = os.path.join(
            self.package_dir, "launch", "include", "heading_calibration.launch"
        )

        self.button_numbers = {
            "l": int(rospy.get_param("~l_button_num", 4)),
            "y": int(rospy.get_param("~y_button_num", 3)),
            "c": int(rospy.get_param("~c_button_num", 2)),
            "k": int(rospy.get_param("~k_button_num", 6)),
            "r": int(rospy.get_param("~r_button_num", 5)),
            "b": int(rospy.get_param("~b_button_num", 1)),
        }
        self.previous_buttons = {
            "main": {name: False for name in self.button_numbers},
            "keyboard": {name: False for name in self.button_numbers},
        }

        self.state = STATE_IDLE
        self.route_ready = False
        self.latest_fix = None
        self.gps_samples = []
        self.heading_samples = []
        self.sample_end_time = rospy.Time(0)
        self.anchor = None
        self.route_points = []
        self.sender_process = None
        self.calibration_process = None
        self.sender_output_thread = None
        self.calibration_output_thread = None
        self.last_reached_index = 0
        self.finish_pending = False
        self.finish_timer = None
        self.sender_exit_observed_at = None
        self.start_cancel_requested = False

        self.tf_listener = tf.TransformListener()
        self.status_pub = rospy.Publisher(
            self.status_topic, String, queue_size=10, latch=True
        )
        self.route_ready_pub = rospy.Publisher(
            self.route_ready_topic, Bool, queue_size=1, latch=True
        )
        self.run_started_pub = rospy.Publisher(
            self.run_started_topic, Bool, queue_size=1, latch=True
        )
        self.marker_pub = rospy.Publisher(
            self.marker_topic, MarkerArray, queue_size=10, latch=True
        )
        self.cmd_vel_pubs = [
            rospy.Publisher("/cmd_vel", Twist, queue_size=1),
            rospy.Publisher("/cmd_vel_intermediate", Twist, queue_size=1),
            rospy.Publisher("/warthog_velocity_controller/cmd_vel", Twist, queue_size=1),
        ]

        rospy.Subscriber(self.gps_topic, NavSatFix, self._gps_cb, queue_size=100)
        rospy.Subscriber("/joy_teleop/joy", Joy, self._main_joy_cb, queue_size=100)
        rospy.Subscriber(
            self.keyboard_joy_topic, Joy, self._keyboard_joy_cb, queue_size=100
        )
        rospy.Subscriber(self.command_topic, String, self._command_cb, queue_size=20)
        rospy.Subscriber(self.finish_topic, Bool, self._finish_cb, queue_size=10)
        rospy.Subscriber(
            self.reached_waypoint_topic,
            Int32,
            self._reached_waypoint_cb,
            queue_size=20,
        )
        rospy.Timer(rospy.Duration(0.05), self._timer_cb)
        rospy.on_shutdown(self._on_shutdown)

        self._publish_route_ready(False)
        self._publish_run_started(False)
        self._clear_markers()
        self._set_state(
            STATE_IDLE,
            "Nhấn l để bắt đầu chuẩn bị nhận WP1; đặt xe đứng yên và hướng theo trục thử nghiệm.",
        )
        self._print_help()

    def _resolve_package_file(self, value):
        value = str(value).strip()
        if not value:
            raise RuntimeError("Waypoint file path cannot be empty")
        relative = value[1:] if value.startswith("/") else value
        candidate = os.path.normpath(os.path.join(self.package_dir, relative))
        package_root = os.path.normpath(self.package_dir)
        if os.path.commonpath((package_root, candidate)) != package_root:
            raise RuntimeError("Waypoint file must stay inside outdoor_waypoint_nav")
        return candidate, "/" + relative

    def _resolve_package_launch_file(self, value):
        """Resolve a package-relative launch file, or a verified absolute one."""
        value = str(value).strip()
        if not value:
            raise RuntimeError("Sender launch file path cannot be empty")
        if os.path.isabs(value) and os.path.isfile(value):
            return os.path.normpath(value)

        relative = value[1:] if value.startswith("/") else value
        candidate = os.path.normpath(os.path.join(self.package_dir, relative))
        package_root = os.path.normpath(self.package_dir)
        if os.path.commonpath((package_root, candidate)) != package_root:
            raise RuntimeError("Sender launch file must stay inside outdoor_waypoint_nav")
        return candidate

    def _print_help(self):
        rospy.loginfo(
            "[scenario4] Key sequence: l -> y -> c -> k -> r | "
            "b aborts a run."
        )
        rospy.loginfo(
            "[scenario4] theta=%.1f deg, total route=%.1f m, generated file=%s",
            self.theta_deg,
            self.route_length_m,
            self.route_file_param,
        )

    def _set_state(self, state, detail):
        self.state = state
        message = "{} | {}".format(state, detail)
        self.status_pub.publish(String(data=message))
        rospy.loginfo("[scenario4] %s", message)

    def _publish_route_ready(self, ready):
        self.route_ready = bool(ready)
        self.route_ready_pub.publish(Bool(data=self.route_ready))

    def _publish_run_started(self, started):
        self.run_started_pub.publish(Bool(data=bool(started)))

    @staticmethod
    def _button_pressed(message, button_number):
        return (
            button_number >= 0
            and button_number < len(message.buttons)
            and message.buttons[button_number] == 1
        )

    def _main_joy_cb(self, message):
        self._process_joy(message, "main")

    def _keyboard_joy_cb(self, message):
        self._process_joy(message, "keyboard")

    def _process_joy(self, message, source):
        for command, button_number in self.button_numbers.items():
            pressed = self._button_pressed(message, button_number)
            previous = self.previous_buttons[source][command]
            self.previous_buttons[source][command] = pressed
            if pressed and not previous:
                self._handle_command(command)

    def _command_cb(self, message):
        command = message.data.strip().lower()
        if command in self.button_numbers:
            self._handle_command(command)
        else:
            rospy.logwarn(
                "[scenario4] Unknown command '%s'. Use l, y, c, k, r, or b.",
                command,
            )

    def _handle_command(self, command):
        if command == "b":
            self._abort_route()
            return
        if self._calibration_is_running():
            rospy.logwarn(
                "[scenario4] Heading calibration is active. Wait for it to finish first."
            )
            return
        if command == "l":
            self._arm_for_wp1()
        elif command == "y":
            self._start_wp1_sampling()
        elif command == "c":
            self._generate_route()
        elif command == "k":
            self._confirm_route()
        elif command == "r":
            self._start_route()

    def _gps_cb(self, message):
        if not _is_valid_fix(message):
            return
        self.latest_fix = message
        if self.state == STATE_SAMPLING:
            self.gps_samples.append(
                (message.latitude, message.longitude, message.header.stamp.to_sec())
            )

    def _timer_cb(self, _event):
        if self.state == STATE_RUNNING:
            self._check_sender_exit()
        if self.state != STATE_SAMPLING:
            return

        heading = self._lookup_utm_yaw()
        if heading is not None:
            self.heading_samples.append(heading)

        if rospy.Time.now() >= self.sample_end_time:
            self._finish_wp1_sampling()

    def _check_sender_exit(self):
        if self.sender_process is None or self.sender_process.poll() is None:
            self.sender_exit_observed_at = None
            return
        if self.finish_pending:
            return
        now = rospy.Time.now()
        if self.sender_exit_observed_at is None:
            self.sender_exit_observed_at = now
            return
        if (now - self.sender_exit_observed_at).to_sec() < 0.5:
            return

        self._publish_route_ready(True)
        self._set_state(
            STATE_ABORTED,
            "Sender dừng mà không có tín hiệu hoàn tất. Run không hợp lệ; "
            "xem log rồi khởi động lại Terminal 2.",
        )

    def _lookup_utm_yaw(self):
        try:
            _, rotation = self.tf_listener.lookupTransform(
                self.utm_frame, self.base_frame, rospy.Time(0)
            )
            return _normalize_angle(tf.transformations.euler_from_quaternion(rotation)[2])
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ) as exc:
            direct_exception = exc

        # If R_ref_base = R_ref_utm * R_utm_base, then the desired heading is
        # yaw_utm_base = yaw_ref_base - yaw_ref_utm. This uses two direct
        # lookups through a shared reference frame instead of one composed
        # lookup, avoiding the Gazebo timestamp issue without changing real
        # robot behavior (where the direct lookup succeeds).
        try:
            _, base_rotation = self.tf_listener.lookupTransform(
                self.heading_fallback_frame, self.base_frame, rospy.Time(0)
            )
            _, utm_rotation = self.tf_listener.lookupTransform(
                self.heading_fallback_frame, self.utm_frame, rospy.Time(0)
            )
            base_yaw = tf.transformations.euler_from_quaternion(base_rotation)[2]
            utm_yaw = tf.transformations.euler_from_quaternion(utm_rotation)[2]
            rospy.logwarn_throttle(
                10.0,
                "[scenario4] Using %s fallback for UTM heading after direct TF lookup failed: %s",
                self.heading_fallback_frame,
                direct_exception,
            )
            return _normalize_angle(base_yaw - utm_yaw)
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ) as fallback_exception:
            rospy.logwarn_throttle(
                2.0,
                "[scenario4] Waiting for TF %s -> %s (fallback %s): %s / %s",
                self.utm_frame,
                self.base_frame,
                self.heading_fallback_frame,
                direct_exception,
                fallback_exception,
            )
            return None

    def _arm_for_wp1(self):
        if self.state in (STATE_STARTING, STATE_RUNNING):
            rospy.logwarn("[scenario4] Không thể tạo route mới khi xe đang khởi/chạy.")
            return
        if self.state in (STATE_FINISHED, STATE_ABORTED):
            rospy.logwarn(
                "[scenario4] Route này đã có kết quả. Khởi động lại Terminal 2 "
                "trước khi tạo một route WP1 mới."
            )
            return

        self.anchor = None
        self.route_points = []
        self.gps_samples = []
        self.heading_samples = []
        self._publish_route_ready(False)
        self._publish_run_started(False)
        self._clear_markers()
        self._set_state(
            STATE_ARMED,
            "Đã sẵn sàng. Đặt xe đứng yên tại WP1, quay đúng hướng trục giữa, rồi nhấn y.",
        )

    def _start_wp1_sampling(self):
        if self.state != STATE_ARMED:
            rospy.logwarn(
                "[scenario4] y chỉ hợp lệ sau l. Trạng thái hiện tại: %s", self.state
            )
            return
        if self.latest_fix is None:
            rospy.logwarn(
                "[scenario4] Chưa có GPS hợp lệ trên %s. Hãy chờ rồi nhấn y lại.",
                self.gps_topic,
            )
            return
        if self._lookup_utm_yaw() is None:
            rospy.logwarn(
                "[scenario4] TF UTM chưa sẵn sàng. Hãy chờ navsat_transform rồi nhấn y lại."
            )
            return

        self.gps_samples = []
        self.heading_samples = []
        self.sample_end_time = rospy.Time.now() + rospy.Duration(self.sample_duration_sec)
        self._set_state(
            STATE_SAMPLING,
            "Đang lấy mẫu WP1 và hướng trong {:.1f} s. Giữ xe đứng yên.".format(
                self.sample_duration_sec
            ),
        )

    def _finish_wp1_sampling(self):
        if len(self.gps_samples) < self.min_gps_samples:
            self._set_state(
                STATE_ARMED,
                "Không đủ mẫu GPS ({}/{}). Giữ xe đứng yên và nhấn y lại.".format(
                    len(self.gps_samples), self.min_gps_samples
                ),
            )
            return
        if len(self.heading_samples) < self.min_heading_samples:
            self._set_state(
                STATE_ARMED,
                "Không đủ mẫu heading TF ({}/{}). Hãy chờ TF rồi nhấn y lại.".format(
                    len(self.heading_samples), self.min_heading_samples
                ),
            )
            return

        latitudes = sorted(sample[0] for sample in self.gps_samples)
        longitudes = sorted(sample[1] for sample in self.gps_samples)
        midpoint = len(latitudes) // 2
        if len(latitudes) % 2:
            latitude = latitudes[midpoint]
            longitude = longitudes[midpoint]
        else:
            latitude = 0.5 * (latitudes[midpoint - 1] + latitudes[midpoint])
            longitude = 0.5 * (longitudes[midpoint - 1] + longitudes[midpoint])

        mean_sin = sum(math.sin(value) for value in self.heading_samples) / len(
            self.heading_samples
        )
        mean_cos = sum(math.cos(value) for value in self.heading_samples) / len(
            self.heading_samples
        )
        resultant_length = math.hypot(mean_sin, mean_cos)
        if resultant_length < 1e-6:
            self._set_state(
                STATE_ARMED,
                "Heading không ổn định. Căn xe lại rồi nhấn y để lấy lại WP1.",
            )
            return

        yaw_utm = math.atan2(mean_sin, mean_cos)
        heading_std_rad = math.sqrt(
            max(0.0, -2.0 * math.log(min(1.0, resultant_length)))
        )
        heading_std_deg = math.degrees(heading_std_rad)
        if heading_std_deg > self.max_heading_std_deg:
            self._set_state(
                STATE_ARMED,
                "Heading dao động {:.2f} deg (> {:.2f} deg). Căn xe lại rồi nhấn y.".format(
                    heading_std_deg, self.max_heading_std_deg
                ),
            )
            return

        easting, northing, zone_epsg = self._latlon_to_utm(latitude, longitude)
        self.anchor = {
            "latitude": latitude,
            "longitude": longitude,
            "easting": easting,
            "northing": northing,
            "epsg": zone_epsg,
            "yaw_utm_rad": yaw_utm,
            "bearing_true_deg": (90.0 - math.degrees(yaw_utm)) % 360.0,
            "heading_std_deg": heading_std_deg,
            "gps_sample_count": len(self.gps_samples),
            "heading_sample_count": len(self.heading_samples),
        }
        self._set_state(
            STATE_CAPTURED,
            "Đã nhận WP1: lat={:.9f}, lon={:.9f}, bearing={:.2f} deg. Nhấn c để tính route.".format(
                latitude, longitude, self.anchor["bearing_true_deg"]
            ),
        )

    def _latlon_to_utm(self, latitude, longitude):
        zone = max(1, min(60, int((longitude + 180.0) / 6.0) + 1))
        epsg = (32600 if latitude >= 0.0 else 32700) + zone
        transformer = Transformer.from_crs(
            "EPSG:4326", "EPSG:{}".format(epsg), always_xy=True
        )
        easting, northing = transformer.transform(longitude, latitude)
        return easting, northing, epsg

    @staticmethod
    def _inverse_utm_transformer(epsg):
        return Transformer.from_crs(
            "EPSG:{}".format(epsg), "EPSG:4326", always_xy=True
        )

    def _generate_route(self):
        if self.state != STATE_CAPTURED or self.anchor is None:
            rospy.logwarn(
                "[scenario4] c chỉ hợp lệ sau khi y đã nhận WP1 và hướng. Trạng thái: %s",
                self.state,
            )
            return

        unit_length = self.route_length_m / 5.0
        theta = math.radians(self.theta_deg)
        first_side_sign = 1.0 if self.first_side == "right" else -1.0

        # (forward, right) offsets exactly match the PDF drawing:
        # [a, a, 2a, a] segment lengths, five waypoint positions, 5a total.
        local_points = [
            (0.0, 0.0),
            (unit_length, 0.0),
            (
                unit_length + unit_length * math.cos(theta),
                first_side_sign * unit_length * math.sin(theta),
            ),
            (
                unit_length + 3.0 * unit_length * math.cos(theta),
                -first_side_sign * unit_length * math.sin(theta),
            ),
            (
                unit_length + 4.0 * unit_length * math.cos(theta),
                0.0,
            ),
        ]
        yaw = self.anchor["yaw_utm_rad"]
        forward_vector = (math.cos(yaw), math.sin(yaw))
        right_vector = (math.sin(yaw), -math.cos(yaw))
        inverse_transformer = self._inverse_utm_transformer(self.anchor["epsg"])

        points = []
        for index, (forward, right) in enumerate(local_points, start=1):
            easting = (
                self.anchor["easting"]
                + forward * forward_vector[0]
                + right * right_vector[0]
            )
            northing = (
                self.anchor["northing"]
                + forward * forward_vector[1]
                + right * right_vector[1]
            )
            longitude, latitude = inverse_transformer.transform(easting, northing)
            points.append(
                {
                    "index": index,
                    "forward_m": forward,
                    "right_m": right,
                    "easting": easting,
                    "northing": northing,
                    "latitude": latitude,
                    "longitude": longitude,
                }
            )

        valid, detail, segment_lengths = self._validate_route(points, unit_length)
        if not valid:
            rospy.logerr("[scenario4] Không thể dùng route vừa tính: %s", detail)
            return

        try:
            self._write_route_files(points, unit_length, segment_lengths)
        except OSError as exc:
            rospy.logerr("[scenario4] Không ghi được file waypoint: %s", exc)
            return

        self.route_points = points
        rospy.set_param("/outdoor_waypoint_nav/coordinates_file", self.route_file_param)
        self._publish_route_ready(False)
        self._publish_markers(points)
        self._set_state(
            STATE_GENERATED,
            "Đã tính 5 waypoint, tổng {:.3f} m. Xem marker/file rồi nhấn k để khóa route.".format(
                sum(segment_lengths)
            ),
        )
        for point in points:
            rospy.loginfo(
                "[scenario4] WP%d lat=%.10f lon=%.10f | forward=%.3f m right=%.3f m",
                point["index"],
                point["latitude"],
                point["longitude"],
                point["forward_m"],
                point["right_m"],
            )

    def _validate_route(self, points, unit_length):
        if len(points) != 5:
            return False, "Route phải có đúng 5 waypoint.", []
        segment_lengths = []
        for previous, current in zip(points, points[1:]):
            segment_lengths.append(
                math.hypot(
                    current["easting"] - previous["easting"],
                    current["northing"] - previous["northing"],
                )
            )
        expected = [unit_length, unit_length, 2.0 * unit_length, unit_length]
        if len(segment_lengths) != len(expected):
            return False, "Số segment không đúng.", segment_lengths
        for index, (actual, wanted) in enumerate(zip(segment_lengths, expected), start=1):
            if abs(actual - wanted) > 0.02:
                return (
                    False,
                    "Segment {} = {:.3f} m, expected {:.3f} m.".format(
                        index, actual, wanted
                    ),
                    segment_lengths,
                )
        total = sum(segment_lengths)
        if abs(total - self.route_length_m) > 0.05:
            return (
                False,
                "Tổng route {:.3f} m, expected {:.3f} m.".format(
                    total, self.route_length_m
                ),
                segment_lengths,
            )
        return True, "ok", segment_lengths

    def _write_route_files(self, points, unit_length, segment_lengths):
        output_dir = os.path.dirname(self.route_file_path)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)

        temporary_path = self.route_file_path + ".tmp"
        with open(temporary_path, "w", encoding="utf-8") as handle:
            for point in points:
                handle.write(
                    "{:.10f} {:.10f}\n".format(
                        point["latitude"], point["longitude"]
                    )
                )
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary_path, self.route_file_path)

        metadata = {
            "scenario": "scenario_4_trajectory_shape",
            "theta_deg": self.theta_deg,
            "first_side": self.first_side,
            "total_route_length_m": self.route_length_m,
            "unit_length_m": unit_length,
            "segment_lengths_m": segment_lengths,
            "corner_heading_changes_deg": [
                self.theta_deg,
                2.0 * self.theta_deg,
                2.0 * self.theta_deg,
            ],
            "route_file": self.route_file_param,
            "utm_epsg": self.anchor["epsg"],
            "anchor_wp1": self.anchor,
            "waypoints": points,
        }
        metadata_dir = os.path.dirname(self.metadata_path)
        if metadata_dir:
            os.makedirs(metadata_dir, exist_ok=True)
        temporary_metadata_path = self.metadata_path + ".tmp"
        with open(temporary_metadata_path, "w", encoding="utf-8") as handle:
            json.dump(metadata, handle, indent=2, sort_keys=True)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary_metadata_path, self.metadata_path)

        rospy.loginfo("[scenario4] Waypoint file written atomically: %s", self.route_file_path)
        rospy.loginfo("[scenario4] Route metadata written: %s", self.metadata_path)

    def _publish_markers(self, points):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        line = Marker()
        line.header.frame_id = self.utm_frame
        line.header.stamp = rospy.Time.now()
        line.ns = "scenario_4_route"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = max(0.08, self.marker_scale * 0.12)
        line.color.r = 0.05
        line.color.g = 0.65
        line.color.b = 0.95
        line.color.a = 1.0
        for route_point in points:
            line.points.append(
                Point(
                    x=route_point["easting"],
                    y=route_point["northing"],
                    z=0.15,
                )
            )
        markers.markers.append(line)

        spheres = Marker()
        spheres.header.frame_id = self.utm_frame
        spheres.header.stamp = rospy.Time.now()
        spheres.ns = "scenario_4_route"
        spheres.id = 1
        spheres.type = Marker.SPHERE_LIST
        spheres.action = Marker.ADD
        spheres.pose.orientation.w = 1.0
        spheres.scale.x = self.marker_scale
        spheres.scale.y = self.marker_scale
        spheres.scale.z = self.marker_scale
        spheres.color.r = 1.0
        spheres.color.g = 0.55
        spheres.color.b = 0.05
        spheres.color.a = 1.0
        for route_point in points:
            spheres.points.append(
                Point(
                    x=route_point["easting"],
                    y=route_point["northing"],
                    z=0.2,
                )
            )
        markers.markers.append(spheres)

        for route_point in points:
            label = Marker()
            label.header.frame_id = self.utm_frame
            label.header.stamp = rospy.Time.now()
            label.ns = "scenario_4_labels"
            label.id = route_point["index"]
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.orientation.w = 1.0
            label.pose.position.x = route_point["easting"]
            label.pose.position.y = route_point["northing"]
            label.pose.position.z = 1.0
            label.scale.z = max(0.6, self.marker_scale)
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = "WP{}".format(route_point["index"])
            markers.markers.append(label)

        self.marker_pub.publish(markers)

    def _clear_markers(self):
        markers = MarkerArray()
        marker = Marker()
        marker.action = Marker.DELETEALL
        markers.markers.append(marker)
        self.marker_pub.publish(markers)

    def _confirm_route(self):
        if self.state != STATE_GENERATED:
            rospy.logwarn(
                "[scenario4] k chỉ hợp lệ sau c. Trạng thái hiện tại: %s", self.state
            )
            return
        valid, detail, segment_lengths = self._validate_route(
            self.route_points, self.route_length_m / 5.0
        )
        if not valid:
            rospy.logerr("[scenario4] Route chưa thể khóa: %s", detail)
            return
        if not os.path.isfile(self.route_file_path):
            rospy.logerr(
                "[scenario4] Không tìm thấy %s. Hãy nhấn c để tạo lại route.",
                self.route_file_path,
            )
            return
        rospy.set_param("/outdoor_waypoint_nav/coordinates_file", self.route_file_param)
        self._publish_route_ready(True)
        self._set_state(
            STATE_CONFIRMED,
            "Đã đủ 5 waypoint. Các đoạn {} m; tổng {:.3f} m. Nhấn r để chạy.".format(
                ", ".join("{:.3f}".format(value) for value in segment_lengths),
                sum(segment_lengths),
            ),
        )

    def _has_move_base_server(self):
        published_topics = dict(rospy.get_published_topics())
        required = ("/move_base/status", "/move_base/goal", "/move_base/result")
        return all(topic in published_topics for topic in required)

    @staticmethod
    def _process_is_running(process):
        return process is not None and process.poll() is None

    def _start_route(self):
        if self.state != STATE_CONFIRMED or not self.route_ready:
            rospy.logwarn(
                "[scenario4] r chỉ hợp lệ sau k khi route đã sẵn sàng. Trạng thái: %s",
                self.state,
            )
            return
        if self._process_is_running(self.sender_process):
            rospy.logwarn("[scenario4] gps_waypoint đang chạy; không thể chạy lần hai.")
            return
        if not self._has_move_base_server():
            rospy.logerr(
                "[scenario4] move_base chưa sẵn sàng. Khởi Terminal 1 trước rồi nhấn r."
            )
            return
        if not os.path.isfile(self.route_file_path):
            rospy.logerr("[scenario4] File route bị mất: %s", self.route_file_path)
            return
        if not os.path.isfile(self.send_launch_path):
            rospy.logerr("[scenario4] Không tìm thấy sender launch: %s", self.send_launch_path)
            return

        self.start_cancel_requested = False
        self._set_state(
            STATE_STARTING,
            "Đang khởi gps_waypoint cho route đã khóa.",
        )
        rospy.set_param("/outdoor_waypoint_nav/coordinates_file", self.route_file_param)
        command = [
            "roslaunch",
            self.send_launch_path,
            "coordinates_file:={}".format(self.route_file_param),
        ]
        try:
            self.last_reached_index = 0
            self.finish_pending = False
            self.sender_exit_observed_at = None
            self.sender_process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                preexec_fn=os.setsid,
            )
        except OSError as exc:
            rospy.logerr("[scenario4] Không khởi được gps_waypoint: %s", exc)
            self.sender_process = None
            self._publish_run_started(False)
            if self.start_cancel_requested or self.state == STATE_ABORTED:
                return
            self._set_state(
                STATE_CONFIRMED,
                "Không khởi được gps_waypoint. Route vẫn khóa; kiểm tra log rồi nhấn r lại.",
            )
            return

        if self.start_cancel_requested or self.state == STATE_ABORTED:
            self._terminate_process(self.sender_process, "gps_waypoint")
            self.sender_process = None
            return

        # Mark RUNNING before the child can publish a completion callback, then
        # arm the evaluator/logger. Publishing happens only after Popen
        # succeeds, so a failed launch cannot create a spurious CSV/PNG run.
        self._set_state(
            STATE_RUNNING,
            "Đang chạy route K4 theta={:.1f} deg từ {}.".format(
                self.theta_deg, self.route_file_param
            ),
        )
        self.sender_output_thread = threading.Thread(
            target=self._relay_process_output,
            args=(self.sender_process, "scenario4 sender"),
            daemon=True,
        )
        self.sender_output_thread.start()
        # This latched event starts the K4 evaluator/logger before
        # gps_waypoint can replace its latched waypoint index 0 with index 1.
        self._publish_run_started(True)
        rospy.sleep(0.1)

    def _reached_waypoint_cb(self, message):
        if self.state != STATE_RUNNING:
            return
        self.last_reached_index = max(self.last_reached_index, int(message.data))

    def _finish_cb(self, message):
        if not message.data or self.state != STATE_RUNNING or self.finish_pending:
            return
        # gps_waypoint publishes the final index and finish status on separate
        # topics. Wait briefly so the final index callback cannot be overtaken
        # by the completion callback.
        self.finish_pending = True
        self.finish_timer = rospy.Timer(
            rospy.Duration(0.2), self._finalize_route_from_status, oneshot=True
        )

    def _finalize_route_from_status(self, _event):
        if self.state != STATE_RUNNING:
            return
        self._publish_route_ready(True)
        expected_last_index = len(self.route_points)
        if self.last_reached_index >= expected_last_index:
            detail = (
                "Đã hoàn tất đủ {}/{} waypoint. CSV/PNG đang được lưu; dừng và "
                "khởi động lại Terminal 2 trước lần đo tiếp theo."
            ).format(self.last_reached_index, expected_last_index)
            next_state = STATE_FINISHED
        else:
            detail = (
                "gps_waypoint đã dừng khi mới đạt {}/{} waypoint; run KHÔNG hoàn "
                "chỉnh. Xem CSV/log, sau đó khởi động lại Terminal 2."
            ).format(self.last_reached_index, expected_last_index)
            next_state = STATE_ABORTED
        self._set_state(next_state, detail)

    def _abort_route(self):
        self._publish_zero_velocity()
        sender_was_running = self._process_is_running(self.sender_process)
        calibration_was_running = self._process_is_running(self.calibration_process)
        if self.state == STATE_STARTING and not sender_was_running:
            self.start_cancel_requested = True
            self._publish_route_ready(bool(self.route_points))
            self._set_state(
                STATE_ABORTED,
                "Đã dừng khẩn cấp khi khởi route. Khởi động lại Terminal 2 trước run tiếp theo.",
            )
            return
        if self._process_is_running(self.sender_process):
            self._terminate_process(self.sender_process, "gps_waypoint")
            self.sender_process = None
        if self._process_is_running(self.calibration_process):
            self._terminate_process(self.calibration_process, "heading calibration")
            self.calibration_process = None
        if sender_was_running:
            self._publish_route_ready(bool(self.route_points))
            self._set_state(
                STATE_ABORTED if self.route_points else STATE_IDLE,
                "Đã dừng khẩn cấp. Dừng rồi khởi động lại Terminal 2 trước run tiếp theo.",
            )
        elif calibration_was_running:
            self._set_state(
                self.state,
                "Đã dừng heading calibration và gửi vận tốc 0.",
            )
        else:
            rospy.logwarn(
                "[scenario4] Đã gửi vận tốc 0. Không có waypoint run đang hoạt động."
            )

    def _publish_zero_velocity(self, repeat=10):
        zero = Twist()
        for _ in range(repeat):
            for publisher in self.cmd_vel_pubs:
                publisher.publish(zero)
            rospy.sleep(0.02)

    def _start_heading_calibration(self):
        if self.state == STATE_RUNNING:
            rospy.logwarn("[scenario4] Không hiệu chuẩn heading khi route đang chạy.")
            return
        if self._calibration_is_running():
            rospy.logwarn("[scenario4] Heading calibration đang chạy.")
            return
        if not os.path.isfile(self.heading_calibration_launch_path):
            rospy.logerr(
                "[scenario4] Không tìm thấy %s", self.heading_calibration_launch_path
            )
            return
        rospy.logwarn(
            "[scenario4] Heading calibration sẽ lái xe tiến/lùi. "
            "Bảo đảm khoảng trống phía trước rồi mới tiếp tục."
        )
        try:
            self.calibration_process = subprocess.Popen(
                ["roslaunch", self.heading_calibration_launch_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                preexec_fn=os.setsid,
            )
        except OSError as exc:
            rospy.logerr("[scenario4] Không khởi được heading calibration: %s", exc)
            self.calibration_process = None
            return
        self.calibration_output_thread = threading.Thread(
            target=self._relay_process_output,
            args=(self.calibration_process, "heading calibration"),
            daemon=True,
        )
        self.calibration_output_thread.start()

    def _calibration_is_running(self):
        return self._process_is_running(self.calibration_process)

    @staticmethod
    def _relay_process_output(process, label):
        if process.stdout is None:
            return
        for raw_line in iter(process.stdout.readline, ""):
            line = raw_line.strip()
            if line:
                sys.stdout.write("[{}] {}\n".format(label, line))
                sys.stdout.flush()
        process.stdout.close()

    @staticmethod
    def _terminate_process(process, label):
        if process is None or process.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        except OSError as exc:
            rospy.logwarn("[scenario4] Cannot stop %s: %s", label, exc)

    def _on_shutdown(self):
        if self._process_is_running(self.sender_process):
            self._publish_zero_velocity()
            self._terminate_process(self.sender_process, "gps_waypoint")
        if self._process_is_running(self.calibration_process):
            self._terminate_process(self.calibration_process, "heading calibration")


def main():
    rospy.init_node("scenario_4_waypoint_manager")
    Scenario4WaypointManager()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("[scenario4] Fatal error: %s", exc)
        raise
