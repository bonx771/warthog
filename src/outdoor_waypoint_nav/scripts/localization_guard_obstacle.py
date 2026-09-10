#!/usr/bin/env python3
"""Keep obstacle navigation disabled until localization is safe to use.

The 3-D costmaps transform every LiDAR cloud at its acquisition time.  If the
IMU/EKF heading changes during startup, cells written with the old transform
remain in the costmap and appear as rotated "ghost" obstacles.  This node
keeps the obstacle clouds and velocity output disabled until IMU/TF yaw has
been stable, clears both move_base costmaps, then enables navigation.

The guard also detects discontinuous TF yaw changes after startup.  A detected
jump closes the gates immediately and repeats the stabilization/clear cycle.
"""

import collections
import math
import threading
import time

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from std_srvs.srv import Empty


def quaternion_yaw(quaternion):
    """Return ROS yaw from a geometry_msgs Quaternion."""
    norm = math.sqrt(
        quaternion.x * quaternion.x
        + quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
        + quaternion.w * quaternion.w
    )
    if norm < 1.0e-6:
        raise ValueError("invalid zero-length quaternion")
    x = quaternion.x / norm
    y = quaternion.y / norm
    z = quaternion.z / norm
    w = quaternion.w / norm
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def angle_difference(lhs, rhs):
    """Shortest signed angle lhs-rhs in radians."""
    return math.atan2(math.sin(lhs - rhs), math.cos(lhs - rhs))


class LocalizationGuardObstacle:
    WAITING = "WAITING_FOR_STABLE_LOCALIZATION"
    WARMING = "WARMING_COSTMAP"
    READY = "READY"

    def __init__(self):
        self.imu_topic = rospy.get_param("~imu_topic", "/imu/data_new")
        self.odom_topic = rospy.get_param(
            "~odom_topic", "/outdoor_waypoint_nav/odometry/filtered"
        )
        self.map_odom_topic = rospy.get_param(
            "~map_odom_topic", "/outdoor_waypoint_nav/odometry/filtered_map"
        )
        self.cloud_enable_topic = rospy.get_param(
            "~cloud_enable_topic",
            "/outdoor_waypoint_nav/lidar_costmap_enabled_obstacle",
        )
        self.navigation_ready_topic = rospy.get_param(
            "~navigation_ready_topic",
            "/outdoor_waypoint_nav/navigation_ready_obstacle",
        )
        self.clear_costmaps_service = rospy.get_param(
            "~clear_costmaps_service", "/move_base/clear_costmaps"
        )
        self.map_frame = str(rospy.get_param("~map_frame", "map")).lstrip("/")
        self.odom_frame = str(rospy.get_param("~odom_frame", "odom")).lstrip("/")
        self.base_frame = str(rospy.get_param("~base_frame", "base_link")).lstrip("/")

        self.check_frequency = float(rospy.get_param("~check_frequency", 10.0))
        self.minimum_startup_duration = float(
            rospy.get_param("~minimum_startup_duration", 12.0)
        )
        self.stability_duration = float(rospy.get_param("~stability_duration", 4.0))
        self.yaw_tolerance = math.radians(
            float(rospy.get_param("~yaw_tolerance_deg", 2.0))
        )
        self.minimum_imu_samples = int(rospy.get_param("~minimum_imu_samples", 30))
        self.minimum_tf_samples = int(rospy.get_param("~minimum_tf_samples", 20))
        self.sensor_timeout = float(rospy.get_param("~sensor_timeout", 0.5))
        self.max_linear_speed = float(rospy.get_param("~max_linear_speed", 0.05))
        self.max_angular_speed = float(rospy.get_param("~max_angular_speed", 0.05))
        self.costmap_warmup_duration = float(
            rospy.get_param("~costmap_warmup_duration", 1.0)
        )
        self.localization_loss_timeout = float(
            rospy.get_param("~localization_loss_timeout", 0.5)
        )
        self.jump_angle = math.radians(
            float(rospy.get_param("~jump_angle_deg", 5.0))
        )
        self.jump_rate = math.radians(
            float(rospy.get_param("~jump_rate_deg_per_sec", 45.0))
        )

        if self.check_frequency <= 0.0:
            raise ValueError("check_frequency must be positive")
        if self.stability_duration <= 0.0:
            raise ValueError("stability_duration must be positive")
        if self.sensor_timeout <= 0.0:
            raise ValueError("sensor_timeout must be positive")

        self.lock = threading.RLock()
        self.start_time = time.monotonic()
        self.state = self.WAITING
        self.cloud_enabled = False
        self.navigation_ready = False
        self.warmup_start = None
        self.loss_start = None
        self.last_wait_reason = "waiting for sensor data"

        self.imu_receipt = None
        self.odom_receipt = None
        self.map_odom_receipt = None
        self.odom_motion = (float("inf"), float("inf"))
        self.map_odom_motion = (float("inf"), float("inf"))

        history_length = max(
            100,
            int(self.check_frequency * (self.stability_duration + 3.0)),
            self.minimum_imu_samples * 3,
        )
        # IMU rates vary widely between drivers. Pruning by elapsed time avoids
        # a high-rate IMU truncating the four-second stability window.
        self.imu_yaw_history = collections.deque()
        self.tf_yaw_history = {
            self.map_frame: collections.deque(maxlen=history_length),
            self.odom_frame: collections.deque(maxlen=history_length),
        }
        self.previous_imu = None
        self.previous_tf = {}
        self.pending_jump_reason = None
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.clear_costmaps = rospy.ServiceProxy(self.clear_costmaps_service, Empty)

        self.cloud_enable_pub = rospy.Publisher(
            self.cloud_enable_topic, Bool, queue_size=1, latch=True
        )
        self.navigation_ready_pub = rospy.Publisher(
            self.navigation_ready_topic, Bool, queue_size=1, latch=True
        )
        self.imu_sub = rospy.Subscriber(
            self.imu_topic, Imu, self.imu_callback, queue_size=100
        )
        self.odom_sub = rospy.Subscriber(
            self.odom_topic, Odometry, self.odom_callback, queue_size=20
        )
        self.map_odom_sub = rospy.Subscriber(
            self.map_odom_topic, Odometry, self.map_odom_callback, queue_size=20
        )

        # Latched false values protect the costmaps even when subscribers start
        # before the first timer tick.
        self._publish_outputs(force=True)
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.check_frequency), self.timer_callback
        )

        rospy.logwarn(
            "[localization_guard_obstacle] Navigation locked. Keep the UGV "
            "stopped while IMU/EKF settle (minimum %.1fs, stable %.1fs).",
            self.minimum_startup_duration,
            self.stability_duration,
        )

    @staticmethod
    def _motion_from_odom(msg):
        linear = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        angular = abs(msg.twist.twist.angular.z)
        return linear, angular

    def imu_callback(self, msg):
        now = time.monotonic()
        try:
            yaw = quaternion_yaw(msg.orientation)
        except ValueError:
            rospy.logwarn_throttle(
                2.0, "[localization_guard_obstacle] IMU quaternion is invalid."
            )
            return
        with self.lock:
            self.imu_receipt = now
            self.imu_yaw_history.append((now, yaw))
            self._prune_history(self.imu_yaw_history, now)
            previous = self.previous_imu
            self.previous_imu = (now, yaw)
            if previous is not None:
                dt = now - previous[0]
                delta = abs(angle_difference(yaw, previous[1]))
                if (
                    0.0 < dt <= self.sensor_timeout
                    and delta >= self.jump_angle
                    and delta / dt >= self.jump_rate
                ):
                    self.pending_jump_reason = (
                        "IMU yaw jumped {:.1f}deg in {:.2f}s".format(
                            math.degrees(delta), dt
                        )
                    )

    def odom_callback(self, msg):
        now = time.monotonic()
        with self.lock:
            self.odom_receipt = now
            self.odom_motion = self._motion_from_odom(msg)

    def map_odom_callback(self, msg):
        now = time.monotonic()
        with self.lock:
            self.map_odom_receipt = now
            self.map_odom_motion = self._motion_from_odom(msg)

    def _prune_history(self, history, now):
        oldest_allowed = now - self.stability_duration - 0.5
        while history and history[0][0] < oldest_allowed:
            history.popleft()

    def _sample_transforms(self, now):
        missing_frames = []
        jump_reason = None
        for target_frame in (self.map_frame, self.odom_frame):
            try:
                transform = self.tf_buffer.lookup_transform(
                    target_frame,
                    self.base_frame,
                    rospy.Time(0),
                    rospy.Duration(0.02),
                )
                yaw = quaternion_yaw(transform.transform.rotation)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException, ValueError):
                missing_frames.append(target_frame)
                continue

            history = self.tf_yaw_history[target_frame]
            history.append((now, yaw))
            self._prune_history(history, now)

            previous = self.previous_tf.get(target_frame)
            self.previous_tf[target_frame] = (now, yaw)
            if previous is None:
                continue
            dt = now - previous[0]
            delta = abs(angle_difference(yaw, previous[1]))
            if (
                0.0 < dt <= self.sensor_timeout
                and delta >= self.jump_angle
                and delta / dt >= self.jump_rate
            ):
                jump_reason = (
                    "{}->{} yaw jumped {:.1f}deg in {:.2f}s".format(
                        target_frame,
                        self.base_frame,
                        math.degrees(delta),
                        dt,
                    )
                )

        if missing_frames:
            return False, "missing TF {}->{}".format(
                "/".join(missing_frames), self.base_frame
            ), jump_reason
        return True, "", jump_reason

    def _sensors_fresh(self, now):
        receipts = (
            ("IMU", self.imu_receipt),
            ("odom EKF", self.odom_receipt),
            ("map EKF", self.map_odom_receipt),
        )
        for label, receipt in receipts:
            if receipt is None:
                return False, "waiting for {}".format(label)
            age = now - receipt
            if age > self.sensor_timeout:
                return False, "{} is stale ({:.2f}s)".format(label, age)
        return True, ""

    def _history_stable(self, history, minimum_samples, label):
        if len(history) < minimum_samples:
            return False, "waiting for {} samples ({}/{})".format(
                label, len(history), minimum_samples
            )
        if history[-1][0] - history[0][0] < self.stability_duration:
            remaining = self.stability_duration - (history[-1][0] - history[0][0])
            return False, "{} stability window needs {:.1f}s".format(
                label, max(0.0, remaining)
            )
        reference_yaw = history[-1][1]
        maximum_error = max(
            abs(angle_difference(sample_yaw, reference_yaw))
            for _, sample_yaw in history
        )
        if maximum_error > self.yaw_tolerance:
            return False, "{} yaw still moving ({:.1f}deg)".format(
                label, math.degrees(maximum_error)
            )
        return True, ""

    def _localization_stable(self, now, transforms_available):
        if now - self.start_time < self.minimum_startup_duration:
            return False, "minimum startup wait {:.1f}s remaining".format(
                self.minimum_startup_duration - (now - self.start_time)
            )
        sensors_ok, reason = self._sensors_fresh(now)
        if not sensors_ok:
            return False, reason
        if not transforms_available:
            return False, self.last_wait_reason

        for label, motion in (
            ("odom EKF", self.odom_motion),
            ("map EKF", self.map_odom_motion),
        ):
            if motion[0] > self.max_linear_speed or motion[1] > self.max_angular_speed:
                return False, "UGV must stop: {} speed {:.2f}m/s {:.2f}rad/s".format(
                    label, motion[0], motion[1]
                )

        stable, reason = self._history_stable(
            self.imu_yaw_history, self.minimum_imu_samples, "IMU"
        )
        if not stable:
            return False, reason
        for target_frame in (self.map_frame, self.odom_frame):
            stable, reason = self._history_stable(
                self.tf_yaw_history[target_frame],
                self.minimum_tf_samples,
                "TF {}->{}".format(target_frame, self.base_frame),
            )
            if not stable:
                return False, reason
        return True, ""

    def _publish_outputs(self, force=False):
        # Re-publishing on state changes keeps late subscribers synchronized;
        # both publishers are also latched.
        if force:
            self.cloud_enable_pub.publish(Bool(data=self.cloud_enabled))
            self.navigation_ready_pub.publish(Bool(data=self.navigation_ready))

    def _set_outputs(self, cloud_enabled, navigation_ready):
        changed = (
            self.cloud_enabled != cloud_enabled
            or self.navigation_ready != navigation_ready
        )
        self.cloud_enabled = cloud_enabled
        self.navigation_ready = navigation_ready
        if changed:
            self._publish_outputs(force=True)

    def _reset_histories(self):
        self.imu_yaw_history.clear()
        for history in self.tf_yaw_history.values():
            history.clear()

    def _lock_navigation(self, reason):
        was_ready = self.state != self.WAITING or self.cloud_enabled
        self.state = self.WAITING
        self.warmup_start = None
        self.loss_start = None
        self._set_outputs(False, False)
        self._reset_histories()
        if was_ready:
            rospy.logerr(
                "[localization_guard_obstacle] Navigation LOCKED: %s. "
                "Stopping LiDAR costmap input until localization is stable again.",
                reason,
            )

    def _try_clear_costmaps(self):
        try:
            rospy.wait_for_service(self.clear_costmaps_service, timeout=0.20)
            self.clear_costmaps()
            return True
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn_throttle(
                2.0,
                "[localization_guard_obstacle] Waiting to clear costmaps via %s: %s",
                self.clear_costmaps_service,
                exc,
            )
            return False

    def timer_callback(self, _event):
        now = time.monotonic()
        with self.lock:
            transforms_available, tf_reason, jump_reason = self._sample_transforms(now)
            if not transforms_available:
                self.last_wait_reason = tf_reason

            if self.pending_jump_reason is not None:
                jump_reason = self.pending_jump_reason
                self.pending_jump_reason = None

            if jump_reason is not None:
                self._lock_navigation(jump_reason)
                return

            sensors_ok, sensor_reason = self._sensors_fresh(now)
            base_ok = transforms_available and sensors_ok

            if self.state == self.READY:
                if base_ok:
                    self.loss_start = None
                    return
                if self.loss_start is None:
                    self.loss_start = now
                if now - self.loss_start >= self.localization_loss_timeout:
                    self._lock_navigation(tf_reason or sensor_reason)
                return

            stable, reason = self._localization_stable(now, transforms_available)
            if not stable:
                self.last_wait_reason = tf_reason or reason
                if self.state == self.WARMING:
                    self._lock_navigation(self.last_wait_reason)
                else:
                    rospy.logwarn_throttle(
                        2.0,
                        "[localization_guard_obstacle] Navigation locked: %s",
                        self.last_wait_reason,
                    )
                return

            if self.state == self.WAITING:
                if not self._try_clear_costmaps():
                    return
                self.state = self.WARMING
                self.warmup_start = now
                self._set_outputs(True, False)
                rospy.logwarn(
                    "[localization_guard_obstacle] Localization stable; costmaps "
                    "cleared. Filling them with correctly transformed LiDAR data."
                )
                return

            if self.state == self.WARMING:
                if now - self.warmup_start < self.costmap_warmup_duration:
                    return
                self.state = self.READY
                self.loss_start = None
                self._set_outputs(True, True)
                rospy.loginfo(
                    "[localization_guard_obstacle] NAVIGATION READY. LiDAR "
                    "costmaps are rebuilt; waypoint motion is now enabled."
                )


if __name__ == "__main__":
    rospy.init_node("localization_guard_obstacle")
    LocalizationGuardObstacle()
    rospy.spin()
