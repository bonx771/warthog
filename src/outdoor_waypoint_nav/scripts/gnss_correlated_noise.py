#!/usr/bin/env python3
"""Inject reproducible, time-correlated horizontal GNSS noise.

The noise model follows the experiment specification:

    n_k = alpha * n_(k-1) + (1 - alpha) * w_k
    w_k ~ N(0, innovation_sigma^2)

The state is maintained independently in local East and North directions.  The
result is converted to latitude/longitude with WGS-84 radii, so the configured
noise is expressed in metres rather than in arbitrary degrees.  The receiver's
reported NavSatFix covariance is retained and an effective variance is added so
downstream filters do not treat temporally correlated samples as independent.
"""

import copy
import math
import random

import rospy
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Int32


WGS84_A_M = 6378137.0
WGS84_E2 = 6.6943799901413165e-3


def _finite_float_param(name, default, minimum=None):
    value = float(rospy.get_param(name, default))
    if not math.isfinite(value) or (minimum is not None and value < minimum):
        raise rospy.ROSException(
            "{} must be finite{}; got {!r}".format(
                name,
                " and >= {}".format(minimum) if minimum is not None else "",
                value,
            )
        )
    return value


def _enu_offset_to_geodetic_delta(latitude_deg, east_m, north_m):
    """Return (delta_lat_deg, delta_lon_deg) for a small local ENU offset."""
    latitude_rad = math.radians(latitude_deg)
    sin_latitude = math.sin(latitude_rad)
    denominator = math.sqrt(1.0 - WGS84_E2 * sin_latitude * sin_latitude)
    transverse_radius = WGS84_A_M / denominator
    meridional_radius = (
        WGS84_A_M * (1.0 - WGS84_E2) / (denominator * denominator * denominator)
    )

    longitude_radius = transverse_radius * math.cos(latitude_rad)
    if abs(longitude_radius) < 1e-6:
        raise ValueError("latitude is too close to a pole for local ENU conversion")

    return (
        math.degrees(north_m / meridional_radius),
        math.degrees(east_m / longitude_radius),
    )


def _valid_fix(msg):
    return (
        msg.status.status != NavSatStatus.STATUS_NO_FIX
        and math.isfinite(msg.latitude)
        and math.isfinite(msg.longitude)
        and -90.0 <= msg.latitude <= 90.0
        and -180.0 <= msg.longitude <= 180.0
    )


class CorrelatedGnssNoise:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/gps/fix")
        self.output_topic = rospy.get_param("~output_topic", "/gps/fix_noisy")
        self.offset_enu_topic = rospy.get_param(
            "~offset_enu_topic", "/outdoor_waypoint_nav/gnss_noise/offset_enu"
        )
        self.active_topic = rospy.get_param(
            "~active_topic", "/outdoor_waypoint_nav/gnss_noise/active"
        )
        self.odom_topic = rospy.get_param(
            "~distance_odom_topic",
            "/outdoor_waypoint_nav/odometry/filtered_odom",
        )
        self.activation_mode = str(
            rospy.get_param("~activation_mode", "distance")
        ).strip().lower()
        if self.activation_mode not in ("distance", "waypoint_interval"):
            raise rospy.ROSException(
                "~activation_mode must be 'distance' or 'waypoint_interval', got {!r}".format(
                    self.activation_mode
                )
            )

        self.reached_waypoint_topic = rospy.get_param(
            "~reached_waypoint_topic",
            "/outdoor_waypoint_nav/waypoint_reached_index",
        )
        self.finish_topic = str(
            rospy.get_param(
                "~finish_topic", "/outdoor_waypoint_nav/waypoint_following_status"
            )
        ).strip()
        self.home_finish_topic = str(
            rospy.get_param(
                "~home_finish_topic", "/outdoor_waypoint_nav/home_navigation_status"
            )
        ).strip()
        self.noise_start_after_waypoint_index = int(
            rospy.get_param("~noise_start_after_waypoint_index", 2)
        )
        self.noise_stop_at_waypoint_index = int(
            rospy.get_param("~noise_stop_at_waypoint_index", 4)
        )
        if self.activation_mode == "waypoint_interval":
            if self.noise_start_after_waypoint_index < 0:
                raise rospy.ROSException(
                    "~noise_start_after_waypoint_index must be >= 0"
                )
            if self.noise_stop_at_waypoint_index <= self.noise_start_after_waypoint_index:
                raise rospy.ROSException(
                    "~noise_stop_at_waypoint_index must be greater than "
                    "~noise_start_after_waypoint_index"
                )

        self.alpha = _finite_float_param("~alpha", 0.9, minimum=0.0)
        if self.alpha >= 1.0:
            raise rospy.ROSException("~alpha must be in [0, 1), got {}".format(self.alpha))

        common_sigma = _finite_float_param("~innovation_sigma_m", 3.0, minimum=0.0)
        self.innovation_sigma_east = _finite_float_param(
            "~innovation_sigma_east_m", common_sigma, minimum=0.0
        )
        self.innovation_sigma_north = _finite_float_param(
            "~innovation_sigma_north_m", common_sigma, minimum=0.0
        )

        self.start_after_time_sec = _finite_float_param(
            "~start_after_time_sec", 0.0, minimum=0.0
        )
        self.start_after_distance_m = _finite_float_param(
            "~start_after_distance_m", 0.0, minimum=0.0
        )
        self.end_after_distance_m = _finite_float_param(
            "~end_after_distance_m", -1.0
        )
        self.fade_out_start_distance_m = _finite_float_param(
            "~fade_out_start_distance_m", -1.0
        )
        self.max_odom_step_m = _finite_float_param("~max_odom_step_m", 5.0, minimum=0.0)

        if 0.0 <= self.end_after_distance_m < self.start_after_distance_m:
            raise rospy.ROSException(
                "~end_after_distance_m must be -1 or >= ~start_after_distance_m"
            )
        if self.fade_out_start_distance_m >= 0.0:
            if self.end_after_distance_m < 0.0:
                raise rospy.ROSException(
                    "~fade_out_start_distance_m requires a non-negative ~end_after_distance_m"
                )
            if not self.start_after_distance_m <= self.fade_out_start_distance_m < self.end_after_distance_m:
                raise rospy.ROSException(
                    "fade-out must satisfy start_after_distance <= fade_out_start < end_after_distance"
                )

        # A fresh generator is used for every node start. Each experimental run
        # therefore receives an independent realization of the same noise model.
        self.rng = random.Random()

        # For n_k = alpha*n_(k-1) + (1-alpha)*w_k, the stationary variance is
        # ((1-alpha)/(1+alpha)) * Var(w).
        self.stationary_variance_scale = (1.0 - self.alpha) / (1.0 + self.alpha)
        self.stationary_variance_east = (
            self.stationary_variance_scale * self.innovation_sigma_east ** 2
        )
        self.stationary_variance_north = (
            self.stationary_variance_scale * self.innovation_sigma_north ** 2
        )
        # Downstream EKFs assume independent measurements. Inflate the reported
        # covariance by the integrated autocorrelation time of the AR(1) noise.
        self.correlation_inflation_factor = (1.0 + self.alpha) / (1.0 - self.alpha)
        self.effective_variance_east = (
            self.correlation_inflation_factor * self.stationary_variance_east
        )
        self.effective_variance_north = (
            self.correlation_inflation_factor * self.stationary_variance_north
        )

        self.first_fix_time = None
        self.last_odom_position = None
        self.distance_travelled_m = 0.0
        self.state_east_m = 0.0
        self.state_north_m = 0.0
        self.was_active = False
        self.reached_waypoint_index = None
        self.completion_received = False
        self.completion_reason = ""

        self.publisher = rospy.Publisher(self.output_topic, NavSatFix, queue_size=20)
        self.offset_publisher = rospy.Publisher(
            self.offset_enu_topic, Vector3Stamped, queue_size=20
        )
        self.active_publisher = rospy.Publisher(
            self.active_topic, Bool, queue_size=1, latch=True
        )
        self.fix_subscriber = rospy.Subscriber(
            self.input_topic, NavSatFix, self._fix_callback, queue_size=20, tcp_nodelay=True
        )
        self.odom_subscriber = rospy.Subscriber(
            self.odom_topic, Odometry, self._odom_callback, queue_size=50, tcp_nodelay=True
        )
        self.waypoint_subscriber = None
        if self.activation_mode == "waypoint_interval":
            self.waypoint_subscriber = rospy.Subscriber(
                self.reached_waypoint_topic,
                Int32,
                self._reached_waypoint_callback,
                queue_size=10,
                tcp_nodelay=True,
            )
        if self.finish_topic:
            rospy.Subscriber(
                self.finish_topic,
                Bool,
                self._completion_callback,
                callback_args="waypoint completion",
                queue_size=5,
                tcp_nodelay=True,
            )
        if self.home_finish_topic:
            rospy.Subscriber(
                self.home_finish_topic,
                Bool,
                self._completion_callback,
                callback_args="home completion",
                queue_size=5,
                tcp_nodelay=True,
            )

        if self.activation_mode == "waypoint_interval":
            rospy.loginfo(
                "GNSS correlated noise: %s -> %s, alpha=%.3f, "
                "stationary sigma E/N=%.3f/%.3f m, effective R E/N=%.3f/%.3f m^2; active after reached WP%d "
                "and clean again at reached WP%d (topic: %s)",
                self.input_topic,
                self.output_topic,
                self.alpha,
                math.sqrt(self.stationary_variance_east),
                math.sqrt(self.stationary_variance_north),
                self.effective_variance_east,
                self.effective_variance_north,
                self.noise_start_after_waypoint_index,
                self.noise_stop_at_waypoint_index,
                self.reached_waypoint_topic,
            )
        else:
            rospy.loginfo(
                "GNSS correlated noise: %s -> %s, alpha=%.3f, "
                "innovation sigma E/N=%.3f/%.3f m, stationary sigma E/N=%.3f/%.3f m, "
                "effective R E/N=%.3f/%.3f m^2, start time/distance=%.1f s/%.1f m, end=%.1f m, fade start=%.1f m",
                self.input_topic,
                self.output_topic,
                self.alpha,
                self.innovation_sigma_east,
                self.innovation_sigma_north,
                math.sqrt(self.stationary_variance_east),
                math.sqrt(self.stationary_variance_north),
                self.effective_variance_east,
                self.effective_variance_north,
                self.start_after_time_sec,
                self.start_after_distance_m,
                self.end_after_distance_m,
                self.fade_out_start_distance_m,
            )

    def _reached_waypoint_callback(self, msg):
        waypoint_index = int(msg.data)
        if waypoint_index < 0:
            rospy.logwarn_throttle(
                5.0,
                "Ignoring invalid reached waypoint index %d",
                waypoint_index,
            )
            return
        if self.reached_waypoint_index != waypoint_index:
            rospy.loginfo("Waypoint status updated: reached WP%d", waypoint_index)
        self.reached_waypoint_index = waypoint_index

    def _completion_callback(self, msg, reason):
        if not msg.data:
            return
        if self.completion_received:
            return
        self.completion_received = True
        self.completion_reason = str(reason)
        if self.was_active:
            rospy.loginfo("GNSS noise disabled after %s; passing clean fixes through", reason)
        else:
            rospy.loginfo("GNSS noise held clean after %s", reason)
        self.was_active = False
        self._reset_noise_state()
        self.active_publisher.publish(Bool(data=False))

    def _odom_callback(self, msg):
        point = msg.pose.pose.position
        current = (float(point.x), float(point.y))
        if not all(math.isfinite(value) for value in current):
            return

        if self.last_odom_position is not None:
            step = math.hypot(
                current[0] - self.last_odom_position[0],
                current[1] - self.last_odom_position[1],
            )
            if step <= self.max_odom_step_m:
                self.distance_travelled_m += step
            else:
                rospy.logwarn_throttle(
                    5.0,
                    "Ignoring %.2f m jump on %s while measuring GNSS-noise distance",
                    step,
                    self.odom_topic,
                )
        self.last_odom_position = current

    def _reset_noise_state(self):
        self.state_east_m = 0.0
        self.state_north_m = 0.0

    def _is_active(self, now):
        if self.completion_received:
            return False
        if self.activation_mode == "waypoint_interval":
            return (
                self.reached_waypoint_index is not None
                and self.noise_start_after_waypoint_index
                <= self.reached_waypoint_index
                < self.noise_stop_at_waypoint_index
            )
        if self.first_fix_time is None:
            return False
        if (now - self.first_fix_time).to_sec() < self.start_after_time_sec:
            return False
        if self.distance_travelled_m < self.start_after_distance_m:
            return False
        if (
            self.end_after_distance_m >= 0.0
            and self.distance_travelled_m >= self.end_after_distance_m
        ):
            return False
        return True

    def _amplitude_scale(self):
        if self.activation_mode == "waypoint_interval":
            return 1.0
        if self.fade_out_start_distance_m < 0.0:
            return 1.0
        if self.distance_travelled_m <= self.fade_out_start_distance_m:
            return 1.0
        span = self.end_after_distance_m - self.fade_out_start_distance_m
        return max(0.0, min(1.0, (self.end_after_distance_m - self.distance_travelled_m) / span))

    def _with_added_covariance(self, msg, scale):
        result = copy.deepcopy(msg)
        covariance = list(result.position_covariance)
        if len(covariance) != 9:
            covariance = [0.0] * 9

        covariance = [value if math.isfinite(value) else 0.0 for value in covariance]
        covariance[0] = max(0.0, covariance[0]) + scale * scale * self.effective_variance_east
        covariance[4] = max(0.0, covariance[4]) + scale * scale * self.effective_variance_north
        result.position_covariance = covariance
        if result.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            result.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        return result

    def _publish_offset(self, header, east_m, north_m, active):
        offset = Vector3Stamped()
        offset.header = copy.deepcopy(header)
        offset.vector.x = east_m
        offset.vector.y = north_m
        offset.vector.z = 0.0
        self.offset_publisher.publish(offset)
        self.active_publisher.publish(Bool(data=active))

    def _fix_callback(self, msg):
        if not _valid_fix(msg):
            self.publisher.publish(msg)
            self._publish_offset(msg.header, 0.0, 0.0, False)
            return

        now = rospy.Time.now()
        if self.first_fix_time is None:
            self.first_fix_time = now

        active = self._is_active(now)
        if not active:
            if self.was_active:
                if self.activation_mode == "waypoint_interval":
                    rospy.loginfo(
                        "GNSS noise disabled at reached WP%d; passing clean fixes through",
                        self.reached_waypoint_index,
                    )
                else:
                    rospy.loginfo(
                        "GNSS noise disabled at travelled distance %.1f m; passing clean fixes through",
                        self.distance_travelled_m,
                    )
                self._reset_noise_state()
            self.was_active = False
            self.publisher.publish(msg)
            self._publish_offset(msg.header, 0.0, 0.0, False)
            return

        if not self.was_active:
            self._reset_noise_state()
            if self.activation_mode == "waypoint_interval":
                rospy.loginfo(
                    "GNSS noise enabled after reached WP%d",
                    self.reached_waypoint_index,
                )
            else:
                rospy.loginfo(
                    "GNSS noise enabled at travelled distance %.1f m",
                    self.distance_travelled_m,
                )
        self.was_active = True

        self.state_east_m = (
            self.alpha * self.state_east_m
            + (1.0 - self.alpha) * self.rng.gauss(0.0, self.innovation_sigma_east)
        )
        self.state_north_m = (
            self.alpha * self.state_north_m
            + (1.0 - self.alpha) * self.rng.gauss(0.0, self.innovation_sigma_north)
        )
        scale = self._amplitude_scale()
        east_m = scale * self.state_east_m
        north_m = scale * self.state_north_m

        try:
            delta_latitude, delta_longitude = _enu_offset_to_geodetic_delta(
                msg.latitude, east_m, north_m
            )
        except ValueError as exc:
            rospy.logerr_throttle(5.0, "Cannot add GNSS noise: %s", exc)
            self.publisher.publish(msg)
            self._publish_offset(msg.header, 0.0, 0.0, False)
            return

        noisy_msg = self._with_added_covariance(msg, scale)
        noisy_msg.latitude += delta_latitude
        noisy_msg.longitude += delta_longitude
        self.publisher.publish(noisy_msg)
        self._publish_offset(msg.header, east_m, north_m, True)

        if self.activation_mode == "waypoint_interval":
            rospy.loginfo_throttle(
                5.0,
                "GNSS noise active: reached WP%d, E/N=%.2f/%.2f m, R_eff E/N=%.2f/%.2f m^2",
                self.reached_waypoint_index,
                east_m,
                north_m,
                scale * scale * self.effective_variance_east,
                scale * scale * self.effective_variance_north,
            )
        else:
            rospy.loginfo_throttle(
                5.0,
                "GNSS noise active: distance=%.1f m, scale=%.2f, E/N=%.2f/%.2f m, R_eff E/N=%.2f/%.2f m^2",
                self.distance_travelled_m,
                scale,
                east_m,
                north_m,
                scale * scale * self.effective_variance_east,
                scale * scale * self.effective_variance_north,
            )


if __name__ == "__main__":
    rospy.init_node("gnss_correlated_noise")
    CorrelatedGnssNoise()
    rospy.spin()
