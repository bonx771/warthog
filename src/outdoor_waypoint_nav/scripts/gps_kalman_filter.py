#!/usr/bin/env python3
"""Smooth GPS odometry before it is fused by the global EKF.

The node expects the output of robot_localization's navsat_transform_node, i.e.
nav_msgs/Odometry in the map frame. It runs a small 2D constant-velocity Kalman
filter over x/y only and republishes an Odometry message for EKF2.

Important covariance rule:
The filtered position is smoother than the raw GPS measurement, but consecutive
outputs are temporally correlated. To avoid making EKF2 over-trust the smoothed
GPS stream, the published x/y covariance is never allowed to be smaller than
the incoming measurement covariance.
"""

import copy
import math

import numpy as np
import rospy
from nav_msgs.msg import Odometry


POSE_SIZE = 6
TWIST_SIZE = 6


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


def _sanitized_covariance(values, size):
    expected = size * size
    if not isinstance(values, (list, tuple)) or len(values) != expected:
        return [0.0] * expected

    result = []
    for value in values:
        try:
            converted = float(value)
        except (TypeError, ValueError):
            converted = 0.0
        result.append(converted if math.isfinite(converted) else 0.0)
    return result


class GpsKalmanFilter:
    def __init__(self):
        self.input_topic = rospy.get_param(
            "~input_topic",
            "/outdoor_waypoint_nav/odometry/gps",
        )
        self.output_topic = rospy.get_param(
            "~output_topic",
            "/outdoor_waypoint_nav/odometry/gps_filter",
        )
        self.motion_odom_topic = rospy.get_param("~motion_odom_topic", "")

        self.process_acceleration_noise = _finite_float_param(
            "~process_acceleration_noise_mps2",
            0.8,
            minimum=0.0,
        )
        self.measurement_covariance_floor = _finite_float_param(
            "~measurement_covariance_floor_m2",
            0.01,
            minimum=0.0,
        )
        self.output_covariance_floor = _finite_float_param(
            "~output_covariance_floor_m2",
            0.01,
            minimum=0.0,
        )
        self.output_min_measurement_covariance_ratio = _finite_float_param(
            "~output_min_measurement_covariance_ratio",
            1.0,
            minimum=0.0,
        )
        self.publish_rate_limit_hz = _finite_float_param(
            "~publish_rate_limit_hz",
            0.0,
            minimum=0.0,
        )
        self.publish_period_sec = (
            1.0 / self.publish_rate_limit_hz
            if self.publish_rate_limit_hz > 0.0
            else 0.0
        )
        self.innovation_covariance_scale = _finite_float_param(
            "~innovation_covariance_scale",
            0.25,
            minimum=0.0,
        )
        self.initial_velocity_variance = _finite_float_param(
            "~initial_velocity_variance",
            1.0,
            minimum=0.0,
        )
        self.reset_timeout_sec = _finite_float_param(
            "~reset_timeout_sec",
            5.0,
            minimum=0.0,
        )
        self.outlier_mahalanobis_threshold = _finite_float_param(
            "~outlier_mahalanobis_threshold",
            6.0,
            minimum=0.0,
        )
        self.outlier_covariance_multiplier = _finite_float_param(
            "~outlier_covariance_multiplier",
            25.0,
            minimum=1.0,
        )
        self.max_consecutive_rejections = int(
            rospy.get_param("~max_consecutive_rejections", 5)
        )
        if self.max_consecutive_rejections < 0:
            raise rospy.ROSException("~max_consecutive_rejections must be >= 0")
        self.motion_odom_timeout_sec = _finite_float_param(
            "~motion_odom_timeout_sec",
            0.5,
            minimum=0.0,
        )
        self.hold_linear_speed_threshold = _finite_float_param(
            "~hold_linear_speed_threshold_mps",
            0.0,
            minimum=0.0,
        )
        self.hold_measurement_covariance_multiplier = _finite_float_param(
            "~hold_measurement_covariance_multiplier",
            8.0,
            minimum=1.0,
        )
        self.hold_skip_measurement_update = bool(
            rospy.get_param("~hold_skip_measurement_update", False)
        )
        self.hold_velocity_decay = _finite_float_param(
            "~hold_velocity_decay",
            0.0,
            minimum=0.0,
        )
        if self.hold_velocity_decay > 1.0:
            raise rospy.ROSException("~hold_velocity_decay must be <= 1.0")
        self.motion_aided_smoothing = bool(
            rospy.get_param("~motion_aided_smoothing", True)
        )
        self.gps_correction_time_constant = _finite_float_param(
            "~gps_correction_time_constant_sec",
            10.0,
            minimum=0.0,
        )
        self.gps_correction_max_speed = _finite_float_param(
            "~gps_correction_max_speed_mps",
            0.10,
            minimum=0.0,
        )
        self.gps_correction_fast_time_constant = _finite_float_param(
            "~gps_correction_fast_time_constant_sec",
            self.gps_correction_time_constant,
            minimum=0.0,
        )
        self.gps_correction_fast_max_speed = _finite_float_param(
            "~gps_correction_fast_max_speed_mps",
            self.gps_correction_max_speed,
            minimum=0.0,
        )
        self.gps_correction_fast_motion_speed = _finite_float_param(
            "~gps_correction_fast_motion_speed_mps",
            0.0,
            minimum=0.0,
        )
        self.gps_correction_deadband = _finite_float_param(
            "~gps_correction_deadband_m",
            0.05,
            minimum=0.0,
        )
        self.gps_correction_residual_clip = _finite_float_param(
            "~gps_correction_residual_clip_m",
            3.0,
            minimum=0.0,
        )
        self.gps_correction_max_dt = _finite_float_param(
            "~gps_correction_max_dt_sec",
            0.1,
            minimum=0.0,
        )
        self.gps_reacquisition_ramp = _finite_float_param(
            "~gps_reacquisition_ramp_sec",
            1.0,
            minimum=0.0,
        )
        self.motion_prediction_position_noise_per_m = _finite_float_param(
            "~motion_prediction_position_noise_m2_per_m",
            0.02,
            minimum=0.0,
        )
        self.motion_prediction_position_noise_per_s = _finite_float_param(
            "~motion_prediction_position_noise_m2_per_s",
            0.005,
            minimum=0.0,
        )
        self.motion_delta_max_speed = _finite_float_param(
            "~motion_delta_max_speed_mps",
            5.0,
            minimum=0.0,
        )

        self.state = None
        self.covariance = None
        self.last_stamp = None
        self.last_publish_stamp = None
        self.consecutive_rejections = 0
        self.latest_motion_speed = None
        self.latest_motion_stamp = None
        self.latest_motion_position = None
        self.last_motion_position_used = None
        self.last_motion_stamp_used = None
        self.reacquisition_elapsed = None

        self.publisher = rospy.Publisher(self.output_topic, Odometry, queue_size=20)
        self.subscriber = rospy.Subscriber(
            self.input_topic,
            Odometry,
            self._callback,
            queue_size=20,
            tcp_nodelay=True,
        )
        self.motion_subscriber = None
        if self.motion_odom_topic and self.hold_linear_speed_threshold > 0.0:
            self.motion_subscriber = rospy.Subscriber(
                self.motion_odom_topic,
                Odometry,
                self._motion_callback,
                queue_size=20,
                tcp_nodelay=True,
            )

        rospy.loginfo(
            "GPS Kalman filter: %s -> %s, q_acc=%.3f m/s^2, "
            "R floor=%.4f m^2, output floor=%.4f m^2, output/input ratio>=%.2f, "
            "publish_rate=%.2f Hz, motion hold=%s, hold skip=%s, "
            "motion aided=%s, gps correction tau=%.2fs max=%.3fm/s "
            "(fast tau=%.2fs max=%.3fm/s above %.2fm/s), correction dt<=%.3fs, "
            "reacquisition ramp=%.2fs",
            self.input_topic,
            self.output_topic,
            self.process_acceleration_noise,
            self.measurement_covariance_floor,
            self.output_covariance_floor,
            self.output_min_measurement_covariance_ratio,
            self.publish_rate_limit_hz,
            self.motion_odom_topic if self.motion_subscriber else "disabled",
            "yes" if self.hold_skip_measurement_update else "no",
            "yes"
            if self.motion_aided_smoothing and self.motion_subscriber
            else "no",
            self.gps_correction_time_constant,
            self.gps_correction_max_speed,
            self.gps_correction_fast_time_constant,
            self.gps_correction_fast_max_speed,
            self.gps_correction_fast_motion_speed,
            self.gps_correction_max_dt,
            self.gps_reacquisition_ramp,
        )

    @staticmethod
    def _stamp_to_sec(msg):
        stamp = msg.header.stamp
        if stamp is None or stamp.to_sec() <= 0.0:
            return rospy.Time.now().to_sec()
        return stamp.to_sec()

    @staticmethod
    def _position_measurement(msg):
        position = msg.pose.pose.position
        x = float(position.x)
        y = float(position.y)
        if not math.isfinite(x) or not math.isfinite(y):
            return None
        return np.array([x, y], dtype=float)

    def _motion_callback(self, msg):
        velocity = msg.twist.twist.linear
        vx = float(velocity.x)
        vy = float(velocity.y)
        if not math.isfinite(vx) or not math.isfinite(vy):
            return

        self.latest_motion_speed = math.hypot(vx, vy)
        self.latest_motion_stamp = self._stamp_to_sec(msg)
        position = msg.pose.pose.position
        x = float(position.x)
        y = float(position.y)
        if math.isfinite(x) and math.isfinite(y):
            self.latest_motion_position = np.array([x, y], dtype=float)

    def _linear_hold_active(self, stamp_sec):
        if (
            self.motion_subscriber is None
            or self.latest_motion_speed is None
            or self.latest_motion_stamp is None
        ):
            return False

        if (
            self.motion_odom_timeout_sec > 0.0
            and abs(stamp_sec - self.latest_motion_stamp) > self.motion_odom_timeout_sec
        ):
            return False

        return self.latest_motion_speed <= self.hold_linear_speed_threshold

    def _measurement_covariance(self, msg):
        cov = _sanitized_covariance(msg.pose.covariance, POSE_SIZE)
        var_x = max(self.measurement_covariance_floor, cov[0])
        var_y = max(self.measurement_covariance_floor, cov[7])

        # Keep the filter conservative. Cross-covariance from navsat_transform is
        # usually zero for horizontal GPS, and dropping it avoids non-positive
        # matrices from malformed upstream data.
        return np.array(
            [
                [var_x, 0.0],
                [0.0, var_y],
            ],
            dtype=float,
        )

    def _innovation_distance(self, innovation, measurement_covariance):
        innovation_covariance = self.covariance[:2, :2] + measurement_covariance
        try:
            innovation_covariance_inv = np.linalg.inv(innovation_covariance)
        except np.linalg.LinAlgError:
            innovation_covariance += np.eye(2) * self.measurement_covariance_floor
            innovation_covariance_inv = np.linalg.inv(innovation_covariance)

        return float(
            math.sqrt(
                max(
                    0.0,
                    innovation.T.dot(innovation_covariance_inv).dot(innovation),
                )
            )
        )

    def _publish_due(self, stamp_sec):
        if self.publish_period_sec <= 0.0 or self.last_publish_stamp is None:
            return True
        if stamp_sec < self.last_publish_stamp:
            return True
        return (stamp_sec - self.last_publish_stamp) >= (self.publish_period_sec - 1e-6)

    def _initialize(self, measurement, measurement_covariance, stamp_sec):
        self.state = np.array(
            [
                measurement[0],
                measurement[1],
                0.0,
                0.0,
            ],
            dtype=float,
        )
        self.covariance = np.diag(
            [
                max(self.output_covariance_floor, measurement_covariance[0, 0]),
                max(self.output_covariance_floor, measurement_covariance[1, 1]),
                self.initial_velocity_variance,
                self.initial_velocity_variance,
            ]
        )
        self.last_stamp = stamp_sec
        self.consecutive_rejections = 0
        self.reacquisition_elapsed = None
        self._sync_motion_reference()

    def _sync_motion_reference(self):
        if self.latest_motion_position is None:
            return
        self.last_motion_position_used = self.latest_motion_position.copy()
        self.last_motion_stamp_used = self.latest_motion_stamp

    def _motion_aided_enabled(self):
        return (
            self.motion_aided_smoothing
            and self.motion_subscriber is not None
            and self.latest_motion_position is not None
        )

    def _predict(self, dt):
        transition = np.array(
            [
                [1.0, 0.0, dt, 0.0],
                [0.0, 1.0, 0.0, dt],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=float,
        )

        acceleration_variance = self.process_acceleration_noise ** 2
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt2 * dt2
        process = acceleration_variance * np.array(
            [
                [0.25 * dt4, 0.0, 0.5 * dt3, 0.0],
                [0.0, 0.25 * dt4, 0.0, 0.5 * dt3],
                [0.5 * dt3, 0.0, dt2, 0.0],
                [0.0, 0.5 * dt3, 0.0, dt2],
            ],
            dtype=float,
        )

        self.state = transition.dot(self.state)
        self.covariance = transition.dot(self.covariance).dot(transition.T) + process

    def _consume_motion_delta(self, dt, stamp_sec):
        if not self._motion_aided_enabled():
            return None

        if (
            self.motion_odom_timeout_sec > 0.0
            and self.latest_motion_stamp is not None
            and abs(stamp_sec - self.latest_motion_stamp)
            > self.motion_odom_timeout_sec
        ):
            return None

        if self.last_motion_position_used is None:
            self._sync_motion_reference()
            return np.zeros(2, dtype=float)

        delta = self.latest_motion_position - self.last_motion_position_used
        if not np.all(np.isfinite(delta)):
            self._sync_motion_reference()
            return None

        elapsed = max(
            1e-3,
            abs((self.latest_motion_stamp or stamp_sec) - (self.last_motion_stamp_used or stamp_sec)),
            dt,
        )
        if self.motion_delta_max_speed > 0.0:
            max_delta = max(1.0, self.motion_delta_max_speed * elapsed + 0.5)
            if float(np.linalg.norm(delta)) > max_delta:
                rospy.logwarn_throttle(
                    2.0,
                    "GPS Kalman ignoring implausible motion odom delta %.2fm over %.2fs.",
                    float(np.linalg.norm(delta)),
                    elapsed,
                )
                self._sync_motion_reference()
                return None

        self._sync_motion_reference()
        return delta

    def _predict_motion_aided(self, dt, stamp_sec):
        delta = self._consume_motion_delta(dt, stamp_sec)
        if delta is None:
            self._predict(dt)
            return False

        self.state[0] += float(delta[0])
        self.state[1] += float(delta[1])
        if dt > 1e-3:
            self.state[2] = float(delta[0]) / dt
            self.state[3] = float(delta[1]) / dt

        distance = float(np.linalg.norm(delta))
        position_growth = (
            self.motion_prediction_position_noise_per_s * max(0.0, dt)
            + self.motion_prediction_position_noise_per_m * distance
        )
        if position_growth > 0.0:
            self.covariance[0, 0] += position_growth
            self.covariance[1, 1] += position_growth
            if dt > 1e-3:
                velocity_growth = position_growth / max(dt * dt, 1e-3)
                self.covariance[2, 2] += velocity_growth
                self.covariance[3, 3] += velocity_growth

        return True

    def _gps_correction_limits(self):
        tau = self.gps_correction_time_constant
        max_speed = self.gps_correction_max_speed
        if (
            self.gps_correction_fast_motion_speed > 0.0
            and self.latest_motion_speed is not None
            and self.latest_motion_speed >= self.gps_correction_fast_motion_speed
        ):
            tau = self.gps_correction_fast_time_constant
            max_speed = self.gps_correction_fast_max_speed
        return tau, max_speed

    def _motion_aided_update(self, measurement, measurement_covariance, dt):
        innovation = measurement - self.state[:2]
        mahalanobis = self._innovation_distance(innovation, measurement_covariance)

        residual = innovation.copy()
        residual_norm = float(np.linalg.norm(residual))
        if residual_norm > 1e-9:
            if self.gps_correction_deadband > 0.0:
                if residual_norm <= self.gps_correction_deadband:
                    residual = np.zeros(2, dtype=float)
                    residual_norm = 0.0
                else:
                    residual *= (
                        residual_norm - self.gps_correction_deadband
                    ) / residual_norm
                    residual_norm = float(np.linalg.norm(residual))

            if (
                self.gps_correction_residual_clip > 0.0
                and residual_norm > self.gps_correction_residual_clip
            ):
                residual *= self.gps_correction_residual_clip / residual_norm
                residual_norm = self.gps_correction_residual_clip

        # Prediction may legitimately span a long GPS outage, but applying
        # that same elapsed time to the GPS correction would permit one large
        # position step when fixes resume.  Limit only the correction clock;
        # motion prediction above still consumes the complete outage delta.
        correction_dt = max(0.0, dt)
        if self.gps_correction_max_dt > 0.0:
            correction_dt = min(correction_dt, self.gps_correction_max_dt)

        # Ease the correction in after a long outage. Besides bounding the
        # position step, this makes correction velocity start at zero so the
        # global trajectory does not acquire a visible corner on reacquisition.
        if self.reacquisition_elapsed is not None:
            if self.gps_reacquisition_ramp <= 0.0:
                self.reacquisition_elapsed = None
            else:
                ramp_scale = min(
                    1.0,
                    self.reacquisition_elapsed / self.gps_reacquisition_ramp,
                )
                self.reacquisition_elapsed += correction_dt
                correction_dt *= ramp_scale
                if self.reacquisition_elapsed >= self.gps_reacquisition_ramp:
                    self.reacquisition_elapsed = None
        correction_tau, correction_max_speed = self._gps_correction_limits()
        if correction_tau <= 0.0:
            gain = 1.0
        else:
            gain = correction_dt / (
                correction_tau + correction_dt
            )
        gain = max(0.0, min(1.0, gain))

        correction = residual * gain
        correction_norm = float(np.linalg.norm(correction))
        if (
            correction_max_speed > 0.0
            and correction_dt > 0.0
            and correction_norm > correction_max_speed * correction_dt
        ):
            correction *= (
                correction_max_speed * correction_dt / correction_norm
            )
            correction_norm = correction_max_speed * correction_dt

        self.state[0] += float(correction[0])
        self.state[1] += float(correction[1])

        covariance_decay = max(0.0, min(0.5, gain))
        self.covariance[0, 0] = max(
            self.output_covariance_floor,
            (1.0 - covariance_decay) * float(self.covariance[0, 0]),
        )
        self.covariance[1, 1] = max(
            self.output_covariance_floor,
            (1.0 - covariance_decay) * float(self.covariance[1, 1]),
        )
        self.consecutive_rejections = 0

        return innovation, mahalanobis, True

    def _update(self, measurement, measurement_covariance):
        observation = np.array(
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
            ],
            dtype=float,
        )

        innovation = measurement - observation.dot(self.state)
        innovation_covariance = (
            observation.dot(self.covariance).dot(observation.T)
            + measurement_covariance
        )

        try:
            innovation_covariance_inv = np.linalg.inv(innovation_covariance)
        except np.linalg.LinAlgError:
            innovation_covariance += np.eye(2) * self.measurement_covariance_floor
            innovation_covariance_inv = np.linalg.inv(innovation_covariance)

        mahalanobis = float(
            math.sqrt(
                max(
                    0.0,
                    innovation.T.dot(innovation_covariance_inv).dot(innovation),
                )
            )
        )

        threshold = self.outlier_mahalanobis_threshold
        if threshold > 0.0 and mahalanobis > threshold:
            self.consecutive_rejections += 1
            if (
                self.max_consecutive_rejections > 0
                and self.consecutive_rejections > self.max_consecutive_rejections
            ):
                rospy.logwarn(
                    "GPS Kalman filter reset after %d consecutive rejected GPS samples.",
                    self.consecutive_rejections,
                )
                self._initialize(
                    measurement,
                    measurement_covariance,
                    self.last_stamp,
                )
                return innovation, mahalanobis, True
            return innovation, mahalanobis, False

        kalman_gain = self.covariance.dot(observation.T).dot(
            innovation_covariance_inv
        )
        self.state = self.state + kalman_gain.dot(innovation)

        identity = np.eye(4)
        joseph = identity - kalman_gain.dot(observation)
        self.covariance = (
            joseph.dot(self.covariance).dot(joseph.T)
            + kalman_gain.dot(measurement_covariance).dot(kalman_gain.T)
        )
        self.consecutive_rejections = 0
        return innovation, mahalanobis, True

    def _output_position_variance(
        self,
        measurement_covariance,
        innovation,
        accepted,
    ):
        variance_x = max(
            self.output_covariance_floor,
            float(self.covariance[0, 0]),
            self.output_min_measurement_covariance_ratio
            * float(measurement_covariance[0, 0]),
        )
        variance_y = max(
            self.output_covariance_floor,
            float(self.covariance[1, 1]),
            self.output_min_measurement_covariance_ratio
            * float(measurement_covariance[1, 1]),
        )

        if innovation is not None:
            variance_x = max(
                variance_x,
                self.innovation_covariance_scale * float(innovation[0] ** 2),
            )
            variance_y = max(
                variance_y,
                self.innovation_covariance_scale * float(innovation[1] ** 2),
            )

        if not accepted:
            variance_x = max(
                variance_x,
                self.outlier_covariance_multiplier
                * float(measurement_covariance[0, 0]),
            )
            variance_y = max(
                variance_y,
                self.outlier_covariance_multiplier
                * float(measurement_covariance[1, 1]),
            )

        return variance_x, variance_y

    def _publish(self, input_msg, measurement_covariance, innovation, accepted, stamp_sec):
        out = copy.deepcopy(input_msg)
        out.pose.pose.position.x = float(self.state[0])
        out.pose.pose.position.y = float(self.state[1])

        pose_covariance = _sanitized_covariance(out.pose.covariance, POSE_SIZE)
        variance_x, variance_y = self._output_position_variance(
            measurement_covariance,
            innovation,
            accepted,
        )
        pose_covariance[0] = variance_x
        pose_covariance[1] = 0.0
        pose_covariance[6] = 0.0
        pose_covariance[7] = variance_y
        out.pose.covariance = pose_covariance

        out.twist.twist.linear.x = float(self.state[2])
        out.twist.twist.linear.y = float(self.state[3])
        twist_covariance = _sanitized_covariance(out.twist.covariance, TWIST_SIZE)
        twist_covariance[0] = max(
            self.output_covariance_floor,
            float(self.covariance[2, 2]),
        )
        twist_covariance[7] = max(
            self.output_covariance_floor,
            float(self.covariance[3, 3]),
        )
        out.twist.covariance = twist_covariance

        self.publisher.publish(out)
        self.last_publish_stamp = stamp_sec

    def _callback(self, msg):
        measurement = self._position_measurement(msg)
        if measurement is None:
            rospy.logwarn_throttle(
                2.0,
                "GPS Kalman filter ignoring non-finite odometry position on %s",
                self.input_topic,
            )
            return

        stamp_sec = self._stamp_to_sec(msg)
        measurement_covariance = self._measurement_covariance(msg)
        hold_active = self._linear_hold_active(stamp_sec)
        effective_measurement_covariance = measurement_covariance
        if hold_active:
            effective_measurement_covariance = (
                measurement_covariance * self.hold_measurement_covariance_multiplier
            )

        if self.state is None:
            self._initialize(measurement, measurement_covariance, stamp_sec)
            self._publish(
                msg,
                effective_measurement_covariance,
                np.zeros(2),
                True,
                stamp_sec,
            )
            return

        dt = stamp_sec - self.last_stamp
        if dt < 0.0:
            rospy.logwarn(
                "GPS Kalman filter received time jump backwards; resetting."
            )
            self._initialize(measurement, measurement_covariance, stamp_sec)
            self._publish(
                msg,
                effective_measurement_covariance,
                np.zeros(2),
                True,
                stamp_sec,
            )
            return

        gps_gap_exceeded = self.reset_timeout_sec > 0.0 and dt > self.reset_timeout_sec
        if gps_gap_exceeded and not self._motion_aided_enabled():
            rospy.logwarn(
                "GPS Kalman filter reset after %.2fs without GPS odometry.",
                dt,
            )
            self._initialize(measurement, measurement_covariance, stamp_sec)
            self._publish(
                msg,
                effective_measurement_covariance,
                np.zeros(2),
                True,
                stamp_sec,
            )
            return
        if gps_gap_exceeded:
            rospy.logwarn(
                "GPS Kalman continuing after %.2fs GPS gap using motion-aided smoothing.",
                dt,
            )
            self.reacquisition_elapsed = 0.0

        update_mode = "normal"
        if hold_active and self.hold_skip_measurement_update:
            self.state[2] *= self.hold_velocity_decay
            self.state[3] *= self.hold_velocity_decay
            self.last_stamp = stamp_sec
            innovation = measurement - self.state[:2]
            mahalanobis = self._innovation_distance(
                innovation,
                effective_measurement_covariance,
            )
            accepted = True
            update_mode = "held"
        else:
            if hold_active:
                self.state[2] *= self.hold_velocity_decay
                self.state[3] *= self.hold_velocity_decay

            motion_prediction_used = False
            if self._motion_aided_enabled():
                motion_prediction_used = self._predict_motion_aided(dt, stamp_sec)
            else:
                self._predict(dt)
            if hold_active:
                self.state[2] *= self.hold_velocity_decay
                self.state[3] *= self.hold_velocity_decay
            self.last_stamp = stamp_sec
            if motion_prediction_used:
                innovation, mahalanobis, accepted = self._motion_aided_update(
                    measurement,
                    effective_measurement_covariance,
                    dt,
                )
                update_mode = "motion"
            else:
                innovation, mahalanobis, accepted = self._update(
                    measurement,
                    effective_measurement_covariance,
                )
            if not accepted:
                update_mode = "rejected"

        if self._publish_due(stamp_sec):
            self._publish(
                msg,
                effective_measurement_covariance,
                innovation,
                accepted,
                stamp_sec,
            )

        rospy.loginfo_throttle(
            5.0,
            "GPS Kalman: raw=(%.2f, %.2f), filtered=(%.2f, %.2f), "
            "R=(%.3f, %.3f), P=(%.3f, %.3f), maha=%.2f, mode=%s",
            measurement[0],
            measurement[1],
            self.state[0],
            self.state[1],
            effective_measurement_covariance[0, 0],
            effective_measurement_covariance[1, 1],
            self.covariance[0, 0],
            self.covariance[1, 1],
            mahalanobis,
            update_mode,
        )


if __name__ == "__main__":
    rospy.init_node("gps_kalman_filter")
    GpsKalmanFilter()
    rospy.spin()
