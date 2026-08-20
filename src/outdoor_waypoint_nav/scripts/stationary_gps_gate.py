#!/usr/bin/env python3

import copy
import math

import rospy
from nav_msgs.msg import Odometry


def _covariance_with_stationary_floor(
    covariance,
    xy_floor,
    z_floor,
    sample_variance=None,
):
    cov = list(covariance)
    if len(cov) != 36:
        cov = [0.0] * 36

    var_x = var_y = var_z = 0.0
    if sample_variance is not None:
        var_x, var_y, var_z = sample_variance

    cov[0] = max(float(cov[0]), xy_floor, var_x)
    cov[7] = max(float(cov[7]), xy_floor, var_y)
    cov[14] = max(float(cov[14]), z_floor, var_z)
    return cov


def _position_tuple(msg):
    pos = msg.pose.pose.position
    return (float(pos.x), float(pos.y), float(pos.z))


def _set_position(msg, position):
    pos = msg.pose.pose.position
    pos.x, pos.y, pos.z = position


def _median(values):
    ordered = sorted(values)
    count = len(ordered)
    middle = count // 2
    if count % 2:
        return ordered[middle]
    return 0.5 * (ordered[middle - 1] + ordered[middle])


class StationaryGpsGate:
    def __init__(self):
        gps_topic = rospy.get_param("~gps_odom_topic", "odometry/gps")
        motion_topic = rospy.get_param("~motion_odom_topic", "odometry/filtered_odom")
        output_topic = rospy.get_param("~output_topic", "odometry/gps_filter")

        self.linear_threshold = float(rospy.get_param("~linear_stop_threshold", 0.03))
        self.angular_threshold = float(rospy.get_param("~angular_stop_threshold", 0.01))
        self.stop_confirm_duration = float(
            rospy.get_param("~stop_confirm_duration", 0.75)
        )
        self.motion_timeout = float(rospy.get_param("~motion_timeout", 0.5))
        self.stationary_xy_covariance = float(
            rospy.get_param("~stationary_xy_covariance", 4.0)
        )
        self.stationary_z_covariance = float(
            rospy.get_param("~stationary_z_covariance", 9.0)
        )
        self.stationary_hold_xy_covariance = float(
            rospy.get_param("~stationary_hold_xy_covariance", 100.0)
        )
        self.stationary_hold_z_covariance = float(
            rospy.get_param("~stationary_hold_z_covariance", 100.0)
        )
        self.stationary_collection_duration = float(
            rospy.get_param("~stationary_collection_duration", 5.0)
        )
        self.stationary_correction_publish_count = max(
            0,
            int(rospy.get_param("~stationary_correction_publish_count", 2)),
        )
        self.stationary_sample_window = max(
            1,
            int(rospy.get_param("~stationary_sample_window", 50)),
        )
        self.stationary_min_samples = max(
            1,
            int(rospy.get_param("~stationary_min_samples", 10)),
        )
        self.stationary_outlier_rejection = float(
            rospy.get_param("~stationary_outlier_rejection_m", 2.0)
        )
        self.release_blend_duration = max(
            0.0,
            float(rospy.get_param("~release_blend_duration", 1.0)),
        )
        self.copy_motion_orientation = bool(
            rospy.get_param("~copy_motion_orientation", True)
        )

        self.last_motion_time = None
        self.last_linear_speed = float("inf")
        self.last_angular_speed = float("inf")
        self.last_motion_orientation = None
        self.stationary_since = None
        self.was_stationary = False
        self.stationary_anchor = None
        self.stationary_samples = []
        self.stationary_collection_start = None
        self.stationary_correction_position = None
        self.stationary_correction_variance = None
        self.stationary_correction_publish_index = 0
        self.last_output_position = None
        self.release_start_time = None
        self.release_start_position = None

        self.publisher = rospy.Publisher(output_topic, Odometry, queue_size=10)
        self.motion_subscriber = rospy.Subscriber(
            motion_topic,
            Odometry,
            self._motion_callback,
            queue_size=20,
            tcp_nodelay=True,
        )
        self.gps_subscriber = rospy.Subscriber(
            gps_topic,
            Odometry,
            self._gps_callback,
            queue_size=20,
            tcp_nodelay=True,
        )

        rospy.loginfo(
            "Stationary GPS gate: gps=%s, motion=%s, output=%s, "
            "linear<=%.3f m/s, angular<=%.3f rad/s, confirm=%.2f s, "
            "collect=%.2f s, corrections=%d, covariance floor xy/z=%.3f/%.3f, "
            "hold covariance xy/z=%.3f/%.3f",
            gps_topic,
            motion_topic,
            output_topic,
            self.linear_threshold,
            self.angular_threshold,
            self.stop_confirm_duration,
            self.stationary_collection_duration,
            self.stationary_correction_publish_count,
            self.stationary_xy_covariance,
            self.stationary_z_covariance,
            self.stationary_hold_xy_covariance,
            self.stationary_hold_z_covariance,
        )

    def _motion_callback(self, msg):
        now = rospy.Time.now()
        twist = msg.twist.twist
        self.last_motion_time = now
        self.last_linear_speed = math.hypot(twist.linear.x, twist.linear.y)
        self.last_angular_speed = abs(twist.angular.z)
        self.last_motion_orientation = copy.deepcopy(msg.pose.pose.orientation)

        stopped = (
            self.last_linear_speed <= self.linear_threshold
            and self.last_angular_speed <= self.angular_threshold
        )

        if stopped:
            if self.stationary_since is None:
                self.stationary_since = now
        else:
            self.stationary_since = None
            self._reset_stationary_state()

    def _is_stationary(self):
        if self.last_motion_time is None or self.stationary_since is None:
            return False

        now = rospy.Time.now()
        if (now - self.last_motion_time).to_sec() > self.motion_timeout:
            return False

        return (now - self.stationary_since).to_sec() >= self.stop_confirm_duration

    def _is_stopped_now(self):
        if self.last_motion_time is None:
            return False

        now = rospy.Time.now()
        if (now - self.last_motion_time).to_sec() > self.motion_timeout:
            return False

        return (
            self.last_linear_speed <= self.linear_threshold
            and self.last_angular_speed <= self.angular_threshold
        )

    def _apply_motion_orientation(self, msg):
        if self.copy_motion_orientation and self.last_motion_orientation is not None:
            msg.pose.pose.orientation = copy.deepcopy(self.last_motion_orientation)

    def _reset_stationary_state(self):
        self.stationary_anchor = None
        self.stationary_samples = []
        self.stationary_collection_start = None
        self.stationary_correction_position = None
        self.stationary_correction_variance = None
        self.stationary_correction_publish_index = 0

    def _append_stationary_sample(self, msg):
        self.stationary_samples.append(_position_tuple(msg))
        if len(self.stationary_samples) > self.stationary_sample_window:
            self.stationary_samples.pop(0)

    def _stationary_estimate(self, fallback_msg):
        if not self.stationary_samples:
            return _position_tuple(fallback_msg), (0.0, 0.0, 0.0)

        samples = self.stationary_samples
        median_x = _median([sample[0] for sample in samples])
        median_y = _median([sample[1] for sample in samples])

        inliers = []
        rejection_sq = self.stationary_outlier_rejection ** 2
        for sample in samples:
            dx = sample[0] - median_x
            dy = sample[1] - median_y
            if dx * dx + dy * dy <= rejection_sq:
                inliers.append(sample)

        if len(inliers) < self.stationary_min_samples:
            inliers = samples

        count = float(len(inliers))
        mean_x = sum(sample[0] for sample in inliers) / count
        mean_y = sum(sample[1] for sample in inliers) / count
        mean_z = sum(sample[2] for sample in inliers) / count

        var_x = sum((sample[0] - mean_x) ** 2 for sample in inliers) / count
        var_y = sum((sample[1] - mean_y) ** 2 for sample in inliers) / count
        var_z = sum((sample[2] - mean_z) ** 2 for sample in inliers) / count
        return (mean_x, mean_y, mean_z), (var_x, var_y, var_z)

    def _start_release_blend(self, now):
        if self.release_blend_duration <= 0.0 or self.last_output_position is None:
            self.release_start_time = None
            self.release_start_position = None
            return

        self.release_start_time = now
        self.release_start_position = self.last_output_position

    def _apply_release_blend(self, msg, now):
        if self.release_start_time is None or self.release_start_position is None:
            return False

        elapsed = (now - self.release_start_time).to_sec()
        alpha = elapsed / self.release_blend_duration
        if alpha >= 1.0:
            self.release_start_time = None
            self.release_start_position = None
            return False

        raw_position = _position_tuple(msg)
        alpha = max(0.0, min(1.0, alpha))
        blended_position = tuple(
            self.release_start_position[index] * (1.0 - alpha)
            + raw_position[index] * alpha
            for index in range(3)
        )
        _set_position(msg, blended_position)
        msg.pose.covariance = _covariance_with_stationary_floor(
            msg.pose.covariance,
            self.stationary_xy_covariance,
            self.stationary_z_covariance,
        )
        return True

    def _publish(self, msg):
        self.last_output_position = _position_tuple(msg)
        self.publisher.publish(msg)

    def _publish_stationary_hold(self, msg, position):
        out = copy.deepcopy(msg)
        _set_position(out, position)
        self._apply_motion_orientation(out)
        out.pose.covariance = _covariance_with_stationary_floor(
            out.pose.covariance,
            self.stationary_hold_xy_covariance,
            self.stationary_hold_z_covariance,
        )
        self._publish(out)

    def _publish_stationary_correction(self, msg):
        out = copy.deepcopy(msg)
        _set_position(out, self.stationary_correction_position)
        self._apply_motion_orientation(out)
        out.pose.covariance = _covariance_with_stationary_floor(
            out.pose.covariance,
            self.stationary_xy_covariance,
            self.stationary_z_covariance,
            self.stationary_correction_variance,
        )
        self.stationary_correction_publish_index += 1
        self._publish(out)

    def _gps_callback(self, msg):
        now = rospy.Time.now()
        stopped_now = self._is_stopped_now()
        stationary = self._is_stationary()

        if not stationary:
            if self.was_stationary:
                rospy.loginfo(
                    "Stationary GPS gate released: speed=%.4f m/s, yaw_rate=%.4f rad/s",
                    self.last_linear_speed,
                    self.last_angular_speed,
                )
                self._start_release_blend(now)
            self.was_stationary = False
            if not stopped_now:
                self._reset_stationary_state()
            out = copy.deepcopy(msg)
            self._apply_release_blend(out, now)
            self._apply_motion_orientation(out)
            self._publish(out)
            return

        if self.stationary_anchor is None:
            self.stationary_anchor = copy.deepcopy(msg)
            self.stationary_collection_start = now
            self.stationary_samples = []
            rospy.loginfo(
                "Stationary GPS gate locked near x=%.3f y=%.3f; collecting GPS for %.2f s",
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                self.stationary_collection_duration,
            )

        self.was_stationary = True
        if self.stationary_correction_position is None:
            self._append_stationary_sample(msg)

            elapsed = (now - self.stationary_collection_start).to_sec()
            enough_time = elapsed >= self.stationary_collection_duration
            enough_samples = len(self.stationary_samples) >= self.stationary_min_samples
            if not enough_time or not enough_samples:
                self._publish_stationary_hold(
                    msg,
                    _position_tuple(self.stationary_anchor),
                )
                return

            position, sample_variance = self._stationary_estimate(msg)
            self.stationary_correction_position = position
            self.stationary_correction_variance = sample_variance
            rospy.loginfo(
                "Stationary GPS correction ready at x=%.3f y=%.3f from %d samples; publishing %d corrections",
                position[0],
                position[1],
                len(self.stationary_samples),
                self.stationary_correction_publish_count,
            )

        if (
            self.stationary_correction_publish_index
            < self.stationary_correction_publish_count
        ):
            self._publish_stationary_correction(msg)
            return

        self._publish_stationary_hold(msg, self.stationary_correction_position)


if __name__ == "__main__":
    rospy.init_node("stationary_gps_gate")
    StationaryGpsGate()
    rospy.spin()
