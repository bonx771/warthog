#!/usr/bin/env python3

import math

import message_filters
import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion


def _normalized_quaternion(quaternion_msg):
    quat = [
        quaternion_msg.x,
        quaternion_msg.y,
        quaternion_msg.z,
        quaternion_msg.w,
    ]
    norm = math.sqrt(sum(value * value for value in quat))
    if norm < 1e-12:
        return None

    return [value / norm for value in quat]


def _yaw_from_imu(msg):
    if (
        len(msg.orientation_covariance) >= 1
        and msg.orientation_covariance[0] < 0.0
    ):
        return None

    quat = _normalized_quaternion(msg.orientation)
    if quat is None:
        return None

    return euler_from_quaternion(quat)[2]


def _format_yaw(label, msg):
    yaw = _yaw_from_imu(msg)
    if yaw is None:
        return "{} = unavailable".format(label)

    return "{} = {:.2f} deg".format(label, math.degrees(yaw))


def _format_yaw_value(label, yaw):
    if yaw is None:
        return "{} = unavailable".format(label)

    return "{} = {:.2f} deg".format(label, math.degrees(yaw))


def _format_imu_wz(label, msg):
    if msg is None:
        return "{} = unavailable".format(label)

    return "{} = {:.5f} rad/s".format(label, msg.angular_velocity.z)


def _message_stamp(msg):
    if msg.header.stamp != rospy.Time():
        return msg.header.stamp
    return rospy.Time.now()


def _sync_span_ms(*msgs):
    stamps = [_message_stamp(msg) for msg in msgs]
    return (max(stamps) - min(stamps)).to_sec() * 1000.0


def _wrap_angle_rad(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class ImuCovarianceLog:
    def __init__(self):
        self.log_rate_hz = rospy.get_param("~log_rate_hz", 10.0)
        self.data_topic = rospy.get_param("~data_topic", "/imu/data")
        self.raw_topic = rospy.get_param("~raw_topic", "/imu/data_raw")
        self.raw_bias_topic = rospy.get_param("~raw_bias_topic", "/imu/data_raw_bias")
        self.filter_madgwick_topic = rospy.get_param(
            "~filter_madgwick_topic", "/imu/data_filter_madgwick"
        )
        self.covariance_topic = rospy.get_param(
            "~covariance_topic", "/imu/data_covariance"
        )
        self.sync_queue_size = int(rospy.get_param("~sync_queue_size", 50))
        self.sync_slop = float(rospy.get_param("~sync_slop", 0.02))

        self.last_log_time = rospy.Time(0)
        self.min_log_period = rospy.Duration(0.0)
        if self.log_rate_hz > 0.0:
            self.min_log_period = rospy.Duration(1.0 / self.log_rate_hz)

        # Raw IMU topics often omit orientation, so keep a gyro-integrated yaw
        # fallback for logging.
        self.raw_yaw_estimate = None
        self.raw_last_stamp = None
        self.raw_bias_yaw_estimate = None
        self.raw_bias_last_stamp = None

        self.data_sub = message_filters.Subscriber(self.data_topic, Imu)
        self.raw_sub = message_filters.Subscriber(self.raw_topic, Imu)
        self.raw_bias_sub = message_filters.Subscriber(self.raw_bias_topic, Imu)
        self.filter_madgwick_sub = message_filters.Subscriber(
            self.filter_madgwick_topic, Imu
        )
        self.covariance_sub = message_filters.Subscriber(self.covariance_topic, Imu)

        # Synchronize the four IMU stages so each log line compares matching samples.
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [
                self.data_sub,
                self.raw_sub,
                self.raw_bias_sub,
                self.filter_madgwick_sub,
                self.covariance_sub,
            ],
            self.sync_queue_size,
            self.sync_slop,
            allow_headerless=False,
        )
        self.sync.registerCallback(self._callback)

        rospy.loginfo(
            "Logging synchronized IMU yaw topics: data=%s, raw=%s, raw_bias=%s, filter_madgwick=%s, covariance=%s, queue=%d, slop=%.3f s",
            self.data_topic,
            self.raw_topic,
            self.raw_bias_topic,
            self.filter_madgwick_topic,
            self.covariance_topic,
            self.sync_queue_size,
            self.sync_slop,
        )

    def _update_yaw_estimate(self, msg, previous_yaw, previous_stamp):
        yaw = _yaw_from_imu(msg)
        stamp = _message_stamp(msg)
        if yaw is not None:
            return yaw, stamp

        if previous_yaw is None or previous_stamp is None:
            return 0.0, stamp

        dt = (stamp - previous_stamp).to_sec()
        if dt <= 0.0:
            return previous_yaw, stamp

        return _wrap_angle_rad(previous_yaw + msg.angular_velocity.z * dt), stamp

    def _callback(
        self,
        data_msg,
        raw_msg,
        raw_bias_msg,
        filter_madgwick_msg,
        covariance_msg,
    ):
        self.raw_yaw_estimate, self.raw_last_stamp = self._update_yaw_estimate(
            raw_msg, self.raw_yaw_estimate, self.raw_last_stamp
        )
        (
            self.raw_bias_yaw_estimate,
            self.raw_bias_last_stamp,
        ) = self._update_yaw_estimate(
            raw_bias_msg,
            self.raw_bias_yaw_estimate,
            self.raw_bias_last_stamp,
        )

        now = rospy.Time.now()
        if (
            self.min_log_period.to_sec() > 0.0
            and now - self.last_log_time < self.min_log_period
        ):
            return

        rospy.loginfo(
            "%s, %s, %s, %s, %s, %s, %s, sync_span_ms = %.3f",
            _format_yaw("imu_data", data_msg),
            _format_yaw_value("imu_raw", self.raw_yaw_estimate),
            _format_yaw_value("imu_raw_bias", self.raw_bias_yaw_estimate),
            _format_yaw("imu_filter_madgwick", filter_madgwick_msg),
            _format_yaw("imu_covariance", covariance_msg),
            _format_imu_wz("imu_wz_raw", raw_msg),
            _format_imu_wz("imu_wz_bias", raw_bias_msg),
            _sync_span_ms(
                data_msg,
                raw_msg,
                raw_bias_msg,
                filter_madgwick_msg,
                covariance_msg,
            ),
        )                                                                            
        self.last_log_time = now


if __name__ == "__main__":
    rospy.init_node("imu_covariance_log")
    ImuCovarianceLog()
    rospy.spin()
