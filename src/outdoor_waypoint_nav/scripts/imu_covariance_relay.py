#!/usr/bin/env python3

import copy

import rospy
from sensor_msgs.msg import Imu


# Edit these defaults if you want one central place to control all IMU covariances.
DEFAULT_ORIENTATION_DIAG = (0.1, 0.1, 0.1)
DEFAULT_ANGULAR_VELOCITY_DIAG = (0.02, 0.02, 0.02)
DEFAULT_LINEAR_ACCELERATION_DIAG = (0.1, 0.1, 0.1)


def _read_diag_param(name, default_diag):
    values = rospy.get_param("~" + name, list(default_diag))
    if not isinstance(values, (list, tuple)) or len(values) != 3:
        raise rospy.ROSException(
            "~{} must be a list of three numbers, got {!r}".format(name, values)
        )

    diag = tuple(float(value) for value in values)
    if any(value < 0.0 for value in diag):
        raise rospy.ROSException(
            "~{} must contain non-negative values, got {!r}".format(name, values)
        )
    return diag


def _diag_to_covariance(diag):
    return [
        diag[0], 0.0, 0.0,
        0.0, diag[1], 0.0,
        0.0, 0.0, diag[2],
    ]


class ImuCovarianceRelay:
    def __init__(self):
        input_topic = rospy.get_param("~input_topic", "/imu_filter_source/imu/data")
        output_topic = rospy.get_param("~output_topic", "/imu/data")

        orientation_diag = _read_diag_param("orientation_diag", DEFAULT_ORIENTATION_DIAG)
        angular_velocity_diag = _read_diag_param(
            "angular_velocity_diag", DEFAULT_ANGULAR_VELOCITY_DIAG
        )
        linear_acceleration_diag = _read_diag_param(
            "linear_acceleration_diag", DEFAULT_LINEAR_ACCELERATION_DIAG
        )

        self.orientation_covariance = _diag_to_covariance(orientation_diag)
        self.angular_velocity_covariance = _diag_to_covariance(angular_velocity_diag)
        self.linear_acceleration_covariance = _diag_to_covariance(
            linear_acceleration_diag
        )

        self.publisher = rospy.Publisher(output_topic, Imu, queue_size=10)
        self.subscriber = rospy.Subscriber(input_topic, Imu, self._callback, queue_size=10)

        rospy.loginfo(
            "Relaying IMU from %s to %s with hard covariance override: orientation diag %s, angular velocity diag %s, linear acceleration diag %s",
            input_topic,
            output_topic,
            orientation_diag,
            angular_velocity_diag,
            linear_acceleration_diag,
        )

    def _callback(self, msg):
        relay_msg = copy.deepcopy(msg)
        relay_msg.orientation_covariance = list(self.orientation_covariance)
        relay_msg.angular_velocity_covariance = list(self.angular_velocity_covariance)
        relay_msg.linear_acceleration_covariance = list(self.linear_acceleration_covariance)
        self.publisher.publish(relay_msg)


if __name__ == "__main__":
    rospy.init_node("imu_covariance_relay")
    ImuCovarianceRelay()
    rospy.spin()
