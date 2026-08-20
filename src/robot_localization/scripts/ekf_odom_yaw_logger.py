#!/usr/bin/env python3

import math

import rospy
from nav_msgs.msg import Odometry


class EkfOdomYawLogger:
    def __init__(self):
        self.log_rate_hz = rospy.get_param("~log_rate_hz", 2.0)
        self.last_log_time = rospy.Time(0)
        self.min_log_period = rospy.Duration(0.0)
        if self.log_rate_hz > 0.0:
            self.min_log_period = rospy.Duration(1.0 / self.log_rate_hz)

        self.sub = rospy.Subscriber(
            "odometry/filtered", Odometry, self.odom_callback, queue_size=10
        )

    def odom_callback(self, msg):
        now = rospy.Time.now()
        if (
            self.min_log_period.to_sec() > 0.0
            and now - self.last_log_time < self.min_log_period
        ):
            return

        q = msg.pose.pose.orientation
        yaw_rad = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        yaw_deg = math.degrees(yaw_rad)

        rospy.loginfo("ekf_odom_yaw_logger: yaw = %.2f deg", yaw_deg)
        self.last_log_time = now


if __name__ == "__main__":
    rospy.init_node("ekf_odom_yaw_logger")
    EkfOdomYawLogger()
    rospy.spin()
