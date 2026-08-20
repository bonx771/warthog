#!/usr/bin/env python3

import copy
import math
from collections import deque

import rospy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def _normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw_from_quaternion(quaternion_msg):
    quat = [
        quaternion_msg.x,
        quaternion_msg.y,
        quaternion_msg.z,
        quaternion_msg.w,
    ]
    norm = math.sqrt(sum(value * value for value in quat))
    if norm < 1e-12:
        return 0.0

    quat = [value / norm for value in quat]
    return euler_from_quaternion(quat)[2]


def _quaternion_from_yaw(yaw):
    quat = quaternion_from_euler(0.0, 0.0, yaw)
    quaternion_msg = Quaternion()
    quaternion_msg.x = quat[0]
    quaternion_msg.y = quat[1]
    quaternion_msg.z = quat[2]
    quaternion_msg.w = quat[3]
    return quaternion_msg


class EncoderOdometryNode:
    def __init__(self):
        self.wheel_odom_topic = rospy.get_param(
            "~wheel_odom_topic", "/warthog_velocity_controller/odom"
        )
        self.filtered_odom_topic = rospy.get_param(
            "~filtered_odom_topic", "odometry/filtered"
        )
        self.output_topic = rospy.get_param("~output_topic", "/odometry/encoder")
        self.imu_topic = rospy.get_param("~imu_topic", "/imu/data_new")
        self.align_yaw_to_imu = bool(rospy.get_param("~align_yaw_to_imu", True))
        self.imu_anchor_timeout_sec = float(
            rospy.get_param("~imu_anchor_timeout_sec", 5.0)
        )
        self.default_frame_id = rospy.get_param("~default_frame_id", "odom")
        self.default_child_frame_id = rospy.get_param(
            "~default_child_frame_id", "base_link"
        )
        self.anchor_ready = False
        self.wheel_history = deque(maxlen=200)

        self.publisher = rospy.Publisher(self.output_topic, Odometry, queue_size=20)
        self.subscriber = rospy.Subscriber(
            self.wheel_odom_topic,
            Odometry,
            self._wheel_odom_callback,
            queue_size=100,
            tcp_nodelay=True,
        )

        self.filtered_anchor_msg = self._wait_for_anchor_message(
            self.filtered_odom_topic, "initial filtered pose"
        )
        self.wheel_anchor_msg = self._select_wheel_anchor(self.filtered_anchor_msg)

        self.output_frame_id = (
            self.filtered_anchor_msg.header.frame_id or self.default_frame_id
        )
        self.output_child_frame_id = (
            self.wheel_anchor_msg.child_frame_id
            or self.filtered_anchor_msg.child_frame_id
            or self.default_child_frame_id
        )

        self.filtered_anchor_x = self.filtered_anchor_msg.pose.pose.position.x
        self.filtered_anchor_y = self.filtered_anchor_msg.pose.pose.position.y
        self.filtered_anchor_z = self.filtered_anchor_msg.pose.pose.position.z
        self.filtered_anchor_yaw = _yaw_from_quaternion(
            self.filtered_anchor_msg.pose.pose.orientation
        )

        self.wheel_anchor_x = self.wheel_anchor_msg.pose.pose.position.x
        self.wheel_anchor_y = self.wheel_anchor_msg.pose.pose.position.y
        self.wheel_anchor_z = self.wheel_anchor_msg.pose.pose.position.z
        self.wheel_anchor_yaw = _yaw_from_quaternion(
            self.wheel_anchor_msg.pose.pose.orientation
        )
        self.imu_anchor_yaw = None

        if self.align_yaw_to_imu:
            self.imu_anchor_yaw = self._wait_for_imu_anchor_yaw()
            if self.imu_anchor_yaw is not None:
                self.filtered_anchor_yaw = self.imu_anchor_yaw

        anchor_dt = (
            self.wheel_anchor_msg.header.stamp - self.filtered_anchor_msg.header.stamp
        ).to_sec()
        rospy.loginfo(
            "Anchored encoder odom: filtered=(%.3f, %.3f, %.3f deg), wheel=(%.3f, %.3f, %.3f deg), imu=%s, dt=%.3f s",
            self.filtered_anchor_x,
            self.filtered_anchor_y,
            math.degrees(self.filtered_anchor_yaw),
            self.wheel_anchor_x,
            self.wheel_anchor_y,
            math.degrees(self.wheel_anchor_yaw),
            "%.3f deg" % math.degrees(self.imu_anchor_yaw)
            if self.imu_anchor_yaw is not None
            else "not used",
            anchor_dt,
        )
        rospy.loginfo(
            "Publishing anchored encoder odometry from %s to %s",
            rospy.resolve_name(self.wheel_odom_topic),
            rospy.resolve_name(self.output_topic),
        )
        self.wheel_history.clear()
        self.anchor_ready = True

        # Publish the initial anchored state immediately so downstream consumers
        # do not have to wait for the next raw wheel-odom message.
        self._publish_anchored_odometry(self.wheel_anchor_msg)

    def _wait_for_anchor_message(self, topic_name, description):
        resolved_topic = rospy.resolve_name(topic_name)
        rospy.loginfo("Waiting for %s on %s", description, resolved_topic)
        return rospy.wait_for_message(topic_name, Odometry)

    def _wait_for_imu_anchor_yaw(self):
        resolved_topic = rospy.resolve_name(self.imu_topic)
        rospy.loginfo("Waiting for initial IMU yaw on %s", resolved_topic)
        try:
            if self.imu_anchor_timeout_sec > 0.0:
                msg = rospy.wait_for_message(
                    self.imu_topic, Imu, timeout=self.imu_anchor_timeout_sec
                )
            else:
                msg = rospy.wait_for_message(self.imu_topic, Imu)
        except rospy.ROSException as exc:
            rospy.logwarn(
                "Could not get initial IMU yaw from %s: %s. Encoder yaw will use filtered yaw.",
                resolved_topic,
                exc,
            )
            return None

        return _yaw_from_quaternion(msg.orientation)

    def _select_wheel_anchor(self, filtered_anchor_msg):
        if self.wheel_history:
            if filtered_anchor_msg.header.stamp != rospy.Time():
                wheel_anchor_msg = min(
                    self.wheel_history,
                    key=lambda msg: abs(
                        (msg.header.stamp - filtered_anchor_msg.header.stamp).to_sec()
                    )
                    if msg.header.stamp != rospy.Time()
                    else float("inf"),
                )
                return copy.deepcopy(wheel_anchor_msg)

            return copy.deepcopy(self.wheel_history[-1])

        resolved_topic = rospy.resolve_name(self.wheel_odom_topic)
        rospy.loginfo("Waiting for first wheel odometry anchor on %s", resolved_topic)

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if self.wheel_history:
                return copy.deepcopy(self.wheel_history[-1])
            rate.sleep()

        raise rospy.ROSInterruptException("ROS shutdown while waiting for wheel odometry anchor")

    def _wheel_odom_callback(self, wheel_msg):
        if not self.anchor_ready:
            self.wheel_history.append(copy.deepcopy(wheel_msg))
            return

        self._publish_anchored_odometry(wheel_msg)

    def _publish_anchored_odometry(self, wheel_msg):
        raw_x = wheel_msg.pose.pose.position.x
        raw_y = wheel_msg.pose.pose.position.y
        raw_z = wheel_msg.pose.pose.position.z
        raw_yaw = _yaw_from_quaternion(wheel_msg.pose.pose.orientation)

        delta_x_world = raw_x - self.wheel_anchor_x
        delta_y_world = raw_y - self.wheel_anchor_y

        # Convert the wheel-odom pose delta into the wheel anchor frame, then
        # reapply that delta in the filtered pose frame captured at startup.
        anchor_cos = math.cos(-self.wheel_anchor_yaw)
        anchor_sin = math.sin(-self.wheel_anchor_yaw)
        delta_x_local = (
            anchor_cos * delta_x_world - anchor_sin * delta_y_world
        )
        delta_y_local = (
            anchor_sin * delta_x_world + anchor_cos * delta_y_world
        )

        filtered_cos = math.cos(self.filtered_anchor_yaw)
        filtered_sin = math.sin(self.filtered_anchor_yaw)
        anchored_x = (
            self.filtered_anchor_x
            + filtered_cos * delta_x_local
            - filtered_sin * delta_y_local
        )
        anchored_y = (
            self.filtered_anchor_y
            + filtered_sin * delta_x_local
            + filtered_cos * delta_y_local
        )
        anchored_z = self.filtered_anchor_z + (raw_z - self.wheel_anchor_z)
        delta_yaw = _normalize_angle(raw_yaw - self.wheel_anchor_yaw)
        anchored_yaw = _normalize_angle(self.filtered_anchor_yaw + delta_yaw)

        anchored_msg = Odometry()
        anchored_msg.header = copy.deepcopy(wheel_msg.header)
        anchored_msg.header.frame_id = self.output_frame_id
        anchored_msg.child_frame_id = self.output_child_frame_id

        anchored_msg.pose.pose.position.x = anchored_x
        anchored_msg.pose.pose.position.y = anchored_y
        anchored_msg.pose.pose.position.z = anchored_z
        anchored_msg.pose.pose.orientation = _quaternion_from_yaw(anchored_yaw)
        anchored_msg.pose.covariance = list(wheel_msg.pose.covariance)

        anchored_msg.twist = copy.deepcopy(wheel_msg.twist)

        self.publisher.publish(anchored_msg)


if __name__ == "__main__":
    rospy.init_node("encoder_odometry_node")
    EncoderOdometryNode()
    rospy.spin()
