#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Drive directly back to the first waypoint in the active waypoint file."""

import math
import os

import actionlib
import rospy
import rospkg
import tf
from geometry_msgs.msg import PointStamped, Twist
from move_base_msgs.msg import MoveBaseAction
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker

try:
    from pyproj import Transformer
except ImportError:
    Transformer = None

try:
    import utm
except ImportError:
    utm = None


def _normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def _clamp(value, low, high):
    return max(low, min(high, value))


class InitialWaypointHome(object):
    def __init__(self):
        self.package_dir = rospkg.RosPack().get_path("outdoor_waypoint_nav")
        self.coordinates_file = rospy.get_param(
            "/outdoor_waypoint_nav/coordinates_file",
            "/waypoint_files/points_outdoor.txt",
        )
        self.waypoint_file_type = rospy.get_param(
            "/outdoor_waypoint_nav/home_waypoint_file_type", "auto"
        ).strip().lower()
        self.goal_frame = rospy.get_param("/outdoor_waypoint_nav/goal_frame", "map")
        self.utm_frame = rospy.get_param("/outdoor_waypoint_nav/utm_frame", "utm")
        self.base_frame = rospy.get_param("/outdoor_waypoint_nav/base_frame", "base_link")
        self.cmd_vel_topic = rospy.get_param(
            "/outdoor_waypoint_nav/home_cmd_topic",
            rospy.get_param("/outdoor_waypoint_nav/waypoint_cmd_topic", "/cmd_vel"),
        )
        self.marker_topic = rospy.get_param(
            "/outdoor_waypoint_nav/home_marker_topic",
            "/outdoor_waypoint_nav/home_waypoint_marker",
        )
        self.status_topic = rospy.get_param(
            "/outdoor_waypoint_nav/home_status_topic",
            "/outdoor_waypoint_nav/home_navigation_status",
        )
        self.move_base_action = rospy.get_param(
            "/outdoor_waypoint_nav/move_base_action", "/move_base"
        )
        self.cancel_move_base_goals = rospy.get_param(
            "/outdoor_waypoint_nav/home_cancel_move_base_goals", True
        )

        self.goal_tolerance = max(
            0.05,
            float(rospy.get_param("/outdoor_waypoint_nav/home_goal_tolerance", 0.25)),
        )
        self.cruise_speed = max(
            0.0, float(rospy.get_param("/outdoor_waypoint_nav/home_cruise_speed", 0.45))
        )
        self.min_speed = min(
            self.cruise_speed,
            max(0.0, float(rospy.get_param("/outdoor_waypoint_nav/home_min_speed", 0.12))),
        )
        self.slow_radius = max(
            self.goal_tolerance + 0.05,
            float(rospy.get_param("/outdoor_waypoint_nav/home_slow_radius", 2.0)),
        )
        self.angular_kp = float(rospy.get_param("/outdoor_waypoint_nav/home_angular_kp", 1.25))
        self.max_angular_speed = max(
            0.01,
            abs(float(rospy.get_param("/outdoor_waypoint_nav/home_max_angular_speed", 0.60))),
        )
        self.min_turn_speed = min(
            self.max_angular_speed,
            max(0.0, abs(float(rospy.get_param("/outdoor_waypoint_nav/home_min_turn_speed", 0.16)))),
        )
        self.in_place_angle_threshold = max(
            0.2,
            min(
                math.pi,
                abs(float(rospy.get_param("/outdoor_waypoint_nav/home_in_place_angle_threshold", 1.55))),
            ),
        )
        self.progress_timeout = max(
            0.0, float(rospy.get_param("/outdoor_waypoint_nav/home_progress_timeout", 30.0))
        )
        self.progress_epsilon = max(
            0.0, float(rospy.get_param("/outdoor_waypoint_nav/home_progress_epsilon", 0.05))
        )
        self.control_frequency = max(
            1.0, float(rospy.get_param("/outdoor_waypoint_nav/home_control_frequency", 20.0))
        )
        self.max_duration = max(
            0.0, float(rospy.get_param("/outdoor_waypoint_nav/home_max_duration", 0.0))
        )

        self.tf_listener = tf.TransformListener()
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.status_pub = rospy.Publisher(self.status_topic, Bool, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher(self.marker_topic, Marker, queue_size=1, latch=True)
        self.status_pub.publish(Bool(data=False))
        rospy.on_shutdown(self._publish_stop)

    def _resolve_package_path(self, value):
        value = str(value).strip()
        if os.path.isabs(value) and os.path.exists(value):
            return value
        package_relative = value[1:] if value.startswith("/") else value
        return os.path.normpath(os.path.join(self.package_dir, package_relative))

    def _load_first_waypoint(self):
        path = self._resolve_package_path(self.coordinates_file)
        if not os.path.isfile(path):
            raise RuntimeError("Waypoint file not found: {}".format(path))

        values = []
        with open(path, "r", encoding="utf-8") as handle:
            for line in handle:
                stripped = line.strip()
                if not stripped or stripped.startswith("#"):
                    continue
                values.extend(float(token) for token in stripped.replace(",", " ").split())
                if len(values) >= 2:
                    break

        if len(values) < 2:
            raise RuntimeError("Waypoint file has no coordinate pair: {}".format(path))
        rospy.loginfo(
            "[home] Loaded initial waypoint from %s: %.10f %.10f",
            path,
            values[0],
            values[1],
        )
        return values[0], values[1]

    def _coordinates_are_gps(self, first, second):
        if self.waypoint_file_type == "gps":
            return True
        if self.waypoint_file_type == "map":
            return False
        return abs(first) <= 90.0 and abs(second) <= 180.0

    @staticmethod
    def _latlon_to_utm(latitude, longitude):
        if Transformer is not None:
            zone = max(1, min(60, int((longitude + 180.0) / 6.0) + 1))
            epsg = (32600 if latitude >= 0.0 else 32700) + zone
            transformer = Transformer.from_crs(
                "EPSG:4326", "EPSG:{}".format(epsg), always_xy=True
            )
            return transformer.transform(longitude, latitude)
        if utm is not None:
            easting, northing, _, _ = utm.from_latlon(latitude, longitude)
            return easting, northing
        raise RuntimeError("Install python3-pyproj or python3-utm for GPS waypoint conversion")

    def _target_point(self):
        first, second = self._load_first_waypoint()
        point = PointStamped()
        point.header.stamp = rospy.Time(0)
        point.point.z = 0.0

        if self._coordinates_are_gps(first, second):
            easting, northing = self._latlon_to_utm(first, second)
            point.header.frame_id = self.utm_frame
            point.point.x = easting
            point.point.y = northing
            self.tf_listener.waitForTransform(
                self.goal_frame,
                self.utm_frame,
                rospy.Time(0),
                rospy.Duration(10.0),
            )
            target = self.tf_listener.transformPoint(self.goal_frame, point)
        else:
            target = point
            target.header.frame_id = self.goal_frame
            target.point.x = first
            target.point.y = second

        rospy.loginfo(
            "[home] Initial waypoint in %s: x=%.3f y=%.3f",
            self.goal_frame,
            target.point.x,
            target.point.y,
        )
        return target

    def _robot_pose(self):
        transform, rotation = self.tf_listener.lookupTransform(
            self.goal_frame, self.base_frame, rospy.Time(0)
        )
        yaw = tf.transformations.euler_from_quaternion(rotation)[2]
        return transform[0], transform[1], yaw

    def _publish_stop(self, repeat=8):
        stop = Twist()
        for _ in range(repeat):
            self.cmd_pub.publish(stop)
            rospy.sleep(0.02)

    def _publish_marker(self, target):
        marker = Marker()
        marker.header.frame_id = self.goal_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "initial_waypoint_home"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = target.point.x
        marker.pose.position.y = target.point.y
        marker.pose.position.z = target.point.z + 0.25
        marker.scale.x = 0.8
        marker.scale.y = 0.8
        marker.scale.z = 0.8
        marker.color.r = 0.15
        marker.color.g = 0.85
        marker.color.b = 0.25
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(0.0)
        self.marker_pub.publish(marker)

    def _cancel_move_base_goals(self):
        if not self.cancel_move_base_goals:
            return
        client = actionlib.SimpleActionClient(self.move_base_action, MoveBaseAction)
        if client.wait_for_server(rospy.Duration(2.0)):
            client.cancel_all_goals()
            rospy.loginfo("[home] Canceled active move_base goals before direct home drive.")
        else:
            rospy.logwarn("[home] move_base action server not available for cancel; continuing.")

    def drive_home(self):
        target = self._target_point()
        self._publish_marker(target)
        self._cancel_move_base_goals()
        self._publish_stop()

        best_distance = float("inf")
        last_progress_time = rospy.Time.now()
        start_time = rospy.Time.now()
        rate = rospy.Rate(self.control_frequency)

        rospy.loginfo(
            "[home] Driving to initial waypoint with tolerance %.2f m on %s.",
            self.goal_tolerance,
            self.cmd_vel_topic,
        )

        while not rospy.is_shutdown():
            try:
                robot_x, robot_y, robot_yaw = self._robot_pose()
            except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
                rospy.logwarn_throttle(2.0, "[home] Waiting for robot pose in %s: %s", self.goal_frame, exc)
                self._publish_stop(repeat=1)
                rate.sleep()
                continue

            dx = target.point.x - robot_x
            dy = target.point.y - robot_y
            distance = math.hypot(dx, dy)
            if distance <= self.goal_tolerance:
                self._publish_stop()
                self.status_pub.publish(Bool(data=True))
                rospy.loginfo(
                    "[home] Reached initial waypoint (%.3f m <= %.3f m).",
                    distance,
                    self.goal_tolerance,
                )
                return True

            now = rospy.Time.now()
            if (best_distance - distance) >= self.progress_epsilon:
                best_distance = distance
                last_progress_time = now

            if self.progress_timeout > 0.0 and (now - last_progress_time).to_sec() > self.progress_timeout:
                self._publish_stop()
                rospy.logerr(
                    "[home] No progress toward initial waypoint for %.1f s. Best %.2f m, current %.2f m.",
                    self.progress_timeout,
                    best_distance,
                    distance,
                )
                return False

            if self.max_duration > 0.0 and (now - start_time).to_sec() > self.max_duration:
                self._publish_stop()
                rospy.logerr("[home] Home drive timed out after %.1f s.", self.max_duration)
                return False

            desired_yaw = math.atan2(dy, dx)
            yaw_error = _normalize_angle(desired_yaw - robot_yaw)
            abs_yaw_error = abs(yaw_error)

            cmd = Twist()
            angular_cmd = _clamp(
                self.angular_kp * yaw_error,
                -self.max_angular_speed,
                self.max_angular_speed,
            )
            if abs_yaw_error > self.in_place_angle_threshold:
                cmd.linear.x = 0.0
                if abs(angular_cmd) < self.min_turn_speed:
                    angular_cmd = (1.0 if yaw_error >= 0.0 else -1.0) * self.min_turn_speed
            else:
                speed = self.cruise_speed
                if distance < self.slow_radius:
                    slow_scale = _clamp(
                        (distance - self.goal_tolerance)
                        / max(0.01, self.slow_radius - self.goal_tolerance),
                        0.0,
                        1.0,
                    )
                    speed = self.min_speed + ((self.cruise_speed - self.min_speed) * slow_scale)
                heading_scale = _clamp(
                    1.0 - (0.65 * abs_yaw_error / self.in_place_angle_threshold),
                    0.25,
                    1.0,
                )
                cmd.linear.x = speed * heading_scale

            cmd.angular.z = angular_cmd
            self.cmd_pub.publish(cmd)
            rospy.loginfo_throttle(
                1.0,
                "[home] dist=%.2f m bearing=%.1f deg yaw=%.1f deg err=%.1f deg cmd=(%.2f, %.2f)",
                distance,
                math.degrees(desired_yaw),
                math.degrees(robot_yaw),
                math.degrees(yaw_error),
                cmd.linear.x,
                cmd.angular.z,
            )
            rate.sleep()

        self._publish_stop()
        return False


def main():
    rospy.init_node("home_to_initial_waypoint", anonymous=False)
    node = InitialWaypointHome()
    success = False
    try:
        success = node.drive_home()
    finally:
        node._publish_stop()
    if not success:
        rospy.signal_shutdown("home drive failed")
        raise SystemExit(1)


if __name__ == "__main__":
    main()
