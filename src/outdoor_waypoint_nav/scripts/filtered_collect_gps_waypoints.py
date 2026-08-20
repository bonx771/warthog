#!/usr/bin/env python3

import math
import os
import statistics

import rospy
import rospkg
import tf
import utm
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Joy, NavSatFix
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker


def is_valid_fix(msg):
    return (
        msg.status.status >= 0
        and math.isfinite(msg.latitude)
        and math.isfinite(msg.longitude)
        and -90.0 <= msg.latitude <= 90.0
        and -180.0 <= msg.longitude <= 180.0
    )


def button_pressed(joy_msg, button_num):
    return (
        button_num >= 0
        and button_num < len(joy_msg.buttons)
        and joy_msg.buttons[button_num] == 1
    )


class FilteredWaypointCollector(object):
    def __init__(self):
        rospy.init_node("filtered_collect_gps_waypoints")

        self.collect_button_num = rospy.get_param("/outdoor_waypoint_nav/collect_button_num", 3)
        self.end_button_num = rospy.get_param("/outdoor_waypoint_nav/end_button_num", 6)
        self.collect_button_sym = rospy.get_param("/outdoor_waypoint_nav/collect_button_sym", "Y")
        self.end_button_sym = rospy.get_param("/outdoor_waypoint_nav/end_button_sym", "BACK")
        self.gps_topic = rospy.get_param(
            "/outdoor_waypoint_nav/gps_topic", "/outdoor_waypoint_nav/gps/filtered"
        )
        self.coordinates_file = rospy.get_param(
            "/outdoor_waypoint_nav/coordinates_file", "/waypoint_files/points_outdoor.txt"
        )
        self.marker_frame = rospy.get_param("/outdoor_waypoint_nav/waypoint_marker_frame", "map")
        self.marker_topic = rospy.get_param(
            "/outdoor_waypoint_nav/waypoint_marker_topic",
            "/outdoor_waypoint_nav/collected_waypoints",
        )
        self.marker_scale = rospy.get_param("/outdoor_waypoint_nav/waypoint_marker_scale", 0.7)
        self.sample_duration = rospy.get_param(
            "/outdoor_waypoint_nav/waypoint_sample_duration", 5.0
        )
        self.min_samples = rospy.get_param("/outdoor_waypoint_nav/waypoint_min_samples", 5)
        self.min_waypoint_distance_m = rospy.get_param(
            "/outdoor_waypoint_nav/waypoint_min_distance_m", 1.0
        )
        self.keyboard_waypoint_control_enabled = rospy.get_param(
            "/outdoor_waypoint_nav/keyboard_waypoint_control_enabled", False
        )
        self.keyboard_joy_topic = rospy.get_param(
            "/outdoor_waypoint_nav/keyboard_joy_topic", "/outdoor_waypoint_nav/keyboard_joy"
        )

        self.collect_prev_main = False
        self.end_prev_main = False
        self.collect_prev_keyboard = False
        self.end_prev_keyboard = False
        self.first_valid_fix_logged = False
        self.have_latest_fix = False
        self.latest_fix = None
        self.sampling_active = False
        self.sample_start = rospy.Time(0)
        self.sample_end = rospy.Time(0)
        self.samples = []
        self.last_utm = None
        self.waypoint_count = 0
        self.closed = False

        package_path = rospkg.RosPack().get_path("outdoor_waypoint_nav")
        self.coordinates_path = self._package_relative_path(package_path, self.coordinates_file)
        coordinates_dir = os.path.dirname(self.coordinates_path)
        if coordinates_dir and not os.path.exists(coordinates_dir):
            os.makedirs(coordinates_dir)

        try:
            self.coord_file = open(self.coordinates_path, "w")
        except IOError as exc:
            rospy.logerr("Unable to open waypoint file %s: %s", self.coordinates_path, exc)
            rospy.signal_shutdown("waypoint_file_error")
            return

        self.tf_listener = tf.TransformListener()
        self.marker_pub = rospy.Publisher(self.marker_topic, Marker, queue_size=100, latch=True)
        self.status_pub = rospy.Publisher(
            "/outdoor_waypoint_nav/collection_status", Bool, queue_size=10
        )

        rospy.Subscriber("/joy_teleop/joy", Joy, self.joy_callback, queue_size=100)
        if self.keyboard_waypoint_control_enabled:
            rospy.Subscriber(
                self.keyboard_joy_topic, Joy, self.keyboard_joy_callback, queue_size=100
            )
            rospy.loginfo("Listening for keyboard waypoint commands on: %s", self.keyboard_joy_topic)

        rospy.Subscriber(self.gps_topic, NavSatFix, self.gps_callback, queue_size=100)
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        rospy.on_shutdown(self.close_file)

        rospy.sleep(0.1)
        self.clear_waypoint_markers()
        self.print_startup_logs()

    def _package_relative_path(self, package_path, path_value):
        if path_value.startswith("/"):
            return os.path.normpath(package_path + path_value)
        return os.path.normpath(os.path.join(package_path, path_value))

    def print_startup_logs(self):
        rospy.loginfo("Initiated filtered_collect_gps_waypoints node")
        rospy.loginfo("Collecting waypoints from GPS topic: %s", self.gps_topic)
        rospy.loginfo("Waypoint sampling window: %.1f s", self.sample_duration)
        rospy.loginfo("Minimum samples required per waypoint: %d", self.min_samples)
        rospy.loginfo("Saving filtered coordinates to: %s", self.coordinates_path)
        rospy.loginfo("Publishing collected waypoint markers to: %s", self.marker_topic)
        rospy.logwarn("Waiting for a valid GPS fix before accepting waypoint saves.")
        print("")
        print("Press {} button to collect and store waypoint.".format(self.collect_button_sym))
        print("Press {} button to end waypoint collection.".format(self.end_button_sym))
        print("")

    def gps_callback(self, msg):
        if not is_valid_fix(msg):
            rospy.logwarn_throttle(
                2.0,
                "GPS fix is not valid yet on %s. Waiting for usable latitude/longitude.",
                self.gps_topic,
            )
            self.have_latest_fix = False
            return

        self.latest_fix = msg
        self.have_latest_fix = True
        if self.sampling_active:
            self.samples.append(msg)

        if not self.first_valid_fix_logged:
            rospy.loginfo(
                "Received valid GPS point on %s: lat=%.8f lon=%.8f",
                self.gps_topic,
                msg.latitude,
                msg.longitude,
            )
            self.first_valid_fix_logged = True

    def joy_callback(self, joy_msg):
        self.process_joy_message(joy_msg, "main")

    def keyboard_joy_callback(self, joy_msg):
        self.process_joy_message(joy_msg, "keyboard")

    def process_joy_message(self, joy_msg, source):
        collect_pressed = button_pressed(joy_msg, self.collect_button_num)
        end_pressed = button_pressed(joy_msg, self.end_button_num)

        if source == "main":
            collect_prev = self.collect_prev_main
            end_prev = self.end_prev_main
            self.collect_prev_main = collect_pressed
            self.end_prev_main = end_pressed
        else:
            collect_prev = self.collect_prev_keyboard
            end_prev = self.end_prev_keyboard
            self.collect_prev_keyboard = collect_pressed
            self.end_prev_keyboard = end_pressed

        if collect_pressed and not collect_prev:
            self.start_sampling()

        if end_pressed and not end_prev:
            self.finish_collection()

    def start_sampling(self):
        if self.sampling_active:
            remaining = max(0.0, (self.sample_end - rospy.Time.now()).to_sec())
            rospy.logwarn(
                "Waypoint sampling is already active. Keep robot stopped for %.1f s more.",
                remaining,
            )
            return

        if not self.have_latest_fix:
            rospy.logerr(
                "Cannot sample waypoint: no valid GPS fix has been received yet on %s.",
                self.gps_topic,
            )
            return

        self.samples = []
        self.sample_start = rospy.Time.now()
        self.sample_end = self.sample_start + rospy.Duration(self.sample_duration)
        self.sampling_active = True

        rospy.logwarn(
            "Waypoint collect button pressed. Keep robot STOPPED and wait %.1f s before moving.",
            self.sample_duration,
        )
        rospy.loginfo(
            "Sampling waypoint GPS now. Do not move until the saved-waypoint log appears."
        )

    def timer_callback(self, _event):
        if not self.sampling_active:
            return

        now = rospy.Time.now()
        remaining = (self.sample_end - now).to_sec()
        if remaining > 0.0:
            rospy.logwarn_throttle(
                1.0,
                "Sampling waypoint... keep robot stopped. %.1f s remaining.",
                remaining,
            )
            return

        self.save_filtered_waypoint()

    def save_filtered_waypoint(self):
        self.sampling_active = False

        if len(self.samples) < self.min_samples:
            rospy.logerr(
                "Waypoint not saved: only %d GPS sample(s) in %.1f s, need at least %d.",
                len(self.samples),
                self.sample_duration,
                self.min_samples,
            )
            rospy.logwarn("You may move now, but collect this waypoint again after GPS is stable.")
            return

        latitudes = [sample.latitude for sample in self.samples]
        longitudes = [sample.longitude for sample in self.samples]

        filtered_lat = statistics.median(latitudes)
        filtered_lon = statistics.median(longitudes)

        try:
            center_easting, center_northing, _, _ = utm.from_latlon(filtered_lat, filtered_lon)
            distances = []
            for sample in self.samples:
                easting, northing, _, _ = utm.from_latlon(sample.latitude, sample.longitude)
                distances.append(math.hypot(easting - center_easting, northing - center_northing))
        except Exception as exc:
            rospy.logerr("Waypoint not saved: unable to convert filtered GPS to UTM: %s", exc)
            rospy.logwarn("You may move now, but collect this waypoint again after GPS is stable.")
            return

        rms_spread = math.sqrt(sum(distance * distance for distance in distances) / len(distances))
        max_spread = max(distances)

        if self.last_utm is not None:
            distance_from_last = math.hypot(
                center_easting - self.last_utm[0], center_northing - self.last_utm[1]
            )
            if distance_from_last < self.min_waypoint_distance_m:
                rospy.logwarn(
                    "Filtered waypoint not saved: %.2f m from previous waypoint, need %.2f m.",
                    distance_from_last,
                    self.min_waypoint_distance_m,
                )
                rospy.logwarn("You may move now, but move farther before collecting the next waypoint.")
                return

        self.waypoint_count += 1
        self.coord_file.write("{:.8f} {:.8f}\n".format(filtered_lat, filtered_lon))
        self.coord_file.flush()
        self.last_utm = (center_easting, center_northing)

        self.publish_waypoint_marker(center_easting, center_northing, self.waypoint_count)

        rospy.loginfo(
            "Collected filtered waypoint %d: lat=%.8f lon=%.8f samples=%d rms=%.2f m max=%.2f m",
            self.waypoint_count,
            filtered_lat,
            filtered_lon,
            len(self.samples),
            rms_spread,
            max_spread,
        )
        rospy.logwarn("Waypoint saved. You may move now.")
        rospy.loginfo(
            "Press %s button to collect next waypoint %d.",
            self.collect_button_sym,
            self.waypoint_count + 1,
        )
        rospy.loginfo("Press %s button to end waypoint collection.", self.end_button_sym)

    def clear_waypoint_markers(self):
        marker = Marker()
        marker.header.frame_id = self.marker_frame
        marker.header.stamp = rospy.Time.now()
        marker.action = Marker.DELETEALL
        self.marker_pub.publish(marker)

    def publish_waypoint_marker(self, easting, northing, waypoint_index):
        utm_point = PointStamped()
        utm_point.header.frame_id = "utm"
        utm_point.header.stamp = rospy.Time(0)
        utm_point.point.x = easting
        utm_point.point.y = northing
        utm_point.point.z = 0.0

        try:
            self.tf_listener.waitForTransform(
                self.marker_frame, "utm", rospy.Time(0), rospy.Duration(1.0)
            )
            marker_point = self.tf_listener.transformPoint(self.marker_frame, utm_point)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
            rospy.logwarn(
                "Saved waypoint, but could not transform marker into %s: %s",
                self.marker_frame,
                exc,
            )
            return

        marker = Marker()
        marker.header.frame_id = self.marker_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "collected_waypoints"
        marker.id = waypoint_index
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = marker_point.point.x
        marker.pose.position.y = marker_point.point.y
        marker.pose.position.z = marker_point.point.z + 0.2
        marker.scale.x = self.marker_scale
        marker.scale.y = self.marker_scale
        marker.scale.z = self.marker_scale
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.lifetime = rospy.Duration(0.0)
        self.marker_pub.publish(marker)

    def finish_collection(self):
        if self.sampling_active:
            self.sampling_active = False
            rospy.logwarn("End request received. Active 5s waypoint sampling was canceled.")

        rospy.loginfo("End request registered.")
        rospy.loginfo("Closed waypoint file, you have collected %d waypoint(s).", self.waypoint_count)
        rospy.loginfo("Waypoint collection complete.")
        rospy.loginfo("Ending node...")

        self.close_file()
        status_msg = Bool()
        status_msg.data = True
        for _ in range(3):
            self.status_pub.publish(status_msg)
            rospy.sleep(0.05)
        rospy.signal_shutdown("collection_complete")

    def close_file(self):
        if self.closed:
            return
        self.closed = True
        try:
            self.coord_file.close()
        except Exception:
            pass


if __name__ == "__main__":
    try:
        FilteredWaypointCollector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
