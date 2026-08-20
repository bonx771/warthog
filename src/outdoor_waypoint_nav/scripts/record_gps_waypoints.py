#!/usr/bin/env python3

import os
import math
import rospy
import rospkg
import tf
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

try:
    import utm
except ImportError:
    utm = None

try:
    from pyproj import Transformer
except ImportError:
    Transformer = None


_utm_transformers = {}


def latlon_to_utm(latitude, longitude):
    """Convert latitude/longitude to UTM even if python3-utm is unavailable."""
    if utm is not None:
        easting, northing, _, _ = utm.from_latlon(latitude, longitude)
        return easting, northing

    if Transformer is None:
        raise RuntimeError("Install python3-utm or python3-pyproj for UTM conversion")

    zone = max(1, min(60, int((longitude + 180.0) / 6.0) + 1))
    epsg = (32600 if latitude >= 0.0 else 32700) + zone
    transformer = _utm_transformers.get(epsg)
    if transformer is None:
        transformer = Transformer.from_crs(
            "EPSG:4326", "EPSG:{}".format(epsg), always_xy=True
        )
        _utm_transformers[epsg] = transformer
    return transformer.transform(longitude, latitude)


def is_valid_fix(msg):
    return (
        msg.status.status >= 0 and
        math.isfinite(msg.latitude) and
        math.isfinite(msg.longitude)
    )


class RecordTrajectory(object):
    def __init__(self):
        rospy.init_node('record_gps_waypoints', anonymous=False)

        self.gps_topic = rospy.get_param('/outdoor_waypoint_nav/gps_topic', '/gps/fix')
        self.odom_topic = rospy.get_param('/outdoor_waypoint_nav/odom_topic', '/outdoor_waypoint_nav/odometry/filtered_map')
        self.record_file = rospy.get_param('/outdoor_waypoint_nav/record_file', '/waypoint_files/record.txt')
        self.trajectory_file = rospy.get_param('/outdoor_waypoint_nav/trajectory_file', '/waypoint_files/trajectory.csv')
        self.record_distance_threshold = rospy.get_param('/outdoor_waypoint_nav/record_distance_threshold', 1.0)
        self.trajectory_record_interval = rospy.get_param('/outdoor_waypoint_nav/trajectory_record_interval', 0.1)

        package_path = rospkg.RosPack().get_path('outdoor_waypoint_nav')
        
        # Prepare GPS waypoint file
        if self.record_file.startswith('/'):
            self.record_path = os.path.normpath(package_path + self.record_file)
        else:
            self.record_path = os.path.normpath(os.path.join(package_path, self.record_file))

        # Prepare trajectory file
        if self.trajectory_file.startswith('/'):
            self.trajectory_path = os.path.normpath(package_path + self.trajectory_file)
        else:
            self.trajectory_path = os.path.normpath(os.path.join(package_path, self.trajectory_file))

        record_dir = os.path.dirname(self.record_path)
        if record_dir and not os.path.exists(record_dir):
            os.makedirs(record_dir)

        try:
            self.gps_file = open(self.record_path, 'w')
            self.trajectory_file_handle = open(self.trajectory_path, 'w')
            self.trajectory_file_handle.write('timestamp,x,y,theta\n')
        except IOError as exc:
            rospy.logerr('Unable to open record files: %s', exc)
            rospy.signal_shutdown('file_error')
            return

        rospy.on_shutdown(self.close_files)

        self.last_utm = None
        self.waypoint_count = 0
        self.last_trajectory_time = rospy.Time(0)
        self.initial_pose_recorded = False

        rospy.loginfo('Recording GPS waypoints to: %s', self.record_path)
        rospy.loginfo('Recording trajectory to: %s', self.trajectory_path)
        
        self.gps_sub = rospy.Subscriber(self.gps_topic, NavSatFix, self.gps_callback, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=10)

        rospy.spin()

    def close_files(self):
        try:
            if hasattr(self, 'gps_file') and self.gps_file:
                self.gps_file.close()
                rospy.loginfo('Closed GPS waypoint file: %s', self.record_path)
            if hasattr(self, 'trajectory_file_handle') and self.trajectory_file_handle:
                self.trajectory_file_handle.close()
                rospy.loginfo('Closed trajectory file: %s', self.trajectory_path)
        except Exception:
            pass

    def gps_callback(self, msg):
        if not is_valid_fix(msg):
            return

        try:
            easting, northing = latlon_to_utm(msg.latitude, msg.longitude)
        except Exception as exc:
            rospy.logwarn('Unable to convert GPS fix to UTM: %s', exc)
            return

        if self.last_utm is None:
            self.write_gps_waypoint(msg.latitude, msg.longitude)
            self.last_utm = (easting, northing)
            return

        dx = easting - self.last_utm[0]
        dy = northing - self.last_utm[1]
        distance = math.hypot(dx, dy)

        if distance >= self.record_distance_threshold:
            self.write_gps_waypoint(msg.latitude, msg.longitude)
            self.last_utm = (easting, northing)

    def write_gps_waypoint(self, latitude, longitude):
        self.waypoint_count += 1
        self.gps_file.write('{:.8f} {:.8f}\n'.format(latitude, longitude))
        self.gps_file.flush()
        rospy.loginfo('Recorded GPS waypoint %d: %.8f %.8f', self.waypoint_count, latitude, longitude)

    def odom_callback(self, msg):
        current_time = rospy.Time.now()
        
        # Record initial pose at timestamp ~0 on first callback
        if not self.initial_pose_recorded:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            quat = msg.pose.pose.orientation
            _, _, theta = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            self.trajectory_file_handle.write('{:.6f},{:.6f},{:.6f},{:.6f}\n'.format(0.0, x, y, theta))
            self.trajectory_file_handle.flush()
            self.initial_pose_recorded = True
            self.last_trajectory_time = current_time
            rospy.loginfo('Recorded initial pose to trajectory: x=%.3f y=%.3f theta=%.3f', x, y, theta)
            return
        
        if (current_time - self.last_trajectory_time).to_sec() < self.trajectory_record_interval:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        quat = msg.pose.pose.orientation
        _, _, theta = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

        timestamp = msg.header.stamp.to_sec()
        self.trajectory_file_handle.write('{:.6f},{:.6f},{:.6f},{:.6f}\n'.format(timestamp, x, y, theta))
        self.trajectory_file_handle.flush()

        self.last_trajectory_time = current_time


if __name__ == '__main__':
    RecordTrajectory()
