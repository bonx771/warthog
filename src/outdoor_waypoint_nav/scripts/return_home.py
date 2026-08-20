#!/usr/bin/env python3

import os
import math
import rospy
import actionlib
import tf
import utm
import rospkg
import csv

from geometry_msgs.msg import PointStamped, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class ReturnHomeNode(object):
    def __init__(self):
        rospy.init_node('return_home', anonymous=False)

        self.coordinates_file = rospy.get_param(
            '/outdoor_waypoint_nav/coordinates_file', '/waypoint_files/record.txt')
        self.trajectory_file = rospy.get_param(
            '/outdoor_waypoint_nav/trajectory_file', '/waypoint_files/trajectory.csv')
        self.goal_frame = rospy.get_param('/outdoor_waypoint_nav/goal_frame', 'map')
        self.waypoint_marker_frame = rospy.get_param(
            '/outdoor_waypoint_nav/waypoint_marker_frame', 'map')
        self.waypoint_marker_topic = rospy.get_param(
            '/outdoor_waypoint_nav/waypoint_marker_topic', '/outdoor_waypoint_nav/collected_waypoints')
        self.waypoint_marker_scale = rospy.get_param('/outdoor_waypoint_nav/waypoint_marker_scale', 0.7)
        self.waypoint_marker_activation_radius = rospy.get_param(
            '/outdoor_waypoint_nav/waypoint_marker_activation_radius', 0.675)
        self.waypoint_advance_radius = rospy.get_param('/outdoor_waypoint_nav/waypoint_advance_radius', 1.0)
        self.max_waypoint_goal_yaw_delta = rospy.get_param(
            '/outdoor_waypoint_nav/max_waypoint_goal_yaw_delta', 0.35)
        self.auto_replan_current_goal_enabled = rospy.get_param(
            '/outdoor_waypoint_nav/auto_replan_current_goal_enabled', True)
        self.auto_replan_current_goal_interval = rospy.get_param(
            '/outdoor_waypoint_nav/auto_replan_current_goal_interval', 0.5)
        self.safety_replan_topic = rospy.get_param(
            '/outdoor_waypoint_nav/safety_replan_topic', '/outdoor_waypoint_nav/replan_requested')
        self.trajectory_drift_threshold = rospy.get_param(
            '/outdoor_waypoint_nav/trajectory_drift_threshold', 2.0)
        self.odom_topic = rospy.get_param(
            '/outdoor_waypoint_nav/odom_topic', '/outdoor_waypoint_nav/odometry/filtered_map')
        self.return_home_yaw_alignment_enabled = rospy.get_param(
            '/outdoor_waypoint_nav/return_home_yaw_alignment_enabled', True)
        self.return_home_yaw_alignment_tolerance = rospy.get_param(
            '/outdoor_waypoint_nav/return_home_yaw_alignment_tolerance', 0.02)
        self.return_home_yaw_alignment_max_angular_speed = rospy.get_param(
            '/outdoor_waypoint_nav/return_home_yaw_alignment_max_angular_speed', 0.5)
        self.return_home_yaw_alignment_p = rospy.get_param(
            '/outdoor_waypoint_nav/return_home_yaw_alignment_p', 1.2)
        self.return_home_yaw_alignment_max_duration = rospy.get_param(
            '/outdoor_waypoint_nav/return_home_yaw_alignment_max_duration', 15.0)
        self.return_home_yaw_alignment_rate = rospy.get_param(
            '/outdoor_waypoint_nav/return_home_yaw_alignment_rate', 10.0)

        self.replan_requested = False
        self.last_auto_replan_time = rospy.Time.now()
        self.current_odom = None
        self.last_trajectory_check = 0

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.tf_listener = tf.TransformListener()
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.marker_pub = rospy.Publisher(self.waypoint_marker_topic, Marker, queue_size=10, latch=True)
        self.replan_sub = rospy.Subscriber(self.safety_replan_topic, Bool, self.safety_replan_cb)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=10)

        self.waypoints = self.load_waypoints(self.coordinates_file)
        self.reversed_waypoints = list(reversed(self.waypoints))
        self.recorded_trajectory = self.load_trajectory(self.trajectory_file)
        self.initial_yaw = None
        if self.recorded_trajectory:
            self.initial_yaw = self.recorded_trajectory[0].get('theta')
            rospy.loginfo('Original recorded heading for return-home alignment: %.3f rad', self.initial_yaw)

    def load_waypoints(self, path_local):
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('outdoor_waypoint_nav')
        if path_local.startswith('/'):
            path_abs = os.path.normpath(pkg_path + path_local)
        else:
            path_abs = os.path.normpath(os.path.join(pkg_path, path_local))

        if not os.path.isfile(path_abs):
            rospy.logerr('Waypoint file does not exist: %s', path_abs)
            rospy.signal_shutdown('waypoint file missing')
            return []

        waypoints = []
        with open(path_abs, 'r') as fh:
            for line in fh:
                tokens = line.strip().split()
                if len(tokens) < 2:
                    continue
                try:
                    latitude = float(tokens[0])
                    longitude = float(tokens[1])
                    waypoints.append((latitude, longitude))
                except ValueError:
                    continue

        if not waypoints:
            rospy.logerr('No valid waypoints loaded from file: %s', path_abs)
            rospy.signal_shutdown('no waypoints')

        rospy.loginfo('Loaded %d waypoints from %s', len(waypoints), path_abs)
        return waypoints

    def load_trajectory(self, path_local):
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('outdoor_waypoint_nav')
        if path_local.startswith('/'):
            path_abs = os.path.normpath(pkg_path + path_local)
        else:
            path_abs = os.path.normpath(os.path.join(pkg_path, path_local))

        trajectory = []
        if not os.path.isfile(path_abs):
            rospy.logwarn('Trajectory file does not exist: %s', path_abs)
            return trajectory

        try:
            with open(path_abs, 'r') as fh:
                reader = csv.DictReader(fh)
                for row in reader:
                    try:
                        trajectory.append({
                            'timestamp': float(row['timestamp']),
                            'x': float(row['x']),
                            'y': float(row['y']),
                            'theta': float(row['theta'])
                        })
                    except (ValueError, KeyError):
                        continue
        except IOError as exc:
            rospy.logwarn('Unable to read trajectory file: %s', exc)
            return trajectory

        rospy.loginfo('Loaded %d trajectory points from %s', len(trajectory), path_abs)
        return trajectory

    def safety_replan_cb(self, msg):
        if msg.data:
            self.replan_requested = True

    def odom_callback(self, msg):
        self.current_odom = msg

    def wait_for_move_base(self):
        rospy.loginfo('Waiting for move_base action server...')
        if not self.move_base_client.wait_for_server(rospy.Duration(30.0)):
            rospy.logerr('move_base action server not available.')
            rospy.signal_shutdown('move_base unavailable')

    def latlon_to_utm_point(self, latitude, longitude):
        easting, northing, zone_number, zone_letter = utm.from_latlon(latitude, longitude)
        point = PointStamped()
        point.header.frame_id = 'utm'
        point.header.stamp = rospy.Time(0)
        point.point.x = easting
        point.point.y = northing
        point.point.z = 0.0
        return point

    def transform_to_goal_frame(self, point):
        target_frame = self.goal_frame
        try:
            self.tf_listener.waitForTransform(target_frame, point.header.frame_id, rospy.Time(0), rospy.Duration(5.0))
            return self.tf_listener.transformPoint(target_frame, point)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn('Unable to transform point from %s to %s: %s', point.header.frame_id, target_frame, e)
            return None

    def get_robot_pose(self):
        try:
            self.tf_listener.waitForTransform(self.goal_frame, 'base_link', rospy.Time(0), rospy.Duration(3.0))
            (trans, rot) = self.tf_listener.lookupTransform(self.goal_frame, 'base_link', rospy.Time(0))
            yaw = tf.transformations.euler_from_quaternion(rot)[2]
            return trans, yaw
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None, None

    def compute_distance_to_goal(self, goal_point):
        try:
            base_point = PointStamped()
            base_point.header.frame_id = 'base_link'
            base_point.header.stamp = rospy.Time(0)
            base_point.point.x = 0.0
            base_point.point.y = 0.0
            base_point.point.z = 0.0
            base_in_goal_frame = self.tf_listener.transformPoint(goal_point.header.frame_id, base_point)
            dx = base_in_goal_frame.point.x - goal_point.point.x
            dy = base_in_goal_frame.point.y - goal_point.point.y
            return math.hypot(dx, dy)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return float('inf')

    def build_goal(self, goal_point, next_point, last_point):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.goal_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = goal_point.point

        if last_point and self.initial_yaw is not None:
            yaw = self.initial_yaw
        elif not last_point and next_point is not None:
            dx = next_point.point.x - goal_point.point.x
            dy = next_point.point.y - goal_point.point.y
            yaw = math.atan2(dy, dx)
        else:
            _, current_yaw = self.get_robot_pose()
            yaw = current_yaw if current_yaw is not None else 0.0

        quaternion = quaternion_from_euler(0.0, 0.0, yaw)
        goal.target_pose.pose.orientation = Quaternion(*quaternion)
        return goal

    def publish_waypoint_marker(self, point, marker_id, color):
        marker = Marker()
        marker.header.frame_id = point.header.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'return_home_waypoints'
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = point.point
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.waypoint_marker_scale
        marker.scale.y = self.waypoint_marker_scale
        marker.scale.z = self.waypoint_marker_scale
        marker.color.a = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.lifetime = rospy.Duration(0)
        self.marker_pub.publish(marker)

    def publish_all_markers(self, map_points):
        for idx, map_point in enumerate(map_points):
            self.publish_waypoint_marker(map_point, idx, (0.0, 0.5, 1.0))

    def check_trajectory_drift(self):
        if not self.current_odom or not self.recorded_trajectory:
            return False

        current_time = rospy.Time.now().to_sec()
        if current_time - self.last_trajectory_check < 5.0:
            return False

        self.last_trajectory_check = current_time

        x_current = self.current_odom.pose.pose.position.x
        y_current = self.current_odom.pose.pose.position.y

        min_distance = float('inf')
        for point in self.recorded_trajectory:
            dx = x_current - point['x']
            dy = y_current - point['y']
            distance = math.hypot(dx, dy)
            if distance < min_distance:
                min_distance = distance

        if min_distance > self.trajectory_drift_threshold:
            rospy.logwarn(
                'Robot trajectory drift detected! Current position (%.2f, %.2f) is %.2f m away from recorded path. '
                'Threshold: %.2f m', x_current, y_current, min_distance, self.trajectory_drift_threshold)
            return True

        return False

    def handle_trajectory_drift(self, goal):
        rospy.logwarn('Trajectory drift exceeded threshold: canceling current move_base goal and resending the same return goal.')
        self.move_base_client.cancel_goal()
        rospy.sleep(0.2)
        self.move_base_client.send_goal(goal)
        self.last_auto_replan_time = rospy.Time.now()

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle <= -math.pi:
            angle += 2.0 * math.pi
        return angle

    def align_yaw_to_initial_heading(self):
        rospy.loginfo('Starting final yaw alignment to original heading: %.3f rad', self.initial_yaw)
        rate = rospy.Rate(self.return_home_yaw_alignment_rate)
        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            _, current_yaw = self.get_robot_pose()
            if current_yaw is None:
                rospy.logwarn('Unable to read current robot yaw for final alignment. Retrying...')
                rospy.sleep(0.1)
                continue

            yaw_error = self.normalize_angle(self.initial_yaw - current_yaw)
            if abs(yaw_error) <= self.return_home_yaw_alignment_tolerance:
                rospy.loginfo('Final yaw aligned within tolerance: %.3f rad', yaw_error)
                break

            twist = Twist()
            twist.angular.z = max(
                -self.return_home_yaw_alignment_max_angular_speed,
                min(self.return_home_yaw_alignment_max_angular_speed,
                    self.return_home_yaw_alignment_p * yaw_error)
            )
            self.cmd_vel_pub.publish(twist)

            if (rospy.Time.now() - start_time).to_sec() >= self.return_home_yaw_alignment_max_duration:
                rospy.logwarn('Yaw alignment timed out after %.1f seconds.', self.return_home_yaw_alignment_max_duration)
                break

            rate.sleep()

        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(0.2)

    def run(self):
        self.wait_for_move_base()
        if not self.reversed_waypoints:
            rospy.logerr('No waypoints available for return path.')
            return

        map_points = []
        for latitude, longitude in self.reversed_waypoints:
            utm_point = self.latlon_to_utm_point(latitude, longitude)
            map_point = self.transform_to_goal_frame(utm_point)
            if map_point is None:
                rospy.signal_shutdown('tf transform unavailable')
                return
            map_points.append(map_point)

        rospy.loginfo('Publishing %d reversed return waypoints as markers.', len(map_points))
        rospy.loginfo('Loaded %d recorded trajectory points for drift detection.', len(self.recorded_trajectory))
        self.publish_all_markers(map_points)
        rospy.sleep(0.5)

        total_points = len(map_points)
        for idx, map_point in enumerate(map_points):
            if rospy.is_shutdown():
                return

            next_point = map_points[idx + 1] if idx + 1 < total_points else None
            last_point = (idx == total_points - 1)

            if idx == 0:
                distance_start = self.compute_distance_to_goal(map_point)
                if distance_start <= self.waypoint_marker_activation_radius:
                    rospy.loginfo('Already at first return waypoint (%.2f m). Skipping first point.', distance_start)
                    continue

            rospy.loginfo('Returning along waypoint %d/%d (%s).', idx + 1, total_points, self.reversed_waypoints[idx])
            goal = self.build_goal(map_point, next_point, last_point)
            self.move_base_client.send_goal(goal)
            self.last_auto_replan_time = rospy.Time.now()
            attempt = 0

            while not rospy.is_shutdown():
                distance = self.compute_distance_to_goal(map_point)
                if distance <= self.waypoint_advance_radius:
                    rospy.loginfo('Reached return waypoint %d/%d by radius (%.2f m <= %.2f m).', idx + 1, total_points, distance, self.waypoint_advance_radius)
                    break

                if self.check_trajectory_drift():
                    self.handle_trajectory_drift(goal)

                if self.replan_requested:
                    rospy.logwarn('Replan request received from safety node. Resending current return goal.')
                    self.move_base_client.send_goal(goal)
                    self.replan_requested = False
                    self.last_auto_replan_time = rospy.Time.now()

                if self.auto_replan_current_goal_enabled and (rospy.Time.now() - self.last_auto_replan_time).to_sec() >= self.auto_replan_current_goal_interval:
                    rospy.loginfo('Auto-refreshing current return goal.')
                    self.move_base_client.send_goal(goal)
                    self.last_auto_replan_time = rospy.Time.now()

                self.move_base_client.wait_for_result(rospy.Duration(0.1))
                state = self.move_base_client.get_state()
                if state == actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo('move_base succeeded for return waypoint %d/%d.', idx + 1, total_points)
                    break
                if state in [actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.REJECTED]:
                    attempt += 1
                    if attempt <= 1:
                        rospy.logwarn('move_base aborted current return goal; retrying (%d/1).', attempt)
                        self.move_base_client.send_goal(goal)
                        continue
                    rospy.logerr('move_base failed for return waypoint %d/%d. State=%d', idx + 1, total_points, state)
                    rospy.signal_shutdown('move_base return failed')
                    return

                rospy.sleep(0.05)

        if self.return_home_yaw_alignment_enabled and self.initial_yaw is not None:
            self.align_yaw_to_initial_heading()

        rospy.loginfo('Return-home path complete. Robot should now be back near the start pose.')


def main():
    node = ReturnHomeNode()
    node.run()


if __name__ == '__main__':
    main()
