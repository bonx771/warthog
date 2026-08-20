#!/usr/bin/env python3

import os
import csv
import math
import rospy
import actionlib
import rospkg
import tf

from geometry_msgs.msg import Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler, euler_from_quaternion


def load_home_pose(path_local):
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('outdoor_waypoint_nav')
    if path_local.startswith('/'):
        path_abs = os.path.normpath(pkg_path + path_local)
    else:
        path_abs = os.path.normpath(os.path.join(pkg_path, path_local))

    if not os.path.isfile(path_abs):
        rospy.logerr('Home trajectory file does not exist: %s', path_abs)
        return None

    with open(path_abs, 'r') as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            try:
                x = float(row['x'])
                y = float(row['y'])
                theta = float(row['theta'])
                rospy.loginfo('Loaded home pose from trajectory file: x=%.3f y=%.3f theta=%.3f', x, y, theta)
                return {'x': x, 'y': y, 'theta': theta}
            except (KeyError, ValueError):
                continue

    rospy.logerr('No valid home pose found in trajectory file: %s', path_abs)
    return None


def build_goal(home_pose, frame_id):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = frame_id
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = home_pose['x']
    goal.target_pose.pose.position.y = home_pose['y']
    goal.target_pose.pose.position.z = 0.0

    quaternion = quaternion_from_euler(0.0, 0.0, home_pose['theta'])
    goal.target_pose.pose.orientation = Quaternion(*quaternion)
    return goal


def wait_for_move_base(action_name, timeout=30.0):
    client = actionlib.SimpleActionClient(action_name, MoveBaseAction)
    rospy.loginfo('Waiting for move_base action server: %s', action_name)
    if not client.wait_for_server(rospy.Duration(timeout)):
        rospy.logerr('Move_base action server %s unavailable after %.1f seconds.', action_name, timeout)
        return None
    return client


def get_robot_pose(goal_frame):
    tf_listener = tf.TransformListener()
    try:
        tf_listener.waitForTransform(goal_frame, 'base_link', rospy.Time(0), rospy.Duration(3.0))
        (trans, rot) = tf_listener.lookupTransform(goal_frame, 'base_link', rospy.Time(0))
        yaw = euler_from_quaternion(rot)[2]
        return trans, yaw
    except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return None, None


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def align_yaw_to_target_heading(target_yaw, goal_frame, tolerance=0.05, max_angular_speed=0.5,
                                 p_gain=1.2, max_duration=15.0, rate_hz=10.0):
    rospy.loginfo('Starting yaw alignment to target heading: %.3f rad', target_yaw)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(rate_hz)
    start_time = rospy.Time.now()

    while not rospy.is_shutdown():
        _, current_yaw = get_robot_pose(goal_frame)
        if current_yaw is None:
            rospy.logwarn('Unable to read current robot yaw for final alignment. Retrying...')
            rospy.sleep(0.1)
            continue

        yaw_error = normalize_angle(target_yaw - current_yaw)
        if abs(yaw_error) <= tolerance:
            rospy.loginfo('Yaw aligned within tolerance: %.3f rad', yaw_error)
            break

        twist = Twist()
        twist.angular.z = max(
            -max_angular_speed,
            min(max_angular_speed, p_gain * yaw_error)
        )
        cmd_vel_pub.publish(twist)

        if (rospy.Time.now() - start_time).to_sec() >= max_duration:
            rospy.logwarn('Yaw alignment timed out after %.1f seconds.', max_duration)
            break

        rate.sleep()

    cmd_vel_pub.publish(Twist())
    rospy.sleep(0.2)


def run():
    rospy.init_node('go_home', anonymous=False)

    trajectory_file = rospy.get_param('/outdoor_waypoint_nav/trajectory_file', '/waypoint_files/trajectory.csv')
    goal_frame = rospy.get_param('/outdoor_waypoint_nav/goal_frame', 'map')
    move_base_action = rospy.get_param('/outdoor_waypoint_nav/move_base_action', '/move_base')
    goal_timeout = rospy.get_param('/outdoor_waypoint_nav/go_home_goal_timeout', 120.0)
    retry_on_failure = rospy.get_param('/outdoor_waypoint_nav/go_home_retry_on_failure', True)
    yaw_alignment_enabled = rospy.get_param('/outdoor_waypoint_nav/go_home_yaw_alignment_enabled', True)
    yaw_alignment_tolerance = rospy.get_param('/outdoor_waypoint_nav/go_home_yaw_alignment_tolerance', 0.05)
    yaw_alignment_max_angular_speed = rospy.get_param('/outdoor_waypoint_nav/go_home_yaw_alignment_max_angular_speed', 0.5)
    yaw_alignment_p = rospy.get_param('/outdoor_waypoint_nav/go_home_yaw_alignment_p', 1.2)
    yaw_alignment_max_duration = rospy.get_param('/outdoor_waypoint_nav/go_home_yaw_alignment_max_duration', 15.0)
    yaw_alignment_rate = rospy.get_param('/outdoor_waypoint_nav/go_home_yaw_alignment_rate', 10.0)

    home_pose = load_home_pose(trajectory_file)
    if home_pose is None:
        rospy.signal_shutdown('home pose missing')
        return

    client = wait_for_move_base(move_base_action)
    if client is None:
        rospy.signal_shutdown('move_base unavailable')
        return

    goal = build_goal(home_pose, goal_frame)

    rospy.loginfo('Sending go-home goal to move_base: x=%.3f y=%.3f theta=%.3f frame=%s',
                  home_pose['x'], home_pose['y'], home_pose['theta'], goal_frame)
    client.send_goal(goal)

    finished_before_timeout = client.wait_for_result(rospy.Duration(goal_timeout))
    if not finished_before_timeout:
        rospy.logerr('go_home goal did not finish within %.1f seconds.', goal_timeout)
        if retry_on_failure:
            rospy.loginfo('Retrying go_home goal once.')
            client.send_goal(goal)
            finished_before_timeout = client.wait_for_result(rospy.Duration(goal_timeout))

    state = client.get_state()
    if finished_before_timeout and state == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo('go_home navigation succeeded.')
        if yaw_alignment_enabled and home_pose['theta'] is not None:
            align_yaw_to_target_heading(
                home_pose['theta'], goal_frame,
                tolerance=yaw_alignment_tolerance,
                max_angular_speed=yaw_alignment_max_angular_speed,
                p_gain=yaw_alignment_p,
                max_duration=yaw_alignment_max_duration,
                rate_hz=yaw_alignment_rate
            )
        rospy.loginfo('go_home complete. Robot is at the original start pose with correct heading.')
    else:
        rospy.logerr('go_home failed. move_base state=%d', state)
        rospy.signal_shutdown('go_home failed')


if __name__ == '__main__':
    run()
