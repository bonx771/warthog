#!/usr/bin/env python3

import copy
from collections import deque

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path


class OdomPathTracker:
    def __init__(self, name, input_topic, output_topic, max_poses):
        self.name = name
        self.max_poses = max_poses
        self.input_topic = input_topic
        self.output_topic = output_topic

        self.poses = deque(maxlen=max_poses)
        self.last_stamp = None
        self.last_frame_id = None

        self.publisher = rospy.Publisher(output_topic, Path, queue_size=10, latch=True)
        self.subscriber = rospy.Subscriber(
            input_topic,
            Odometry,
            self._callback,
            queue_size=100,
            tcp_nodelay=True,
        )

        rospy.loginfo(
            "[%s] Building path from %s to %s (max poses: %d)",
            self.name,
            rospy.resolve_name(self.input_topic),
            rospy.resolve_name(self.output_topic),
            self.max_poses,
        )

    def _callback(self, msg):
        frame_id = msg.header.frame_id or "odom"
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()

        if self.last_frame_id is not None and frame_id != self.last_frame_id:
            rospy.logwarn(
                "[%s] Frame changed from %s to %s. Clearing accumulated path.",
                self.name,
                self.last_frame_id,
                frame_id,
            )
            self.poses.clear()
            self.last_stamp = None

        if self.last_stamp is not None and stamp < self.last_stamp:
            rospy.logwarn(
                "[%s] Timestamp went backwards. Clearing accumulated path.",
                self.name,
            )
            self.poses.clear()
            self.last_stamp = None

        if self.last_stamp is not None and stamp == self.last_stamp:
            return

        pose_msg = PoseStamped()
        pose_msg.header = copy.deepcopy(msg.header)
        pose_msg.header.frame_id = frame_id
        pose_msg.header.stamp = stamp
        pose_msg.pose = copy.deepcopy(msg.pose.pose)

        self.poses.append(pose_msg)
        self.last_stamp = stamp
        self.last_frame_id = frame_id

        path_msg = Path()
        path_msg.header.frame_id = frame_id
        path_msg.header.stamp = stamp
        path_msg.poses = list(self.poses)
        self.publisher.publish(path_msg)


class PathNode:
    def __init__(self):
        max_encoder_poses = int(rospy.get_param("~max_encoder_poses", 20000))
        max_filtered_poses = int(rospy.get_param("~max_filtered_poses", 20000))

        if max_encoder_poses <= 0 or max_filtered_poses <= 0:
            raise rospy.ROSException("max pose limits must be positive integers")

        self.encoder_tracker = OdomPathTracker(
            name="encoder_path",
            input_topic=rospy.get_param("~encoder_odom_topic", "/odometry/encoder"),
            output_topic=rospy.get_param("~encoder_path_topic", "/odometry/encoder_path"),
            max_poses=max_encoder_poses,
        )
        self.filtered_tracker = OdomPathTracker(
            name="filtered_path",
            input_topic=rospy.get_param("~filtered_odom_topic", "odometry/filtered"),
            output_topic=rospy.get_param(
                "~filtered_path_topic", "odometry/filtered_path"
            ),
            max_poses=max_filtered_poses,
        )


if __name__ == "__main__":
    rospy.init_node("path_node")
    PathNode()
    rospy.spin()
