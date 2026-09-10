#!/usr/bin/env python3
"""Obstacle-only entry point for the shared joystick/keyboard controller."""

import os

import joy_launch_control as controller
import rospkg
import rospy
from std_msgs.msg import Bool


navigation_ready = False
navigation_ready_subscriber = None


def navigation_ready_callback(msg):
    global navigation_ready
    navigation_ready = bool(msg.data)


def install_obstacle_navigation_gate():
    """Add a readiness interlock without changing the shared scenario control."""
    original_launch_subscribers = controller.launch_subscribers
    original_start_launch_process = controller.start_launch_process

    def launch_subscribers_with_guard():
        global navigation_ready_subscriber
        original_launch_subscribers()
        navigation_ready_subscriber = rospy.Subscriber(
            "/outdoor_waypoint_nav/navigation_ready_obstacle",
            Bool,
            navigation_ready_callback,
            queue_size=1,
        )

    def start_launch_process_when_ready(launch_file, label, launch_args=None):
        if label in ("send_goals.launch", "home_to_initial_waypoint.launch"):
            if not navigation_ready:
                rospy.logerr(
                    "Obstacle navigation is not ready: keep the UGV stopped and "
                    "wait for the NAVIGATION READY log before sending waypoints."
                )
                controller.publish_zero_velocity()
                return
        return original_start_launch_process(launch_file, label, launch_args)

    controller.launch_subscribers = launch_subscribers_with_guard
    controller.start_launch_process = start_launch_process_when_ready


def main():
    controller.getParameter()
    controller.getPaths()
    install_obstacle_navigation_gate()

    package_path = rospkg.RosPack().get_path("outdoor_waypoint_nav")
    controller.location_collect = os.path.join(
        package_path, "launch/include/collect_goals_obstacle.launch"
    )
    controller.location_send = os.path.join(
        package_path, "launch/include/send_goals_obstacle.launch"
    )

    controller.print_instructions()
    controller.main()


if __name__ == "__main__":
    main()
