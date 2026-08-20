#!/usr/bin/env python3
"""Temporarily suppress GNSS fixes for the Scenario 3 dead-reckoning test.

The node normally relays ``input_topic`` to ``output_topic``. After the waypoint
controller confirms that the configured start waypoint has been reached, it
stops publishing NavSatFix messages. It measures the outage length from the
GPS-independent first EKF odometry stream, then resumes relaying raw fixes.

Suppressing messages, rather than publishing a fake valid fix, makes
navsat_transform and the map EKF behave as they would during an actual GNSS
receiver outage.
"""

import math

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float64, Int32


class GnssOutageGate:
    WAITING = "waiting"
    OUTAGE = "outage"
    RECOVERED = "recovered"

    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/gps/fix")
        self.output_topic = rospy.get_param("~output_topic", "/gps/fix_off")
        self.distance_odom_topic = rospy.get_param(
            "~distance_odom_topic",
            "/outdoor_waypoint_nav/odometry/filtered_odom",
        )
        self.reached_waypoint_topic = rospy.get_param(
            "~reached_waypoint_topic",
            "/outdoor_waypoint_nav/waypoint_reached_index",
        )
        self.active_topic = rospy.get_param(
            "~active_topic", "/outdoor_waypoint_nav/gnss_outage/active"
        )
        self.distance_topic = rospy.get_param(
            "~distance_topic", "/outdoor_waypoint_nav/gnss_outage/distance_m"
        )
        self.start_after_waypoint_index = int(
            rospy.get_param("~start_after_waypoint_index", 2)
        )
        self.outage_distance_m = float(rospy.get_param("~outage_distance_m", 20.0))
        self.max_odom_step_m = float(rospy.get_param("~max_odom_step_m", 5.0))

        if self.start_after_waypoint_index < 0:
            raise rospy.ROSException("~start_after_waypoint_index must be >= 0")
        if not math.isfinite(self.outage_distance_m) or self.outage_distance_m <= 0.0:
            raise rospy.ROSException("~outage_distance_m must be finite and > 0")
        if not math.isfinite(self.max_odom_step_m) or self.max_odom_step_m <= 0.0:
            raise rospy.ROSException("~max_odom_step_m must be finite and > 0")

        self.state = self.WAITING
        self.start_requested = False
        self.latest_odom_position = None
        self.last_outage_odom_position = None
        self.outage_distance_travelled_m = 0.0
        self.reached_waypoint_index = None

        self.publisher = rospy.Publisher(self.output_topic, NavSatFix, queue_size=20)
        self.active_publisher = rospy.Publisher(
            self.active_topic, Bool, queue_size=1, latch=True
        )
        self.distance_publisher = rospy.Publisher(
            self.distance_topic, Float64, queue_size=10, latch=True
        )
        self.fix_subscriber = rospy.Subscriber(
            self.input_topic,
            NavSatFix,
            self._fix_callback,
            queue_size=20,
            tcp_nodelay=True,
        )
        self.odom_subscriber = rospy.Subscriber(
            self.distance_odom_topic,
            Odometry,
            self._odom_callback,
            queue_size=50,
            tcp_nodelay=True,
        )
        self.waypoint_subscriber = rospy.Subscriber(
            self.reached_waypoint_topic,
            Int32,
            self._reached_waypoint_callback,
            queue_size=10,
            tcp_nodelay=True,
        )

        self.active_publisher.publish(Bool(data=False))
        self.distance_publisher.publish(Float64(data=0.0))
        rospy.loginfo(
            "GNSS outage gate: %s -> %s; suppress after reached WP%d for %.1f m "
            "measured from %s",
            self.input_topic,
            self.output_topic,
            self.start_after_waypoint_index,
            self.outage_distance_m,
            self.distance_odom_topic,
        )

    @staticmethod
    def _position_from_odom(msg):
        point = msg.pose.pose.position
        position = (float(point.x), float(point.y))
        if not all(math.isfinite(value) for value in position):
            return None
        return position

    def _start_outage(self):
        if self.state != self.WAITING or self.latest_odom_position is None:
            return

        self.state = self.OUTAGE
        self.last_outage_odom_position = self.latest_odom_position
        self.outage_distance_travelled_m = 0.0
        self.active_publisher.publish(Bool(data=True))
        self.distance_publisher.publish(Float64(data=0.0))
        rospy.logwarn(
            "GNSS OFF started after reached WP%d. Suppressing fixes for %.1f m.",
            self.reached_waypoint_index,
            self.outage_distance_m,
        )

    def _finish_outage(self):
        if self.state != self.OUTAGE:
            return

        self.state = self.RECOVERED
        self.active_publisher.publish(Bool(data=False))
        self.distance_publisher.publish(Float64(data=self.outage_distance_travelled_m))
        rospy.logwarn(
            "GNSS ON restored after %.2f m of dead reckoning.",
            self.outage_distance_travelled_m,
        )

    def _reached_waypoint_callback(self, msg):
        waypoint_index = int(msg.data)
        if waypoint_index < 0:
            rospy.logwarn_throttle(5.0, "Ignoring invalid waypoint index %d", waypoint_index)
            return

        self.reached_waypoint_index = waypoint_index
        if (
            self.state == self.WAITING
            and waypoint_index >= self.start_after_waypoint_index
        ):
            self.start_requested = True
            if self.latest_odom_position is not None:
                self._start_outage()
            else:
                rospy.logwarn(
                    "Reached WP%d; waiting for %s before starting GNSS outage.",
                    waypoint_index,
                    self.distance_odom_topic,
                )

    def _odom_callback(self, msg):
        current = self._position_from_odom(msg)
        if current is None:
            return

        self.latest_odom_position = current
        if self.state == self.WAITING and self.start_requested:
            self._start_outage()
            return

        if self.state != self.OUTAGE:
            return

        if self.last_outage_odom_position is None:
            self.last_outage_odom_position = current
            return

        step = math.hypot(
            current[0] - self.last_outage_odom_position[0],
            current[1] - self.last_outage_odom_position[1],
        )
        self.last_outage_odom_position = current
        if step > self.max_odom_step_m:
            rospy.logwarn_throttle(
                5.0,
                "Ignoring %.2f m odometry jump while measuring GNSS outage distance",
                step,
            )
            return

        self.outage_distance_travelled_m += step
        self.distance_publisher.publish(Float64(data=self.outage_distance_travelled_m))
        if self.outage_distance_travelled_m >= self.outage_distance_m:
            self._finish_outage()

    def _fix_callback(self, msg):
        if self.state == self.OUTAGE:
            rospy.loginfo_throttle(
                5.0,
                "GNSS OFF: suppressing %s at %.2f / %.2f m",
                self.input_topic,
                self.outage_distance_travelled_m,
                self.outage_distance_m,
            )
            return
        self.publisher.publish(msg)


if __name__ == "__main__":
    rospy.init_node("gnss_outage_gate")
    GnssOutageGate()
    rospy.spin()
