#!/usr/bin/env python3
"""Relay GNSS fixes in alternating available/unavailable time windows.

Before waypoint following starts, fixes are relayed continuously. The first
timed window after the start signal is always GNSS available. During an
unavailable window this node publishes no NavSatFix message at all, which
models loss of reception for navsat_transform more faithfully than publishing
an invalid or frozen fix.
"""

import math
import threading

import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float64, Int32


class GnssPeriodicGate:
    def __init__(self):
        self.input_topic = str(rospy.get_param("~input_topic", "/gps/fix"))
        self.output_topic = str(
            rospy.get_param("~output_topic", "/gps/fix_scenario_5")
        )
        self.start_topic = str(
            rospy.get_param(
                "~start_topic", "/outdoor_waypoint_nav/waypoint_reached_index"
            )
        )
        self.start_waypoint_index = int(
            rospy.get_param("~start_waypoint_index", 0)
        )
        self.stop_waypoint_index = int(
            rospy.get_param("~stop_waypoint_index", -1)
        )
        self.finish_topic = str(
            rospy.get_param(
                "~finish_topic",
                "/outdoor_waypoint_nav/waypoint_following_status",
            )
        ).strip()
        self.available_duration_sec = float(
            rospy.get_param("~available_duration_sec", 5.0)
        )
        self.unavailable_duration_sec = float(
            rospy.get_param("~unavailable_duration_sec", 5.0)
        )
        self.available_topic = str(
            rospy.get_param(
                "~available_topic",
                "/outdoor_waypoint_nav/scenario_5/gnss_available",
            )
        )
        self.outage_active_topic = str(
            rospy.get_param(
                "~outage_active_topic",
                "/outdoor_waypoint_nav/scenario_5/gnss_outage_active",
            )
        )
        self.phase_elapsed_topic = str(
            rospy.get_param(
                "~phase_elapsed_topic",
                "/outdoor_waypoint_nav/scenario_5/phase_elapsed_sec",
            )
        )

        self._validate_parameters()
        self.period_sec = self.available_duration_sec + self.unavailable_duration_sec
        self.lock = threading.RLock()
        self.started = False
        self.cycle_stopped = False
        self.start_time = None
        self.gnss_available = True

        self.fix_publisher = rospy.Publisher(
            self.output_topic, NavSatFix, queue_size=20
        )
        self.available_publisher = rospy.Publisher(
            self.available_topic, Bool, queue_size=1, latch=True
        )
        self.outage_active_publisher = rospy.Publisher(
            self.outage_active_topic, Bool, queue_size=1, latch=True
        )
        self.phase_elapsed_publisher = rospy.Publisher(
            self.phase_elapsed_topic, Float64, queue_size=1, latch=True
        )
        self.fix_subscriber = rospy.Subscriber(
            self.input_topic,
            NavSatFix,
            self._fix_callback,
            queue_size=20,
            tcp_nodelay=True,
        )
        self.start_subscriber = rospy.Subscriber(
            self.start_topic,
            Int32,
            self._start_callback,
            queue_size=10,
            tcp_nodelay=True,
        )
        self.finish_subscriber = None
        if self.finish_topic:
            self.finish_subscriber = rospy.Subscriber(
                self.finish_topic,
                Bool,
                self._finish_callback,
                queue_size=10,
                tcp_nodelay=True,
            )

        self.available_publisher.publish(Bool(data=True))
        self.outage_active_publisher.publish(Bool(data=False))
        self.phase_elapsed_publisher.publish(Float64(data=0.0))
        self.status_timer = rospy.Timer(
            rospy.Duration(0.1), self._status_timer_callback, reset=True
        )

        rospy.loginfo(
            "Scenario 5 periodic GNSS gate: %s -> %s; waiting for %s=%d while "
            "relaying GPS continuously; then ON %.3f s, OFF %.3f s until %s",
            self.input_topic,
            self.output_topic,
            self.start_topic,
            self.start_waypoint_index,
            self.available_duration_sec,
            self.unavailable_duration_sec,
            self.finish_topic or "node shutdown",
        )

    def _validate_parameters(self):
        if not self.input_topic or not self.output_topic:
            raise rospy.ROSException("~input_topic and ~output_topic must not be empty")
        if self.input_topic == self.output_topic:
            raise rospy.ROSException("~input_topic and ~output_topic must be different")
        if not self.start_topic:
            raise rospy.ROSException("~start_topic must not be empty")
        if self.start_waypoint_index < 0:
            raise rospy.ROSException("~start_waypoint_index must be >= 0")
        if self.stop_waypoint_index < -1:
            raise rospy.ROSException("~stop_waypoint_index must be >= -1")
        for name, value in (
            ("available_duration_sec", self.available_duration_sec),
            ("unavailable_duration_sec", self.unavailable_duration_sec),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise rospy.ROSException("~{} must be finite and > 0".format(name))

    def _phase_at(self, now):
        """Return (available, elapsed seconds in current phase)."""
        if self.cycle_stopped:
            return self.gnss_available, 0.0
        if not self.started:
            return True, 0.0

        elapsed_sec = (now - self.start_time).to_sec()
        if elapsed_sec < 0.0:
            # Gazebo can reset /clock when the world is reset. Restart the
            # schedule in its documented initial ON state in that case.
            rospy.logwarn("ROS time moved backwards; restarting GNSS cycle ON")
            self.start_time = now
            elapsed_sec = 0.0

        position_sec = elapsed_sec % self.period_sec
        if position_sec < self.available_duration_sec:
            return True, position_sec
        return False, position_sec - self.available_duration_sec

    def _start_callback(self, msg):
        waypoint_index = int(msg.data)
        if (
            self.stop_waypoint_index >= 0
            and waypoint_index == self.stop_waypoint_index
        ):
            with self.lock:
                if not self.started:
                    return
                self.cycle_stopped = True
                held_available = self.gnss_available
                self.phase_elapsed_publisher.publish(Float64(data=0.0))
            rospy.logwarn(
                "Scenario 5 route complete: holding GNSS gate %s until next run",
                "ON" if held_available else "OFF",
            )
            return

        if waypoint_index != self.start_waypoint_index:
            return

        with self.lock:
            # A new gps_waypoint node publishes index 0 after every r/RB start.
            # Restart the schedule so every run begins with a full ON window.
            self.started = True
            self.start_time = rospy.Time.now()
            self.cycle_stopped = False
            self.gnss_available = True
            self.available_publisher.publish(Bool(data=True))
            self.outage_active_publisher.publish(Bool(data=False))
            self.phase_elapsed_publisher.publish(Float64(data=0.0))

        rospy.logwarn(
            "Scenario 5 waypoint run started: GNSS cycle reset to ON for %.3f s",
            self.available_duration_sec,
        )

    def _finish_callback(self, msg):
        if not msg.data:
            return

        with self.lock:
            if not self.started or self.cycle_stopped:
                return
            available, _phase_elapsed_sec = self._phase_at(rospy.Time.now())
            self.gnss_available = available
            self.cycle_stopped = True
            self.available_publisher.publish(Bool(data=available))
            self.outage_active_publisher.publish(Bool(data=not available))
            self.phase_elapsed_publisher.publish(Float64(data=0.0))

        rospy.logwarn(
            "Scenario 5 route complete: holding GNSS gate %s until next run",
            "ON" if available else "OFF",
        )

    def _update_status(self):
        # Read time only after acquiring the lock. This prevents the timer and
        # GPS subscriber threads from applying phase states out of order at a
        # five-second boundary.
        with self.lock:
            available, phase_elapsed_sec = self._phase_at(rospy.Time.now())
            changed = available != self.gnss_available
            self.gnss_available = available
            if changed:
                rospy.logwarn(
                    "Scenario 5 GNSS %s for the next %.3f s",
                    "ON" if available else "OFF",
                    self.available_duration_sec
                    if available
                    else self.unavailable_duration_sec,
                )
                self.available_publisher.publish(Bool(data=available))
                self.outage_active_publisher.publish(Bool(data=not available))

            self.phase_elapsed_publisher.publish(Float64(data=phase_elapsed_sec))
        return available

    def _status_timer_callback(self, _event):
        self._update_status()

    def _fix_callback(self, msg):
        if self._update_status():
            self.fix_publisher.publish(msg)
        else:
            rospy.loginfo_throttle(
                5.0, "Scenario 5 GNSS OFF: suppressing fixes from %s", self.input_topic
            )


if __name__ == "__main__":
    rospy.init_node("gnss_periodic_gate")
    GnssPeriodicGate()
    rospy.spin()
