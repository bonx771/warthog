#!/usr/bin/env python3
"""Acceleration limiter used only by outdoor_waypoint_obstacle.launch.

Normal velocity changes, including short zero commands from move_base, are
ramped.  A stale input or a false localization-ready signal still stops
immediately.  This prevents one-cycle planner zeros from repeatedly resetting
the heavy UGV to zero speed.
"""

import threading
import time

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


def step_towards(current, target, accel_limit, decel_limit, dt):
    """Move current toward a non-zero target with bounded acceleration."""
    if dt <= 0.0 or current == target:
        return current

    same_direction = current == 0.0 or current * target > 0.0
    speeding_up = same_direction and abs(target) > abs(current)
    limit = accel_limit if speeding_up else decel_limit
    max_step = max(0.0, limit) * dt
    delta = target - current
    if abs(delta) <= max_step:
        return target
    return current + max_step * (1.0 if delta > 0.0 else -1.0)


class ObstacleCmdVelSmoother:
    def __init__(self):
        self.input_topic = rospy.get_param(
            "~input_topic", "/cmd_vel_raw_obstacle"
        )
        self.output_topic = rospy.get_param("~output_topic", "/cmd_vel")
        self.frequency = float(rospy.get_param("~frequency", 30.0))
        self.linear_accel = float(rospy.get_param("~linear_accel", 0.35))
        self.linear_decel = float(rospy.get_param("~linear_decel", 0.65))
        self.angular_accel = float(rospy.get_param("~angular_accel", 0.55))
        self.angular_decel = float(rospy.get_param("~angular_decel", 0.90))
        self.max_linear = float(rospy.get_param("~max_linear", 0.70))
        self.max_angular = float(rospy.get_param("~max_angular", 0.35))
        self.command_timeout = float(rospy.get_param("~command_timeout", 0.50))
        self.deadband = float(rospy.get_param("~deadband", 0.005))
        self.smooth_zero_commands = bool(
            rospy.get_param("~smooth_zero_commands", True)
        )
        self.navigation_ready_topic = rospy.get_param(
            "~navigation_ready_topic",
            "/outdoor_waypoint_nav/navigation_ready_obstacle",
        )
        self.require_navigation_ready = bool(
            rospy.get_param("~require_navigation_ready", True)
        )

        if self.frequency <= 0.0:
            raise ValueError("frequency must be positive")
        for name, value in (
            ("linear_accel", self.linear_accel),
            ("linear_decel", self.linear_decel),
            ("angular_accel", self.angular_accel),
            ("angular_decel", self.angular_decel),
            ("max_linear", self.max_linear),
            ("max_angular", self.max_angular),
            ("command_timeout", self.command_timeout),
        ):
            if value <= 0.0:
                raise ValueError("{} must be positive".format(name))

        self.lock = threading.Lock()
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.output_linear = 0.0
        self.output_angular = 0.0
        self.last_command_time = None
        self.last_update_time = time.monotonic()
        self.navigation_ready = not self.require_navigation_ready

        self.publisher = rospy.Publisher(self.output_topic, Twist, queue_size=1)
        self.ready_subscriber = rospy.Subscriber(
            self.navigation_ready_topic,
            Bool,
            self.navigation_ready_callback,
            queue_size=1,
        )
        self.subscriber = rospy.Subscriber(
            self.input_topic, Twist, self.command_callback, queue_size=1
        )
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.frequency), self.update
        )
        rospy.on_shutdown(self.stop_now)

        rospy.loginfo(
            "[cmd_vel_smoother_obstacle] %s -> %s at %.1f Hz | "
            "linear accel/decel=%.2f/%.2f m/s^2 | "
            "angular accel/decel=%.2f/%.2f rad/s^2 | planner-zero=%s",
            self.input_topic,
            self.output_topic,
            self.frequency,
            self.linear_accel,
            self.linear_decel,
            self.angular_accel,
            self.angular_decel,
            "ramped" if self.smooth_zero_commands else "immediate",
        )
        if self.require_navigation_ready:
            rospy.logwarn(
                "[cmd_vel_smoother_obstacle] Velocity LOCKED until %s is true.",
                self.navigation_ready_topic,
            )

    @staticmethod
    def clamp(value, limit):
        return max(-limit, min(limit, value))

    def navigation_ready_callback(self, msg):
        ready = bool(msg.data) or not self.require_navigation_ready
        with self.lock:
            changed = ready != self.navigation_ready
            self.navigation_ready = ready
            if not ready:
                self.target_linear = 0.0
                self.target_angular = 0.0
                self.output_linear = 0.0
                self.output_angular = 0.0
                self.last_command_time = None

        if not ready:
            # The callback path provides an immediate stop; the timer continues
            # sending zero while localization remains unavailable.
            self.publisher.publish(Twist())
            if changed:
                rospy.logerr(
                    "[cmd_vel_smoother_obstacle] Velocity LOCKED by localization guard."
                )
        elif changed:
            rospy.loginfo(
                "[cmd_vel_smoother_obstacle] Localization ready; velocity ENABLED."
            )

    def command_callback(self, msg):
        now = time.monotonic()
        linear = self.clamp(float(msg.linear.x), self.max_linear)
        angular = self.clamp(float(msg.angular.z), self.max_angular)
        is_zero = abs(linear) <= self.deadband and abs(angular) <= self.deadband

        with self.lock:
            if not self.navigation_ready:
                self.last_command_time = None
                return
            self.last_command_time = now
            if is_zero:
                self.target_linear = 0.0
                self.target_angular = 0.0
                if not self.smooth_zero_commands:
                    self.output_linear = 0.0
                    self.output_angular = 0.0
            else:
                self.target_linear = linear
                self.target_angular = angular

        if is_zero and not self.smooth_zero_commands:
            self.publisher.publish(Twist())

    def update(self, _event):
        now = time.monotonic()
        with self.lock:
            dt = min(max(now - self.last_update_time, 0.0), 0.20)
            self.last_update_time = now

            stale = (
                not self.navigation_ready
                or self.last_command_time is None
                or now - self.last_command_time >= self.command_timeout
            )
            if stale:
                self.target_linear = 0.0
                self.target_angular = 0.0
                self.output_linear = 0.0
                self.output_angular = 0.0
            else:
                self.output_linear = step_towards(
                    self.output_linear,
                    self.target_linear,
                    self.linear_accel,
                    self.linear_decel,
                    dt,
                )
                self.output_angular = step_towards(
                    self.output_angular,
                    self.target_angular,
                    self.angular_accel,
                    self.angular_decel,
                    dt,
                )

            output = Twist()
            output.linear.x = self.output_linear
            output.angular.z = self.output_angular

        self.publisher.publish(output)

    def stop_now(self):
        stop = Twist()
        for _ in range(3):
            self.publisher.publish(stop)


if __name__ == "__main__":
    rospy.init_node("cmd_vel_smoother_obstacle")
    ObstacleCmdVelSmoother()
    rospy.spin()
