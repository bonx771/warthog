#!/usr/bin/env python3
import sys
import termios
import tty
import textwrap
import signal
import ast

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class KeyboardToJoy:
    def __init__(self):
        self.output_topic = self._get_param("output_topic", "/joy_teleop/joy")
        self.cmd_topic = self._get_param("cmd_topic", "/joy_teleop/cmd_vel")
        if isinstance(self.cmd_topic, str):
            self.cmd_topic = self.cmd_topic.strip()
        self.button_count = self._get_param("button_count", 8)
        self.press_repeats = self._get_param("press_repeats", 5)
        self.release_repeats = self._get_param("release_repeats", 2)
        self.repeat_delay = self._get_param("repeat_delay", 0.03)
        self.linear_speed = self._get_param("linear_speed", 0.8)
        self.angular_speed = self._get_param("angular_speed", 0.8)
        self.publish_rate = self._get_param("publish_rate", 10.0)
        self.enable_motion_keys = self._coerce_bool(self._get_param("enable_motion_keys", True))

        default_key_to_button = {
            "l": 4,  # LB
            "r": 5,  # RB
            "s": 7,  # START
            "b": 1,  # B
            "c": 2,  # X (reserved for Scenario 4 route calculation)
            "y": 3,  # Y
            "k": 6,  # BACK
        }
        enabled_button_keys = self._get_param("enabled_button_keys", list(default_key_to_button.keys()))
        if isinstance(enabled_button_keys, str):
            try:
                enabled_button_keys = ast.literal_eval(enabled_button_keys)
            except (ValueError, SyntaxError):
                enabled_button_keys = list(default_key_to_button.keys())
        self.key_to_button = {
            key: button_index
            for key, button_index in default_key_to_button.items()
            if key in enabled_button_keys
        }

        self.pub = rospy.Publisher(self.output_topic, Joy, queue_size=10)
        self.cmd_pub = None
        if self.enable_motion_keys and self.cmd_topic:
            self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        self.motion_controls_enabled = self.cmd_pub is not None
        self._running = True
        self.current_twist = Twist()
        self.manual_active = False
        self.stop_hold_active = False

        signal.signal(signal.SIGINT, self._handle_sigint)
        rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self._publish_current_twist)
        rospy.loginfo(
            "Keyboard waypoint control config: motion_keys=%s, enabled_buttons=%s, output_topic=%s, cmd_topic=%s",
            self.motion_controls_enabled,
            sorted(self.key_to_button.keys()),
            self.output_topic,
            self.cmd_topic if self.cmd_topic else "<disabled>",
        )

    @staticmethod
    def _coerce_bool(value):
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    @staticmethod
    def _candidate_param_names(name):
        private_name = f"~{name}"
        absolute_names = [
            f"/outdoor_waypoint_nav/keyboard_waypoint_control/{name}",
            f"/outdoor_waypoint_nav/keyboard_to_joy/{name}",
        ]
        return [private_name] + absolute_names

    def _get_param(self, name, default):
        for param_name in self._candidate_param_names(name):
            if rospy.has_param(param_name):
                return rospy.get_param(param_name)
        return default

    def _handle_sigint(self, _signum, _frame):
        self._stop_motion()
        self._running = False
        rospy.signal_shutdown("Ctrl-C pressed")

    def _publish_current_twist(self, _event):
        if self.cmd_pub is not None and (self.manual_active or self.stop_hold_active):
            self.cmd_pub.publish(self.current_twist)

    def _set_motion(self, linear=0.0, angular=0.0):
        if self.cmd_pub is None:
            return
        self.current_twist.linear.x = linear
        self.current_twist.angular.z = angular
        self.stop_hold_active = False
        self.manual_active = True
        self.cmd_pub.publish(self.current_twist)

    def _stop_motion(self, hold_zero=False):
        if self.cmd_pub is None:
            self.manual_active = False
            self.stop_hold_active = False
            return
        self.current_twist.linear.x = 0.0
        self.current_twist.angular.z = 0.0
        self.cmd_pub.publish(self.current_twist)
        self.manual_active = False
        self.stop_hold_active = hold_zero

    def _release_stop_hold(self):
        self.stop_hold_active = False

    def _publish_button(self, button_index):
        pressed_msg = Joy()
        pressed_msg.axes = []
        pressed_msg.buttons = [0] * self.button_count
        pressed_msg.buttons[button_index] = 1

        released_msg = Joy()
        released_msg.axes = []
        released_msg.buttons = [0] * self.button_count

        for _ in range(self.press_repeats):
            pressed_msg.header.stamp = rospy.Time.now()
            self.pub.publish(pressed_msg)
            rospy.sleep(self.repeat_delay)

        for _ in range(self.release_repeats):
            released_msg.header.stamp = rospy.Time.now()
            self.pub.publish(released_msg)
            rospy.sleep(self.repeat_delay)

    def _getch(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            char = sys.stdin.read(1)
            # In raw mode Ctrl-C arrives as '\x03' instead of raising KeyboardInterrupt.
            if char == "\x03":
                self._handle_sigint(None, None)
                return None
            return char
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def run(self):
        banner_lines = [
            "==================== Keyboard To Joy ===================="
            if self.motion_controls_enabled
            else "================ Keyboard Waypoint Control ================"
        ]
        if self.motion_controls_enabled:
            banner_lines.extend(
                [
                    "w -> drive forward",
                    "x -> drive backward",
                    "a -> turn left",
                    "d -> turn right",
                    "space -> stop motion",
                ]
            )
        else:
            banner_lines.append("joystick -> drive robot motion")

        key_descriptions = {
            "l": "start waypoint collection",
            "r": "start waypoint following",
            "s": "heading calibration",
            "b": "stop robot motion",
            "c": "calculate Scenario 4 waypoints",
            "y": "continue or collect waypoint",
            "k": "end waypoint collection",
        }

        for key in ["l", "r", "s", "b", "c", "y", "k"]:
            if key in self.key_to_button:
                banner_lines.append(f"{key} -> {key_descriptions[key]}")

        banner_lines.extend(
            [
                "q -> quit",
                "========================================================",
            ]
        )
        banner = "\n".join(banner_lines)
        sys.stdout.write("\n" + banner + "\n\n")
        sys.stdout.flush()

        while not rospy.is_shutdown() and self._running:
            key = self._getch()
            if key is None:
                continue
            key = key.lower()
            if key == "q":
                break
            if self.motion_controls_enabled and key == "w":
                self._set_motion(self.linear_speed, 0.0)
                rospy.loginfo("Keyboard drive: forward")
                continue
            if self.motion_controls_enabled and key == "x":
                self._set_motion(-self.linear_speed, 0.0)
                rospy.loginfo("Keyboard drive: backward")
                continue
            if self.motion_controls_enabled and key == "a":
                self._set_motion(0.0, self.angular_speed)
                rospy.loginfo("Keyboard drive: turn left")
                continue
            if self.motion_controls_enabled and key == "d":
                self._set_motion(0.0, -self.angular_speed)
                rospy.loginfo("Keyboard drive: turn right")
                continue
            if self.motion_controls_enabled and key == " ":
                self._stop_motion(hold_zero=True)
                rospy.loginfo("Keyboard drive: stop")
                continue
            if key in self.key_to_button:
                if key == "b":
                    self._stop_motion(hold_zero=True)
                else:
                    self._release_stop_hold()
                rospy.loginfo("Sent Joy button for key: %s", key)
                self._publish_button(self.key_to_button[key])

        self._stop_motion()


if __name__ == "__main__":
    rospy.init_node("keyboard_to_joy")
    KeyboardToJoy().run()
