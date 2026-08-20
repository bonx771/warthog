#!/usr/bin/env python3
"""
imu_republisher.py
------------------
Đọc /imu/data_raw, trừ bias đã tính sẵn, publish ra /imu/data_raw_bias.
Khi phát hiện đứng yên: set angular_velocity.z = 0 cứng.

Subscribes : /imu/data_raw          sensor_msgs/Imu
             /warthog_velocity_controller/odom  nav_msgs/Odometry  (optional)
Publishes  : /imu/data_raw_bias      sensor_msgs/Imu

Bias được load từ file YAML riêng (config/gyro_bias_static.yaml).
Không có online update – chỉ dùng giá trị tính trước.
"""

import os
import yaml
import numpy as np
from collections import deque

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


class ImuRepublisher:
    def __init__(self):
        rospy.init_node("imu_republisher", anonymous=False)

        # ── Bias file ─────────────────────────────────────────────────────────
        default_bias_file = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "..", "config", "gyro_bias.yaml"
        )
        bias_file = rospy.get_param("~bias_file", default_bias_file)
        self.bias = self._load_bias(bias_file)

        # ── Topics ────────────────────────────────────────────────────────────
        self.imu_topic  = rospy.get_param("~imu_topic",  "/imu/data_raw")
        self.odom_topic = rospy.get_param("~odom_topic",
                                          "/warthog_velocity_controller/odom")
        self.use_odom   = rospy.get_param("~use_odom", True)

        # ── Static detection params ───────────────────────────────────────────
        self.accel_var_thresh = rospy.get_param("~accel_var_thresh", 0.02)
        self.gyro_var_thresh  = rospy.get_param("~gyro_var_thresh",  0.001)
        self.odom_vel_thresh  = rospy.get_param("~odom_vel_thresh",  0.02)
        self.window_size      = rospy.get_param("~window_size",      50)
        self.static_confirm   = rospy.get_param("~static_confirm",   20)

        # ── State ─────────────────────────────────────────────────────────────
        self.accel_win      = deque(maxlen=self.window_size)
        self.gyro_win       = deque(maxlen=self.window_size)
        self.static_odom    = True
        self.static_counter = 0

        # ── ROS ───────────────────────────────────────────────────────────────
        self.pub = rospy.Publisher("/imu/data_raw_bias", Imu, queue_size=10)

        rospy.Subscriber(self.imu_topic, Imu, self._imu_cb, queue_size=200)
        if self.use_odom:
            rospy.Subscriber(self.odom_topic, Odometry,
                             self._odom_cb, queue_size=10)

        rospy.loginfo("=" * 50)
        rospy.loginfo(" IMU Republisher  (imu/data_raw → imu/data_raw_bias)")
        rospy.loginfo(f"  bias x : {self.bias[0]:+.6f} rad/s")
        rospy.loginfo(f"  bias y : {self.bias[1]:+.6f} rad/s")
        rospy.loginfo(f"  bias z : {self.bias[2]:+.6f} rad/s")
        rospy.loginfo(f"  use_odom: {self.use_odom}")
        rospy.loginfo("=" * 50)

    # ── Load bias YAML ────────────────────────────────────────────────────────
    def _load_bias(self, path: str) -> np.ndarray:
        abs_path = os.path.abspath(path)
        if not os.path.isfile(abs_path):
            rospy.logwarn(f"[imu_republisher] Bias file not found: {abs_path}")
            rospy.logwarn("  Falling back to zero bias.")
            return np.zeros(3)

        with open(abs_path, "r") as f:
            d = yaml.safe_load(f)

        bias = np.array([
            float(d.get("gyro_bias_x", 0.0)),
            float(d.get("gyro_bias_y", 0.0)),
            float(d.get("gyro_bias_z", 0.0)),
        ])
        rospy.loginfo(f"[imu_republisher] Loaded bias: {bias}")
        return bias

    # ── Odometry callback ─────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        lv = msg.twist.twist.linear
        av = msg.twist.twist.angular
        speed = (lv.x**2 + lv.y**2 + lv.z**2 +
                 av.x**2 + av.y**2 + av.z**2) ** 0.5
        self.static_odom = speed < self.odom_vel_thresh

    # ── IMU static detection ──────────────────────────────────────────────────
    def _is_static_imu(self) -> bool:
        if len(self.gyro_win) < self.window_size:
            return False
        av = np.var(np.array(self.accel_win), axis=0)
        gv = np.var(np.array(self.gyro_win),  axis=0)
        return (np.all(av < self.accel_var_thresh) and
                np.all(gv < self.gyro_var_thresh))

    # ── IMU callback ──────────────────────────────────────────────────────────
    def _imu_cb(self, msg: Imu):
        gx_raw = msg.angular_velocity.x
        gy_raw = msg.angular_velocity.y
        gz_raw = msg.angular_velocity.z

        self.accel_win.append([msg.linear_acceleration.x,
                                msg.linear_acceleration.y,
                                msg.linear_acceleration.z])
        self.gyro_win.append([gx_raw, gy_raw, gz_raw])

        # ── Detect static ────────────────────────────────────────────────────
        static_imu = self._is_static_imu()
        is_static  = static_imu and (self.static_odom if self.use_odom else True)

        if is_static:
            self.static_counter += 1
        else:
            self.static_counter = 0

        confirmed_static = is_static and self.static_counter > self.static_confirm

        # ── Build output message ─────────────────────────────────────────────
        out = Imu()
        out.header             = msg.header
        out.linear_acceleration            = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        out.orientation            = msg.orientation
        out.orientation_covariance = msg.orientation_covariance

        if confirmed_static:
            # Đứng yên xác nhận → zero cứng toàn bộ angular velocity
            out.angular_velocity.x = 0.0
            out.angular_velocity.y = 0.0
            out.angular_velocity.z = 0.0
            out.angular_velocity_covariance = [
                1e-9, 0.0, 0.0,
                0.0, 1e-9, 0.0,
                0.0, 0.0, 1e-9,
            ]
        else:
            # Di chuyển → trừ bias tính sẵn
            out.angular_velocity.x = gx_raw - self.bias[0]
            out.angular_velocity.y = gy_raw - self.bias[1]
            out.angular_velocity.z = gz_raw - self.bias[2]
            out.angular_velocity_covariance = msg.angular_velocity_covariance

        self.pub.publish(out)

        rospy.logdebug(
            f"static={confirmed_static}  "
            f"gz_raw={gz_raw:+.5f}  "
            f"gz_out={out.angular_velocity.z:+.5f}"
        )


# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    try:
        ImuRepublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
