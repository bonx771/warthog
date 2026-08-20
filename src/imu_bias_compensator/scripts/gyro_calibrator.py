#!/usr/bin/env python3
"""
gyro_calibrator.py
------------------
Static gyroscope bias calibration node.
Keep the robot COMPLETELY STILL during calibration.

Subscribes : /imu/data_raw  (sensor_msgs/Imu)
Publishes  : nothing (writes result to YAML file)

Usage:
  roslaunch imu_bias_compensator calibrate.launch
"""

import os
import yaml
import numpy as np
import rospy
from sensor_msgs.msg import Imu


class GyroBiasCalibrator:
    def __init__(self):
        rospy.init_node("gyro_bias_calibrator", anonymous=False)

        # ── Parameters ───────────────────────────────────────────────────────
        self.duration = rospy.get_param("~duration", 60.0)          # seconds
        default_out = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "..", "config", "gyro_bias.yaml"
        )
        self.output_path = rospy.get_param("~output_path", default_out)
        self.imu_topic   = rospy.get_param("~imu_topic", "/imu/data_raw")

        # ── State ─────────────────────────────────────────────────────────────
        self.gyro_samples  = []
        self.accel_samples = []
        self.done          = False
        self.start_time    = None

        # ── ROS ───────────────────────────────────────────────────────────────
        rospy.Subscriber(self.imu_topic, Imu, self._imu_cb, queue_size=200)

        rospy.loginfo("=" * 55)
        rospy.loginfo(" IMU Gyroscope Bias Calibrator")
        rospy.loginfo(f" Topic    : {self.imu_topic}")
        rospy.loginfo(f" Duration : {self.duration} s")
        rospy.loginfo(f" Output   : {self.output_path}")
        rospy.loginfo("=" * 55)
        rospy.loginfo(" >>> Chờ đến khi hoàn thành! <<<")

    # ─────────────────────────────────────────────────────────────────────────
    def _imu_cb(self, msg: Imu):
        if self.done:
            return

        if self.start_time is None:
            self.start_time = rospy.Time.now()
            rospy.loginfo("Collecting samples …")

        elapsed = (rospy.Time.now() - self.start_time).to_sec()

        # Progress every 10 s
        if int(elapsed) % 10 == 0 and int(elapsed) > 0:
            pct = int(elapsed / self.duration * 100)
            rospy.loginfo_throttle(10, f"  {pct}%  ({int(elapsed)}/{int(self.duration)} s) "
                                       f"  samples: {len(self.gyro_samples)}")

        if elapsed >= self.duration:
            self.done = True
            self._save_bias()
            rospy.signal_shutdown("Calib hoàn thành")
            return

        self.gyro_samples.append([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ])
        self.accel_samples.append([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ])

    # ─────────────────────────────────────────────────────────────────────────
    def _save_bias(self):
        if not self.gyro_samples:
            rospy.logerr("No samples collected – is the IMU publishing?")
            return

        gyro  = np.array(self.gyro_samples)
        accel = np.array(self.accel_samples)

        bias     = np.mean(gyro,  axis=0)
        std_gyro = np.std(gyro,   axis=0)
        std_acc  = np.std(accel,  axis=0)

        # Drift estimate for yaw axis (z)
        drift_deg_hr = float(np.degrees(bias[2]) * 3600)

        data = {
            "gyro_bias_x":     float(bias[0]),
            "gyro_bias_y":     float(bias[1]),
            "gyro_bias_z":     float(bias[2]),
            "gyro_std_x":      float(std_gyro[0]),
            "gyro_std_y":      float(std_gyro[1]),
            "gyro_std_z":      float(std_gyro[2]),
            "accel_std_x":     float(std_acc[0]),
            "accel_std_y":     float(std_acc[1]),
            "accel_std_z":     float(std_acc[2]),
            "samples":         len(self.gyro_samples),
            "duration_s":      self.duration,
            "drift_deg_per_hr": drift_deg_hr,
        }

        os.makedirs(os.path.dirname(os.path.abspath(self.output_path)), exist_ok=True)
        with open(self.output_path, "w") as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)

        rospy.loginfo("=" * 55)
        rospy.loginfo(f" Calibration result ({len(self.gyro_samples)} samples)")
        rospy.loginfo(f"   bias  x: {bias[0]:+.6f}  std: {std_gyro[0]:.6f}  rad/s")
        rospy.loginfo(f"   bias  y: {bias[1]:+.6f}  std: {std_gyro[1]:.6f}  rad/s")
        rospy.loginfo(f"   bias  z: {bias[2]:+.6f}  std: {std_gyro[2]:.6f}  rad/s  "
                      f"(yaw drift ≈ {drift_deg_hr:+.1f} °/hr)")
        rospy.loginfo(f" Saved → {self.output_path}")
        rospy.loginfo("=" * 55)


# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    try:
        node = GyroBiasCalibrator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
