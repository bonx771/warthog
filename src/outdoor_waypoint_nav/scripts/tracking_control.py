#!/usr/bin/env python3
"""
Closed-loop square tracking controller for outdoor EKF evaluation.

This node replaces the open-loop square driving in square_eval.py with:
  - line tracking on each square side using EKF pose feedback
  - heading correction through a lookahead point on the current side
  - closed-loop 90 degree turns using EKF yaw feedback
  - the same result YAML/CSV format used by plot_results.py
"""

import csv
import math
import os
import time
from enum import Enum, auto

import rospy
import tf.transformations
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


R = "\033[91m"
Y = "\033[93m"
G = "\033[92m"
C = "\033[96m"
B = "\033[0m"


# NOTE: State machine chính của node; mỗi vòng _loop() xử lý theo state hiện tại.
class State(Enum):
    WAIT_EKF = auto()
    PAUSE = auto()
    DRIVE = auto()
    TURN = auto()
    DONE = auto()


# NOTE: Chuẩn hóa góc về [-pi, pi] để tránh nhảy sai số tại biên +/-180 độ.
def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, low, high):
    return max(low, min(high, value))


def sign(value):
    return 1.0 if value >= 0.0 else -1.0


class TrackingControl:
    def __init__(self):
        rospy.init_node("tracking_control")

        # NOTE: Nhóm tham số vận tốc và kích thước hình vuông, lấy từ ROS param/launch.
        self.side_length = float(rospy.get_param("~side_length", 3.0))
        self.linear_vel = float(rospy.get_param("~linear_vel", 0.5))
        self.min_linear_vel = float(rospy.get_param("~min_linear_vel", 0.10))
        self.angular_vel = float(rospy.get_param("~angular_vel", 0.25))
        self.max_drive_angular_vel = float(
            rospy.get_param("~max_drive_angular_vel", self.angular_vel)
        )
        self.turn_dir = 1 if int(rospy.get_param("~turn_direction", 1)) >= 0 else -1

        # NOTE: Các khoảng dừng giúp robot dừng hẳn và EKF ổn định giữa các pha.
        self.startup_pause = float(rospy.get_param("~startup_pause", 3.0))
        self.pause_side = float(rospy.get_param("~pause_after_side", 2.0))
        self.pause_turn = float(rospy.get_param("~pause_after_turn", 3.0))

        # NOTE: Nhóm tham số bám đường: dung sai, lookahead, giảm tốc và gain P.
        self.side_tolerance = float(rospy.get_param("~side_tolerance", 0.03))
        self.turn_tolerance = math.radians(
            float(rospy.get_param("~turn_tolerance_deg", 1.0))
        )
        self.lookahead_distance = float(rospy.get_param("~lookahead_distance", 0.8))
        self.min_lookahead = float(rospy.get_param("~min_lookahead", 0.30))
        self.slow_down_distance = float(rospy.get_param("~slow_down_distance", 0.7))
        self.drive_heading_kp = float(rospy.get_param("~drive_heading_kp", 1.4))
        self.turn_kp = float(rospy.get_param("~turn_kp", 1.2))
        self.min_turn_vel = float(rospy.get_param("~min_turn_vel", 0.07))
        self.heading_slow_angle = math.radians(
            float(rospy.get_param("~heading_slow_angle_deg", 25.0))
        )

        # ROS convention is positive cmd_vel.angular.z -> positive yaw.
        # Set to -1 if EKF yaw decreases when angular.z is positive.
        # NOTE: Đổi dấu tại đây nếu chiều yaw EKF ngược với chiều cmd_vel.angular.z.
        self.yaw_response_sign = 1.0
        if float(rospy.get_param("~yaw_response_sign", 1.0)) < 0.0:
            self.yaw_response_sign = -1.0

        # NOTE: Topic cmd_vel và odom EKF là hai điểm nối chính với hệ ROS bên ngoài.
        self.output_dir = rospy.get_param("~output_dir", "/tmp/ekf_eval")
        self.mode = rospy.get_param("~mode", "outdoor")
        self.cmd_topic = rospy.get_param(
            "~cmd_vel_topic", "/warthog_velocity_controller/cmd_vel"
        )
        self.ekf_odom_topic = rospy.get_param(
            "~ekf_odom_topic", "/outdoor_waypoint_nav/odometry/filtered"
        )

        os.makedirs(self.output_dir, exist_ok=True)

        # NOTE: Các biến runtime phục vụ state machine và pause giữa các pha.
        self.state = State.WAIT_EKF
        self.pause_start = None
        self.pause_dur = 0.0
        self.pause_next = None

        # NOTE: Pose EKF mới nhất và toàn bộ path được lưu để tính/lưu kết quả cuối bài.
        self.ekf_pos = None
        self.ekf_yaw = None
        self.ekf_path = []

        # NOTE: Các mốc pose/yaw của toàn bài, từng cạnh và từng lượt quay.
        self.start_pos = None
        self.start_yaw = None
        self.side_start_pos = None
        self.side_start_yaw = None
        self.turn_start_yaw = None
        self.turn_target_yaw = None

        # NOTE: Bộ đếm tiến độ và các lỗi dùng để đánh giá chất lượng tracking.
        self.side_done = 0
        self.turn_done = 0
        self.cross_track_errors = []
        self.heading_errors = []
        self.side_end_errors = []
        self.turn_end_errors = []

        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        rospy.Subscriber(self.ekf_odom_topic, Odometry, self._cb_ekf)
        rospy.on_shutdown(self._stop)

        rospy.loginfo(
            "[tracking_control] mode=%s side=%.2fm v=%.2fm/s max_w=%.2frad/s yaw_sign=%+.0f",
            self.mode,
            self.side_length,
            self.linear_vel,
            self.angular_vel,
            self.yaw_response_sign,
        )
        rospy.loginfo(
            "[tracking_control] cmd=%s odom=%s",
            rospy.resolve_name(self.cmd_topic),
            rospy.resolve_name(self.ekf_odom_topic),
        )

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            # NOTE: Vòng điều khiển chính chạy 20 Hz, phát cmd_vel theo state hiện tại.
            self._loop()
            if self.state == State.DONE:
                break
            rate.sleep()

    def _cb_ekf(self, msg):
        # NOTE: Đây là nguồn feedback chính: lấy x, y, yaw từ odometry EKF.
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )
        self.ekf_pos = (p.x, p.y)
        self.ekf_yaw = yaw
        self.ekf_path.append((p.x, p.y, rospy.get_time()))

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _begin_pause(self, duration, next_state):
        # NOTE: Mỗi lần chuyển pha đều stop robot trước, rồi đợi hết pause mới sang state kế tiếp.
        self._stop()
        self.pause_start = rospy.get_time()
        self.pause_dur = duration
        self.pause_next = next_state
        self.state = State.PAUSE

    def _start_drive(self):
        # NOTE: Lần DRIVE đầu tiên lưu mốc xuất phát để cuối bài tính loop closure error.
        if self.start_pos is None:
            self.start_pos = self.ekf_pos
            self.start_yaw = self.ekf_yaw
            rospy.loginfo(
                "[tracking_control] %sStart at (%.3f, %.3f), yaw %.2f deg%s",
                G,
                self.start_pos[0],
                self.start_pos[1],
                math.degrees(self.start_yaw),
                B,
            )

        # NOTE: Mỗi cạnh bắt đầu từ pose EKF hiện tại và có heading kế hoạch riêng.
        self.side_start_pos = self.ekf_pos
        self.side_start_yaw = self._planned_yaw_for_side(self.side_done)
        self.state = State.DRIVE
        rospy.loginfo(
            "[tracking_control] Drive side %d/4, target heading %.2f deg",
            self.side_done + 1,
            math.degrees(self.side_start_yaw),
        )

    def _start_turn(self):
        # NOTE: Target quay là heading của cạnh tiếp theo, cách cạnh hiện tại 90 độ.
        self.turn_start_yaw = self.ekf_yaw
        self.turn_target_yaw = self._planned_yaw_for_side(self.turn_done + 1)
        self.state = State.TURN
        rospy.loginfo(
            "[tracking_control] Turn %d/4 to %.2f deg",
            self.turn_done + 1,
            math.degrees(self.turn_target_yaw),
        )

    def _planned_yaw_for_side(self, side_index):
        # NOTE: Heading kế hoạch = yaw ban đầu + số cạnh * 90 độ, có xét chiều quay thực tế.
        signed_turn = self.yaw_response_sign * self.turn_dir * math.pi / 2.0
        return normalize_angle(self.start_yaw + side_index * signed_turn)

    def _side_progress(self):
        # NOTE: Chiếu pose hiện tại lên trục cạnh để biết đã đi được bao xa và lệch ngang bao nhiêu.
        x, y = self.ekf_pos
        x0, y0 = self.side_start_pos
        yaw = self.side_start_yaw
        ux = math.cos(yaw)
        uy = math.sin(yaw)
        dx = x - x0
        dy = y - y0
        along = dx * ux + dy * uy
        cross = -dx * uy + dy * ux
        return along, cross, ux, uy

    def _publish_drive(self, along, cross, ux, uy):
        # NOTE: Chọn một điểm lookahead nằm trên cạnh, rồi lái robot hướng tới điểm đó.
        x, y = self.ekf_pos
        x0, y0 = self.side_start_pos
        remaining = max(0.0, self.side_length - along)
        lookahead = clamp(remaining, self.min_lookahead, self.lookahead_distance)
        target_along = clamp(along + lookahead, 0.0, self.side_length)
        target_x = x0 + target_along * ux
        target_y = y0 + target_along * uy

        desired_yaw = math.atan2(target_y - y, target_x - x)
        heading_error = normalize_angle(desired_yaw - self.ekf_yaw)

        cmd = Twist()
        cmd.linear.x = self._drive_speed(remaining, heading_error)
        # NOTE: P-control theo sai số heading; yaw_response_sign đảm bảo lệnh quay đúng chiều EKF.
        angular_cmd = self.yaw_response_sign * self.drive_heading_kp * heading_error
        cmd.angular.z = clamp(
            angular_cmd, -self.max_drive_angular_vel, self.max_drive_angular_vel
        )
        self.cmd_pub.publish(cmd)

        self.cross_track_errors.append(abs(cross))
        self.heading_errors.append(abs(heading_error))

    def _drive_speed(self, remaining, heading_error):
        # NOTE: Giảm tốc khi gần cuối cạnh để tránh overshoot qua điểm cần quay.
        speed = self.linear_vel
        if self.slow_down_distance > 1e-6 and remaining < self.slow_down_distance:
            speed *= remaining / self.slow_down_distance
            speed = max(self.min_linear_vel, speed)

        # NOTE: Nếu lệch hướng lớn, giảm tốc để ưu tiên sửa heading trước.
        abs_heading_error = abs(heading_error)
        if abs_heading_error > self.heading_slow_angle:
            scale = clamp(self.heading_slow_angle / abs_heading_error, 0.35, 1.0)
            speed *= scale

        return clamp(speed, 0.0, self.linear_vel)

    def _publish_turn(self):
        # NOTE: Quay tại chỗ bằng P-control tới yaw target của cạnh kế tiếp.
        turn_error = normalize_angle(self.turn_target_yaw - self.ekf_yaw)
        cmd_z = self.yaw_response_sign * self.turn_kp * turn_error
        cmd_z = clamp(cmd_z, -self.angular_vel, self.angular_vel)
        if abs(cmd_z) < self.min_turn_vel:
            cmd_z = self.min_turn_vel * sign(cmd_z)

        cmd = Twist()
        cmd.angular.z = cmd_z
        self.cmd_pub.publish(cmd)

    def _loop(self):
        # NOTE: WAIT_EKF chỉ chờ callback EKF có dữ liệu đầu tiên, chưa phát lệnh chạy.
        if self.state == State.WAIT_EKF:
            if self.ekf_pos is not None:
                rospy.loginfo("[tracking_control] EKF ready. Starting after %.1f s.", self.startup_pause)
                self._begin_pause(self.startup_pause, State.DRIVE)
            return

        # NOTE: PAUSE là state trung gian sau khởi động, sau mỗi cạnh và sau mỗi lượt quay.
        if self.state == State.PAUSE:
            if rospy.get_time() - self.pause_start < self.pause_dur:
                return
            if self.pause_next == State.DRIVE:
                if self.side_done >= 4:
                    self.state = State.DONE
                    self._compute_and_report()
                else:
                    self._start_drive()
            elif self.pause_next == State.TURN:
                self._start_turn()
            elif self.pause_next == State.DONE:
                self.state = State.DONE
                self._compute_and_report()
            return

        # NOTE: DRIVE bám một cạnh hình vuông cho tới khi along đạt side_length.
        if self.state == State.DRIVE:
            along, cross, ux, uy = self._side_progress()
            remaining = self.side_length - along
            if remaining <= self.side_tolerance:
                self.side_done += 1
                self.side_end_errors.append(cross)
                self._stop()
                rospy.loginfo(
                    "[tracking_control] %sSide %d/4 done: along=%.3fm cross=%+.3fm%s",
                    G,
                    self.side_done,
                    along,
                    cross,
                    B,
                )
                self._begin_pause(self.pause_side, State.TURN)
            else:
                self._publish_drive(along, cross, ux, uy)
            return

        # NOTE: TURN quay 90 độ theo EKF yaw; đủ chính xác thì chuyển sang cạnh tiếp theo.
        if self.state == State.TURN:
            turn_error = normalize_angle(self.turn_target_yaw - self.ekf_yaw)
            if abs(turn_error) <= self.turn_tolerance:
                self.turn_done += 1
                self.turn_end_errors.append(math.degrees(turn_error))
                self._stop()
                rospy.loginfo(
                    "[tracking_control] %sTurn %d/4 done: error=%+.2f deg%s",
                    G,
                    self.turn_done,
                    math.degrees(turn_error),
                    B,
                )
                next_state = State.DONE if self.turn_done >= 4 else State.DRIVE
                self._begin_pause(self.pause_turn, next_state)
            else:
                self._publish_turn()

    def _compute_and_report(self):
        # NOTE: Tổng kết bài chạy: so vị trí/yaw cuối với mốc xuất phát và lưu log đánh giá.
        if not self.ekf_pos or not self.start_pos:
            rospy.logerr("[tracking_control] Not enough EKF data to compute results.")
            return

        total_path = self.side_length * 4.0
        dx = self.ekf_pos[0] - self.start_pos[0]
        dy = self.ekf_pos[1] - self.start_pos[1]
        lce = math.hypot(dx, dy)
        lce_pct = lce / total_path * 100.0
        dyaw = normalize_angle(self.ekf_yaw - self.start_yaw)

        max_cross = max(self.cross_track_errors) if self.cross_track_errors else 0.0
        mean_cross = (
            sum(self.cross_track_errors) / len(self.cross_track_errors)
            if self.cross_track_errors
            else 0.0
        )
        max_heading = max(self.heading_errors) if self.heading_errors else 0.0

        grade, verdict = self._grade(lce_pct)
        sep = "=" * 62
        print("\n{}{}{}".format(C, sep, B))
        print("{}  EKF EVALUATION - Closed-loop Square Tracking{}".format(C, B))
        print("{}{}{}".format(C, sep, B))
        print("  Mode              : {}".format(self.mode.upper()))
        print("  Side length       : {:.2f} m x 4 = {:.0f} m".format(self.side_length, total_path))
        print("  Speed             : {:.2f} m/s linear | {:.2f} rad/s max turn".format(self.linear_vel, self.angular_vel))
        print("  Loop Closure Error: {:.4f} m ({:.2f}%)".format(lce, lce_pct))
        print("  Heading Error     : {:.2f} deg".format(math.degrees(dyaw)))
        print("  Cross-track max   : {:.4f} m".format(max_cross))
        print("  Cross-track mean  : {:.4f} m".format(mean_cross))
        print("  Max heading corr  : {:.2f} deg".format(math.degrees(max_heading)))
        print("  Grade             : {}{}{}".format(G if lce_pct < 2.0 else Y, grade, B))
        print("  Verdict           : {}".format(verdict))
        print("{}{}{}\n".format(C, sep, B))

        ts = int(time.time())
        results = {
            "timestamp": ts,
            "mode": self.mode,
            "controller": "tracking_control",
            "side_length_m": self.side_length,
            "total_path_m": total_path,
            "linear_vel_ms": self.linear_vel,
            "angular_vel_rads": self.angular_vel,
            "turn_direction": self.turn_dir,
            "yaw_response_sign": self.yaw_response_sign,
            "start_pos": {"x": self.start_pos[0], "y": self.start_pos[1]},
            "start_yaw_rad": self.start_yaw,
            "start_yaw_deg": round(math.degrees(self.start_yaw), 4),
            "end_pos_ekf": {"x": self.ekf_pos[0], "y": self.ekf_pos[1]},
            "gt_start_pos": None,
            "loop_closure_error_m": round(lce, 6),
            "loop_closure_error_pct": round(lce_pct, 4),
            "heading_error_deg": round(math.degrees(dyaw), 4),
            "max_cross_track_error_m": round(max_cross, 6),
            "mean_cross_track_error_m": round(mean_cross, 6),
            "max_heading_correction_deg": round(math.degrees(max_heading), 4),
            "side_end_cross_track_errors_m": [round(v, 6) for v in self.side_end_errors],
            "turn_end_errors_deg": [round(v, 4) for v in self.turn_end_errors],
            "grade": grade,
            "verdict": verdict,
        }

        yaml_path = os.path.join(self.output_dir, "result_{}_{}.yaml".format(self.mode, ts))
        with open(yaml_path, "w") as f:
            yaml.dump(results, f, allow_unicode=True, default_flow_style=False)

        csv_path = os.path.join(self.output_dir, "ekf_path_{}_{}.csv".format(self.mode, ts))
        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["x", "y", "t"])
            writer.writerows(self.ekf_path)

        rospy.loginfo("[tracking_control] Results saved in: %s", self.output_dir)
        rospy.loginfo("[tracking_control] YAML: %s", yaml_path)
        rospy.loginfo("[tracking_control] CSV : %s", csv_path)
        rospy.loginfo(
            "[tracking_control] Plot: rosrun outdoor_waypoint_nav plot_results.py %s",
            yaml_path,
        )

    def _grade(self, lce_pct):
        if lce_pct < 1.0:
            return "XUẤT SẮC", "Đủ điều kiện dẫn đường chính xác cao"
        if lce_pct < 2.0:
            return "TỐT", "Đủ điều kiện dẫn đường thông thường"
        if lce_pct < 4.0:
            return "CHẤP NHẬN", "Dùng được nhưng nên kết hợp GPS hiệu chỉnh"
        if lce_pct < 6.0:
            return "YẾU", "Cần hiệu chỉnh IMU/encoder trước khi dẫn đường"
        return "KÉM", "Không đủ dẫn đường - kiểm tra lại cấu hình EKF"


if __name__ == "__main__":
    try:
        TrackingControl()
    except rospy.ROSInterruptException:
        pass
