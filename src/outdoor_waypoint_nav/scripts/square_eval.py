#!/usr/bin/env python3
"""
square_eval.py — Đánh giá EKF bằng Loop Closure Test hình vuông

Luồng sim:
  - Điều khiển xe chạy hình vuông trong Gazebo
  - So sánh EKF vs Gazebo ground truth
  - Tính: loop closure error, RMSE theo GT, heading error

Luồng outdoor:
  - Điều khiển xe chạy hình vuông ngoài trời
  - Không có ground truth, chỉ tính loop closure error từ EKF

State machine:
  WAIT_EKF → READY → DRIVE → TURN → DRIVE → ... (×4) → DONE

Cách chạy:
  rosrun outdoor_waypoint_nav square_eval.py       (dùng param mặc định)
  rosparam load sim_params.yaml && rosrun ...       (load từ file)
"""
import os
import math
import time
import csv
import yaml
import rospy
import tf.transformations
from enum import Enum, auto
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates

# ── Màu terminal ──────────────────────────────────────────────────────────────
R  = "\033[91m"; Y = "\033[93m"; G = "\033[92m"; C = "\033[96m"; B = "\033[0m"

# ── State machine ─────────────────────────────────────────────────────────────
class State(Enum):
    WAIT_EKF = auto()   # chờ nhận dữ liệu EKF lần đầu
    READY    = auto()   # đếm ngược 3 giây
    DRIVE    = auto()   # đang chạy thẳng
    PAUSE    = auto()   # dừng ngắn giữa cạnh và góc
    TURN     = auto()   # đang quay 90°
    DONE     = auto()   # hoàn thành, tính kết quả


def _normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class SquareEval:
    def __init__(self):
        rospy.init_node("square_eval")

        # ── Tham số ───────────────────────────────────────────────────────────
        self.side_length      = rospy.get_param("~side_length",      5.0)
        self.linear_vel       = rospy.get_param("~linear_vel",       0.4)
        self.angular_vel      = rospy.get_param("~angular_vel",      0.3)
        self.turn_dir         = rospy.get_param("~turn_direction",   1)    # 1=trái, -1=phải
        self.pause_side       = rospy.get_param("~pause_after_side", 1.0)
        self.pause_turn       = rospy.get_param("~pause_after_turn", 3.0)
        self.output_dir       = rospy.get_param("~output_dir",       "/tmp/ekf_eval")
        self.mode             = rospy.get_param("~mode",             "outdoor")
        self.gt_topic         = rospy.get_param("~gt_topic",         "/gazebo/model_states")
        self.gt_model_name    = rospy.get_param("~gt_model_name",    "warthog")
        self.cmd_topic        = rospy.get_param(
            "~cmd_vel_topic", "/warthog_velocity_controller/cmd_vel"
        )
        self.ekf_odom_topic   = rospy.get_param(
            "~ekf_odom_topic", "/outdoor_waypoint_nav/odometry/filtered"
        )

        os.makedirs(self.output_dir, exist_ok=True)

        # ── State ─────────────────────────────────────────────────────────────
        self.state        = State.WAIT_EKF
        self.side_done    = 0       # số cạnh đã hoàn thành (0-3)
        self.turn_done    = 0       # số lần quay đã hoàn thành (0-3)
        self.pause_start  = None    # thời điểm bắt đầu dừng
        self.pause_dur    = 0.0

        # ── EKF ───────────────────────────────────────────────────────────────
        self.ekf_pos      = None    # (x, y) hiện tại
        self.ekf_yaw      = None    # yaw hiện tại (rad)
        self.ekf_path     = []      # [(x, y, t), ...]

        # Mốc đầu mỗi cạnh / mỗi góc quay
        self.seg_start_pos = None
        self.seg_start_yaw = None

        # Vị trí xuất phát và kết thúc
        self.start_pos    = None
        self.start_yaw    = None

        # ── Ground truth (chỉ sim) ────────────────────────────────────────────
        self.gt_pos       = None
        self.gt_path      = []      # [(x, y, t), ...]
        self.gt_start_pos = None

        # ── Publisher / Subscriber ────────────────────────────────────────────
        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        rospy.Subscriber(self.ekf_odom_topic, Odometry, self._cb_ekf)

        if self.mode == "sim" and self.gt_topic:
            rospy.Subscriber(self.gt_topic, ModelStates, self._cb_gt)

        rospy.loginfo("[square_eval] mode=%s  cạnh=%.1fm  v=%.1fm/s  ω=%.2frad/s",
                      self.mode, self.side_length, self.linear_vel, self.angular_vel)

        # ── Vòng điều khiển 20 Hz ─────────────────────────────────────────────
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self._loop()
            if self.state == State.DONE:
                break
            rate.sleep()

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_ekf(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w])
        self.ekf_pos = (p.x, p.y)
        self.ekf_yaw = yaw
        self.ekf_path.append((p.x, p.y, rospy.get_time()))

    def _cb_gt(self, msg):
        try:
            idx = msg.name.index(self.gt_model_name)
            p = msg.pose[idx].position
            self.gt_pos = (p.x, p.y)
            self.gt_path.append((p.x, p.y, rospy.get_time()))
        except ValueError:
            pass

    # ── Hàm hỗ trợ ────────────────────────────────────────────────────────────

    def _dist_from_seg(self):
        """Khoảng cách từ vị trí hiện tại đến điểm đầu cạnh."""
        if not self.seg_start_pos or not self.ekf_pos:
            return 0.0
        dx = self.ekf_pos[0] - self.seg_start_pos[0]
        dy = self.ekf_pos[1] - self.seg_start_pos[1]
        return math.sqrt(dx*dx + dy*dy)

    def _yaw_turned(self):
        """Góc đã quay kể từ đầu lượt quay (rad, luôn dương)."""
        if self.seg_start_yaw is None or self.ekf_yaw is None:
            return 0.0
        # Chỉ đo độ lớn góc quay; dấu cmd_vel và dấu yaw EKF có thể ngược nhau
        # trên xe thật.
        return abs(_normalize_angle(self.ekf_yaw - self.seg_start_yaw))

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _publish_drive(self):
        cmd = Twist()
        cmd.linear.x = self.linear_vel
        self.cmd_pub.publish(cmd)

    def _publish_turn(self):
        cmd = Twist()
        cmd.angular.z = self.angular_vel * self.turn_dir
        self.cmd_pub.publish(cmd)

    def _begin_pause(self, duration):
        self._stop()
        self.pause_start = rospy.get_time()
        self.pause_dur   = duration
        self.state       = State.PAUSE

    # ── Vòng lặp chính ────────────────────────────────────────────────────────

    def _loop(self):
        if self.state == State.WAIT_EKF:
            if self.ekf_pos is not None:
                rospy.loginfo("[square_eval] EKF sẵn sàng. Bắt đầu sau 3 giây...")
                self._begin_pause(3.0)
                self.state = State.PAUSE
                # Sau khi pause xong sẽ chuyển sang READY

        elif self.state == State.PAUSE:
            if rospy.get_time() - self.pause_start >= self.pause_dur:
                # Kết thúc pause → quyết định trạng thái tiếp theo
                if self.start_pos is None:
                    # Pause đầu tiên (sau WAIT_EKF) → bắt đầu lái
                    self.start_pos     = self.ekf_pos
                    self.start_yaw     = self.ekf_yaw
                    self.seg_start_pos = self.ekf_pos
                    self.seg_start_yaw = self.ekf_yaw
                    if self.gt_pos is not None:
                        self.gt_start_pos = self.gt_pos
                    rospy.loginfo("[square_eval] %sBắt đầu! Xuất phát: (%.3f, %.3f)%s",
                                  G, self.start_pos[0], self.start_pos[1], B)
                    self.state = State.DRIVE

                elif self.state == State.PAUSE:
                    # Pause giữa cạnh và góc → chuyển sang TURN
                    self.seg_start_yaw = self.ekf_yaw
                    self.state = State.TURN

        elif self.state == State.DRIVE:
            dist = self._dist_from_seg()
            if dist < self.side_length:
                self._publish_drive()
            else:
                self.side_done += 1
                rospy.loginfo("[square_eval] %sCạnh %d/4 hoàn thành — %.3f m%s",
                              G, self.side_done, dist, B)
                # Sau mỗi cạnh (kể cả cạnh 4) đều quay để về đúng hướng ban đầu
                self._stop()
                rospy.sleep(self.pause_side)
                self.seg_start_yaw = self.ekf_yaw
                self.state = State.TURN

        elif self.state == State.TURN:
            turned = self._yaw_turned()
            target = math.pi / 2 - 0.01   # 90° - 2.3° dung sai
            if turned < target:
                self._publish_turn()
            else:
                self.turn_done += 1
                rospy.loginfo("[square_eval] Quay %d/4 hoàn thành — %.1f°",
                              self.turn_done, math.degrees(turned))
                self._stop()
                rospy.sleep(self.pause_turn)
                if self.turn_done >= 4:
                    # Đã quay đủ 4 lần → về đúng hướng ban đầu → tính kết quả
                    self.state = State.DONE
                    self._compute_and_report()
                else:
                    self.seg_start_pos = self.ekf_pos
                    self.seg_start_yaw = self.ekf_yaw
                    self.state = State.DRIVE

    # ── Tính kết quả ──────────────────────────────────────────────────────────

    def _compute_and_report(self):
        """Tính loop closure error và các chỉ số, in báo cáo, lưu file."""
        if not self.ekf_pos or not self.start_pos:
            rospy.logerr("[square_eval] Không đủ dữ liệu để tính kết quả!")
            return

        total_path = self.side_length * 4

        # ── Loop Closure Error (EKF) ──────────────────────────────────────────
        dx  = self.ekf_pos[0] - self.start_pos[0]
        dy  = self.ekf_pos[1] - self.start_pos[1]
        lce = math.sqrt(dx*dx + dy*dy)
        lce_pct = lce / total_path * 100

        # ── Heading Error ─────────────────────────────────────────────────────
        dyaw = self.ekf_yaw - self.start_yaw
        dyaw = (dyaw + math.pi) % (2*math.pi) - math.pi  # normalize [-π, π]

        # ── So sánh với Ground Truth (sim only) ───────────────────────────────
        gt_lce    = None
        ekf_rmse  = None
        if self.mode == "sim" and self.gt_pos and self.gt_start_pos:
            # Loop closure error của GT
            dx_gt = self.gt_pos[0] - self.gt_start_pos[0]
            dy_gt = self.gt_pos[1] - self.gt_start_pos[1]
            gt_lce = math.sqrt(dx_gt*dx_gt + dy_gt*dy_gt)

            # RMSE giữa EKF path và GT path (nội suy theo thời gian)
            ekf_rmse = self._compute_rmse()

        # ── Đánh giá chất lượng ───────────────────────────────────────────────
        if lce_pct < 1.0:
            grade = f"{G}XUẤT SẮC{B}"
            verdict = "Đủ điều kiện dẫn đường chính xác cao"
        elif lce_pct < 2.0:
            grade = f"{G}TỐT{B}"
            verdict = "Đủ điều kiện dẫn đường thông thường"
        elif lce_pct < 4.0:
            grade = f"{Y}CHẤP NHẬN{B}"
            verdict = "Dùng được nhưng nên kết hợp GPS hiệu chỉnh"
        elif lce_pct < 6.0:
            grade = f"{Y}YẾU{B}"
            verdict = "Cần hiệu chỉnh IMU/encoder trước khi dẫn đường"
        else:
            grade = f"{R}KÉM{B}"
            verdict = "Không đủ dẫn đường — kiểm tra lại cấu hình EKF"

        # ── In báo cáo ────────────────────────────────────────────────────────
        sep = "═" * 62
        print(f"\n{C}{sep}{B}")
        print(f"{C}  KẾT QUẢ ĐÁNH GIÁ EKF — Loop Closure Test (hình vuông){B}")
        print(f"{C}{sep}{B}")
        print(f"  Chế độ          : {self.mode.upper()}")
        print(f"  Kích thước      : {self.side_length} m/cạnh × 4 = {total_path:.0f} m tổng")
        print(f"  Vận tốc         : {self.linear_vel} m/s thẳng  |  {self.angular_vel} rad/s quay")
        print()
        print(f"  Vị trí xuất phát (EKF) : ({self.start_pos[0]:+.4f},  {self.start_pos[1]:+.4f})")
        print(f"  Vị trí kết thúc (EKF)  : ({self.ekf_pos[0]:+.4f},  {self.ekf_pos[1]:+.4f})")
        print()
        print(f"  Loop Closure Error      : {self._color_val(lce, 0.1, 0.3):.4f} m")
        print(f"  Loop Closure Error %    : {self._color_val(lce_pct, 1.0, 3.0):.2f}%  (tổng {total_path:.0f} m)")
        print(f"  Heading Error           : {self._color_val(abs(math.degrees(dyaw)), 2.0, 5.0):.2f}°")

        if self.mode == "sim":
            print()
            print(f"  ── So sánh với Gazebo Ground Truth ──")
            if gt_lce is not None:
                print(f"  GT Loop Closure Error   : {gt_lce:.4f} m")
            if ekf_rmse is not None:
                print(f"  RMSE (EKF vs GT)        : {self._color_val(ekf_rmse, 0.05, 0.2):.4f} m")

        print()
        print(f"  Đánh giá: {grade}  —  {verdict}")
        print(f"{C}{sep}{B}\n")

        # ── Lưu kết quả ──────────────────────────────────────────────────────
        ts = int(time.time())
        results = {
            "timestamp": ts,
            "mode": self.mode,
            "side_length_m": self.side_length,
            "total_path_m": total_path,
            "linear_vel_ms": self.linear_vel,
            "angular_vel_rads": self.angular_vel,
            "turn_direction": self.turn_dir,
            "start_pos": {"x": self.start_pos[0], "y": self.start_pos[1]},
            "start_yaw_rad": self.start_yaw,
            "start_yaw_deg": round(math.degrees(self.start_yaw), 4),
            "end_pos_ekf": {"x": self.ekf_pos[0], "y": self.ekf_pos[1]},
            "gt_start_pos": {"x": self.gt_start_pos[0], "y": self.gt_start_pos[1]}
                             if self.gt_start_pos else None,
            "loop_closure_error_m": round(lce, 6),
            "loop_closure_error_pct": round(lce_pct, 4),
            "heading_error_deg": round(math.degrees(dyaw), 4),
            "grade": grade.replace("\033[92m","").replace("\033[93m","")
                         .replace("\033[91m","").replace("\033[0m",""),
            "verdict": verdict,
        }
        if gt_lce is not None:
            results["gt_loop_closure_error_m"] = round(gt_lce, 6)
        if ekf_rmse is not None:
            results["ekf_rmse_vs_gt_m"] = round(ekf_rmse, 6)

        yaml_path = os.path.join(self.output_dir, f"result_{self.mode}_{ts}.yaml")
        with open(yaml_path, "w") as f:
            yaml.dump(results, f, allow_unicode=True, default_flow_style=False)

        # Lưu path EKF dạng CSV để vẽ đồ thị
        csv_path = os.path.join(self.output_dir, f"ekf_path_{self.mode}_{ts}.csv")
        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["x", "y", "t"])
            w.writerows(self.ekf_path)

        if self.mode == "sim" and self.gt_path:
            gt_csv = os.path.join(self.output_dir, f"gt_path_{self.mode}_{ts}.csv")
            with open(gt_csv, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["x", "y", "t"])
                w.writerows(self.gt_path)

        rospy.loginfo("[square_eval] Kết quả lưu tại: %s", self.output_dir)
        rospy.loginfo("[square_eval] YAML: %s", yaml_path)
        rospy.loginfo("[square_eval] CSV : %s", csv_path)
        rospy.loginfo(
            "[square_eval] Vẽ đồ thị: rosrun outdoor_waypoint_nav plot_results.py %s",
            yaml_path,
        )

    def _compute_rmse(self):
        """RMSE giữa EKF path và GT path, nội suy GT theo timestamp EKF."""
        if len(self.ekf_path) < 2 or len(self.gt_path) < 2:
            return None

        errors = []
        gt_times = [p[2] for p in self.gt_path]
        gt_xs    = [p[0] for p in self.gt_path]
        gt_ys    = [p[1] for p in self.gt_path]

        for ex, ey, et in self.ekf_path:
            # Nội suy tuyến tính GT tại thời điểm et
            if et < gt_times[0] or et > gt_times[-1]:
                continue
            # Tìm đoạn GT bao quanh et
            for i in range(len(gt_times)-1):
                if gt_times[i] <= et <= gt_times[i+1]:
                    alpha = (et - gt_times[i]) / (gt_times[i+1] - gt_times[i] + 1e-9)
                    gx = gt_xs[i] + alpha * (gt_xs[i+1] - gt_xs[i])
                    gy = gt_ys[i] + alpha * (gt_ys[i+1] - gt_ys[i])
                    errors.append((ex-gx)**2 + (ey-gy)**2)
                    break

        if not errors:
            return None
        return math.sqrt(sum(errors) / len(errors))

    def _color_val(self, val, thr_good, thr_warn):
        """Trả về giá trị màu terminal theo ngưỡng."""
        if val < thr_good:
            return val   # sẽ print màu xanh ở caller
        return val       # đơn giản trả về số, màu đã in trong _report

    def _color_str(self, val, thr_good, thr_warn, fmt=".4f"):
        c = G if val < thr_good else (Y if val < thr_warn else R)
        return f"{c}{val:{fmt}}{B}"


if __name__ == "__main__":
    try:
        SquareEval()
    except rospy.ROSInterruptException:
        pass
