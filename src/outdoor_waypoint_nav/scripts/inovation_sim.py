#!/usr/bin/env python3

import csv
import math
import os
import threading
from collections import deque

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry


COMPONENT_LABELS = {
    "position": ("x", "y", "z"),
    "angle": ("roll", "pitch", "yaw"),
    "velocity": ("vx", "vy", "vz"),
    "angle_velocity": ("wx", "wy", "wz"),
    "acceleration": ("ax", "ay", "az"),
}

MEASUREMENT_COLOR = "tab:blue"
PREDICTION_COLOR = "tab:red"
PLOT_ROWS = (
    ("position", "Position", "m"),
    ("angle", "Angle", "rad"),
    ("velocity", "Velocity", "m/s"),
    ("angle_velocity", "Angle Velocity", "rad/s"),
    ("acceleration", "Acceleration", "m/s^2"),
)


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_to_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def vector_subtract(vector_a, vector_b):
    return tuple(vector_a[i] - vector_b[i] for i in range(3))


def rotate_vector(rotation_matrix, vector):
    return tuple(
        sum(rotation_matrix[row][col] * vector[col] for col in range(3))
        for row in range(3)
    )


def transpose_matrix(matrix):
    return tuple(
        tuple(matrix[col][row] for col in range(3))
        for row in range(3)
    )


def multiply_matrices(matrix_a, matrix_b):
    return tuple(
        tuple(
            sum(matrix_a[row][idx] * matrix_b[idx][col] for idx in range(3))
            for col in range(3)
        )
        for row in range(3)
    )


def quaternion_to_matrix(quaternion_msg):
    x_value = quaternion_msg.x
    y_value = quaternion_msg.y
    z_value = quaternion_msg.z
    w_value = quaternion_msg.w
    norm = math.sqrt(
        x_value * x_value
        + y_value * y_value
        + z_value * z_value
        + w_value * w_value
    )
    if norm < 1e-12:
        return (
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
        )

    x_value /= norm
    y_value /= norm
    z_value /= norm
    w_value /= norm

    xx = x_value * x_value
    yy = y_value * y_value
    zz = z_value * z_value
    xy = x_value * y_value
    xz = x_value * z_value
    yz = y_value * z_value
    wx = w_value * x_value
    wy = w_value * y_value
    wz = w_value * z_value

    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def matrix_to_rpy(rotation_matrix):
    pitch = math.asin(clamp(-rotation_matrix[2][0], -1.0, 1.0))
    if abs(abs(rotation_matrix[2][0]) - 1.0) < 1e-9:
        roll = math.atan2(-rotation_matrix[1][2], rotation_matrix[1][1])
        yaw = 0.0
    else:
        roll = math.atan2(rotation_matrix[2][1], rotation_matrix[2][2])
        yaw = math.atan2(rotation_matrix[1][0], rotation_matrix[0][0])
    return (
        wrap_to_pi(roll),
        wrap_to_pi(pitch),
        wrap_to_pi(yaw),
    )


def nan_vector():
    return (float("nan"), float("nan"), float("nan"))


class InovationSim:
    """Simulation innovation monitor: measurement - prediction."""

    def __init__(self):
        rospy.init_node("inovation_sim")

        self.model_name = rospy.get_param("~model_name", "warthog")
        self.model_states_topic = rospy.get_param(
            "~model_states_topic", "/gazebo/model_states"
        )
        self.ekf_topic = rospy.get_param(
            "~ekf_topic", "/outdoor_waypoint_nav/odometry/filtered"
        )
        self.report_hz = float(rospy.get_param("~report_hz", 5.0))
        self.max_pair_dt = float(rospy.get_param("~max_pair_dt", 0.5))
        self.output_frame = rospy.get_param("~output_frame", "innovation_local")

        self.save_plot = bool(rospy.get_param("~save_plot", False))
        self.show_plot = bool(rospy.get_param("~show_plot", False))
        self.plot_backend = rospy.get_param("~plot_backend", "Agg")
        self.realtime_plot = bool(rospy.get_param("~realtime_plot", True))
        self.realtime_plot_backend = rospy.get_param(
            "~realtime_plot_backend", "TkAgg"
        )
        self.realtime_plot_hz = float(rospy.get_param("~realtime_plot_hz", 5.0))
        self.realtime_window_sec = float(rospy.get_param("~realtime_window_sec", 0.0))
        self.realtime_max_points = max(
            1,
            int(rospy.get_param("~realtime_max_points", 500)),
        )
        max_history_points = int(rospy.get_param("~max_history_points", 0))
        self.max_history_points = (
            max_history_points if max_history_points > 0 else None
        )
        self.plot_file = rospy.get_param(
            "~plot_file", "/tmp/inovation_sim_measurement_vs_prediction.png"
        )
        self.save_csv = bool(rospy.get_param("~save_csv", True))
        self.csv_file = rospy.get_param(
            "~csv_file", "/tmp/inovation_sim_measurement_vs_prediction.csv"
        )

        self.position_topic = rospy.get_param(
            "~inovation_position_topic", "/outdoor_waypoint_nav/inovation_position"
        )
        self.angle_topic = rospy.get_param(
            "~inovation_angle_topic", "/outdoor_waypoint_nav/inovation_angle"
        )
        self.velocity_topic = rospy.get_param(
            "~inovation_velocity_topic", "/outdoor_waypoint_nav/inovation_velocity"
        )
        self.angle_velocity_topic = rospy.get_param(
            "~inovation_angle_velocity_topic",
            "/outdoor_waypoint_nav/inovation_angle_velocity",
        )
        self.acceleration_topic = rospy.get_param(
            "~inovation_acceleration_topic",
            "/outdoor_waypoint_nav/inovation_acceleration",
        )

        self.measurement_state = None
        self.prediction_state = None
        self.measurement_reference = None
        self.prediction_reference = None
        self.measurement_prev_velocity = None
        self.prediction_prev_velocity = None
        self.measurement_prev_stamp = None
        self.prediction_prev_stamp = None
        self.model_index = None

        self.measurement_history = deque(maxlen=self.max_history_points)
        self.prediction_history = deque(maxlen=self.max_history_points)
        self.history_lock = threading.Lock()
        self.history_t0 = None
        self.last_measurement_stamp_ns = None
        self.last_prediction_stamp_ns = None

        self.matplotlib = None
        self.plt = None
        self.realtime_figure = None
        self.realtime_axes = None
        self.realtime_lines = None
        self.realtime_y_limits = {}
        self.last_realtime_measurement_count = 0
        self.last_realtime_prediction_count = 0
        self.realtime_plot_closed_logged = False

        self.position_pub = rospy.Publisher(
            self.position_topic, Vector3Stamped, queue_size=10
        )
        self.angle_pub = rospy.Publisher(self.angle_topic, Vector3Stamped, queue_size=10)
        self.velocity_pub = rospy.Publisher(
            self.velocity_topic, Vector3Stamped, queue_size=10
        )
        self.angle_velocity_pub = rospy.Publisher(
            self.angle_velocity_topic, Vector3Stamped, queue_size=10
        )
        self.acceleration_pub = rospy.Publisher(
            self.acceleration_topic, Vector3Stamped, queue_size=10
        )

        rospy.Subscriber(self.model_states_topic, ModelStates, self._measurement_cb)
        rospy.Subscriber(self.ekf_topic, Odometry, self._prediction_cb)
        rospy.Timer(rospy.Duration(1.0 / self.report_hz), self._report_timer_cb)
        rospy.on_shutdown(self._on_shutdown)

        self._initialize_plotting()

        rospy.loginfo(
            "[inovation_sim] measurement=%s prediction=%s model=%s report_hz=%.1f plot=%s csv=%s realtime=%s history=%s",
            rospy.resolve_name(self.model_states_topic),
            rospy.resolve_name(self.ekf_topic),
            self.model_name,
            self.report_hz,
            self.plot_file if self.save_plot else "disabled",
            self.csv_file if self.save_csv else "disabled",
            "enabled" if self.realtime_plot else "disabled",
            "all" if self.max_history_points is None else self.max_history_points,
        )

    def _measurement_cb(self, msg):
        if self.model_index is None:
            try:
                self.model_index = msg.name.index(self.model_name)
            except ValueError:
                rospy.logwarn_throttle(
                    5.0,
                    "[inovation_sim] model '%s' not found on %s",
                    self.model_name,
                    rospy.resolve_name(self.model_states_topic),
                )
                return

        if self.model_index >= len(msg.pose) or self.model_index >= len(msg.twist):
            rospy.logwarn_throttle(
                5.0,
                "[inovation_sim] model index out of range, waiting next sample.",
            )
            self.model_index = None
            return

        pose = msg.pose[self.model_index]
        twist = msg.twist[self.model_index]
        stamp = rospy.Time.now()

        state = self._build_measurement_state(pose, twist, stamp)
        self.measurement_state = state
        self._append_history(
            self.measurement_history,
            "measurement",
            stamp,
            state,
        )

    def _prediction_cb(self, msg):
        pose = msg.pose.pose
        twist = msg.twist.twist
        stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0.0 else rospy.Time.now()

        state = self._build_prediction_state(pose, twist, stamp)
        self.prediction_state = state
        self._append_history(
            self.prediction_history,
            "prediction",
            stamp,
            state,
        )

    def _build_measurement_state(self, pose, twist, stamp):
        position_world = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
        )
        rotation_world = quaternion_to_matrix(pose.orientation)

        if self.measurement_reference is None:
            self.measurement_reference = {
                "position": position_world,
                "rotation": rotation_world,
            }

        reference_rotation_t = transpose_matrix(self.measurement_reference["rotation"])
        position_local = rotate_vector(
            reference_rotation_t,
            vector_subtract(position_world, self.measurement_reference["position"]),
        )
        relative_rotation = multiply_matrices(reference_rotation_t, rotation_world)
        angle_local = matrix_to_rpy(relative_rotation)

        velocity_local = rotate_vector(
            reference_rotation_t,
            (
                twist.linear.x,
                twist.linear.y,
                twist.linear.z,
            ),
        )
        angle_velocity_local = rotate_vector(
            reference_rotation_t,
            (
                twist.angular.x,
                twist.angular.y,
                twist.angular.z,
            ),
        )
        acceleration_local = self._differentiate_vector(
            velocity_local,
            stamp,
            self.measurement_prev_velocity,
            self.measurement_prev_stamp,
        )

        self.measurement_prev_velocity = velocity_local
        self.measurement_prev_stamp = stamp
        return {
            "stamp": stamp,
            "position": position_local,
            "angle": angle_local,
            "velocity": velocity_local,
            "angle_velocity": angle_velocity_local,
            "acceleration": acceleration_local,
        }

    def _build_prediction_state(self, pose, twist, stamp):
        position_world = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
        )
        rotation_world = quaternion_to_matrix(pose.orientation)

        if self.prediction_reference is None:
            self.prediction_reference = {
                "position": position_world,
                "rotation": rotation_world,
            }

        reference_rotation_t = transpose_matrix(self.prediction_reference["rotation"])
        position_local = rotate_vector(
            reference_rotation_t,
            vector_subtract(position_world, self.prediction_reference["position"]),
        )
        relative_rotation = multiply_matrices(reference_rotation_t, rotation_world)
        angle_local = matrix_to_rpy(relative_rotation)

        velocity_local = rotate_vector(
            relative_rotation,
            (
                twist.linear.x,
                twist.linear.y,
                twist.linear.z,
            ),
        )
        angle_velocity_local = rotate_vector(
            relative_rotation,
            (
                twist.angular.x,
                twist.angular.y,
                twist.angular.z,
            ),
        )
        acceleration_local = self._differentiate_vector(
            velocity_local,
            stamp,
            self.prediction_prev_velocity,
            self.prediction_prev_stamp,
        )

        self.prediction_prev_velocity = velocity_local
        self.prediction_prev_stamp = stamp
        return {
            "stamp": stamp,
            "position": position_local,
            "angle": angle_local,
            "velocity": velocity_local,
            "angle_velocity": angle_velocity_local,
            "acceleration": acceleration_local,
        }

    def _append_history(self, history, source, stamp, state):
        with self.history_lock:
            stamp_ns = stamp.to_nsec()
            if source == "measurement":
                if stamp_ns == self.last_measurement_stamp_ns:
                    return
                self.last_measurement_stamp_ns = stamp_ns
            else:
                if stamp_ns == self.last_prediction_stamp_ns:
                    return
                self.last_prediction_stamp_ns = stamp_ns

            if self.history_t0 is None:
                self.history_t0 = stamp

            relative_time = max(0.0, (stamp - self.history_t0).to_sec())
            history.append(
                {
                    "time": relative_time,
                    "position": state["position"],
                    "angle": state["angle"],
                    "velocity": state["velocity"],
                    "angle_velocity": state["angle_velocity"],
                    "acceleration": (
                        state["acceleration"]
                        if state["acceleration"] is not None
                        else nan_vector()
                    ),
                }
            )

    def _report_timer_cb(self, _event):
        if self.measurement_state is None or self.prediction_state is None:
            rospy.logwarn_throttle(
                5.0,
                "[inovation_sim] waiting for both Gazebo ground truth and EKF.",
            )
            return

        pair_dt = abs(
            (
                self.measurement_state["stamp"] - self.prediction_state["stamp"]
            ).to_sec()
        )
        if pair_dt > self.max_pair_dt:
            rospy.logwarn_throttle(
                5.0,
                "[inovation_sim] measurement/prediction dt too large: %.3f s",
                pair_dt,
            )
            return

        innovation_position = self._subtract_triplet(
            self.measurement_state["position"],
            self.prediction_state["position"],
        )
        innovation_angle = self._subtract_angles(
            self.measurement_state["angle"],
            self.prediction_state["angle"],
        )
        innovation_velocity = self._subtract_triplet(
            self.measurement_state["velocity"],
            self.prediction_state["velocity"],
        )
        innovation_angle_velocity = self._subtract_triplet(
            self.measurement_state["angle_velocity"],
            self.prediction_state["angle_velocity"],
        )
        if (
            self.measurement_state["acceleration"] is None
            or self.prediction_state["acceleration"] is None
        ):
            innovation_acceleration = nan_vector()
            rospy.logwarn_throttle(
                5.0,
                "[inovation_sim] acceleration needs one more sample, publishing NaN for now.",
            )
        else:
            innovation_acceleration = self._subtract_triplet(
                self.measurement_state["acceleration"],
                self.prediction_state["acceleration"],
            )

        stamp = rospy.Time.now()
        self.position_pub.publish(
            self._vector_msg(stamp, innovation_position, self.output_frame)
        )
        self.angle_pub.publish(
            self._vector_msg(stamp, innovation_angle, self.output_frame)
        )
        self.velocity_pub.publish(
            self._vector_msg(stamp, innovation_velocity, self.output_frame)
        )
        self.angle_velocity_pub.publish(
            self._vector_msg(stamp, innovation_angle_velocity, self.output_frame)
        )
        self.acceleration_pub.publish(
            self._vector_msg(stamp, innovation_acceleration, self.output_frame)
        )

        rospy.loginfo(
            "[inovation_sim] "
            "pos=(%.3f, %.3f, %.3f)m | "
            "angle=(%.3f, %.3f, %.3f)rad | "
            "vel=(%.3f, %.3f, %.3f)m/s | "
            "w=(%.3f, %.3f, %.3f)rad/s | "
            "acc=(%.3f, %.3f, %.3f)m/s^2",
            innovation_position[0],
            innovation_position[1],
            innovation_position[2],
            innovation_angle[0],
            innovation_angle[1],
            innovation_angle[2],
            innovation_velocity[0],
            innovation_velocity[1],
            innovation_velocity[2],
            innovation_angle_velocity[0],
            innovation_angle_velocity[1],
            innovation_angle_velocity[2],
            innovation_acceleration[0],
            innovation_acceleration[1],
            innovation_acceleration[2],
        )

    def _initialize_plotting(self):
        if self.realtime_plot:
            try:
                self._setup_realtime_plot()
            except Exception as exc:
                self.realtime_plot = False
                self.matplotlib = None
                self.plt = None
                rospy.logerr(
                    "[inovation_sim] realtime plot disabled: %s",
                    exc,
                )
                if not self.save_plot and not self.show_plot:
                    raise

        if self.plt is None and (self.save_plot or self.show_plot):
            self._ensure_plot_modules(self.plot_backend)

    def _ensure_plot_modules(self, backend):
        if self.plt is not None:
            return self.plt

        import matplotlib

        matplotlib.use(backend)
        import matplotlib.pyplot as plt

        self.matplotlib = matplotlib
        self.plt = plt
        return plt

    def _setup_realtime_plot(self):
        if self.realtime_plot_hz <= 0.0:
            raise ValueError("realtime_plot_hz must be > 0")

        plt = self._ensure_plot_modules(self.realtime_plot_backend)
        plt.ion()

        figure, axes = plt.subplots(
            len(PLOT_ROWS),
            3,
            figsize=(14, 10),
            sharex=True,
        )
        figure.subplots_adjust(
            left=0.055,
            right=0.99,
            bottom=0.055,
            top=0.93,
            hspace=0.62,
            wspace=0.25,
        )
        figure.suptitle(
            "Measurement vs Prediction - Inovation Sim (Realtime)",
            fontsize=15,
        )

        self.realtime_figure = figure
        self.realtime_axes = axes
        self.realtime_lines = {}
        for row_index, (key, title, unit) in enumerate(PLOT_ROWS):
            self.realtime_lines[key] = self._create_triplet_row(
                axes[row_index],
                title,
                unit,
                COMPONENT_LABELS[key],
            )

        self._enable_time_axis_on_all_plots(axes)

        figure.canvas.draw()
        figure.canvas.flush_events()
        plt.show(block=False)

    def _create_triplet_row(self, axes, title, unit, labels):
        row_lines = []
        for index, label in enumerate(labels):
            axis = axes[index]
            lines = {
                "measurement": axis.plot(
                    [],
                    [],
                    color=MEASUREMENT_COLOR,
                    linestyle="-",
                    linewidth=1.9,
                    label="measurement",
                )[0],
                "prediction": axis.plot(
                    [],
                    [],
                    color=PREDICTION_COLOR,
                    linestyle="--",
                    linewidth=1.7,
                    label="prediction",
                )[0],
            }
            axis.set_title("{} {}".format(title, label))
            axis.set_ylabel(unit)
            axis.grid(True, alpha=0.35)
            axis.legend(loc="upper right")
            row_lines.append(lines)

        return row_lines

    def _enable_time_axis_on_all_plots(self, axes):
        for axis_row in axes:
            for axis in axis_row:
                axis.set_xlabel("Time [s]")
                axis.tick_params(axis="x", labelbottom=True)

    def _update_realtime_plot(self):
        if not self.realtime_plot or self.realtime_figure is None:
            return

        if not self.plt.fignum_exists(self.realtime_figure.number):
            self.realtime_plot = False
            if not self.realtime_plot_closed_logged:
                rospy.logwarn("[inovation_sim] realtime plot window closed by user.")
                self.realtime_plot_closed_logged = True
            return

        measurement_history, prediction_history = self._history_snapshot()
        if not measurement_history and not prediction_history:
            return
        if (
            len(measurement_history) == self.last_realtime_measurement_count
            and len(prediction_history) == self.last_realtime_prediction_count
        ):
            return
        self.last_realtime_measurement_count = len(measurement_history)
        self.last_realtime_prediction_count = len(prediction_history)

        current_time = self._latest_history_time(
            measurement_history,
            prediction_history,
        )
        measurement_history = self._realtime_display_history(
            measurement_history,
            current_time,
        )
        prediction_history = self._realtime_display_history(
            prediction_history,
            current_time,
        )

        for row_index, (key, _title, _unit) in enumerate(PLOT_ROWS):
            self._update_triplet_row(
                self.realtime_axes[row_index],
                self.realtime_lines[key],
                measurement_history,
                prediction_history,
                key,
                current_time,
            )

        self.realtime_figure.canvas.draw()
        self.realtime_figure.canvas.flush_events()

    def _latest_history_time(self, *histories):
        latest_time = 0.0
        for history in histories:
            if history:
                latest_time = max(latest_time, history[-1]["time"])
        return latest_time

    def _realtime_display_history(self, history, current_time):
        if not history:
            return []

        if self.realtime_window_sec > 0.0:
            min_time = max(0.0, current_time - self.realtime_window_sec)
            visible = [entry for entry in history if entry["time"] >= min_time]
        else:
            visible = list(history)

        if len(visible) <= self.realtime_max_points:
            return visible

        step = int(math.ceil(float(len(visible)) / self.realtime_max_points))
        thinned = visible[::step]
        if thinned[-1]["time"] != visible[-1]["time"]:
            thinned.append(visible[-1])
        return thinned

    def _update_triplet_row(
        self, axes, row_lines, measurement_history, prediction_history, key, current_time
    ):
        measurement_times = [entry["time"] for entry in measurement_history]
        prediction_times = [entry["time"] for entry in prediction_history]
        measurement_series = [entry[key] for entry in measurement_history]
        prediction_series = [entry[key] for entry in prediction_history]
        x_min, x_max = self._realtime_x_limits(current_time)

        for index, axis in enumerate(axes):
            measurement_values = [value[index] for value in measurement_series]
            prediction_values = [value[index] for value in prediction_series]
            row_lines[index]["measurement"].set_data(measurement_times, measurement_values)
            row_lines[index]["prediction"].set_data(prediction_times, prediction_values)
            axis.set_xlim(x_min, x_max)
            self._set_realtime_y_limits(axis, measurement_values, prediction_values)

    def _realtime_x_limits(self, current_time):
        if self.realtime_window_sec > 0.0:
            if current_time <= self.realtime_window_sec:
                return 0.0, self.realtime_window_sec
            return current_time - self.realtime_window_sec, current_time

        return 0.0, max(1.0, current_time)

    def _set_realtime_y_limits(self, axis, *series_list):
        finite_values = []
        for series in series_list:
            finite_values.extend(value for value in series if math.isfinite(value))

        if not finite_values:
            return

        data_min = min(finite_values)
        data_max = max(finite_values)
        if data_min == data_max:
            padding = max(1e-3, abs(data_min) * 0.1)
        else:
            padding = (data_max - data_min) * 0.15

        y_min = data_min - padding
        y_max = data_max + padding
        axis_id = id(axis)
        if axis_id in self.realtime_y_limits:
            old_min, old_max = self.realtime_y_limits[axis_id]
            y_min = min(old_min, y_min)
            y_max = max(old_max, y_max)

        self.realtime_y_limits[axis_id] = (y_min, y_max)
        axis.set_ylim(y_min, y_max)

    def run(self):
        if self.realtime_plot and self.realtime_figure is not None:
            draw_pause = max(0.001, 1.0 / self.realtime_plot_hz)
            while not rospy.is_shutdown() and self.realtime_plot:
                self._update_realtime_plot()
                self.plt.pause(draw_pause)

            if not rospy.is_shutdown():
                rospy.spin()
            return

        rospy.spin()

    def _on_shutdown(self):
        if not self.save_plot and not self.show_plot and not self.save_csv:
            return

        measurement_history, prediction_history = self._history_snapshot()
        if not measurement_history and not prediction_history:
            rospy.logwarn("[inovation_sim] no history collected, skip saving.")
            return

        if self.save_csv:
            try:
                self._save_history_csv(measurement_history, prediction_history)
            except Exception as exc:
                rospy.logwarn("[inovation_sim] failed to save csv: %s", exc)

        if self.save_plot or self.show_plot:
            try:
                self._plot_measurement_vs_prediction(
                    measurement_history,
                    prediction_history,
                )
            except Exception as exc:
                rospy.logwarn("[inovation_sim] failed to create plot: %s", exc)

    def _history_snapshot(self):
        with self.history_lock:
            return list(self.measurement_history), list(self.prediction_history)

    def _save_history_csv(self, measurement_history, prediction_history):
        csv_dir = os.path.dirname(self.csv_file)
        if csv_dir:
            os.makedirs(csv_dir, exist_ok=True)

        with open(self.csv_file, "w", newline="") as csv_handle:
            writer = csv.writer(csv_handle)
            writer.writerow(
                [
                    "source",
                    "time",
                    "position_x",
                    "position_y",
                    "position_z",
                    "angle_roll",
                    "angle_pitch",
                    "angle_yaw",
                    "velocity_x",
                    "velocity_y",
                    "velocity_z",
                    "angular_velocity_x",
                    "angular_velocity_y",
                    "angular_velocity_z",
                    "acceleration_x",
                    "acceleration_y",
                    "acceleration_z",
                ]
            )

            for source, history in (
                ("measurement", measurement_history),
                ("prediction", prediction_history),
            ):
                for entry in history:
                    writer.writerow(
                        [
                            source,
                            entry["time"],
                            entry["position"][0],
                            entry["position"][1],
                            entry["position"][2],
                            entry["angle"][0],
                            entry["angle"][1],
                            entry["angle"][2],
                            entry["velocity"][0],
                            entry["velocity"][1],
                            entry["velocity"][2],
                            entry["angle_velocity"][0],
                            entry["angle_velocity"][1],
                            entry["angle_velocity"][2],
                            entry["acceleration"][0],
                            entry["acceleration"][1],
                            entry["acceleration"][2],
                        ]
                    )

        rospy.loginfo("[inovation_sim] saved csv: %s", self.csv_file)

    def _plot_measurement_vs_prediction(self, measurement_history, prediction_history):
        plt = self._ensure_plot_modules(self.plot_backend)

        figure, axes = plt.subplots(
            len(PLOT_ROWS),
            3,
            figsize=(18, 14),
            sharex=True,
            constrained_layout=True,
        )
        figure.suptitle("Measurement vs Prediction - Inovation Sim", fontsize=15)

        for row_index, (key, title, unit) in enumerate(PLOT_ROWS):
            self._plot_triplet_row(
                axes[row_index],
                measurement_history,
                prediction_history,
                key,
                title,
                unit,
                COMPONENT_LABELS[key],
            )

        self._enable_time_axis_on_all_plots(axes)

        if self.save_plot:
            plot_dir = os.path.dirname(self.plot_file)
            if plot_dir:
                os.makedirs(plot_dir, exist_ok=True)
            figure.savefig(self.plot_file, dpi=150, bbox_inches="tight")
            rospy.loginfo("[inovation_sim] saved plot: %s", self.plot_file)

        if self.show_plot:
            plt.show()
        else:
            plt.close(figure)

    def _plot_triplet_row(
        self,
        axes,
        measurement_history,
        prediction_history,
        key,
        title,
        unit,
        labels,
    ):
        measurement_times = [entry["time"] for entry in measurement_history]
        prediction_times = [entry["time"] for entry in prediction_history]
        measurement_series = [entry[key] for entry in measurement_history]
        prediction_series = [entry[key] for entry in prediction_history]

        for index, label in enumerate(labels):
            axis = axes[index]
            axis.plot(
                measurement_times,
                [value[index] for value in measurement_series],
                color=MEASUREMENT_COLOR,
                linestyle="-",
                linewidth=1.9,
                label="measurement",
            )
            axis.plot(
                prediction_times,
                [value[index] for value in prediction_series],
                color=PREDICTION_COLOR,
                linestyle="--",
                linewidth=1.7,
                label="prediction",
            )
            axis.set_title("{} {}".format(title, label))
            axis.set_ylabel(unit)
            axis.grid(True, alpha=0.35)
            axis.legend(loc="upper right")

    def _differentiate_vector(self, current_vector, current_stamp, prev_vector, prev_stamp):
        if prev_vector is None or prev_stamp is None:
            return None

        delta_t = (current_stamp - prev_stamp).to_sec()
        if delta_t <= 0.0:
            return None

        return tuple(
            (current_vector[index] - prev_vector[index]) / delta_t
            for index in range(3)
        )

    def _subtract_triplet(self, measurement, prediction):
        return tuple(
            measurement[index] - prediction[index]
            for index in range(3)
        )

    def _subtract_angles(self, measurement, prediction):
        return tuple(
            wrap_to_pi(measurement[index] - prediction[index])
            for index in range(3)
        )

    def _vector_msg(self, stamp, vector, frame_id):
        msg = Vector3Stamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.vector.x = vector[0]
        msg.vector.y = vector[1]
        msg.vector.z = vector[2]
        return msg


if __name__ == "__main__":
    try:
        node = InovationSim()
        node.run()
    except rospy.ROSInterruptException:
        pass
