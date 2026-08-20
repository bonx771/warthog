#!/usr/bin/env python3
import json
import os
import shlex
import signal
import socket
import subprocess
import threading

import rospy
from std_msgs.msg import String


COMMAND_TOPIC = "/hmi/launch_command"
STATUS_TOPIC = "/hmi/launch_status"

DEFAULT_ROS_MASTER_URI = "http://192.168.166.1:11311"
DEFAULT_ROS_IP = "192.168.166.100"
DEFAULT_MACHINE_NAME = "cuong"
DEFAULT_MAIN_SETUP = "/home/cuong/warthog_full/devel/setup.bash"
DEFAULT_EKF_SETUP = "/home/cuong/warthog/devel/setup.bash"

# Giup node nay van ket noi dung ROS master tren may .166.100
# ngay ca khi ban quen export bien moi truong truoc khi chay script.
os.environ.setdefault("ROS_MASTER_URI", DEFAULT_ROS_MASTER_URI)
os.environ.setdefault("ROS_IP", DEFAULT_ROS_IP)
os.environ.setdefault("ROS_HOSTNAME", os.environ.get("ROS_IP", DEFAULT_ROS_IP))

LAUNCH_SPECS = {
    "aloam_velodyne_HDL_32": {
        "setup_key": "main",
        "command": "roslaunch aloam_velodyne aloam_velodyne_HDL_32.launch",
    },
    "outdoor_waypoint_nav": {
        "setup_key": "ekf",
        "command": "roslaunch outdoor_waypoint_nav outdoor_waypoint_nav.launch",
    },
    "imu_start_calib": {
        "setup_key": "ekf",
        "command": "roslaunch imu_pkg start_calib.launch",
    },
    "outdoor_waypoint_nav_sim": {
        "setup_key": "ekf",
        "command": "roslaunch outdoor_waypoint_nav outdoor_waypoint_nav_sim.launch",
    },
    "joy_launch_control_sim": {
        "setup_key": "ekf",
        "command": "roslaunch outdoor_waypoint_nav joy_launch_control_sim.launch",
    },
    "outdoor_waypoint_nav_tracking": {
        "setup_key": "ekf",
        "command": "rosrun outdoor_waypoint_nav tracking_control.py",
    },
    "rslidar_start": {
        "setup_key": "main",
        "command": "roslaunch rslidar_sdk start.launch",
    },
    "save_aloam_2d_map": {
        "setup_key": "main",
        "command": "roslaunch export_2d_map save_aloam_2d_map.launch",
    },
    "warthog_real_amcl_checked": {
        "setup_key": "main",
        "command": (
            "roslaunch export_2d_map warthog_real_amcl_checked.launch "
            "use_rf2o_odom:=false "
            "wheel_odom_topic:=/odometry/filtered "
            "publish_wheel_odom_tf:=false "
            "publish_lidar_static_tf:=false"
        ),
    },
}


class LaunchManager:
    def __init__(self):
        self.hostname = socket.gethostname().strip()
        self.machine_name = str(
            rospy.get_param("~machine_name", DEFAULT_MACHINE_NAME)
        ).strip()

        raw_aliases = rospy.get_param(
            "~machine_aliases",
            [
                DEFAULT_MACHINE_NAME,
                DEFAULT_ROS_IP,
                "166.100",
                self.hostname,
            ],
        )
        if isinstance(raw_aliases, str):
            raw_aliases = [
                item.strip() for item in raw_aliases.split(",") if item.strip()
            ]

        self.machine_aliases = {
            alias.strip()
            for alias in raw_aliases
            if isinstance(alias, str) and alias.strip()
        }
        self.machine_aliases.add(self.machine_name)
        self.machine_aliases.add(self.hostname)

        self.workspace_setup_main = str(
            rospy.get_param("~workspace_setup_main", DEFAULT_MAIN_SETUP)
        ).strip()
        self.workspace_setup_ekf = str(
            rospy.get_param("~workspace_setup_ekf", DEFAULT_EKF_SETUP)
        ).strip()

        self.ros_master_uri = os.environ.get("ROS_MASTER_URI", DEFAULT_ROS_MASTER_URI)
        self.ros_ip = os.environ.get("ROS_IP", DEFAULT_ROS_IP)
        self.ros_hostname = os.environ.get("ROS_HOSTNAME", self.ros_ip)

        self.processes = {}
        self.process_lock = threading.Lock()

        self.status_pub = rospy.Publisher(STATUS_TOPIC, String, queue_size=20)
        self.command_sub = rospy.Subscriber(
            COMMAND_TOPIC, String, self.handle_command, queue_size=20
        )

        rospy.on_shutdown(self.stop_all)
        rospy.loginfo(
            "Launch manager ready: machine_name=%s aliases=%s ROS_MASTER_URI=%s ROS_IP=%s",
            self.machine_name,
            sorted(self.machine_aliases),
            self.ros_master_uri,
            self.ros_ip,
        )

    def publish_status(self, payload):
        payload["machine_name"] = self.machine_name
        payload["machine_aliases"] = sorted(self.machine_aliases)
        payload["timestamp"] = rospy.Time.now().to_sec()
        self.status_pub.publish(String(data=json.dumps(payload)))

    def matches_target(self, target_machine):
        target = str(target_machine or "").strip()
        if target in ("*", "all"):
            return True

        return target in self.machine_aliases

    def get_setup_path(self, setup_key):
        if setup_key == "main":
            return self.workspace_setup_main
        if setup_key == "ekf":
            return self.workspace_setup_ekf
        raise ValueError("Khong biet setup key: {}".format(setup_key))

    def build_launch_command(self, launch_name):
        spec = LAUNCH_SPECS.get(launch_name)
        if not spec:
            raise KeyError(launch_name)

        setup_path = self.get_setup_path(spec["setup_key"])
        if not os.path.exists(setup_path):
            raise FileNotFoundError(
                "Khong tim thay file setup: {}. Hay sua lai bien DEFAULT_*_SETUP hoac ROS param."
                .format(setup_path)
            )

        script = "\n".join(
            [
                "set -e",
                "source {}".format(shlex.quote(setup_path)),
                "export ROS_MASTER_URI={}".format(shlex.quote(self.ros_master_uri)),
                "export ROS_IP={}".format(shlex.quote(self.ros_ip)),
                "export ROS_HOSTNAME={}".format(shlex.quote(self.ros_hostname)),
                "exec {}".format(spec["command"]),
            ]
        )

        command = ["bash", "-lc", script]
        command_display = "source {} && {}".format(setup_path, spec["command"])
        return command, command_display

    def read_log_tail(self, log_path, max_lines=20):
        if not log_path or not os.path.exists(log_path):
            return ""

        try:
            with open(log_path, "r") as log_file:
                lines = log_file.readlines()
        except OSError as error:
            return "Khong doc duoc log: {}".format(error)

        return "".join(lines[-max_lines:]).strip()

    def close_launch_log(self, launch_info):
        log_file = launch_info.get("log_file")
        if not log_file or log_file.closed:
            return

        try:
            log_file.flush()
        except OSError:
            pass

        try:
            log_file.close()
        except OSError:
            pass

    def get_launch_info(self, launch_name):
        with self.process_lock:
            return self.processes.get(launch_name)

    def cleanup_exited_process(self, launch_name):
        launch_info = self.get_launch_info(launch_name)
        if not launch_info:
            return

        process = launch_info["process"]
        return_code = process.poll()
        if return_code is None:
            return

        with self.process_lock:
            current_info = self.processes.get(launch_name)
            if current_info is not launch_info:
                return
            self.processes.pop(launch_name, None)

        self.close_launch_log(launch_info)
        self.publish_status(
            {
                "event": "exited",
                "launch": launch_name,
                "return_code": return_code,
                "pid": process.pid,
                "log_path": launch_info.get("log_path"),
                "log_tail": self.read_log_tail(launch_info.get("log_path")),
                "command": launch_info.get("command_display", ""),
            }
        )

    def watch_process(self, launch_name, process):
        return_code = process.wait()

        with self.process_lock:
            launch_info = self.processes.get(launch_name)
            if not launch_info or launch_info["process"] is not process:
                return

            expected_stop = launch_info.get("expected_stop", False)
            self.processes.pop(launch_name, None)

        self.close_launch_log(launch_info)
        if expected_stop:
            return

        self.publish_status(
            {
                "event": "exited",
                "launch": launch_name,
                "return_code": return_code,
                "pid": process.pid,
                "log_path": launch_info.get("log_path"),
                "log_tail": self.read_log_tail(launch_info.get("log_path")),
                "command": launch_info.get("command_display", ""),
            }
        )

    def handle_command(self, msg):
        try:
            payload = json.loads(msg.data)
        except ValueError:
            rospy.logwarn("Invalid launch command JSON: %s", msg.data)
            return

        target_machine = payload.get("target_machine")
        if not self.matches_target(target_machine):
            return

        action = str(payload.get("action", "")).strip().lower()
        launch_name = str(payload.get("launch", "")).strip()
        request_id = payload.get("request_id", "")

        if action == "start":
            self.start_launch(launch_name, request_id)
            return

        if action == "stop":
            self.stop_launch(launch_name, request_id)
            return

        self.publish_status(
            {
                "event": "rejected",
                "reason": "unknown_action",
                "action": action,
                "launch": launch_name,
                "request_id": request_id,
            }
        )

    def start_launch(self, launch_name, request_id=""):
        self.cleanup_exited_process(launch_name)

        if launch_name not in LAUNCH_SPECS:
            self.publish_status(
                {
                    "event": "rejected",
                    "reason": "launch_not_whitelisted",
                    "launch": launch_name,
                    "request_id": request_id,
                }
            )
            return

        running_info = self.get_launch_info(launch_name)
        if running_info:
            self.publish_status(
                {
                    "event": "already_running",
                    "launch": launch_name,
                    "pid": running_info["process"].pid,
                    "log_path": running_info.get("log_path"),
                    "request_id": request_id,
                }
            )
            return

        try:
            command, command_display = self.build_launch_command(launch_name)
        except (FileNotFoundError, ValueError) as error:
            self.publish_status(
                {
                    "event": "start_failed",
                    "launch": launch_name,
                    "request_id": request_id,
                    "error": str(error),
                }
            )
            return

        log_path = "/tmp/hmi_launch_{}.log".format(launch_name)
        try:
            log_file = open(log_path, "a")
        except OSError as error:
            self.publish_status(
                {
                    "event": "start_failed",
                    "launch": launch_name,
                    "request_id": request_id,
                    "error": "Khong mo duoc file log: {}".format(error),
                    "command": command_display,
                    "log_path": log_path,
                }
            )
            return

        env = os.environ.copy()
        env["ROS_MASTER_URI"] = self.ros_master_uri
        env["ROS_IP"] = self.ros_ip
        env["ROS_HOSTNAME"] = self.ros_hostname

        rospy.loginfo("Starting launch %s: %s", launch_name, command_display)
        try:
            process = subprocess.Popen(
                command,
                stdout=log_file,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,
                env=env,
            )
        except OSError as error:
            self.close_launch_log({"log_file": log_file})
            self.publish_status(
                {
                    "event": "start_failed",
                    "launch": launch_name,
                    "request_id": request_id,
                    "error": str(error),
                    "command": command_display,
                    "log_path": log_path,
                    "log_tail": self.read_log_tail(log_path),
                }
            )
            return

        launch_info = {
            "process": process,
            "log_file": log_file,
            "log_path": log_path,
            "command_display": command_display,
            "expected_stop": False,
        }

        with self.process_lock:
            self.processes[launch_name] = launch_info

        watcher = threading.Thread(target=self.watch_process, args=(launch_name, process))
        watcher.daemon = True
        watcher.start()

        self.publish_status(
            {
                "event": "started",
                "launch": launch_name,
                "pid": process.pid,
                "log_path": log_path,
                "command": command_display,
                "request_id": request_id,
            }
        )

    def stop_launch(self, launch_name, request_id=""):
        self.cleanup_exited_process(launch_name)

        launch_info = self.get_launch_info(launch_name)
        if not launch_info:
            self.publish_status(
                {
                    "event": "not_running",
                    "launch": launch_name,
                    "request_id": request_id,
                }
            )
            return

        process = launch_info["process"]
        with self.process_lock:
            current_info = self.processes.get(launch_name)
            if current_info:
                current_info["expected_stop"] = True

        rospy.loginfo("Stopping launch %s pid=%s", launch_name, process.pid)
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            process.wait(timeout=5)
        except ProcessLookupError:
            pass
        finally:
            with self.process_lock:
                self.processes.pop(launch_name, None)

        self.close_launch_log(launch_info)
        self.publish_status(
            {
                "event": "stopped",
                "launch": launch_name,
                "log_path": launch_info.get("log_path"),
                "request_id": request_id,
            }
        )

    def stop_all(self):
        for launch_name in list(self.processes.keys()):
            self.stop_launch(launch_name)


if __name__ == "__main__":
    rospy.init_node("hmi_launch_manager_166_100")
    LaunchManager()
    rospy.spin()
