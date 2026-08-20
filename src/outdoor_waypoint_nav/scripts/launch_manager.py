#!/usr/bin/env python3
import json
import os
import signal
import subprocess

import rospy
from std_msgs.msg import String

COMMAND_TOPIC = "/hmi/launch_command"
STATUS_TOPIC = "/hmi/launch_status"
WORKSPACE_SETUP = "/home/cuong/warthog_full/devel/setup.bash"
WORKSPACE_SETUP_EKF = "/home/cuong/warthog/devel/setup.bash"


LAUNCH_MAP = {
    "aloam_velodyne_HDL_32": [
        "bash",
        "-lc",
        "source {} && roslaunch aloam_velodyne aloam_velodyne_HDL_32.launch".format(
            WORKSPACE_SETUP
        ),
    ],
    "outdoor_waypoint_nav": [
        "bash",                                      
        "-lc",
        "source {} && roslaunch outdoor_waypoint_nav outdoor_waypoint_nav.launch".format(
            WORKSPACE_SETUP_EKF
        ),
    ],
    "imu_start_calib": [
        "bash",                                      
        "-lc",
        "source {} && roslaunch imu_pkg start_calib.launch".format(
            WORKSPACE_SETUP_EKF
        ),
    ],
    "outdoor_waypoint_nav_sim": [
        "bash",
        "-lc",
        "source {} && roslaunch outdoor_waypoint_nav outdoor_waypoint_nav_sim.launch".format(
            WORKSPACE_SETUP_EKF
        ),
    ],
    "joy_launch_control_sim": [
        "bash",
        "-lc",
        "source {} && roslaunch outdoor_waypoint_nav joy_launch_control_sim.launch".format(
            WORKSPACE_SETUP_EKF
        ),
    ],
    "outdoor_waypoint_nav_tracking": [
        "bash",
        "-lc",
        "source {} && rosrun outdoor_waypoint_nav tracking_control.py".format(
            WORKSPACE_SETUP_EKF
        ),
    ],
    "rslidar_start": [
        "bash",
        "-lc",
        "source {} && roslaunch rslidar_sdk start.launch".format(WORKSPACE_SETUP),
    ],
    "save_aloam_2d_map": [
        "bash",
        "-lc",
        "source {} && roslaunch export_2d_map save_aloam_2d_map.launch".format(
            WORKSPACE_SETUP
        ),
    ],
    "warthog_real_amcl_checked": [
        "bash",
        "-lc",
        "source {} && roslaunch export_2d_map warthog_real_amcl_checked.launch use_rf2o_odom:=false wheel_odom_topic:=/odometry/filtered publish_wheel_odom_tf:=false publish_lidar_static_tf:=false".format(
            WORKSPACE_SETUP
        ),
    ],
}


class LaunchManager:
    def __init__(self):
        default_machine_name = "cuong"
        self.machine_name = rospy.get_param("~machine_name", default_machine_name)
        self.machine_name = self.machine_name.strip()
        self.processes = {}

        self.status_pub = rospy.Publisher(STATUS_TOPIC, String, queue_size=20)
        self.command_sub = rospy.Subscriber(
            COMMAND_TOPIC, String, self.handle_command, queue_size=20
        )
        self.process_timer = rospy.Timer(rospy.Duration(1.0), self.check_processes)

        rospy.on_shutdown(self.stop_all)
        rospy.loginfo(
            "Launch manager ready: machine_name=%s topic=%s",
            self.machine_name,
            COMMAND_TOPIC,
        )

    def publish_status(self, payload):
        payload["machine_name"] = self.machine_name
        payload["timestamp"] = rospy.Time.now().to_sec()
        self.status_pub.publish(String(data=json.dumps(payload)))

    def matches_target(self, target_machine):
        target = str(target_machine or "").strip()
        if target in ("*", "all"):
            return True

        return target == self.machine_name

    def cleanup_exited_process(self, launch_name):
        process = self.processes.get(launch_name)
        if not process:
            return

        return_code = process.poll()
        if return_code is not None:
            self.processes.pop(launch_name, None)
            self.publish_status(
                {
                    "event": "exited",
                    "launch": launch_name,
                    "return_code": return_code,
                }
            )

    def check_processes(self, _event):
        for launch_name in list(self.processes.keys()):
            self.cleanup_exited_process(launch_name)

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

        if launch_name not in LAUNCH_MAP:
            self.publish_status(
                {
                    "event": "rejected",
                    "reason": "launch_not_whitelisted",
                    "launch": launch_name,
                    "request_id": request_id,
                }
            )
            return

        if launch_name in self.processes:
            self.publish_status(
                {
                    "event": "already_running",
                    "launch": launch_name,
                    "pid": self.processes[launch_name].pid,
                    "request_id": request_id,
                }
            )
            return

        command = LAUNCH_MAP[launch_name]
        log_path = "/tmp/hmi_launch_{}.log".format(launch_name)
        log_file = open(log_path, "a")

        rospy.loginfo("Starting launch %s: %s", launch_name, " ".join(command))
        process = subprocess.Popen(
            command,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
        self.processes[launch_name] = process

        self.publish_status(
            {
                "event": "started",
                "launch": launch_name,
                "pid": process.pid,
                "log_path": log_path,
                "request_id": request_id,
            }
        )

    def stop_launch(self, launch_name, request_id=""):
        self.cleanup_exited_process(launch_name)

        process = self.processes.get(launch_name)
        if not process:
            self.publish_status(
                {
                    "event": "not_running",
                    "launch": launch_name,
                    "request_id": request_id,
                }
            )
            return

        rospy.loginfo("Stopping launch %s pid=%s", launch_name, process.pid)
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            process.wait(timeout=5)
        finally:
            self.processes.pop(launch_name, None)

        self.publish_status(
            {
                "event": "stopped",
                "launch": launch_name,
                "request_id": request_id,
            }
        )

    def stop_all(self):
        for launch_name in list(self.processes.keys()):
            self.stop_launch(launch_name)


if __name__ == "__main__":
    rospy.init_node("hmi_launch_manager")
    LaunchManager()
    rospy.spin()
