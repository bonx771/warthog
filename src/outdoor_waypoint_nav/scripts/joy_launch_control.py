#!/usr/bin/env python3
import rospy
import roslaunch
import rospkg
import os
import signal
import subprocess
import sys
import threading
import textwrap
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

# Initialize variables

button_events = [False, False, False, False, False]
main_prev_buttons = [0, 0, 0, 0, 0]
keyboard_prev_buttons = [0, 0, 0, 0, 0]
collect_btn_num = 0
collect_btn_sym = ""
send_btn_num = 0
send_btn_sym = ""
calibrate_btn_num = 0
calibrate_btn_sym = ""
abort_btn_num = 0
abort_btn_sym = ""
sim_enabled = False
keyboard_waypoint_control_enabled = False
keyboard_joy_topic = "/outdoor_waypoint_nav/keyboard_joy"

location_collect = ""
location_send = ""
location_calibrate = ""
location_safety_node = ""

calibrate_complete = False
collect_complete = False
send_complete = False
velocity_paused = False
launch_process = None
launch_label = ""
launch_output_thread = None
cmd_vel_pub = None
cmd_vel_intermediate_pub = None
warthog_cmd_vel_pub = None

def getParameter():
    global collect_btn_num
    global collect_btn_sym
    global send_btn_num
    global send_btn_sym
    global calibrate_btn_num
    global calibrate_btn_sym
    global abort_btn_num
    global abort_btn_sym
    global continue_btn_num
    global continue_btn_sym
    global sim_enabled
    global keyboard_waypoint_control_enabled
    global keyboard_joy_topic

    collect_btn_num = rospy.get_param("/outdoor_waypoint_nav/collect_button_num")
    collect_btn_sym = rospy.get_param("/outdoor_waypoint_nav/collect_button_sym")
    send_btn_num = rospy.get_param("/outdoor_waypoint_nav/send_button_num")
    send_btn_sym = rospy.get_param("/outdoor_waypoint_nav/send_button_sym")
    calibrate_btn_num = rospy.get_param("/outdoor_waypoint_nav/calibrate_button_num")
    calibrate_btn_sym = rospy.get_param("/outdoor_waypoint_nav/calibrate_button_sym")
    abort_btn_num = rospy.get_param("/outdoor_waypoint_nav/abort_button_num")
    abort_btn_sym = rospy.get_param("/outdoor_waypoint_nav/abort_button_sym")
    continue_btn_num = rospy.get_param("/outdoor_waypoint_nav/continue_button_num")
    continue_btn_sym = rospy.get_param("/outdoor_waypoint_nav/continue_button_sym")
    
    sim_enabled = rospy.get_param("/outdoor_waypoint_nav/sim_enabled")
    keyboard_waypoint_control_enabled = rospy.get_param("/outdoor_waypoint_nav/keyboard_waypoint_control_enabled", False)
    keyboard_joy_topic = rospy.get_param("/outdoor_waypoint_nav/keyboard_joy_topic", "/outdoor_waypoint_nav/keyboard_joy")

def getPaths():
    global location_collect
    global location_send
    global location_calibrate
    global location_safety_node
    rospack = rospkg.RosPack()
    
    # Define location of launch files
    if sim_enabled == True:
        location_collect = rospack.get_path('outdoor_waypoint_nav') + "/launch/include/collect_goals_sim.launch"
        location_send = rospack.get_path('outdoor_waypoint_nav') + "/launch/include/send_goals_sim.launch"
        location_calibrate = rospack.get_path('outdoor_waypoint_nav') + "/launch/include/heading_calibration_sim.launch"
        location_safety_node = rospack.get_path('outdoor_waypoint_nav') + "/launch/include/safety_node.launch"

    elif sim_enabled == False:
        location_collect = rospack.get_path('outdoor_waypoint_nav') + "/launch/include/collect_goals.launch"
        location_send = rospack.get_path('outdoor_waypoint_nav') + "/launch/include/send_goals.launch"
        location_calibrate = rospack.get_path('outdoor_waypoint_nav') + "/launch/include/heading_calibration.launch"
        location_safety_node = rospack.get_path('outdoor_waypoint_nav') + "/launch/include/safety_node.launch"

    else:
        print("ERROR: PLEASE SPECIFY SIM_ENABLED PARAMETER.")

    # Scenario launchers can select a controller launch with the same direct
    # pursuit settings as the outdoor experiments. The legacy simulation
    # control keeps send_goals_sim.launch as its default.
    send_launch_override = str(
        rospy.get_param("/outdoor_waypoint_nav/send_goals_launch_file", "")
    ).strip()
    if send_launch_override:
        if not os.path.isabs(send_launch_override):
            send_launch_override = os.path.join(
                rospack.get_path("outdoor_waypoint_nav"), send_launch_override
            )
        location_send = send_launch_override

def _button_value(joy_msg, index):
    if index < len(joy_msg.buttons):
        return joy_msg.buttons[index]
    return 0

def _make_joy_cb(previous_state):
    def joy_cb(joy_msg):
        global button_events

        current_buttons = [
            _button_value(joy_msg, collect_btn_num),
            _button_value(joy_msg, send_btn_num),
            _button_value(joy_msg, calibrate_btn_num),
            _button_value(joy_msg, abort_btn_num),
            _button_value(joy_msg, continue_btn_num),
        ]

        for idx, current_value in enumerate(current_buttons):
            if current_value == 1 and previous_state[idx] == 0:
                button_events[idx] = True
            previous_state[idx] = current_value

    return joy_cb

def calibrate_status_CB(calibrate_status_msg):
    global calibrate_complete
    calibrate_complete = calibrate_status_msg.data

def collection_status_CB(collection_status_msg):
    global collect_complete
    collect_complete = collection_status_msg.data

def waypoint_following_status_CB(waypoint_following_status_msg):
    global send_complete
    send_complete = waypoint_following_status_msg.data

def launch_subscribers():
    global cmd_vel_pub, cmd_vel_intermediate_pub, warthog_cmd_vel_pub
    rospy.init_node('joy_launch_control')
    rospy.Subscriber("/joy_teleop/joy", Joy, _make_joy_cb(main_prev_buttons))
    if keyboard_waypoint_control_enabled:
        rospy.Subscriber(keyboard_joy_topic, Joy, _make_joy_cb(keyboard_prev_buttons))
    rospy.Subscriber("/outdoor_waypoint_nav/calibrate_status",Bool, calibrate_status_CB )
    rospy.Subscriber("/outdoor_waypoint_nav/collection_status",Bool, collection_status_CB )
    rospy.Subscriber("/outdoor_waypoint_nav/waypoint_following_status",Bool, waypoint_following_status_CB )
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    cmd_vel_intermediate_pub = rospy.Publisher('/cmd_vel_intermediate', Twist, queue_size=1)
    warthog_cmd_vel_pub = rospy.Publisher('/warthog_velocity_controller/cmd_vel', Twist, queue_size=1)

def print_instructions():
    return

def relay_launch_output(process, label):
    for raw_line in iter(process.stdout.readline, ""):
        stripped = raw_line.strip()
        if not stripped:
            continue

        # Show only actionable node output and hide roslaunch boilerplate.
        if (
            "[INFO]" in stripped
            or "[WARN]" in stripped
            or "[ERROR]" in stripped
            or stripped.startswith("Press ")
        ):
            sys.stdout.write(stripped + "\n")
            sys.stdout.flush()

    process.stdout.close()


def publish_zero_velocity(repeat=10, interval=0.02):
    global cmd_vel_pub, cmd_vel_intermediate_pub, warthog_cmd_vel_pub
    zero_twist = Twist()
    for _ in range(repeat):
        for pub in (cmd_vel_pub, cmd_vel_intermediate_pub, warthog_cmd_vel_pub):
            if pub is None:
                continue
            try:
                pub.publish(zero_twist)
            except Exception:
                pass
        rospy.sleep(interval)


def shutdown_launch_process():
    global launch_process
    global launch_label

    if launch_process is None:
        return

    if launch_process.poll() is None:
        try:
            publish_zero_velocity()
            os.killpg(os.getpgid(launch_process.pid), signal.SIGINT)
            rospy.sleep(0.2)
            if launch_process.poll() is None:
                os.killpg(os.getpgid(launch_process.pid), signal.SIGTERM)
                rospy.sleep(0.2)
            if launch_process.poll() is None:
                os.killpg(os.getpgid(launch_process.pid), signal.SIGKILL)
        except OSError:
            pass
    launch_process = None
    launch_label = ""

def start_launch_process(launch_file, label, launch_args=None):
    global launch_process
    global launch_label
    global launch_output_thread

    shutdown_launch_process()
    rospy.loginfo("Starting %s...", label)
    command = ["roslaunch", launch_file]
    if launch_args:
        command.extend(launch_args)
    launch_process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        preexec_fn=os.setsid,
    )
    launch_label = label
    launch_output_thread = threading.Thread(
        target=relay_launch_output,
        args=(launch_process, label),
        daemon=True,
    )
    launch_output_thread.start()

def get_coordinates_file_path():
    path_local = rospy.get_param("/outdoor_waypoint_nav/coordinates_file", None)
    if path_local:
        return rospkg.RosPack().get_path("outdoor_waypoint_nav") + path_local

    filename = "/waypoint_files/points_sim.txt" if sim_enabled else "/waypoint_files/points_outdoor.txt"
    return rospkg.RosPack().get_path("outdoor_waypoint_nav") + filename

def count_waypoint_tokens(filepath):
    if not os.path.exists(filepath):
        return 0

    with open(filepath, "r", encoding="utf-8") as waypoint_file:
        return len(waypoint_file.read().split())

def has_move_base_server():
    published_topics = dict(rospy.get_published_topics())
    required_topics = [
        "/move_base/status",
        "/move_base/goal",
        "/move_base/result",
    ]
    return all(topic in published_topics for topic in required_topics)

def check_buttons():

    global button_events
    global launch 
    global calibrate_complete
    global collect_complete
    global send_complete
    global velocity_paused

    current_events = button_events[:]
    button_events = [False, False, False, False, False]
    
    # Check abort button
    if current_events[3]:
        rospy.logerr("STOP BUTTON SELECTED, blocking velocity commands...")
        publish_zero_velocity()
        os.system("rosnode kill safety_node")
        rospy.sleep(1) # Sleep for 1 second to allow time for node to shutdown
        publish_zero_velocity()
        sys.stdout.write("\nPress %s to continue following waypoints\n\n" % continue_btn_sym)
        sys.stdout.flush()
        velocity_paused = True
    
    elif current_events[4] and velocity_paused == True:
        rospy.loginfo("continuing to follow wapoints...")
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid,[location_safety_node])
        launch.start()
        velocity_paused = False

    # Start collecting goals
    if current_events[0]:
        start_launch_process(location_collect, "collect_goals.launch")

    # Start sending goals
    elif current_events[1]:
        if not has_move_base_server():
            launch_name = "outdoor_waypoint_nav_sim.launch" if sim_enabled else "outdoor_waypoint_nav.launch"
            rospy.logerr("move_base is not running. Start %s before pressing %s.", launch_name, send_btn_sym)
            return

        waypoint_path = get_coordinates_file_path()
        waypoint_token_count = count_waypoint_tokens(waypoint_path)
        if waypoint_token_count < 2:
            rospy.logerr("No waypoint available in %s. Collect waypoint(s) before pressing %s.", waypoint_path, send_btn_sym)
            return

        rospy.loginfo("Using waypoint file: %s", waypoint_path)
        rospy.loginfo("Waypoint count detected: %d", waypoint_token_count // 2)
        start_launch_process(
            location_send,
            "send_goals.launch",
            ["coordinates_file:={}".format(
                rospy.get_param(
                    "/outdoor_waypoint_nav/coordinates_file",
                    "/waypoint_files/points_sim.txt"
                    if sim_enabled
                    else "/waypoint_files/points_outdoor.txt",
                )
            )],
        )

    # Start Heading Calbration
    elif current_events[2]:
        if sim_enabled:
            rospy.logwarn("Heading calibration is disabled in simulation. Using navsat_params_sim.yaml defaults instead.")
            return
        start_launch_process(location_calibrate, "heading_calibration.launch")

    # Check if end notice has been published by other nodes
    if (calibrate_complete or collect_complete or send_complete):
        rospy.sleep(2) # Sleep for 2 seconds to allow time for other nodes to shutdown
        shutdown_launch_process()
        print_instructions()
        # Reset all parameters
        calibrate_complete = False
        collect_complete = False
        send_complete = False

def main():

    # start node to subscribe to joy messages node end messages 
    launch_subscribers()
    rospy.on_shutdown(shutdown_launch_process)

    # check buttons and launch the appropriate file
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        check_buttons()
        rate.sleep()

if __name__ == '__main__':

    getParameter()
    getPaths()

    print_instructions()

    if sim_enabled == False:
        sys.stdout.write(
            "NOTE: It is recommended to perform one or two heading calibrations\n"
            "      each time the robot is starting from a new heading.\n"
        )
        sys.stdout.flush()
    
    main()
    
