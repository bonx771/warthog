#!/usr/bin/env python3

import copy
import json
import subprocess
import sys

import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header, String

try:
    import pyrealsense2 as rs
except ImportError as exc:
    rs = None
    REALSENSE_IMPORT_ERROR = exc
else:
    REALSENSE_IMPORT_ERROR = None


def _positive_int_param(name, default_value):
    value = int(rospy.get_param("~" + name, default_value))
    if value <= 0:
        raise rospy.ROSException("~{} must be > 0, got {}".format(name, value))
    return value


def _bool_param(name, default_value):
    return bool(rospy.get_param("~" + name, default_value))


def _safe_device_info(device, info_key):
    try:
        if device.supports(info_key):
            return device.get_info(info_key)
    except RuntimeError:
        pass
    return ""


def _safe_device_info_by_name(device, info_name):
    info_key = getattr(rs.camera_info, info_name, None)
    if info_key is None:
        return ""
    return _safe_device_info(device, info_key)


def _header(stamp, frame_id):
    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id
    return header


def _distortion_model_name(intrinsics):
    model = str(intrinsics.model).lower()
    if "kannala" in model or "fisheye" in model or "ftheta" in model:
        return "equidistant"
    return "plumb_bob"


def _camera_info_from_profile(stream_profile, frame_id):
    video_profile = stream_profile.as_video_stream_profile()
    intrinsics = video_profile.get_intrinsics()

    camera_info = CameraInfo()
    camera_info.header.frame_id = frame_id
    camera_info.width = intrinsics.width
    camera_info.height = intrinsics.height
    camera_info.distortion_model = _distortion_model_name(intrinsics)

    coeffs = list(intrinsics.coeffs)
    while len(coeffs) < 5:
        coeffs.append(0.0)
    camera_info.D = coeffs

    camera_info.K = [
        intrinsics.fx,
        0.0,
        intrinsics.ppx,
        0.0,
        intrinsics.fy,
        intrinsics.ppy,
        0.0,
        0.0,
        1.0,
    ]
    camera_info.R = [
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
    camera_info.P = [
        intrinsics.fx,
        0.0,
        intrinsics.ppx,
        0.0,
        0.0,
        intrinsics.fy,
        intrinsics.ppy,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
    ]
    return camera_info


def _image_from_array(array, encoding, header):
    if not array.flags["C_CONTIGUOUS"]:
        array = np.ascontiguousarray(array)

    msg = Image()
    msg.header = header
    msg.height = int(array.shape[0])
    msg.width = int(array.shape[1])
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = int(array.strides[0])
    msg.data = array.tobytes()
    return msg


class RealSenseD455Node:
    def __init__(self):
        if rs is None:
            raise rospy.ROSException(
                "pyrealsense2 is not installed or cannot be imported: {}".format(
                    REALSENSE_IMPORT_ERROR
                )
            )

        self.serial_no = rospy.get_param("~serial_no", "")
        self.color_width = _positive_int_param("color_width", 640)
        self.color_height = _positive_int_param("color_height", 480)
        self.depth_width = _positive_int_param("depth_width", self.color_width)
        self.depth_height = _positive_int_param("depth_height", self.color_height)
        self.fps = _positive_int_param("fps", 30)
        self.depth_fps = _positive_int_param("depth_fps", self.fps)
        self.timeout_ms = _positive_int_param("timeout_ms", 1000)
        self.queue_size = _positive_int_param("queue_size", 5)

        self.enable_depth = _bool_param("enable_depth", True)
        self.align_depth_to_color = _bool_param("align_depth_to_color", True)

        self.color_frame_id = rospy.get_param(
            "~color_frame_id", "camera_color_optical_frame"
        )
        self.depth_frame_id = rospy.get_param(
            "~depth_frame_id", "camera_depth_optical_frame"
        )
        self.camera_topic = rospy.get_param("~camera_topic", "/camera")
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/camera/camera_info"
        )
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/depth")
        self.depth_info_topic = rospy.get_param(
            "~depth_info_topic", "/camera/depth/camera_info"
        )
        self.metadata_topic = rospy.get_param("~metadata_topic", "/camera/metadata")

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.profile = None
        self.align = None
        self.depth_scale = None
        self.device_info = {}
        self.color_info = None
        self.depth_info = None
        self.depth_output_frame_id = self.depth_frame_id
        self.pipeline_started = False

        self.image_pub = rospy.Publisher(
            self.camera_topic, Image, queue_size=self.queue_size
        )
        self.camera_info_pub = rospy.Publisher(
            self.camera_info_topic, CameraInfo, queue_size=self.queue_size
        )
        self.metadata_pub = rospy.Publisher(
            self.metadata_topic, String, queue_size=1, latch=True
        )
        self.depth_pub = None
        self.depth_info_pub = None
        if self.enable_depth:
            self.depth_pub = rospy.Publisher(
                self.depth_topic, Image, queue_size=self.queue_size
            )
            self.depth_info_pub = rospy.Publisher(
                self.depth_info_topic, CameraInfo, queue_size=self.queue_size
            )

        self._start_camera()
        rospy.on_shutdown(self.shutdown)

    def _start_camera(self):
        if self.serial_no:
            self.config.enable_device(self.serial_no)

        self.config.enable_stream(
            rs.stream.color,
            self.color_width,
            self.color_height,
            rs.format.bgr8,
            self.fps,
        )

        if self.enable_depth:
            self.config.enable_stream(
                rs.stream.depth,
                self.depth_width,
                self.depth_height,
                rs.format.z16,
                self.depth_fps,
            )

        try:
            self.profile = self.pipeline.start(self.config)
        except RuntimeError as exc:
            raise rospy.ROSException("Could not start RealSense camera: {}".format(exc))
        self.pipeline_started = True

        device = self.profile.get_device()
        self.device_info = {
            "name": _safe_device_info_by_name(device, "name"),
            "serial_number": _safe_device_info_by_name(device, "serial_number"),
            "firmware_version": _safe_device_info_by_name(device, "firmware_version"),
            "physical_port": _safe_device_info_by_name(device, "physical_port"),
            "product_id": _safe_device_info_by_name(device, "product_id"),
            "product_line": _safe_device_info_by_name(device, "product_line"),
            "usb_type_descriptor": _safe_device_info_by_name(
                device, "usb_type_descriptor"
            ),
        }

        color_profile = self.profile.get_stream(rs.stream.color)
        self.color_info = _camera_info_from_profile(
            color_profile, self.color_frame_id
        )

        if self.enable_depth:
            try:
                depth_sensor = device.first_depth_sensor()
                self.depth_scale = float(depth_sensor.get_depth_scale())
            except RuntimeError:
                self.depth_scale = 0.001

            rospy.set_param("~depth_scale_meters_per_unit", self.depth_scale)

            if self.align_depth_to_color:
                self.align = rs.align(rs.stream.color)
                self.depth_info = copy.deepcopy(self.color_info)
                self.depth_info.header.frame_id = self.color_frame_id
                self.depth_output_frame_id = self.color_frame_id
            else:
                depth_profile = self.profile.get_stream(rs.stream.depth)
                self.depth_info = _camera_info_from_profile(
                    depth_profile, self.depth_frame_id
                )
                self.depth_output_frame_id = self.depth_frame_id

        self._publish_metadata()
        rospy.loginfo(
            "RealSense camera started: color %dx%d@%dHz -> %s, depth %s",
            self.color_width,
            self.color_height,
            self.fps,
            rospy.resolve_name(self.camera_topic),
            "{} -> {}".format(
                "{}x{}@{}Hz".format(
                    self.depth_width, self.depth_height, self.depth_fps
                ),
                rospy.resolve_name(self.depth_topic),
            )
            if self.enable_depth
            else "disabled",
        )
        rospy.loginfo("Camera metadata published on %s", self.metadata_topic)

    def _publish_metadata(self):
        metadata = {
            "device": self.device_info,
            "topics": {
                "color_image": rospy.resolve_name(self.camera_topic),
                "color_camera_info": rospy.resolve_name(self.camera_info_topic),
                "depth_image": rospy.resolve_name(self.depth_topic)
                if self.enable_depth
                else "",
                "depth_camera_info": rospy.resolve_name(self.depth_info_topic)
                if self.enable_depth
                else "",
            },
            "color": {
                "width": self.color_info.width,
                "height": self.color_info.height,
                "fps": self.fps,
                "frame_id": self.color_frame_id,
                "encoding": "bgr8",
                "distortion_model": self.color_info.distortion_model,
                "D": list(self.color_info.D),
                "K": list(self.color_info.K),
                "P": list(self.color_info.P),
            },
            "depth": {
                "enabled": self.enable_depth,
                "width": self.depth_width if self.enable_depth else 0,
                "height": self.depth_height if self.enable_depth else 0,
                "fps": self.depth_fps if self.enable_depth else 0,
                "frame_id": self.depth_output_frame_id if self.enable_depth else "",
                "encoding": "16UC1" if self.enable_depth else "",
                "depth_scale_meters_per_unit": self.depth_scale
                if self.enable_depth
                else 0.0,
                "aligned_to_color": self.align_depth_to_color
                if self.enable_depth
                else False,
            },
        }
        msg = String()
        msg.data = json.dumps(metadata, sort_keys=True)
        self.metadata_pub.publish(msg)

    def spin(self):
        while not rospy.is_shutdown():
            try:
                frames = self.pipeline.wait_for_frames(self.timeout_ms)
            except RuntimeError as exc:
                rospy.logwarn_throttle(
                    5.0, "Waiting for RealSense frames timed out: %s", exc
                )
                continue

            if self.align is not None:
                frames = self.align.process(frames)

            color_frame = frames.get_color_frame()
            if not color_frame:
                rospy.logwarn_throttle(5.0, "RealSense color frame is missing")
                continue

            stamp = rospy.Time.now()
            color_header = _header(stamp, self.color_frame_id)
            color_image = np.asanyarray(color_frame.get_data())
            self.image_pub.publish(_image_from_array(color_image, "bgr8", color_header))

            self.color_info.header = color_header
            self.camera_info_pub.publish(self.color_info)

            if self.enable_depth:
                depth_frame = frames.get_depth_frame()
                if not depth_frame:
                    rospy.logwarn_throttle(5.0, "RealSense depth frame is missing")
                    continue

                depth_header = _header(stamp, self.depth_output_frame_id)
                depth_image = np.asanyarray(depth_frame.get_data())
                self.depth_pub.publish(
                    _image_from_array(depth_image, "16UC1", depth_header)
                )

                self.depth_info.header = depth_header
                self.depth_info_pub.publish(self.depth_info)

    def shutdown(self):
        if self.pipeline_started:
            try:
                self.pipeline.stop()
            except RuntimeError:
                pass
            self.pipeline_started = False


class RealSenseWrapperRelayNode:
    def __init__(self):
        self.serial_no = rospy.get_param("~serial_no", "")
        self.color_width = _positive_int_param("color_width", 640)
        self.color_height = _positive_int_param("color_height", 480)
        self.depth_width = _positive_int_param("depth_width", self.color_width)
        self.depth_height = _positive_int_param("depth_height", self.color_height)
        self.fps = _positive_int_param("fps", 30)
        self.depth_fps = _positive_int_param("depth_fps", self.fps)
        self.queue_size = _positive_int_param("queue_size", 5)

        self.enable_depth = _bool_param("enable_depth", True)
        self.align_depth_to_color = _bool_param("align_depth_to_color", True)
        self.launch_driver = _bool_param("launch_driver", True)
        self.initial_reset = _bool_param("initial_reset", True)
        self.enable_pointcloud = _bool_param("enable_pointcloud", False)
        self.enable_sync = _bool_param("enable_sync", True)

        self.color_frame_id = rospy.get_param(
            "~color_frame_id", "camera_color_optical_frame"
        )
        self.depth_frame_id = rospy.get_param(
            "~depth_frame_id", "camera_depth_optical_frame"
        )
        self.override_frame_id = _bool_param("override_frame_id", False)

        self.camera_topic = rospy.get_param("~camera_topic", "/camera")
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/camera/camera_info"
        )
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/depth")
        self.depth_info_topic = rospy.get_param(
            "~depth_info_topic", "/camera/depth/camera_info"
        )
        self.metadata_topic = rospy.get_param("~metadata_topic", "/camera/metadata")

        self.driver_camera_name = rospy.get_param("~driver_camera_name", "realsense")
        driver_ns = "/" + self.driver_camera_name.strip("/")
        self.driver_color_topic = rospy.get_param(
            "~driver_color_topic", driver_ns + "/color/image_raw"
        )
        self.driver_color_info_topic = rospy.get_param(
            "~driver_color_info_topic", driver_ns + "/color/camera_info"
        )
        default_depth_topic = (
            driver_ns + "/aligned_depth_to_color/image_raw"
            if self.align_depth_to_color
            else driver_ns + "/depth/image_rect_raw"
        )
        default_depth_info_topic = (
            driver_ns + "/aligned_depth_to_color/camera_info"
            if self.align_depth_to_color
            else driver_ns + "/depth/camera_info"
        )
        self.driver_depth_topic = rospy.get_param(
            "~driver_depth_topic", default_depth_topic
        )
        self.driver_depth_info_topic = rospy.get_param(
            "~driver_depth_info_topic", default_depth_info_topic
        )

        self.driver_process = None
        self.driver_exit_reported = False

        self.image_pub = rospy.Publisher(
            self.camera_topic, Image, queue_size=self.queue_size
        )
        self.camera_info_pub = rospy.Publisher(
            self.camera_info_topic, CameraInfo, queue_size=self.queue_size
        )
        self.metadata_pub = rospy.Publisher(
            self.metadata_topic, String, queue_size=1, latch=True
        )
        self.depth_pub = None
        self.depth_info_pub = None
        if self.enable_depth:
            self.depth_pub = rospy.Publisher(
                self.depth_topic, Image, queue_size=self.queue_size
            )
            self.depth_info_pub = rospy.Publisher(
                self.depth_info_topic, CameraInfo, queue_size=self.queue_size
            )

        if self.launch_driver:
            self._launch_realsense_driver()

        self.image_sub = rospy.Subscriber(
            self.driver_color_topic,
            Image,
            self._color_image_callback,
            queue_size=self.queue_size,
            tcp_nodelay=True,
        )
        self.camera_info_sub = rospy.Subscriber(
            self.driver_color_info_topic,
            CameraInfo,
            self._color_info_callback,
            queue_size=self.queue_size,
            tcp_nodelay=True,
        )
        self.depth_sub = None
        self.depth_info_sub = None
        if self.enable_depth:
            self.depth_sub = rospy.Subscriber(
                self.driver_depth_topic,
                Image,
                self._depth_image_callback,
                queue_size=self.queue_size,
                tcp_nodelay=True,
            )
            self.depth_info_sub = rospy.Subscriber(
                self.driver_depth_info_topic,
                CameraInfo,
                self._depth_info_callback,
                queue_size=self.queue_size,
                tcp_nodelay=True,
            )

        self.monitor_timer = rospy.Timer(rospy.Duration(2.0), self._monitor_driver)
        rospy.on_shutdown(self.shutdown)
        self._publish_metadata()

        rospy.loginfo(
            "Relaying RealSense wrapper: %s -> %s, %s -> %s",
            rospy.resolve_name(self.driver_color_topic),
            rospy.resolve_name(self.camera_topic),
            rospy.resolve_name(self.driver_color_info_topic),
            rospy.resolve_name(self.camera_info_topic),
        )
        if self.enable_depth:
            rospy.loginfo(
                "Relaying RealSense depth: %s -> %s, %s -> %s",
                rospy.resolve_name(self.driver_depth_topic),
                rospy.resolve_name(self.depth_topic),
                rospy.resolve_name(self.driver_depth_info_topic),
                rospy.resolve_name(self.depth_info_topic),
            )

    def _launch_realsense_driver(self):
        try:
            import rospkg

            rospkg.RosPack().get_path("realsense2_camera")
        except Exception as exc:
            raise rospy.ROSException(
                "Cannot launch realsense2_camera package: {}. Install it with "
                "`sudo apt install ros-noetic-realsense2-camera` or run this node "
                "with `~backend:=pyrealsense` after installing pyrealsense2.".format(
                    exc
                )
            )

        args = [
            "roslaunch",
            "realsense2_camera",
            "rs_camera.launch",
            "camera:={}".format(self.driver_camera_name),
            "enable_color:=true",
            "enable_depth:={}".format(str(self.enable_depth).lower()),
            "align_depth:={}".format(str(self.align_depth_to_color).lower()),
            "enable_pointcloud:={}".format(str(self.enable_pointcloud).lower()),
            "enable_sync:={}".format(str(self.enable_sync).lower()),
            "initial_reset:={}".format(str(self.initial_reset).lower()),
            "color_width:={}".format(self.color_width),
            "color_height:={}".format(self.color_height),
            "color_fps:={}".format(self.fps),
            "depth_width:={}".format(self.depth_width),
            "depth_height:={}".format(self.depth_height),
            "depth_fps:={}".format(self.depth_fps),
        ]
        if self.serial_no:
            args.append("serial_no:={}".format(self.serial_no))

        try:
            self.driver_process = subprocess.Popen(args)
        except OSError as exc:
            raise rospy.ROSException(
                "Could not start roslaunch for realsense2_camera: {}".format(exc)
            )

        rospy.loginfo("Started realsense2_camera driver with PID %s", self.driver_process.pid)

    def _publish_metadata(self):
        metadata = {
            "backend": "realsense2_camera",
            "driver_launched_by_node": self.launch_driver,
            "driver_camera_name": self.driver_camera_name,
            "serial_no": self.serial_no,
            "topics": {
                "driver_color_image": rospy.resolve_name(self.driver_color_topic),
                "driver_color_camera_info": rospy.resolve_name(
                    self.driver_color_info_topic
                ),
                "driver_depth_image": rospy.resolve_name(self.driver_depth_topic)
                if self.enable_depth
                else "",
                "driver_depth_camera_info": rospy.resolve_name(
                    self.driver_depth_info_topic
                )
                if self.enable_depth
                else "",
                "color_image": rospy.resolve_name(self.camera_topic),
                "color_camera_info": rospy.resolve_name(self.camera_info_topic),
                "depth_image": rospy.resolve_name(self.depth_topic)
                if self.enable_depth
                else "",
                "depth_camera_info": rospy.resolve_name(self.depth_info_topic)
                if self.enable_depth
                else "",
            },
            "color": {
                "width": self.color_width,
                "height": self.color_height,
                "fps": self.fps,
                "frame_id": self.color_frame_id,
                "encoding": "bgr8",
            },
            "depth": {
                "enabled": self.enable_depth,
                "width": self.depth_width if self.enable_depth else 0,
                "height": self.depth_height if self.enable_depth else 0,
                "fps": self.depth_fps if self.enable_depth else 0,
                "frame_id": self.depth_frame_id if self.enable_depth else "",
                "encoding": "16UC1" if self.enable_depth else "",
                "aligned_to_color": self.align_depth_to_color
                if self.enable_depth
                else False,
            },
        }
        msg = String()
        msg.data = json.dumps(metadata, sort_keys=True)
        self.metadata_pub.publish(msg)

    def _relay(self, publisher, msg, frame_id=""):
        output = copy.deepcopy(msg)
        if self.override_frame_id and frame_id:
            output.header.frame_id = frame_id
        publisher.publish(output)

    def _color_image_callback(self, msg):
        self._relay(self.image_pub, msg, self.color_frame_id)

    def _color_info_callback(self, msg):
        self._relay(self.camera_info_pub, msg, self.color_frame_id)

    def _depth_image_callback(self, msg):
        self._relay(self.depth_pub, msg, self.depth_frame_id)

    def _depth_info_callback(self, msg):
        self._relay(self.depth_info_pub, msg, self.depth_frame_id)

    def _monitor_driver(self, _event):
        if self.driver_process is None or self.driver_exit_reported:
            return

        return_code = self.driver_process.poll()
        if return_code is None:
            return

        self.driver_exit_reported = True
        rospy.logerr("realsense2_camera driver exited with code %s", return_code)
        rospy.signal_shutdown("realsense2_camera driver exited")

    def spin(self):
        rospy.spin()

    def shutdown(self):
        if self.driver_process is None:
            return

        if self.driver_process.poll() is None:
            self.driver_process.terminate()
            try:
                self.driver_process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self.driver_process.kill()
                self.driver_process.wait()
        self.driver_process = None


def _select_backend():
    backend = rospy.get_param("~backend", "auto").strip().lower()
    if backend == "auto":
        if rs is not None:
            return "pyrealsense"
        return "wrapper"

    valid_backends = ("pyrealsense", "wrapper")
    if backend not in valid_backends:
        raise rospy.ROSException(
            "~backend must be one of {}, got {!r}".format(valid_backends, backend)
        )

    if backend == "pyrealsense" and rs is None:
        raise rospy.ROSException(
            "Cannot use pyrealsense backend because pyrealsense2 cannot be imported: "
            "{}. Use `~backend:=wrapper` with ros-noetic-realsense2-camera, or "
            "install pyrealsense2.".format(REALSENSE_IMPORT_ERROR)
        )

    return backend


if __name__ == "__main__":
    rospy.init_node("camera_node")
    try:
        selected_backend = _select_backend()
        rospy.loginfo("camera_node selected backend: %s", selected_backend)
        if selected_backend == "pyrealsense":
            node = RealSenseD455Node()
        else:
            node = RealSenseWrapperRelayNode()
        node.spin()
    except rospy.ROSException as exc:
        rospy.logerr("%s", exc)
        sys.exit(1)
