#!/usr/bin/env python3

import math
import struct

import rospy
import serial
from sensor_msgs.msg import Imu


HEADER = b"\x10\x10"
CHECKSUM_INDEX = 4
STATE_CMD = 0x02
MIN_FRAME_LEN = 5
MIN_PAYLOAD_LEN = 68

YAW_OFF = 28
ROLL_OFF = 32
PITCH_OFF = 36
GYRO_OFFS = (44, 48, 52)
ACCEL_OFFS = (56, 60, 64)


def checksum(frame):
    return sum(byte for i, byte in enumerate(frame) if i != CHECKSUM_INDEX) & 0xFF


def f32(payload, offset):
    return struct.unpack_from(">f", payload, offset)[0]


def diag_cov(values):
    return [
        values[0], 0.0, 0.0,
        0.0, values[1], 0.0,
        0.0, 0.0, values[2],
    ]


def quaternion_from_euler(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class InsImuNode:
    def __init__(self):
        self.port = rospy.get_param("~serial_port", "/dev/ttyUSB0")
        self.baud = int(rospy.get_param("~baud_rate", 115200))
        self.timeout = float(rospy.get_param("~serial_timeout_sec", 0.05))
        self.chunk_size = int(rospy.get_param("~read_chunk_size", 256))
        self.frame_id = rospy.get_param("~frame_id", "imu_link")
        self.topic = rospy.get_param("~state_topic", "/imu/data_new")
        self.yaw_sign = float(rospy.get_param("~yaw_sign", -1.0))
        self.startup_commands = rospy.get_param("~startup_commands", [])
        self.startup_delay = float(rospy.get_param("~startup_command_delay_sec", 0.05))

        self.orientation_cov = diag_cov(
            rospy.get_param("~orientation_covariance_diagonal", [1.0, 1.0, 0.8])
        )
        self.angular_velocity_cov = diag_cov(
            rospy.get_param("~angular_velocity_covariance_diagonal", [0.05, 0.05, 0.0002])
        )
        self.linear_acceleration_cov = diag_cov(
            rospy.get_param("~linear_acceleration_covariance_diagonal", [0.1, 0.1, 0.1])
        )

        self.buffer = bytearray()
        self.serial = None
        self.publisher = rospy.Publisher(self.topic, Imu, queue_size=10)
        rospy.on_shutdown(self.close)

    def open(self):
        if self.serial and self.serial.is_open:
            return
        self.serial = serial.Serial(self.port, self.baud, timeout=self.timeout)
        rospy.loginfo("INS serial opened on %s @ %d baud.", self.port, self.baud)
        for command in self.startup_commands:
            self.serial.write(bytes.fromhex(command.replace(" ", "")))
            self.serial.flush()
            rospy.sleep(self.startup_delay)

    def close(self):
        if self.serial and self.serial.is_open:
            self.serial.close()

    def feed(self, data):
        frames = []
        if data:
            self.buffer.extend(data)

        while True:
            start = self.buffer.find(HEADER)
            if start < 0:
                self.buffer = bytearray(b"\x10") if self.buffer[-1:] == b"\x10" else bytearray()
                return frames
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 4:
                return frames

            frame_len = self.buffer[3]
            if frame_len < MIN_FRAME_LEN:
                del self.buffer[0]
                continue
            if len(self.buffer) < frame_len:
                return frames

            frame = bytes(self.buffer[:frame_len])
            del self.buffer[:frame_len]
            if frame[2] == STATE_CMD and checksum(frame) == frame[CHECKSUM_INDEX]:
                frames.append(frame)

    def publish_frame(self, frame):
        payload = frame[5:]
        if len(payload) < MIN_PAYLOAD_LEN:
            return

        roll = f32(payload, ROLL_OFF)
        pitch = f32(payload, PITCH_OFF)
        yaw = self.yaw_sign * f32(payload, YAW_OFF)
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)

        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.orientation_covariance = self.orientation_cov
        msg.angular_velocity.x = f32(payload, GYRO_OFFS[0])
        msg.angular_velocity.y = f32(payload, GYRO_OFFS[1])
        msg.angular_velocity.z = self.yaw_sign * f32(payload, GYRO_OFFS[2])
        msg.angular_velocity_covariance = self.angular_velocity_cov
        msg.linear_acceleration.x = f32(payload, ACCEL_OFFS[0])
        msg.linear_acceleration.y = f32(payload, ACCEL_OFFS[1])
        msg.linear_acceleration.z = f32(payload, ACCEL_OFFS[2])
        msg.linear_acceleration_covariance = self.linear_acceleration_cov
        self.publisher.publish(msg)

    def spin(self):
        rate = rospy.Rate(200)
        while not rospy.is_shutdown():
            try:
                self.open()
                for frame in self.feed(self.serial.read(self.chunk_size)):
                    self.publish_frame(frame)
            except serial.SerialException as exc:
                rospy.logerr_throttle(2.0, "INS serial error: %s", exc)
                self.close()
                rospy.sleep(1.0)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("imu_node")
    InsImuNode().spin()
