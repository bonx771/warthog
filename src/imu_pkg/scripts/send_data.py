#!/usr/bin/env python3

import struct

import rospy
import serial


HEADER = b"\x10\x10"
CHECKSUM_INDEX = 4
OUTPUT_CONTROL_CMD = 0x84
OUTPUT_ONCE = 0x00
OUTPUT_START = 0x01
OUTPUT_STOP = 0x81
ALLOWED_OUTPUT_HZ = (1, 2, 5, 10, 25, 50)
GYROCOMPASS_START_FRAME = bytes.fromhex("10 10 86 09 AF 00 00 00 00")


def checksum(frame):
    return sum(byte for i, byte in enumerate(frame) if i != CHECKSUM_INDEX) & 0xFF


def hex_bytes(data):
    return " ".join(f"{byte:02X}" for byte in data)


def as_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "y", "on")


def output_control_frame(command, hz, data_mask):
    payload = struct.pack(">BHI", command, hz, data_mask)
    frame = bytearray(HEADER + bytes([OUTPUT_CONTROL_CMD, 12, 0]) + payload)
    frame[CHECKSUM_INDEX] = checksum(frame)
    return bytes(frame)


class SendDataNode:
    def __init__(self):
        self.port = rospy.get_param("~serial_port", "/dev/ttyUSB0")
        self.baud = int(rospy.get_param("~baud_rate", 115200))
        self.timeout = float(rospy.get_param("~serial_timeout_sec", 0.05))
        self.output_hz = int(rospy.get_param("~output_hz", 25))
        self.data_mask = int(rospy.get_param("~data_mask", 1))
        self.send_gyrocompass_start = as_bool(
            rospy.get_param("~send_gyrocompass_start", True)
        )
        self.gyrocompass_start_delay_sec = float(
            rospy.get_param("~gyrocompass_start_delay_sec", 0.2)
        )
        self.send_stop_first = as_bool(rospy.get_param("~send_stop_first", True))
        self.wait_after_send_sec = float(
            rospy.get_param("~wait_after_send_sec", 0.1)
        )
        self.serial = None

        rospy.on_shutdown(self.close)

    def validate_params(self):
        if self.output_hz not in ALLOWED_OUTPUT_HZ:
            raise ValueError(
                f"output_hz={self.output_hz} is unsupported; "
                f"allowed values: {ALLOWED_OUTPUT_HZ}"
            )
        if not 0 <= self.data_mask <= 0xFFFFFFFF:
            raise ValueError("data_mask must be in the range 0..0xFFFFFFFF")
        if self.gyrocompass_start_delay_sec < 0.0:
            raise ValueError("gyrocompass_start_delay_sec must be non-negative")
        if self.wait_after_send_sec < 0.0:
            raise ValueError("wait_after_send_sec must be non-negative")

    def open(self):
        self.serial = serial.Serial(self.port, self.baud, timeout=self.timeout)
        rospy.loginfo("INS command serial opened on %s @ %d baud.", self.port, self.baud)

    def close(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
            rospy.loginfo("INS command serial closed.")

    def write(self, data, description):
        self.serial.write(data)
        self.serial.flush()
        rospy.loginfo("Sent %s: %s", description, hex_bytes(data))

    def build_cyclic_output_command(self):
        frames = []
        if self.send_stop_first:
            frames.append(output_control_frame(OUTPUT_STOP, 0, 0))

        frames.extend(
            [
                output_control_frame(OUTPUT_START, self.output_hz, 0),
                output_control_frame(OUTPUT_ONCE, self.output_hz, self.data_mask),
            ]
        )
        return b"".join(frames)

    def run(self):
        try:
            self.validate_params()
            self.open()

            if self.send_gyrocompass_start:
                rospy.logwarn(
                    "Sending 0x86, which the INS protocol identifies as the "
                    "gyrocompass start command. Set send_gyrocompass_start:=false "
                    "to restore cyclic output without sending 0x86."
                )
                self.write(GYROCOMPASS_START_FRAME, "gyrocompass start command 0x86")
                rospy.sleep(self.gyrocompass_start_delay_sec)

            cyclic_command = self.build_cyclic_output_command()
            self.write(
                cyclic_command,
                f"cyclic INS output command at {self.output_hz} Hz "
                f"with data_mask=0x{self.data_mask:08X}",
            )

            rospy.sleep(self.wait_after_send_sec)
            rospy.loginfo(
                "INS COMMANDS SENT SUCCESSFULLY at %d Hz. "
                "Closing the serial port now. Next, run: "
                "roslaunch imu_pkg imu_ins.launch",
                self.output_hz,
            )
        except (serial.SerialException, ValueError, struct.error) as exc:
            rospy.logerr("Failed to start INS cyclic output: %s", exc)
        finally:
            self.close()


if __name__ == "__main__":
    rospy.init_node("send_data")
    SendDataNode().run()
