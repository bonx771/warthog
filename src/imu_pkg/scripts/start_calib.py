#!/usr/bin/env python3

import sys
import struct

import rospy
import serial


HEADER = b"\x10\x10"
CHECKSUM_INDEX = 4
STATUS_CMD = 0x00
STATUS_ALT_CMD = 0x01
STATUS_FRAME_LEN = 9
GYROCOMPASS_EVENT_CMD = 0x03
OUTPUT_CONTROL_CMD = 0x84
ALLOWED_OUTPUT_HZ = (1, 2, 5, 10, 25, 50)
GYROCOMPASS_START_FRAME = bytes.fromhex("10 10 86 09 AF 00 00 00 00")
GYROCOMPASS_STARTED_FRAME = bytes.fromhex("10 10 03 09 2C 00 00 00 00")
GYROCOMPASS_FINISHED_FRAME = bytes.fromhex("10 10 03 09 2D 00 00 00 01")
WORK_MODE_LABELS = {
    0: "idle",
    1: "gyrocompass",
    2: "navigation",
}


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


def format_remaining(seconds):
    seconds = max(0, int(round(seconds)))
    minutes, seconds = divmod(seconds, 60)
    return f"{minutes:02d}:{seconds:02d}"


def screen(message):
    print(message, flush=True)


class StartCalibNode:
    def __init__(self):
        self.port = rospy.get_param("~serial_port", "/dev/ttyUSB0")
        self.baud = int(rospy.get_param("~baud_rate", 115200))
        self.timeout = float(rospy.get_param("~serial_timeout_sec", 0.05))
        self.chunk_size = int(rospy.get_param("~read_chunk_size", 256))
        self.countdown_sec = float(rospy.get_param("~countdown_sec", 480.0))
        self.finish_wait_timeout_sec = float(rospy.get_param("~finish_wait_timeout_sec", 60.0))
        self.response_required = as_bool(rospy.get_param("~response_required", False))
        self.auto_send = as_bool(rospy.get_param("~auto_send", False))
        self.shutdown_on_finish = as_bool(rospy.get_param("~shutdown_on_finish", True))
        self.countdown_log_period_sec = float(rospy.get_param("~countdown_log_period_sec", 10.0))
        self.log_all_frames = as_bool(rospy.get_param("~log_all_frames", True))
        self.log_raw_rx = as_bool(rospy.get_param("~log_raw_rx", False))
        self.no_rx_warn_period_sec = float(rospy.get_param("~no_rx_warn_period_sec", 10.0))
        self.reapply_output_after_start = as_bool(rospy.get_param("~reapply_output_after_start", True))
        self.reapply_output_on_finish = as_bool(rospy.get_param("~reapply_output_on_finish", True))
        self.reapply_output_hz = int(rospy.get_param("~reapply_output_hz", 25))
        self.reapply_output_data_mask = int(rospy.get_param("~reapply_output_data_mask", 1))
        self.reapply_output_send_stop = as_bool(rospy.get_param("~reapply_output_send_stop", True))
        self.reapply_output_delay_sec = float(rospy.get_param("~reapply_output_delay_sec", 0.2))

        self.serial = None
        self.buffer = bytearray()
        self.sent = False
        self.started_ack = False
        self.finished_ack = False
        self.sent_time = None
        self.last_remaining = None
        self.last_countdown_log_time = None
        self.countdown_done = False
        self.last_status = None
        self.seen_gyrocompass_mode = False
        self.fail_reported = False
        self.unknown_reported = False
        self.frame_counts = {}
        self.last_rx_time = None
        self.last_no_rx_warn_time = None
        self.output_reapplied_on_finish = False

        rospy.on_shutdown(self.close)

    def open(self):
        if self.serial and self.serial.is_open:
            return
        self.serial = serial.Serial(self.port, self.baud, timeout=self.timeout)
        rospy.loginfo("INS calib serial opened on %s @ %d baud.", self.port, self.baud)

    def close(self):
        if self.serial and self.serial.is_open:
            self.serial.close()

    def wait_for_send(self):
        if self.auto_send:
            message = "auto_send=true, sending gyrocompass start immediately."
            screen(message)
            rospy.logwarn(message)
            return True

        prompt = "Type 'send' then press Enter to send gyrocompass start command: "
        screen("")
        screen("============================================================")
        screen("IMU gyrocompass calibration is ready.")
        screen("Type exactly: send")
        screen("============================================================")
        while not rospy.is_shutdown():
            try:
                with open("/dev/tty", "r+") as tty:
                    tty.write(prompt)
                    tty.flush()
                    answer = tty.readline()
            except OSError:
                sys.stdout.write(prompt)
                sys.stdout.flush()
                answer = sys.stdin.readline()

            if not answer:
                rospy.logerr("No interactive terminal available. Set auto_send:=true to send without prompt.")
                return False

            command = answer.strip().lower()
            if command == "send":
                message = "Operator command received: send. Sending gyrocompass start command now."
                return True

            message = f"Command not sent. Received '{answer.strip()}'. Type exactly: send"
            rospy.logwarn(message)

        return False

    def send_start(self):
        self.open()
        self.serial.write(GYROCOMPASS_START_FRAME)
        self.serial.flush()
        self.sent = True
        self.sent_time = rospy.Time.now()
        self.last_remaining = None
        self.last_countdown_log_time = None
        self.countdown_done = False
        sent_message = f"Sent gyrocompass start 0x86"
        countdown_message = f"Gyrocompass started: {format_remaining(self.countdown_sec)} remaining."
        screen(sent_message)
        screen(countdown_message)

        if self.reapply_output_after_start:
            rospy.sleep(self.reapply_output_delay_sec)
            self.send_output_config("after gyrocompass start")

    def send_output_config(self, reason):
        if self.reapply_output_hz not in ALLOWED_OUTPUT_HZ:
            rospy.logerr(
                "Cannot reapply INS output frequency: reapply_output_hz=%s is unsupported. Allowed: %s.",
                self.reapply_output_hz,
                ", ".join(str(hz) for hz in ALLOWED_OUTPUT_HZ),
            )
            return

        frames = []
        if self.reapply_output_send_stop:
            frames.append(output_control_frame(0x81, 0, 0))
        frames.extend(
            [
                output_control_frame(0x01, self.reapply_output_hz, 0),
                output_control_frame(0x00, self.reapply_output_hz, self.reapply_output_data_mask),
            ]
        )
        data = b"".join(frames)
        self.serial.write(data)
        self.serial.flush()
        message = (
            f"Reapplied INS output {self.reapply_output_hz} Hz {reason}: "        )
        screen(message)

    def reapply_output_on_terminal_state(self, reason):
        if not self.reapply_output_on_finish or self.output_reapplied_on_finish:
            return
        self.output_reapplied_on_finish = True
        self.send_output_config(reason)

    def feed(self, data):
        frames = []
        if data:
            self.last_rx_time = rospy.Time.now()
            if self.log_raw_rx:
                rospy.loginfo_throttle(
                    1.0,
                    "RX raw bytes: %d bytes: %s%s",
                    len(data),
                    hex_bytes(data[:64]),
                    " ..." if len(data) > 64 else "",
                )
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
            if frame_len < 5:
                del self.buffer[0]
                continue
            if len(self.buffer) < frame_len:
                return frames

            frame = bytes(self.buffer[:frame_len])
            del self.buffer[:frame_len]
            if checksum(frame) == frame[CHECKSUM_INDEX]:
                frames.append(frame)
            else:
                rospy.logwarn("Dropped INS frame with bad checksum: %s", hex_bytes(frame))

    def log_received_frame(self, frame):
        if not self.log_all_frames:
            return

        cmd = frame[2]
        count = self.frame_counts.get(cmd, 0) + 1
        self.frame_counts[cmd] = count
        should_log = cmd in (STATUS_CMD, STATUS_ALT_CMD, GYROCOMPASS_EVENT_CMD) or count <= 5 or count % 50 == 0
        if should_log:
            rospy.loginfo(
                "RX frame cmd=0x%02X len=%d count=%d: %s",
                cmd,
                len(frame),
                count,
                hex_bytes(frame),
            )

    def handle_frame(self, frame):
        self.log_received_frame(frame)

        if frame[2] in (STATUS_CMD, STATUS_ALT_CMD) and frame[3] == STATUS_FRAME_LEN:
            self.handle_status_frame(frame)
            return

        if frame[2] == GYROCOMPASS_EVENT_CMD and frame[3] == STATUS_FRAME_LEN:
            self.handle_gyrocompass_event_frame(frame)
            return

        rospy.logdebug("Received INS frame while calibrating: %s", hex_bytes(frame))

    def report_success(self, reason):
        if self.finished_ack:
            return
        self.finished_ack = True
        self.reapply_output_on_terminal_state("after gyrocompass finish")
        success_message = f"CALIBRATION SUCCESS: {reason}"
        screen(success_message)
        if self.shutdown_on_finish:
            rospy.signal_shutdown("gyrocompass calibration finished")

    def handle_gyrocompass_event_frame(self, frame):
        rospy.loginfo("Received INS gyrocompass event frame: %s", hex_bytes(frame))

        if frame == GYROCOMPASS_STARTED_FRAME:
            if not self.started_ack:
                rospy.loginfo("INS confirmed gyrocompass start: %s", hex_bytes(frame))
            self.started_ack = True
            return

        if frame == GYROCOMPASS_FINISHED_FRAME:
            rospy.loginfo("INS confirmed gyrocompass finish: %s", hex_bytes(frame))
            self.report_success("gyrocompass finish frame received.")
            return

        rospy.logwarn("Unknown INS gyrocompass event frame: %s", hex_bytes(frame))

    def handle_status_frame(self, frame):
        status = struct.unpack(">I", frame[5:9])[0]
        gyrocompass_minutes = (status >> 8) & 0x0F
        work_mode = (status >> 12) & 0x03
        if work_mode == 1:
            self.seen_gyrocompass_mode = True

        key = (work_mode, gyrocompass_minutes)
        if key != self.last_status:
            self.last_status = key
            rospy.loginfo(
                "INS status cmd=0x%02X: work_mode=%s(%d), gyrocompass_wait=%d min, raw=0x%08X.",
                frame[2],
                WORK_MODE_LABELS.get(work_mode, "unknown"),
                work_mode,
                gyrocompass_minutes,
                status,
            )

        if (
            self.sent
            and self.seen_gyrocompass_mode
            and work_mode != 1
            and gyrocompass_minutes == 0
            and not self.finished_ack
        ):
            self.report_success(
                f"INS status left gyrocompass mode; work_mode={WORK_MODE_LABELS.get(work_mode, 'unknown')}({work_mode})."
            )
            return

    def update_countdown(self):
        if not self.sent or self.finished_ack:
            return

        elapsed = (rospy.Time.now() - self.sent_time).to_sec()
        remaining = max(0.0, self.countdown_sec - elapsed)
        remaining_int = int(remaining)
        now = rospy.Time.now()

        should_log_remaining = (
            self.last_countdown_log_time is None
            or (now - self.last_countdown_log_time).to_sec() >= self.countdown_log_period_sec
            or remaining <= 0.0
        )
        if remaining_int != self.last_remaining and should_log_remaining:
            rospy.loginfo("Gyrocompass remaining: %s", format_remaining(remaining))
            self.last_remaining = remaining_int
            self.last_countdown_log_time = now

        if remaining <= 0.0 and not self.countdown_done:
            rospy.loginfo(
                "Waiting %.0f seconds for finish frame.",
                self.finish_wait_timeout_sec,
            )
            self.countdown_done = True

        if (
            self.countdown_done
            and not self.finished_ack
            and not self.fail_reported
            and not self.unknown_reported
            and elapsed >= self.countdown_sec + self.finish_wait_timeout_sec
        ):
            if self.response_required and not self.started_ack and not self.seen_gyrocompass_mode:
                fail_message = (
                    "CALIBRATION FAILED: no gyrocompass start ACK/status and no finish frame were received."
                )
                rospy.logerr(fail_message)
                self.fail_reported = True
                shutdown_reason = "gyrocompass calibration failed"
                self.reapply_output_on_terminal_state("after gyrocompass failure")
            else:
                unknown_message = (
                    "CALIBRATION UNKNOWN: countdown ended, but finish frame "
                    "10 10 03 09 2D 00 00 00 01 was not received. "
                    "The start command was sent, but this node cannot prove the final device state."
                )
                rospy.logwarn(unknown_message)
                self.unknown_reported = True
                shutdown_reason = "gyrocompass calibration status unknown"
                self.reapply_output_on_terminal_state("after gyrocompass countdown")
            if self.shutdown_on_finish:
                rospy.signal_shutdown(shutdown_reason)

    def warn_if_no_rx(self):
        if not self.sent or self.finished_ack:
            return

        now = rospy.Time.now()
        if self.last_rx_time is not None:
            return

        if (
            self.last_no_rx_warn_time is None
            or (now - self.last_no_rx_warn_time).to_sec() >= self.no_rx_warn_period_sec
        ):
            rospy.logwarn(
                "No bytes received from INS since sending gyrocompass start. "
                "If this persists, the device is not sending ACK/status on this serial stream."
            )
            self.last_no_rx_warn_time = now

    def spin(self):
        if not self.wait_for_send():
            rospy.signal_shutdown("start command not sent")
            return

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            try:
                self.open()
                if not self.sent:
                    self.send_start()
                for frame in self.feed(self.serial.read(self.chunk_size)):
                    self.handle_frame(frame)
                self.warn_if_no_rx()
                self.update_countdown()
            except serial.SerialException as exc:
                rospy.logerr_throttle(2.0, "INS calib serial error: %s", exc)
                self.close()
                rospy.sleep(1.0)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("start_calib")
    StartCalibNode().spin()
