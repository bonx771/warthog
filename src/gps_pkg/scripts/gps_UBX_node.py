#!/usr/bin/env python3

import math
import struct
import time

import rospy
import serial
from sensor_msgs.msg import NavSatFix, NavSatStatus


# ============================================================
# CẤU HÌNH MẶC ĐỊNH
# ============================================================

GPS_PORT = "/dev/ttyACM0"
BAUD_RATE = 9600
FRAME_ID = "gps_link"

SERIAL_TIMEOUT = 0.2
READ_CHUNK_SIZE = 512
STALE_DATA_WARN_SEC = 1.0

NAVIGATION_RATE_HZ = 10
CONFIGURE_RECEIVER = True

RTK_FIXED_MIN_HORIZONTAL_ACCURACY_M = 0.05
RTK_FIXED_MIN_VERTICAL_ACCURACY_M = 0.10
RTK_FLOAT_MIN_HORIZONTAL_ACCURACY_M = 1.00
RTK_FLOAT_MIN_VERTICAL_ACCURACY_M = 2.00
NO_CARRIER_MIN_HORIZONTAL_ACCURACY_M = 2.00
NO_CARRIER_MIN_VERTICAL_ACCURACY_M = 3.00
WEAK_FIX_MIN_HORIZONTAL_ACCURACY_M = 100.00
WEAK_FIX_MIN_VERTICAL_ACCURACY_M = 150.00


# ============================================================
# UBX CONSTANTS
# ============================================================

UBX_SYNC = b"\xB5\x62"

UBX_CLASS_NAV = 0x01
UBX_ID_NAV_PVT = 0x07

UBX_CLASS_ACK = 0x05
UBX_ID_ACK_NAK = 0x00
UBX_ID_ACK_ACK = 0x01

UBX_CLASS_CFG = 0x06
UBX_ID_CFG_MSG = 0x01
UBX_ID_CFG_RATE = 0x08
UBX_ID_CFG_VALSET = 0x8A

# CFG-TMODE-MODE:
# 0 = DISABLED: rover/positioning mode
# 1 = SURVEY_IN: base station survey-in
# 2 = FIXED: fixed base station
CFG_TMODE_MODE_KEY = 0x20030001

TMODE_DISABLED = 0
TMODE_SURVEY_IN = 1
TMODE_FIXED = 2

UBX_CLASS_NMEA_STANDARD = 0xF0

NAV_PVT_PAYLOAD_LEN = 92
MAX_UBX_PAYLOAD_LEN = 4096


NMEA_MESSAGES = {
    "GGA": 0x00,
    "GLL": 0x01,
    "GSA": 0x02,
    "GSV": 0x03,
    "RMC": 0x04,
    "VTG": 0x05,
    "GRS": 0x06,
    "GST": 0x07,
    "ZDA": 0x08,
    "GBS": 0x09,
    "DTM": 0x0A,
}


# ============================================================
# UBX FRAME
# ============================================================

def ubx_checksum(data):
    ck_a = 0
    ck_b = 0

    for byte in data:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF

    return ck_a, ck_b


def build_ubx_frame(msg_class, msg_id, payload=b""):
    header = struct.pack(
        "<BBH",
        msg_class,
        msg_id,
        len(payload),
    )

    ck_a, ck_b = ubx_checksum(header + payload)

    return (
        UBX_SYNC
        + header
        + payload
        + bytes([ck_a, ck_b])
    )


def extract_ubx_frames(buffer):
    frames = []

    while True:
        sync_index = buffer.find(UBX_SYNC)

        if sync_index < 0:
            if len(buffer) > 1:
                del buffer[:-1]
            break

        if sync_index > 0:
            del buffer[:sync_index]

        if len(buffer) < 8:
            break

        payload_length = struct.unpack_from(
            "<H",
            buffer,
            4,
        )[0]

        if payload_length > MAX_UBX_PAYLOAD_LEN:
            rospy.logwarn_throttle(
                2,
                f"UBX payload length không hợp lệ: {payload_length}",
            )
            del buffer[0]
            continue

        frame_length = payload_length + 8

        if len(buffer) < frame_length:
            break

        frame = bytes(buffer[:frame_length])

        received_ck_a = frame[-2]
        received_ck_b = frame[-1]

        calculated_ck_a, calculated_ck_b = ubx_checksum(
            frame[2:-2]
        )

        if (
            received_ck_a != calculated_ck_a
            or received_ck_b != calculated_ck_b
        ):
            rospy.logwarn_throttle(
                2,
                "UBX frame sai checksum",
            )
            del buffer[0]
            continue

        msg_class = frame[2]
        msg_id = frame[3]
        payload = frame[6:-2]

        frames.append(
            (
                msg_class,
                msg_id,
                payload,
            )
        )

        del buffer[:frame_length]

    return frames


# ============================================================
# UBX ACK
# ============================================================

def wait_for_ack(
    ser,
    expected_class,
    expected_id,
    timeout_sec=1.0,
):
    deadline = time.monotonic() + timeout_sec
    ack_buffer = bytearray()

    while (
        not rospy.is_shutdown()
        and time.monotonic() < deadline
    ):
        try:
            available = ser.in_waiting

            if available <= 0:
                time.sleep(0.01)
                continue

            chunk = ser.read(
                min(
                    available,
                    READ_CHUNK_SIZE,
                )
            )

            if not chunk:
                continue

            ack_buffer.extend(chunk)

            frames = extract_ubx_frames(ack_buffer)

            for msg_class, msg_id, payload in frames:
                if msg_class != UBX_CLASS_ACK:
                    continue

                if len(payload) < 2:
                    continue

                acknowledged_class = payload[0]
                acknowledged_id = payload[1]

                if (
                    acknowledged_class != expected_class
                    or acknowledged_id != expected_id
                ):
                    continue

                if msg_id == UBX_ID_ACK_ACK:
                    return True

                if msg_id == UBX_ID_ACK_NAK:
                    rospy.logerr(
                        "F9P trả về ACK-NAK cho "
                        f"class=0x{expected_class:02X}, "
                        f"id=0x{expected_id:02X}"
                    )
                    return False

        except serial.SerialException as exc:
            rospy.logerr(
                f"Lỗi serial khi chờ ACK: {exc}"
            )
            return False

    rospy.logwarn(
        "Không nhận được ACK cho "
        f"class=0x{expected_class:02X}, "
        f"id=0x{expected_id:02X}"
    )

    return False


def send_ubx_command(
    ser,
    msg_class,
    msg_id,
    payload,
    wait_ack=True,
    ack_timeout_sec=1.0,
):
    try:
        frame = build_ubx_frame(
            msg_class,
            msg_id,
            payload,
        )

        ser.write(frame)
        ser.flush()

        if not wait_ack:
            return True

        return wait_for_ack(
            ser,
            msg_class,
            msg_id,
            timeout_sec=ack_timeout_sec,
        )

    except serial.SerialException as exc:
        rospy.logerr(
            f"Không gửi được UBX command: {exc}"
        )
        return False

def disable_time_mode(ser):
    """
    Tắt Time Mode / Survey-In / Fixed Base bằng UBX-CFG-VALSET.

    CFG-TMODE-MODE = 0:
        Receiver trở về chế độ rover và tính nghiệm vị trí 2D/3D.

    Cấu hình được ghi vào RAM nên sẽ được gửi lại mỗi lần node chạy.
    Không ghi flash, tránh hao chu kỳ ghi bộ nhớ.
    """

    # UBX-CFG-VALSET payload:
    #
    # byte 0: version = 0
    # byte 1: layers  = 0x01 -> RAM
    # byte 2: transaction = 0
    # byte 3: reserved = 0
    # byte 4..7: key ID, little-endian
    # byte 8: value E1 = 0, TMODE disabled

    payload = struct.pack(
        "<BBBBIB",
        0,                      # version
        0x01,                   # layer: RAM
        0,                      # transaction
        0,                      # reserved
        CFG_TMODE_MODE_KEY,     # 0x20030001
        TMODE_DISABLED,         # mode = 0
    )

    success = send_ubx_command(
        ser,
        UBX_CLASS_CFG,
        UBX_ID_CFG_VALSET,
        payload,
        wait_ack=True,
        ack_timeout_sec=1.0,
    )

    if success:
        rospy.loginfo(
            "Đã tắt Time Mode/Survey-In/Fixed Base; "
            "F9P chuyển về rover positioning mode"
        )
    else:
        rospy.logerr(
            "Không tắt được Time Mode bằng UBX-CFG-VALSET"
        )

    return success

# ============================================================
# F9P CONFIGURATION
# ============================================================

def configure_navigation_rate(
    ser,
    frequency_hz,
):
    frequency_hz = max(
        1,
        min(
            int(frequency_hz),
            20,
        ),
    )

    measurement_period_ms = int(
        round(1000.0 / frequency_hz)
    )

    payload = struct.pack(
        "<HHH",
        measurement_period_ms,
        1,
        1,
    )

    success = send_ubx_command(
        ser,
        UBX_CLASS_CFG,
        UBX_ID_CFG_RATE,
        payload,
    )

    if success:
        rospy.loginfo(
            "Đã cấu hình navigation rate: "
            f"{frequency_hz} Hz "
            f"({measurement_period_ms} ms)"
        )

    return success


def configure_message_rate(
    ser,
    target_class,
    target_id,
    usb_rate,
):
    payload = struct.pack(
        "<BBBBBBBB",
        target_class,
        target_id,
        0,          # I2C/DDC
        0,          # UART1
        0,          # UART2
        usb_rate,   # USB
        0,          # SPI
        0,          # reserved
    )

    return send_ubx_command(
        ser,
        UBX_CLASS_CFG,
        UBX_ID_CFG_MSG,
        payload,
    )


def configure_f9p(
    ser,
    navigation_rate_hz=10,
):
    rospy.loginfo("=" * 60)
    rospy.loginfo("Bắt đầu cấu hình u-blox ZED-F9P")
    rospy.loginfo("=" * 60)

    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except serial.SerialException as exc:
        rospy.logwarn(
            f"Không reset được serial buffer: {exc}"
        )

    success = True

    # --------------------------------------------------------
    # Bước 1: tắt Time Mode để F9P trở lại rover mode
    # --------------------------------------------------------

    if not disable_time_mode(ser):
        rospy.logerr(
            "Không thể chuyển F9P về rover mode"
        )
        success = False

    # Cho navigation engine một khoảng ngắn để áp dụng cấu hình
    time.sleep(0.2)

    # --------------------------------------------------------
    # Bước 2: đặt navigation rate
    # --------------------------------------------------------

    if not configure_navigation_rate(
        ser,
        navigation_rate_hz,
    ):
        rospy.logwarn(
            "Không xác nhận được navigation rate"
        )
        success = False

    nav_pvt_enabled = configure_message_rate(
        ser,
        UBX_CLASS_NAV,
        UBX_ID_NAV_PVT,
        usb_rate=1,
    )

    if nav_pvt_enabled:
        rospy.loginfo(
            "Đã bật UBX-NAV-PVT trên cổng USB"
        )
    else:
        rospy.logerr(
            "Không bật được UBX-NAV-PVT trên USB"
        )
        success = False

    for message_name, message_id in NMEA_MESSAGES.items():
        disabled = configure_message_rate(
            ser,
            UBX_CLASS_NMEA_STANDARD,
            message_id,
            usb_rate=0,
        )

        if disabled:
            rospy.loginfo(
                f"Đã tắt NMEA-{message_name} trên USB"
            )
        else:
            rospy.logwarn(
                f"Không tắt được NMEA-{message_name}"
            )
            success = False

    time.sleep(0.3)

    try:
        ser.reset_input_buffer()
    except serial.SerialException as exc:
        rospy.logwarn(
            f"Không xóa được input buffer: {exc}"
        )

    if success:
        rospy.loginfo("=" * 60)
        rospy.loginfo(
            "Cấu hình F9P hoàn tất: "
            f"UBX-NAV-PVT, {navigation_rate_hz} Hz"
        )
        rospy.loginfo("=" * 60)
    else:
        rospy.logwarn(
            "Một số cấu hình F9P không nhận được ACK. "
            "Node vẫn tiếp tục đọc NAV-PVT."
        )

    return success


# ============================================================
# NAV-PVT PARSER
# ============================================================

def parse_nav_pvt(payload):
    if len(payload) < NAV_PVT_PAYLOAD_LEN:
        rospy.logwarn_throttle(
            2,
            "NAV-PVT payload quá ngắn: "
            f"{len(payload)} byte"
        )
        return None


    valid = payload[11]
    flags = payload[21]
    flags2 = payload[22]

    return {
        "valid": valid,
        "valid_date": bool(valid & 0x01),
        "valid_time": bool(valid & 0x02),
        "fully_resolved": bool(valid & 0x04),
        "valid_mag": bool(valid & 0x08),

        "flags": flags,
        "flags2": flags2,

        "fix_type": payload[20],
        "gnss_fix_ok": bool(flags & 0x01),
        "diff_soln": bool(flags & 0x02),
        "psm_state": (flags >> 2) & 0x07,
        "head_vehicle_valid": bool(flags & 0x20),
        "carr_soln": (flags >> 6) & 0x03,

        "confirmed_available": bool(flags2 & 0x20),
        "confirmed_date": bool(flags2 & 0x40),
        "confirmed_time": bool(flags2 & 0x80),

        "num_sv": payload[23],

        "longitude": (
            struct.unpack_from(
                "<i",
                payload,
                24,
            )[0]
            * 1e-7
        ),

        "latitude": (
            struct.unpack_from(
                "<i",
                payload,
                28,
            )[0]
            * 1e-7
        ),

        "height_ellipsoid": (
            struct.unpack_from(
                "<i",
                payload,
                32,
            )[0]
            / 1000.0
        ),

        "height_msl": (
            struct.unpack_from(
                "<i",
                payload,
                36,
            )[0]
            / 1000.0
        ),

        "horizontal_accuracy": (
            struct.unpack_from(
                "<I",
                payload,
                40,
            )[0]
            / 1000.0
        ),

        "vertical_accuracy": (
            struct.unpack_from(
                "<I",
                payload,
                44,
            )[0]
            / 1000.0
        ),

        "velocity_north": (
            struct.unpack_from(
                "<i",
                payload,
                48,
            )[0]
            / 1000.0
        ),

        "velocity_east": (
            struct.unpack_from(
                "<i",
                payload,
                52,
            )[0]
            / 1000.0
        ),

        "velocity_down": (
            struct.unpack_from(
                "<i",
                payload,
                56,
            )[0]
            / 1000.0
        ),

        "ground_speed": (
            struct.unpack_from(
                "<i",
                payload,
                60,
            )[0]
            / 1000.0
        ),

        "heading_motion": (
            struct.unpack_from(
                "<i",
                payload,
                64,
            )[0]
            * 1e-5
        ),
    }


def has_valid_position_fix(nav_pvt):
    return (
        nav_pvt["gnss_fix_ok"]
        and nav_pvt["fix_type"] in (2, 3, 4)
        and nav_pvt["num_sv"] > 0
        and math.isfinite(nav_pvt["latitude"])
        and math.isfinite(nav_pvt["longitude"])
        and -90.0 <= nav_pvt["latitude"] <= 90.0
        and -180.0 <= nav_pvt["longitude"] <= 180.0
    )


def get_fix_description(nav_pvt):
    if not has_valid_position_fix(nav_pvt):
        if nav_pvt["fix_type"] == 0:
            return "NO FIX"

        if nav_pvt["fix_type"] == 1:
            return "DEAD RECKONING ONLY"

        if nav_pvt["fix_type"] == 5:
            return "TIME ONLY"

        return f"INVALID FIX TYPE {nav_pvt['fix_type']}"

    if nav_pvt["carr_soln"] == 2:
        return "RTK FIXED"

    if nav_pvt["carr_soln"] == 1:
        return "RTK FLOAT"

    if nav_pvt["diff_soln"]:
        return "DGNSS"

    if nav_pvt["fix_type"] == 2:
        return "2D FIX"

    if nav_pvt["fix_type"] == 3:
        return "3D FIX"

    if nav_pvt["fix_type"] == 4:
        return "GNSS + DR"

    return "GNSS FIX"


def map_nav_status(nav_pvt):
    if not has_valid_position_fix(nav_pvt):
        return NavSatStatus.STATUS_NO_FIX

    if nav_pvt["carr_soln"] in (1, 2):
        return NavSatStatus.STATUS_GBAS_FIX

    if nav_pvt["diff_soln"]:
        return NavSatStatus.STATUS_SBAS_FIX

    return NavSatStatus.STATUS_FIX


# ============================================================
# ROS NAVSATFIX
# ============================================================

def get_positive_float_param(name, default_value):
    value = float(
        rospy.get_param(
            name,
            default_value,
        )
    )

    if (
        not math.isfinite(value)
        or value <= 0.0
    ):
        rospy.logwarn(
            "%s không hợp lệ (%.3f), dùng mặc định %.3f",
            name,
            value,
            default_value,
        )
        return default_value

    return value


def get_covariance_config():
    return {
        "rtk_fixed_min_horizontal": get_positive_float_param(
            "~rtk_fixed_min_horizontal_accuracy_m",
            RTK_FIXED_MIN_HORIZONTAL_ACCURACY_M,
        ),
        "rtk_fixed_min_vertical": get_positive_float_param(
            "~rtk_fixed_min_vertical_accuracy_m",
            RTK_FIXED_MIN_VERTICAL_ACCURACY_M,
        ),
        "rtk_float_min_horizontal": get_positive_float_param(
            "~rtk_float_min_horizontal_accuracy_m",
            RTK_FLOAT_MIN_HORIZONTAL_ACCURACY_M,
        ),
        "rtk_float_min_vertical": get_positive_float_param(
            "~rtk_float_min_vertical_accuracy_m",
            RTK_FLOAT_MIN_VERTICAL_ACCURACY_M,
        ),
        "no_carrier_min_horizontal": get_positive_float_param(
            "~no_carrier_min_horizontal_accuracy_m",
            NO_CARRIER_MIN_HORIZONTAL_ACCURACY_M,
        ),
        "no_carrier_min_vertical": get_positive_float_param(
            "~no_carrier_min_vertical_accuracy_m",
            NO_CARRIER_MIN_VERTICAL_ACCURACY_M,
        ),
        "weak_fix_min_horizontal": get_positive_float_param(
            "~weak_fix_min_horizontal_accuracy_m",
            WEAK_FIX_MIN_HORIZONTAL_ACCURACY_M,
        ),
        "weak_fix_min_vertical": get_positive_float_param(
            "~weak_fix_min_vertical_accuracy_m",
            WEAK_FIX_MIN_VERTICAL_ACCURACY_M,
        ),
    }


def get_valid_covariance_floor(nav_pvt, covariance_config):
    if nav_pvt["carr_soln"] == 2:
        return (
            covariance_config["rtk_fixed_min_horizontal"],
            covariance_config["rtk_fixed_min_vertical"],
        )

    if nav_pvt["carr_soln"] == 1:
        return (
            covariance_config["rtk_float_min_horizontal"],
            covariance_config["rtk_float_min_vertical"],
        )

    return (
        covariance_config["no_carrier_min_horizontal"],
        covariance_config["no_carrier_min_vertical"],
    )


def build_valid_covariance(
    horizontal_accuracy,
    vertical_accuracy,
    min_horizontal_accuracy,
    min_vertical_accuracy,
):
    if (
        not math.isfinite(horizontal_accuracy)
        or horizontal_accuracy <= 0.0
    ):
        horizontal_accuracy = min_horizontal_accuracy

    if (
        not math.isfinite(vertical_accuracy)
        or vertical_accuracy <= 0.0
    ):
        vertical_accuracy = min_vertical_accuracy

    sigma_h = max(
        horizontal_accuracy,
        min_horizontal_accuracy,
    )

    sigma_v = max(
        vertical_accuracy,
        min_vertical_accuracy,
    )

    return [
        sigma_h ** 2, 0.0,          0.0,
        0.0,          sigma_h ** 2, 0.0,
        0.0,          0.0,          sigma_v ** 2,
    ]


def build_weak_covariance(
    horizontal_accuracy,
    vertical_accuracy,
    min_horizontal_accuracy,
    min_vertical_accuracy,
):
    """
    Covariance yếu cho dữ liệu chưa có position fix hợp lệ.

    Lưu ý:
    - Message vẫn mang status = STATUS_NO_FIX.
    - Tọa độ không nên được EKF sử dụng khi status = -1.
    """
    if (
        not math.isfinite(horizontal_accuracy)
        or horizontal_accuracy <= 0.0
    ):
        horizontal_accuracy = min_horizontal_accuracy

    if (
        not math.isfinite(vertical_accuracy)
        or vertical_accuracy <= 0.0
    ):
        vertical_accuracy = min_vertical_accuracy

    sigma_h = max(
        horizontal_accuracy,
        min_horizontal_accuracy,
    )

    sigma_v = max(
        vertical_accuracy,
        min_vertical_accuracy,
    )

    return [
        sigma_h ** 2, 0.0,          0.0,
        0.0,          sigma_h ** 2, 0.0,
        0.0,          0.0,          sigma_v ** 2,
    ]


def build_navsat_fix(
    nav_pvt,
    frame_id,
    covariance_config,
):
    msg = NavSatFix()

    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id

    valid_position = has_valid_position_fix(nav_pvt)

    msg.status.status = map_nav_status(nav_pvt)

    msg.status.service = (
        NavSatStatus.SERVICE_GPS
        | NavSatStatus.SERVICE_GLONASS
        | NavSatStatus.SERVICE_COMPASS
        | NavSatStatus.SERVICE_GALILEO
    )

    # Luôn đưa các giá trị receiver trả về vào message.
    # Khi không có position fix, status sẽ bằng STATUS_NO_FIX.
    msg.latitude = nav_pvt["latitude"]
    msg.longitude = nav_pvt["longitude"]
    msg.altitude = nav_pvt["height_ellipsoid"]

    if valid_position:
        min_horizontal_accuracy, min_vertical_accuracy = (
            get_valid_covariance_floor(
                nav_pvt,
                covariance_config,
            )
        )

        msg.position_covariance = build_valid_covariance(
            nav_pvt["horizontal_accuracy"],
            nav_pvt["vertical_accuracy"],
            min_horizontal_accuracy,
            min_vertical_accuracy,
        )

        msg.position_covariance_type = (
            NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        )

    else:
        msg.position_covariance = build_weak_covariance(
            nav_pvt["horizontal_accuracy"],
            nav_pvt["vertical_accuracy"],
            covariance_config["weak_fix_min_horizontal"],
            covariance_config["weak_fix_min_vertical"],
        )

        msg.position_covariance_type = (
            NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        )

    return msg


# ============================================================
# SERIAL
# ============================================================

def open_serial(
    port,
    baud_rate,
    timeout,
):
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud_rate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
            write_timeout=1.0,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )

        rospy.loginfo(
            f"Đã mở GPS tại {port}, baud={baud_rate}"
        )

        return ser

    except serial.SerialException as exc:
        rospy.logerr(
            f"Không thể mở GPS tại {port}: {exc}"
        )
        return None


# ============================================================
# MAIN NODE
# ============================================================

def gps_publisher():
    rospy.init_node(
        "gps_UBX_node",
        anonymous=False,
    )

    pub = rospy.Publisher(
        "/gps/fix",
        NavSatFix,
        queue_size=10,
    )

    port = rospy.get_param(
        "~port",
        GPS_PORT,
    )

    baud_rate = int(
        rospy.get_param(
            "~baud_rate",
            BAUD_RATE,
        )
    )

    frame_id = rospy.get_param(
        "~frame_id",
        FRAME_ID,
    )

    serial_timeout = float(
        rospy.get_param(
            "~serial_timeout",
            SERIAL_TIMEOUT,
        )
    )

    read_chunk_size = max(
        1,
        int(
            rospy.get_param(
                "~read_chunk_size",
                READ_CHUNK_SIZE,
            )
        ),
    )

    stale_data_warn_sec = max(
        0.1,
        float(
            rospy.get_param(
                "~stale_data_warn_sec",
                STALE_DATA_WARN_SEC,
            )
        ),
    )

    navigation_rate_hz = max(
        1,
        min(
            20,
            int(
                rospy.get_param(
                    "~navigation_rate_hz",
                    NAVIGATION_RATE_HZ,
                )
            ),
        ),
    )

    configure_receiver = bool(
        rospy.get_param(
            "~configure_receiver",
            CONFIGURE_RECEIVER,
        )
    )

    covariance_config = get_covariance_config()
    rospy.loginfo(
        "GPS covariance floors | "
        "no_carrier: horizontal=%.3f m vertical=%.3f m, "
        "rtk_float: horizontal=%.3f m vertical=%.3f m, "
        "rtk_fixed: horizontal=%.3f m vertical=%.3f m",
        covariance_config["no_carrier_min_horizontal"],
        covariance_config["no_carrier_min_vertical"],
        covariance_config["rtk_float_min_horizontal"],
        covariance_config["rtk_float_min_vertical"],
        covariance_config["rtk_fixed_min_horizontal"],
        covariance_config["rtk_fixed_min_vertical"],
    )

    ser = open_serial(
        port,
        baud_rate,
        serial_timeout,
    )

    if ser is None:
        return

    if configure_receiver:
        configure_f9p(
            ser,
            navigation_rate_hz=navigation_rate_hz,
        )
    else:
        rospy.logwarn(
            "Tự động cấu hình F9P đang tắt"
        )

    buffer = bytearray()
    loop_rate = rospy.Rate(200)

    startup_time = rospy.Time.now()
    last_nav_pvt_time = None
    last_valid_fix_time = None

    try:
        while not rospy.is_shutdown():
            try:
                available_bytes = ser.in_waiting

                if available_bytes > 0:
                    bytes_to_read = min(
                        available_bytes,
                        read_chunk_size,
                    )
                else:
                    bytes_to_read = 1

                chunk = ser.read(bytes_to_read)

                if chunk:
                    buffer.extend(chunk)

                    frames = extract_ubx_frames(buffer)

                    for msg_class, msg_id, payload in frames:
                        if (
                            msg_class != UBX_CLASS_NAV
                            or msg_id != UBX_ID_NAV_PVT
                        ):
                            continue

                        last_nav_pvt_time = rospy.Time.now()

                        nav_pvt = parse_nav_pvt(payload)

                        if nav_pvt is None:
                            continue

                        valid_position = has_valid_position_fix(
                            nav_pvt
                        )

                        # Luôn publish, kể cả fixType 0, 1 hoặc 5.
                        msg = build_navsat_fix(
                            nav_pvt,
                            frame_id,
                            covariance_config,
                        )

                        pub.publish(msg)

                        # ==========================================================
                        # GPS QUALITY REPORT
                        # ==========================================================

                        sigma_x = math.sqrt(msg.position_covariance[0])
                        sigma_y = math.sqrt(msg.position_covariance[4])
                        sigma_z = math.sqrt(msg.position_covariance[8])

                        h_acc = nav_pvt["horizontal_accuracy"]
                        v_acc = nav_pvt["vertical_accuracy"]

                        if valid_position:
                            accuracy_text = (
                                f"Receiver hAcc/vAcc ±{h_acc:.3f}/±{v_acc:.3f} m; "
                                f"EKF sigma ±{sigma_x:.3f}/±{sigma_z:.3f} m"
                            )
                        else:
                            accuracy_text = (
                                "KHÔNG CÓ NGHIỆM VỊ TRÍ HỢP LỆ; "
                                f"hAcc={h_acc:.3f} m, vAcc={v_acc:.3f} m "
                                "chỉ dùng để theo dõi"
                            )

                        rospy.loginfo_throttle(
                            1,
                            "\n"
                            "================ GPS QUALITY ================\n"
                            f"Fix           : {get_fix_description(nav_pvt)}\n"
                            f"Status        : {msg.status.status}\n"
                            f"Satellites    : {nav_pvt['num_sv']}\n"
                            f"Latitude      : {msg.latitude:.8f}\n"
                            f"Longitude     : {msg.longitude:.8f}\n"
                            f"Altitude      : {msg.altitude:.3f} m\n"
                            f"hAcc          : {nav_pvt['horizontal_accuracy']:.3f} m\n"
                            f"vAcc          : {nav_pvt['vertical_accuracy']:.3f} m\n"
                            f"Cov(X)        : {msg.position_covariance[0]:.4f} m²\n"
                            f"Cov(Y)        : {msg.position_covariance[4]:.4f} m²\n"
                            f"Cov(Z)        : {msg.position_covariance[8]:.4f} m²\n"
                            f"Sigma(X)      : {sigma_x:.3f} m\n"
                            f"Sigma(Y)      : {sigma_y:.3f} m\n"
                            f"Sigma(Z)      : {sigma_z:.3f} m\n"
                            f"Sigma floor   : carrSoln={nav_pvt['carr_soln']}\n"
                            f"Sai số ước lượng: {accuracy_text}\n"
                            f"valid raw     : 0x{nav_pvt['valid']:02X}\n"
                            f"flags raw     : 0x{nav_pvt['flags']:02X}\n"
                            f"flags2 raw    : 0x{nav_pvt['flags2']:02X}\n"
                            f"validDate     : {nav_pvt['valid_date']}\n"
                            f"validTime     : {nav_pvt['valid_time']}\n"
                            f"fullyResolved : {nav_pvt['fully_resolved']}\n"
                            f"gnssFixOK     : {nav_pvt['gnss_fix_ok']}\n"
                            f"diffSoln      : {nav_pvt['diff_soln']}\n"
                            f"carrSoln      : {nav_pvt['carr_soln']}\n"
                            "============================================="
                        )

                        fix_description = get_fix_description(
                            nav_pvt
                        )

                        if valid_position:
                            last_valid_fix_time = msg.header.stamp

                            rospy.loginfo_throttle(
                                2,
                                "Publishing UBX-NAV-PVT | "
                                f"{fix_description} | "
                                f"lat={msg.latitude:.8f}, "
                                f"lon={msg.longitude:.8f}, "
                                f"alt={msg.altitude:.3f} m, "
                                f"hAcc={nav_pvt['horizontal_accuracy']:.3f} m, "
                                f"vAcc={nav_pvt['vertical_accuracy']:.3f} m, "
                                f"SV={nav_pvt['num_sv']}"
                            )

                        else:
                            rospy.logwarn_throttle(
                                2,
                                "Publishing weak/no-position NAV-PVT | "
                                f"{fix_description} | "
                                f"fixType={nav_pvt['fix_type']}, "
                                f"lat={msg.latitude:.8f}, "
                                f"lon={msg.longitude:.8f}, "
                                f"hAcc={nav_pvt['horizontal_accuracy']:.3f} m, "
                                f"vAcc={nav_pvt['vertical_accuracy']:.3f} m, "
                                f"SV={nav_pvt['num_sv']}, "
                                "status=NO_FIX"
                            )

                now = rospy.Time.now()

                if last_nav_pvt_time is None:
                    startup_elapsed = (
                        now - startup_time
                    ).to_sec()

                    if startup_elapsed > stale_data_warn_sec:
                        rospy.logwarn_throttle(
                            5,
                            "Chưa nhận được UBX-NAV-PVT"
                        )

                else:
                    nav_pvt_age = (
                        now - last_nav_pvt_time
                    ).to_sec()

                    if nav_pvt_age > stale_data_warn_sec:
                        rospy.logwarn_throttle(
                            2,
                            "UBX-NAV-PVT quá cũ: "
                            f"{nav_pvt_age:.2f} giây"
                        )

                if last_valid_fix_time is not None:
                    valid_fix_age = (
                        now - last_valid_fix_time
                    ).to_sec()

                    if valid_fix_age > stale_data_warn_sec:
                        rospy.logwarn_throttle(
                            2,
                            "Không có position fix hợp lệ trong "
                            f"{valid_fix_age:.2f} giây"
                        )

            except serial.SerialTimeoutException as exc:
                rospy.logwarn_throttle(
                    2,
                    f"Serial timeout: {exc}"
                )

            except serial.SerialException as exc:
                rospy.logerr(
                    f"Lỗi serial GPS: {exc}"
                )
                break

            except Exception as exc:
                rospy.logerr_throttle(
                    1,
                    f"Lỗi xử lý GPS UBX: {exc}"
                )

            loop_rate.sleep()

    finally:
        if ser is not None and ser.is_open:
            ser.close()
            rospy.loginfo(
                "Đã đóng cổng GPS"
            )


if __name__ == "__main__":
    try:
        gps_publisher()
    except rospy.ROSInterruptException:
        pass
