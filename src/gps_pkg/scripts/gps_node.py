#!/usr/bin/env python3

import rospy
import serial
from sensor_msgs.msg import NavSatFix, NavSatStatus

GPS_PORT = "/dev/ttyACM0"
BAUD_RATE = 9600

# Hệ số sai số giả định cho GPS dân dụng (m)
UERE = 4.0

# Lưu DOP mới nhất
latest_hdop = None
latest_vdop = None
latest_fix_quality = 0


def parse_gngga(sentence):
    global latest_hdop, latest_fix_quality

    parts = sentence.split(',')

    if parts[0].endswith("GGA") and len(parts) > 9 and parts[2] and parts[4]:
        try:
            # Latitude
            lat_raw = parts[2]
            lat_dir = parts[3]
            lat_deg = float(lat_raw[:2])
            lat_min = float(lat_raw[2:])
            latitude = lat_deg + lat_min / 60.0
            if lat_dir == "S":
                latitude = -latitude

            # Longitude
            lon_raw = parts[4]
            lon_dir = parts[5]
            lon_deg = float(lon_raw[:3])
            lon_min = float(lon_raw[3:])
            longitude = lon_deg + lon_min / 60.0
            if lon_dir == "W":
                longitude = -longitude

            # Fix quality
            latest_fix_quality = int(parts[6]) if parts[6] else 0

            # HDOP
            latest_hdop = float(parts[8]) if parts[8] else None

            # Altitude
            altitude = float(parts[9]) if parts[9] else 0.0

            return latitude, longitude, altitude

        except (ValueError, IndexError):
            pass

    return None, None, None


def parse_gngsa(sentence):
    global latest_hdop, latest_vdop

    parts = sentence.split(',')

    if parts[0].endswith("GSA") and len(parts) >= 18:
        try:
            # Theo chuẩn NMEA:
            # parts[-3] = PDOP
            # parts[-2] = HDOP
            # parts[-1] = VDOP*checksum
            pdop = parts[-3] if len(parts) >= 3 else None
            hdop = parts[-2] if len(parts) >= 2 else None
            vdop_raw = parts[-1] if len(parts) >= 1 else None

            if hdop:
                latest_hdop = float(hdop)

            if vdop_raw:
                vdop = vdop_raw.split('*')[0]
                latest_vdop = float(vdop)

        except (ValueError, IndexError):
            pass


def build_covariance(hdop, vdop):
    """
    Ước lượng covariance từ DOP
    sigma_h = HDOP * UERE
    sigma_v = VDOP * UERE
    covariance = sigma^2
    """
    # fallback nếu chưa có dữ liệu
    if hdop is None:
        hdop = 2.0
    if vdop is None:
        vdop = hdop * 1.5

    sigma_h = hdop * UERE
    sigma_v = vdop * UERE

    cov_xx = sigma_h ** 2
    cov_yy = sigma_h ** 2
    cov_zz = sigma_v ** 2

    return [
        cov_xx, 0.0,    0.0,
        0.0,    cov_yy, 0.0,
        0.0,    0.0,    cov_zz
    ]


def gps_publisher():
    rospy.init_node('gps_node', anonymous=True)
    pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=10)

    last_fix_time = None

    try:
        ser = serial.Serial(GPS_PORT, BAUD_RATE, timeout=1)
        rospy.loginfo(f"Đang đọc GPS từ {GPS_PORT}")
    except serial.SerialException as e:
        rospy.logerr(f"Không thể mở GPS: {e}")
        return

    while not rospy.is_shutdown():
        try:
            line = ser.readline().decode('ascii', errors='ignore').strip()

            if not line:
                if last_fix_time is not None and (rospy.Time.now() - last_fix_time).to_sec() > 1.0:
                    rospy.logwarn_throttle(2, "GPS data too old!")
                continue

            if line.startswith("$GNGSA") or line.startswith("$GPGSA") or line.startswith("$GLGSA"):
                parse_gngsa(line)
                continue

            if line.startswith("$GNGGA") or line.startswith("$GPGGA"):
                lat, lon, alt = parse_gngga(line)

                if lat is None or lon is None:
                    continue

                msg = NavSatFix()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "gps_link"

                if latest_fix_quality > 0:
                    msg.status.status = NavSatStatus.STATUS_FIX
                else:
                    msg.status.status = NavSatStatus.STATUS_NO_FIX

                msg.status.service = NavSatStatus.SERVICE_GPS

                msg.latitude = lat
                msg.longitude = lon
                msg.altitude = alt

                msg.position_covariance = build_covariance(latest_hdop, latest_vdop)
                msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

                pub.publish(msg)
                last_fix_time = msg.header.stamp

                rospy.loginfo_throttle(
                    2,
                    f"Publishing new GGA fix | lat={msg.latitude:.8f}, lon={msg.longitude:.8f}"
                )

        except Exception as e:
            rospy.logwarn(f"Lỗi khi đọc GPS: {e}")


if __name__ == "__main__":
    try:
        gps_publisher()
    except rospy.ROSInterruptException:
        pass
