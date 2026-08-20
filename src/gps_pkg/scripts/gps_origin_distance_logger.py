#!/usr/bin/env python3

import math

import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus


EARTH_RADIUS_M = 6371000.0


def _is_valid_fix(msg):
    return (
        msg.status.status != NavSatStatus.STATUS_NO_FIX
        and math.isfinite(msg.latitude)
        and math.isfinite(msg.longitude)
        and math.isfinite(msg.altitude)
    )


def _haversine_distance_m(lat1_deg, lon1_deg, lat2_deg, lon2_deg):
    lat1 = math.radians(lat1_deg)
    lon1 = math.radians(lon1_deg)
    lat2 = math.radians(lat2_deg)
    lon2 = math.radians(lon2_deg)

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    sin_dlat = math.sin(dlat / 2.0)
    sin_dlon = math.sin(dlon / 2.0)

    a = (
        sin_dlat * sin_dlat
        + math.cos(lat1) * math.cos(lat2) * sin_dlon * sin_dlon
    )
    a = min(1.0, max(0.0, a))
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return EARTH_RADIUS_M * c


class GpsOriginDistanceLogger:
    def __init__(self):
        self.gps_topic = rospy.get_param("~gps_topic", "/gps/fix")

        # Với GPS 10 Hz: 300 mẫu tương đương khoảng 30 giây.
        self.origin_sample_count = max(
            1,
            int(rospy.get_param("~origin_sample_count", 300)),
        )

        self.min_position_change_deg = float(
            rospy.get_param("~min_position_change_deg", 1e-10)
        )
        self.min_altitude_change_m = float(
            rospy.get_param("~min_altitude_change_m", 1e-6)
        )

        self.origin_samples = []
        self.origin_fix = None
        self.last_logged_fix = None
        self.fix_count = 0

        self.subscriber = rospy.Subscriber(
            self.gps_topic,
            NavSatFix,
            self._callback,
            queue_size=100,
        )

        rospy.loginfo(
            "GPS logger đang thu %d mẫu hợp lệ để lấy trung bình làm gốc trên topic %s",
            self.origin_sample_count,
            self.gps_topic,
        )

    def _set_averaged_origin(self):
        sample_count = len(self.origin_samples)

        mean_lat = sum(sample[0] for sample in self.origin_samples) / sample_count
        mean_lon = sum(sample[1] for sample in self.origin_samples) / sample_count
        mean_alt = sum(sample[2] for sample in self.origin_samples) / sample_count

        self.origin_fix = (mean_lat, mean_lon, mean_alt)
        self.last_logged_fix = self.origin_fix
        self.fix_count = 0

        rospy.loginfo(
            "GPS averaged origin set from %d samples: lat=%.8f, lon=%.8f, alt=%.3f",
            sample_count,
            mean_lat,
            mean_lon,
            mean_alt,
        )

        self.origin_samples.clear()

    def _callback(self, msg):
        if not _is_valid_fix(msg):
            rospy.logwarn_throttle(
                5.0,
                "Skipping GPS sample without valid fix.",
            )
            return

        current_fix = (
            msg.latitude,
            msg.longitude,
            msg.altitude,
        )

        # Giai đoạn warm-up: chỉ thu mẫu, chưa tính khoảng cách.
        if self.origin_fix is None:
            self.origin_samples.append(current_fix)
            collected = len(self.origin_samples)

            rospy.loginfo_throttle(
                1.0,
                "Collecting GPS origin samples: %d/%d",
                collected,
                self.origin_sample_count,
            )

            if collected >= self.origin_sample_count:
                self._set_averaged_origin()

            return

        if self.last_logged_fix is not None:
            lat_changed = abs(current_fix[0] - self.last_logged_fix[0])
            lon_changed = abs(current_fix[1] - self.last_logged_fix[1])
            alt_changed = abs(current_fix[2] - self.last_logged_fix[2])

            if (
                lat_changed < self.min_position_change_deg
                and lon_changed < self.min_position_change_deg
                and alt_changed < self.min_altitude_change_m
            ):
                return

        self.fix_count += 1
        self.last_logged_fix = current_fix

        horizontal_distance_m = _haversine_distance_m(
            self.origin_fix[0],
            self.origin_fix[1],
            msg.latitude,
            msg.longitude,
        )

        altitude_difference_m = msg.altitude - self.origin_fix[2]
        distance_3d_m = math.sqrt(
            horizontal_distance_m ** 2
            + altitude_difference_m ** 2
        )

        rospy.loginfo(
            "GPS fix #%d | horizontal distance: %.3f m | altitude difference: %+.3f m | "
            "3D distance: %.3f m | lat=%.8f, lon=%.8f, alt=%.3f",
            self.fix_count,
            horizontal_distance_m,
            altitude_difference_m,
            distance_3d_m,
            msg.latitude,
            msg.longitude,
            msg.altitude,
        )


if __name__ == "__main__":
    rospy.init_node("gps_origin_distance_logger")
    GpsOriginDistanceLogger()
    rospy.spin()
    