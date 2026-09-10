#!/usr/bin/env python3
"""Prepare a 3-D RoboSense cloud for the obstacle-only navigation stack.

The node estimates the local ground plane, removes ground and self returns,
keeps points that intersect the UGV vertical envelope, and voxel-downsamples
the result.  A second cloud retains all non-self returns for costmap clearing
and explicitly raytraces the recently marked obstacle endpoints.  Clearing the
old endpoints before marking the current scan makes moving obstacles decay in
one or two LiDAR frames instead of becoming permanent VoxelLayer trails.
Both outputs contain XYZ float32 fields in the original LiDAR frame.
"""

import math
import threading

import numpy as np
import rospy
import tf
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, Header


class LidarPreprocessObstacle:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/rslidar_points")
        self.obstacle_topic = rospy.get_param(
            "~obstacle_topic", "/outdoor_waypoint_nav/obstacle_points_obstacle"
        )
        self.clearing_topic = rospy.get_param(
            "~clearing_topic", "/outdoor_waypoint_nav/clearing_points_obstacle"
        )
        self.enable_topic = rospy.get_param(
            "~enable_topic",
            "/outdoor_waypoint_nav/lidar_costmap_enabled_obstacle",
        )
        # Fail closed.  The localization guard publishes a latched True only
        # after IMU/EKF yaw is stable and old costmap cells have been cleared.
        self.state_lock = threading.Lock()
        self.output_enabled = False
        self.previous_obstacle_frames = []

        # Physical dimensions supplied for the real UGV.
        self.ugv_length = float(rospy.get_param("~ugv_length", 1.52))
        self.ugv_width = float(rospy.get_param("~ugv_width", 1.38))
        self.ugv_height = float(rospy.get_param("~ugv_height", 0.83))
        self.lidar_height_above_ground = float(
            rospy.get_param("~lidar_height_above_ground", 1.23)
        )

        self.min_obstacle_height = float(
            rospy.get_param("~min_obstacle_height", 0.08)
        )
        self.max_obstacle_height = float(
            rospy.get_param(
                "~max_obstacle_height",
                max(self.ugv_height, self.lidar_height_above_ground) + 0.10,
            )
        )
        self.self_filter_padding = float(
            rospy.get_param("~self_filter_padding", 0.08)
        )
        self.self_min_z = float(
            rospy.get_param("~self_min_z", -self.lidar_height_above_ground - 0.10)
        )
        self.self_max_z = float(rospy.get_param("~self_max_z", 0.10))

        self.min_range = float(rospy.get_param("~min_range", 0.50))
        self.max_range = float(rospy.get_param("~max_range", 15.0))
        self.marking_leaf_size = float(
            rospy.get_param("~marking_leaf_size", 0.08)
        )
        self.clearing_leaf_size = float(
            rospy.get_param("~clearing_leaf_size", 0.12)
        )
        self.clear_previous_obstacles = bool(
            rospy.get_param("~clear_previous_obstacles", True)
        )
        self.previous_clear_frames = int(
            rospy.get_param("~previous_clear_frames", 2)
        )
        # VoxelLayer deliberately shortens every clearing ray by two costmap
        # cells.  The obstacle stack uses 0.10 m local and 0.20 m global cells,
        # so an old endpoint must be extended by more than 0.40 m before it
        # can clear the voxel that was marked at that endpoint.
        self.previous_clear_extension = float(
            rospy.get_param("~previous_clear_extension", 0.60)
        )
        # Historical endpoints must live in a non-moving frame.  Reusing old
        # coordinates in rslidar would clear the wrong world cells after the
        # vehicle translates or rotates.
        self.clearing_fixed_frame = rospy.get_param(
            "~clearing_fixed_frame", "odom"
        )
        self.transform_timeout = float(
            rospy.get_param("~transform_timeout", 0.05)
        )

        self.ground_candidate_band = float(
            rospy.get_param("~ground_candidate_band", 0.60)
        )
        self.ground_fit_min_range = float(
            rospy.get_param("~ground_fit_min_range", 0.80)
        )
        self.ground_fit_max_range = float(
            rospy.get_param("~ground_fit_max_range", 12.0)
        )
        self.ground_distance_threshold = float(
            rospy.get_param("~ground_distance_threshold", 0.07)
        )
        self.max_ground_tilt_deg = float(
            rospy.get_param("~max_ground_tilt_deg", 18.0)
        )
        self.ground_origin_tolerance = float(
            rospy.get_param("~ground_origin_tolerance", 0.45)
        )
        self.ransac_iterations = int(rospy.get_param("~ransac_iterations", 80))
        self.max_ground_candidates = int(
            rospy.get_param("~max_ground_candidates", 12000)
        )
        self.min_ground_inliers = int(
            rospy.get_param("~min_ground_inliers", 300)
        )
        self.random_generator = np.random.default_rng(
            int(rospy.get_param("~random_seed", 17))
        )

        if self.min_obstacle_height < 0.0:
            raise ValueError("min_obstacle_height must be non-negative")
        if self.max_obstacle_height <= self.min_obstacle_height:
            raise ValueError("max_obstacle_height must exceed min_obstacle_height")
        if self.max_range <= self.min_range:
            raise ValueError("max_range must exceed min_range")
        if self.previous_clear_frames < 1:
            raise ValueError("previous_clear_frames must be at least 1")
        if self.previous_clear_extension <= 0.0:
            raise ValueError("previous_clear_extension must be positive")

        self.obstacle_pub = rospy.Publisher(
            self.obstacle_topic, PointCloud2, queue_size=1
        )
        self.clearing_pub = rospy.Publisher(
            self.clearing_topic, PointCloud2, queue_size=1
        )
        self.tf_listener = tf.TransformListener(cache_time=rospy.Duration(5.0))
        self.enable_subscriber = rospy.Subscriber(
            self.enable_topic, Bool, self.enable_callback, queue_size=1
        )
        self.subscriber = rospy.Subscriber(
            self.input_topic,
            PointCloud2,
            self.cloud_callback,
            queue_size=1,
            buff_size=32 * 1024 * 1024,
            tcp_nodelay=True,
        )

        rospy.loginfo(
            "[lidar_obstacle] input=%s marking=%s clearing=%s | "
            "UGV=%.2fx%.2fx%.2fm lidar_height=%.2fm obstacle_height=%.2f..%.2fm",
            self.input_topic,
            self.obstacle_topic,
            self.clearing_topic,
            self.ugv_length,
            self.ugv_width,
            self.ugv_height,
            self.lidar_height_above_ground,
            self.min_obstacle_height,
            self.max_obstacle_height,
        )
        rospy.logwarn(
            "[lidar_obstacle] Costmap output is LOCKED until %s is true.",
            self.enable_topic,
        )
        rospy.loginfo(
            "[lidar_obstacle] Current-snapshot clearing=%s history=%d frame(s) "
            "extension=%.2fm fixed_frame=%s.",
            self.clear_previous_obstacles,
            self.previous_clear_frames,
            self.previous_clear_extension,
            self.clearing_fixed_frame,
        )

    def enable_callback(self, msg):
        enabled = bool(msg.data)
        with self.state_lock:
            if enabled == self.output_enabled:
                return
            self.output_enabled = enabled
            if not enabled:
                # The guard clears both costmaps before enabling output again;
                # do not replay endpoints transformed with the old yaw.
                self.previous_obstacle_frames = []
        if enabled:
            rospy.loginfo(
                "[lidar_obstacle] Costmap output ENABLED after localization stabilization."
            )
        else:
            rospy.logerr(
                "[lidar_obstacle] Costmap output LOCKED because localization is not ready."
            )

    @staticmethod
    def _field(msg, name):
        for field in msg.fields:
            if field.name == name:
                if field.datatype != PointField.FLOAT32 or field.count != 1:
                    raise ValueError("PointCloud2 field '{}' must be FLOAT32[1]".format(name))
                return field
        raise ValueError("PointCloud2 is missing '{}' field".format(name))

    @classmethod
    def _read_xyz(cls, msg):
        if msg.width == 0 or msg.height == 0:
            return np.empty((0, 3), dtype=np.float32)

        byte_order = ">" if msg.is_bigendian else "<"
        shape = (msg.height, msg.width)
        strides = (msg.row_step, msg.point_step)
        output = np.empty((msg.height * msg.width, 3), dtype=np.float32)
        for column, name in enumerate(("x", "y", "z")):
            field = cls._field(msg, name)
            values = np.ndarray(
                shape=shape,
                dtype=np.dtype(byte_order + "f4"),
                buffer=msg.data,
                offset=field.offset,
                strides=strides,
            )
            output[:, column] = values.reshape(-1)
        return output

    @staticmethod
    def _build_cloud(header, points):
        points = np.ascontiguousarray(points, dtype="<f4")
        output = PointCloud2()
        output.header = header
        output.height = 1
        output.width = int(points.shape[0])
        output.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        output.is_bigendian = False
        output.point_step = 12
        output.row_step = output.point_step * output.width
        output.data = points.tobytes()
        output.is_dense = True
        return output

    @staticmethod
    def _voxel_downsample(points, leaf_size):
        if points.shape[0] == 0 or leaf_size <= 0.0:
            return points
        voxel_keys = np.floor(points / leaf_size).astype(np.int32)
        _, first_indices = np.unique(voxel_keys, axis=0, return_index=True)
        return points[np.sort(first_indices)]

    def _output_is_enabled(self):
        with self.state_lock:
            return self.output_enabled

    @staticmethod
    def _transform_points(points, translation, quaternion):
        if points.shape[0] == 0:
            return points.copy()
        rotation = tf.transformations.quaternion_matrix(quaternion)[:3, :3]
        transformed = np.dot(points.astype(np.float64, copy=False), rotation.T)
        transformed += np.asarray(translation, dtype=np.float64)
        return transformed.astype(np.float32)

    def _points_in_clearing_frame(self, header, obstacle_points, clearing_points):
        if header.frame_id == self.clearing_fixed_frame:
            return obstacle_points.copy(), clearing_points.copy(), np.zeros(3)
        try:
            self.tf_listener.waitForTransform(
                self.clearing_fixed_frame,
                header.frame_id,
                header.stamp,
                rospy.Duration(self.transform_timeout),
            )
            translation, quaternion = self.tf_listener.lookupTransform(
                self.clearing_fixed_frame, header.frame_id, header.stamp
            )
        except (tf.Exception, tf.LookupException, tf.ConnectivityException) as exc:
            rospy.logwarn_throttle(
                2.0,
                "[lidar_obstacle] Cannot transform clearing snapshot %s -> %s: %s. "
                "Publishing current clearing only; history is not replayed.",
                header.frame_id,
                self.clearing_fixed_frame,
                exc,
            )
            return None

        return (
            self._transform_points(obstacle_points, translation, quaternion),
            self._transform_points(clearing_points, translation, quaternion),
            np.asarray(translation, dtype=np.float64),
        )

    def _extend_historical_endpoints(self, points, sensor_origin):
        """Move old endpoints farther along their current sensor rays.

        costmap_2d::VoxelLayer subtracts 2 * costmap resolution from every
        clearing ray.  Without this extension, replaying an old hit stops just
        before the voxel that the same hit marked and a moving person leaves a
        permanent trail.
        """
        if points.shape[0] == 0:
            return points
        vectors = points.astype(np.float64, copy=False) - sensor_origin
        distances = np.linalg.norm(vectors, axis=1)
        valid = distances > 1e-3
        extended = points.astype(np.float64, copy=True)
        scale = np.ones(points.shape[0], dtype=np.float64)
        scale[valid] += self.previous_clear_extension / distances[valid]
        extended[valid] = sensor_origin + vectors[valid] * scale[valid, None]
        return extended.astype(np.float32)

    def _publish_current_snapshot(self, header, obstacle_points, clearing_points):
        """Clear recent marks, then publish only the current obstacle snapshot.

        costmap_2d's VoxelLayer processes clearing observations before marking
        observations during each update.  Its raytracer includes the endpoint,
        so replaying a previous obstacle point on the clearing-only topic
        removes that exact old voxel.  A static obstacle is immediately marked
        again by the current scan, while a person who moved away is not.
        """
        empty = np.empty((0, 3), dtype=np.float32)
        fixed_points = self._points_in_clearing_frame(
            header, obstacle_points, clearing_points
        )
        if fixed_points is None:
            obstacle_points_fixed = None
            sensor_origin_fixed = None
            output_clearing_points = clearing_points
            clearing_header = header
        else:
            (
                obstacle_points_fixed,
                output_clearing_points,
                sensor_origin_fixed,
            ) = fixed_points
            clearing_header = Header(
                seq=header.seq,
                stamp=header.stamp,
                frame_id=self.clearing_fixed_frame,
            )

        with self.state_lock:
            if not self.output_enabled:
                return None

            historical_points = empty
            if (
                obstacle_points_fixed is not None
                and self.clear_previous_obstacles
                and self.previous_obstacle_frames
            ):
                historical_points = np.concatenate(
                    self.previous_obstacle_frames, axis=0
                )

            if self.clear_previous_obstacles and obstacle_points_fixed is not None:
                self.previous_obstacle_frames.append(obstacle_points_fixed)
                if len(self.previous_obstacle_frames) > self.previous_clear_frames:
                    self.previous_obstacle_frames.pop(0)
            elif not self.clear_previous_obstacles:
                self.previous_obstacle_frames = []

            if historical_points.shape[0] > 0:
                # Keep the historical endpoints separate from the current
                # clearing downsample so none are discarded by voxel merging.
                # Extend them past the formerly occupied voxel because
                # VoxelLayer shortens clearing rays by two map cells.
                historical_points = self._extend_historical_endpoints(
                    historical_points, sensor_origin_fixed
                )
                output_clearing_points = np.concatenate(
                    (output_clearing_points, historical_points), axis=0
                )

            obstacle_msg = self._build_cloud(header, obstacle_points)
            clearing_msg = self._build_cloud(
                clearing_header, output_clearing_points
            )
            # Publishing under the state lock prevents one stale cloud from
            # escaping after the localization guard has locked the pipeline.
            self.obstacle_pub.publish(obstacle_msg)
            self.clearing_pub.publish(clearing_msg)

        return historical_points.shape[0], output_clearing_points.shape[0]

    def _fallback_ground_plane(self):
        # Plane equation n.p + d = 0.  The expected ground in the LiDAR frame
        # is z=-lidar_height_above_ground.
        return (
            np.array([0.0, 0.0, 1.0], dtype=np.float64),
            self.lidar_height_above_ground,
            False,
        )

    def _fit_ground_plane(self, points, ranges):
        expected_ground_z = -self.lidar_height_above_ground
        candidate_mask = (
            (ranges >= self.ground_fit_min_range)
            & (ranges <= self.ground_fit_max_range)
            & (np.abs(points[:, 2] - expected_ground_z) <= self.ground_candidate_band)
        )
        candidates = points[candidate_mask].astype(np.float64, copy=False)
        if candidates.shape[0] < max(3, self.min_ground_inliers):
            return self._fallback_ground_plane()

        if candidates.shape[0] > self.max_ground_candidates:
            selected = self.random_generator.choice(
                candidates.shape[0], self.max_ground_candidates, replace=False
            )
            candidates = candidates[selected]

        cosine_max_tilt = math.cos(math.radians(self.max_ground_tilt_deg))
        best_normal = None
        best_offset = 0.0
        best_inlier_mask = None
        best_count = 0

        for _ in range(max(1, self.ransac_iterations)):
            sample_indices = self.random_generator.integers(0, candidates.shape[0], 3)
            if len(set(int(value) for value in sample_indices)) != 3:
                continue
            p0, p1, p2 = candidates[sample_indices]
            normal = np.cross(p1 - p0, p2 - p0)
            norm = np.linalg.norm(normal)
            if norm < 1e-8:
                continue
            normal /= norm
            if normal[2] < 0.0:
                normal = -normal
            if normal[2] < cosine_max_tilt:
                continue

            offset = -float(np.dot(normal, p0))
            ground_z_at_sensor = -offset / normal[2]
            if abs(ground_z_at_sensor - expected_ground_z) > self.ground_origin_tolerance:
                continue

            distances = np.abs(np.dot(candidates, normal) + offset)
            inlier_mask = distances <= self.ground_distance_threshold
            count = int(np.count_nonzero(inlier_mask))
            if count > best_count:
                best_count = count
                best_normal = normal
                best_offset = offset
                best_inlier_mask = inlier_mask

        required_inliers = max(
            self.min_ground_inliers, int(0.08 * candidates.shape[0])
        )
        if best_normal is None or best_count < required_inliers:
            return self._fallback_ground_plane()

        # Least-squares refinement using the RANSAC inliers.
        inliers = candidates[best_inlier_mask]
        centroid = np.mean(inliers, axis=0)
        covariance = np.cov((inliers - centroid).T)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        normal = eigenvectors[:, int(np.argmin(eigenvalues))]
        if normal[2] < 0.0:
            normal = -normal
        if normal[2] < cosine_max_tilt:
            return best_normal, best_offset, True
        offset = -float(np.dot(normal, centroid))
        return normal, offset, True

    def cloud_callback(self, msg):
        if not self._output_is_enabled():
            rospy.logwarn_throttle(
                2.0,
                "[lidar_obstacle] Receiving raw LiDAR but withholding costmap clouds "
                "until localization is ready.",
            )
            return

        try:
            points = self._read_xyz(msg)
        except (TypeError, ValueError) as exc:
            rospy.logerr_throttle(2.0, "[lidar_obstacle] invalid cloud: %s", exc)
            return

        if points.shape[0] == 0:
            # Even an empty current scan must replay the recent endpoints on
            # the clearing topic; otherwise their costmap cells can linger.
            self._publish_current_snapshot(msg.header, points, points)
            return

        finite_mask = np.all(np.isfinite(points), axis=1)
        points = points[finite_mask]
        ranges = np.linalg.norm(points[:, :2], axis=1)
        range_mask = (ranges >= self.min_range) & (ranges <= self.max_range)
        points = points[range_mask]
        ranges = ranges[range_mask]

        half_length = 0.5 * self.ugv_length + self.self_filter_padding
        half_width = 0.5 * self.ugv_width + self.self_filter_padding
        inside_self = (
            (np.abs(points[:, 0]) <= half_length)
            & (np.abs(points[:, 1]) <= half_width)
            & (points[:, 2] >= self.self_min_z)
            & (points[:, 2] <= self.self_max_z)
        )
        points = points[~inside_self]
        ranges = ranges[~inside_self]

        # The clearing cloud retains ground endpoints.  VoxelLayer uses them
        # only to raytrace free space; they are never allowed to mark cells.
        clearing_points = self._voxel_downsample(
            points, self.clearing_leaf_size
        )

        normal, offset, ground_was_fitted = self._fit_ground_plane(points, ranges)
        vertical_height = (np.dot(points, normal) + offset) / max(normal[2], 1e-6)
        obstacle_mask = (
            (vertical_height >= self.min_obstacle_height)
            & (vertical_height <= self.max_obstacle_height)
        )
        obstacle_points = self._voxel_downsample(
            points[obstacle_mask], self.marking_leaf_size
        )

        publish_counts = self._publish_current_snapshot(
            msg.header, obstacle_points, clearing_points
        )
        if publish_counts is None:
            return
        historical_count, clearing_count = publish_counts

        tilt_deg = math.degrees(math.acos(np.clip(normal[2], -1.0, 1.0)))
        rospy.loginfo_throttle(
            2.0,
            "[lidar_obstacle] input=%d current_marking=%d clearing=%d "
            "expired_marking=%d ground=%s tilt=%.1fdeg",
            msg.width * msg.height,
            obstacle_points.shape[0],
            clearing_count,
            historical_count,
            "ransac" if ground_was_fitted else "fixed-height fallback",
            tilt_deg,
        )


if __name__ == "__main__":
    rospy.init_node("lidar_preprocess_obstacle")
    LidarPreprocessObstacle()
    rospy.spin()
