#!/usr/bin/env python3
"""Find the red target block in the camera image and publish its map-frame pose.

The math lives in module-level functions so check_detector_math.py can exercise
it without ROS or a running sim.
"""

import cv2
import numpy as np

# Red wraps around hue 0 in HSV, so it takes two bands. They are tight on
# purpose: the dark-orange distractor sits at hue ~13 and brown at ~10, right
# next to the low band, and magenta at ~150 below the high one. Widening
# either band past this is how distractors start leaking through.
LOW1, HIGH1 = (0, 120, 70), (6, 255, 255)
LOW2, HIGH2 = (174, 120, 70), (180, 255, 255)

MIN_AREA = 400.0   # px^2 — below this it's a reflection or speckle, not the block
GROUND_Z = 0.025   # block center height: 5 cm cube sitting on the floor

# Log-readability only: name what's in view when it isn't the target.
# Orange and brown overlap in hue (~8-22); brightness tells them apart —
# the SDF materials work out to V~255 vs V~115, split with slack for
# lighting. Every candidate still has to pass the size-distance check, or
# the orange chairs and wood floor of the house get called blocks.
DISTRACTOR_BANDS = [
    ('orange',  (8, 150, 140), (22, 255, 255)),
    ('brown',   (5, 100, 40),  (22, 255, 139)),
    ('magenta', (135, 80, 70), (172, 255, 255)),
]

_KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))


def red_mask(hsv, low1=LOW1, high1=HIGH1, low2=LOW2, high2=HIGH2):
    """Binary mask of red pixels in an HSV image, speckle-cleaned."""
    mask = cv2.inRange(hsv, np.array(low1), np.array(high1)) \
         | cv2.inRange(hsv, np.array(low2), np.array(high2))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _KERNEL)
    return mask


def largest_blob_centroid(mask, min_area=MIN_AREA):
    """Centroid (u, v, area) of the biggest contour, or None if nothing big enough."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    best = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(best)
    if area < min_area:
        return None
    m = cv2.moments(best)
    if m['m00'] == 0:
        return None
    return m['m10'] / m['m00'], m['m01'] / m['m00'], area


def pixel_to_ground(u, v, K, R_mo, t_mo, ground_z=GROUND_Z):
    """Back-project pixel (u, v) onto the horizontal plane z = ground_z.

    A pixel only pins down a ray, not a point: inv(K) @ [u, v, 1] in the
    optical frame. R_mo / t_mo (optical -> map) rotate that ray into the map
    frame, then we solve for where it crosses the ground plane. Returns the
    3-vector in map, or None when the ray points at or above the horizon —
    without that guard a sky pixel "detects" a block far behind the camera.
    """
    ray_opt = np.linalg.inv(K) @ np.array([u, v, 1.0])
    ray_map = R_mo @ ray_opt
    if ray_map[2] > -1e-3:
        return None
    s = (ground_z - t_mo[2]) / ray_map[2]
    if s <= 0:
        return None
    return t_mo + s * ray_map


def quat_to_rot(x, y, z, w):
    """3x3 rotation matrix from a quaternion. Just enough numpy to skip scipy."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


# Everything below is the ROS wrapper around the math above. Imports are
# guarded so check_detector_math.py can import this module without a sourced
# ROS environment; only the base-class name is needed at class-definition
# time, so aliasing Node keeps the file importable.
try:
    import rclpy
    import tf2_ros
    from cv_bridge import CvBridge
    from geometry_msgs.msg import PoseStamped
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
    from visualization_msgs.msg import Marker
except ImportError:
    Node = object


class BlockDetector(Node):
    """HSV detector: camera frames in, map-frame pose of the red block out."""

    def __init__(self):
        super().__init__('block_detector')
        # Thresholds as parameters so tuning is a `ros2 param set`, not an
        # edit-and-rebuild. Read back every frame — at ~8 Hz that's free.
        self.declare_parameter('hue_low_min', LOW1[0])
        self.declare_parameter('hue_low_max', HIGH1[0])
        self.declare_parameter('hue_high_min', LOW2[0])
        self.declare_parameter('hue_high_max', HIGH2[0])
        self.declare_parameter('sat_min', LOW1[1])
        self.declare_parameter('val_min', LOW1[2])
        self.declare_parameter('min_area', MIN_AREA)

        self._bridge = CvBridge()
        self._K = None
        self._cam_frame = None
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._pose_pub = self.create_publisher(PoseStamped, '/target_block_pose', 10)
        self._marker_pub = self.create_publisher(Marker, '/target_block_marker', 10)

        # gazebo_ros_camera publishes best-effort; a default (reliable)
        # subscription would never receive a single frame.
        self.create_subscription(CameraInfo, '/camera/camera_info',
                                 self._info_cb, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/image_raw',
                                 self._image_cb, qos_profile_sensor_data)

    def _info_cb(self, msg):
        self._K = np.array(msg.k).reshape(3, 3)
        self._cam_frame = msg.header.frame_id

    def _image_cb(self, msg):
        if self._K is None:
            self.get_logger().info('waiting for /camera/camera_info',
                                   throttle_duration_sec=5.0)
            return

        h0, h1, h2, h3, s_min, v_min = (self.get_parameter(n).value for n in (
            'hue_low_min', 'hue_low_max', 'hue_high_min', 'hue_high_max',
            'sat_min', 'val_min'))

        bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = red_mask(hsv, (h0, s_min, v_min), (h1, 255, 255),
                        (h2, s_min, v_min), (h3, 255, 255))
        cam = self._camera_pose(msg.header.stamp)
        if cam is None:
            return
        R_mo, t_mo = cam

        hit = largest_blob_centroid(mask, self.get_parameter('min_area').value)
        if hit is None:
            self._log_whats_in_view(hsv, R_mo, t_mo)
            return
        u, v, area = hit

        p_map = pixel_to_ground(u, v, self._K, R_mo, t_mo)
        if p_map is None:
            return

        # Size-distance consistency: a 5 cm cube at the projected distance
        # has a predictable pixel area. A red sofa cushion across the room
        # makes a huge blob that passes the raw area gate but claims to be
        # a "block" metres away — this is what rejects it.
        dist = float(np.linalg.norm(p_map - t_mo))
        if not self._block_sized(area, dist):
            expected = (self._K[0, 0] * 0.05 / max(dist, 0.05)) ** 2
            self.get_logger().info(
                f'red blob rejected: {area:.0f} px^2 at {dist:.2f} m, '
                f'a block there would be ~{expected:.0f} px^2',
                throttle_duration_sec=5.0)
            return

        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(p_map[0])
        pose.pose.position.y = float(p_map[1])
        pose.pose.position.z = GROUND_Z
        pose.pose.orientation.w = 1.0
        self._pose_pub.publish(pose)

        marker = Marker()
        marker.header = pose.header
        marker.ns = 'target_block'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = marker.scale.y = marker.scale.z = 0.05
        marker.color.r = 1.0
        marker.color.a = 0.9
        marker.lifetime = Duration(seconds=1.0).to_msg()
        self._marker_pub.publish(marker)

        self.get_logger().info(
            f'TARGET block (red) at map ({p_map[0]:.2f}, {p_map[1]:.2f}), '
            f'{area:.0f} px^2 at {dist:.2f} m — size checks out',
            throttle_duration_sec=2.0)

    def _camera_pose(self, stamp):
        """(R, t) taking optical-frame points to map, or None.

        Prefer the image stamp — while the robot turns, 100 ms of pose error
        smears the projection sideways. But AMCL only refreshes map->odom on
        filter updates, so a parked robot's TF goes seconds stale and
        exact-stamp lookups die on extrapolation; fall back to the latest
        transform then, since a stationary camera hasn't moved.
        """
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', self._cam_frame, stamp, timeout=Duration(seconds=0.2))
        except tf2_ros.ExtrapolationException:
            try:
                tf = self._tf_buffer.lookup_transform(
                    'map', self._cam_frame, rclpy.time.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as exc:
                self.get_logger().warning(f'no map->{self._cam_frame} TF: {exc}',
                                          throttle_duration_sec=2.0)
                return None
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException) as exc:
            self.get_logger().warning(f'no map->{self._cam_frame} TF: {exc}',
                                      throttle_duration_sec=2.0)
            return None
        q = tf.transform.rotation
        t = tf.transform.translation
        return quat_to_rot(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z])

    def _block_sized(self, area, dist):
        expected = (self._K[0, 0] * 0.05 / max(dist, 0.05)) ** 2
        return 0.3 * expected <= area <= 4.0 * expected

    def _log_whats_in_view(self, hsv, R_mo, t_mo):
        min_area = self.get_parameter('min_area').value
        for name, lo, hi in DISTRACTOR_BANDS:
            m = cv2.inRange(hsv, np.array(lo), np.array(hi))
            m = cv2.morphologyEx(m, cv2.MORPH_OPEN, _KERNEL)
            hit = largest_blob_centroid(m, min_area)
            if hit is None:
                continue
            u, v, area = hit
            p = pixel_to_ground(u, v, self._K, R_mo, t_mo)
            if p is None:
                continue
            if self._block_sized(area, float(np.linalg.norm(p - t_mo))):
                self.get_logger().info(
                    f'{name} block in view — a distractor, not the target',
                    throttle_duration_sec=5.0)
                return
        self.get_logger().info('no block in view', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = BlockDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
