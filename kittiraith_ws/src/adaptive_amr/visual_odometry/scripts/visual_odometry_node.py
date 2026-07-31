#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
visual_odometry_node.py — stereo visual odometry node.

Subscribes to the left/right rectified camera streams + their CameraInfos,
runs StereoVisualOdometry and publishes:

    /visual_odometry           (nav_msgs/Odometry, frame odom, child base_link)
    /visual_odometry/path      (nav_msgs/Path)
    /visual_odometry/statistics (DiagnosticArray)
    TF odom -> base_link       (when enable_tf:=true)

Run:
    rosrun visual_odometry visual_odometry_node.py
"""

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster, TransformStamped

from visual_odometry.geometry_utils import invert_pose
from visual_odometry.stereo_vo import StereoVisualOdometry


class VisualOdometryNode:
    """Runs stereo visual odometry on the KITTI camera streams."""

    def __init__(self):
        # ---- configuration ----------------------------------------------------
        self.left_topic = rospy.get_param("~left_image_topic", "/camera/image_raw")
        self.right_topic = rospy.get_param("~right_image_topic", "/camera_right/image_raw")
        self.left_ci_topic = rospy.get_param("~left_camera_info",
                                             "/camera/camera_02/camera_info")
        self.right_ci_topic = rospy.get_param("~right_camera_info",
                                              "/camera/camera_03/camera_info")
        self.queue_size = rospy.get_param("~queue_size", 20)
        self.approx_slop = rospy.get_param("~approx_slop", 0.05)
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.camera_frame = rospy.get_param("~camera_frame", "camera_optical_frame")
        self.enable_tf = rospy.get_param("~enable_tf", True)
        self.diagnostics_rate = rospy.get_param("~diagnostics_rate", 1.0)
        self.output_ns = rospy.get_param("~output_ns", "/visual_odometry")

        # feature/geometry params
        max_features = rospy.get_param("~max_features", 1500)
        quality = rospy.get_param("~quality_level", 0.01)
        min_dist = rospy.get_param("~min_distance", 10)
        min_inliers = rospy.get_param("~min_inliers", 15)

        # ---- calibration (filled by the first CameraInfo) ----------------------
        self.P_left = None
        self.P_right = None
        self.T_base_cam = None      # camera_optical -> base_link

        self.vo = None
        self.frames_processed = 0
        self.skipped_frames = 0
        self.last_ms = 0.0
        self.path = Path()

        # ---- publishers ----------------------------------------------------------
        self.pub_odom = rospy.Publisher(self.output_ns, Odometry, queue_size=5)
        self.pub_path = rospy.Publisher(self.output_ns + "/path", Path, queue_size=1)
        self.pub_stats = rospy.Publisher(self.output_ns + "/statistics",
                                         DiagnosticArray, queue_size=5)
        self.tf_broadcaster = TransformBroadcaster()

        # ---- subscribers ------------------------------------------------------------
        self.sub_left = Subscriber(self.left_topic, Image)
        self.sub_right = Subscriber(self.right_topic, Image)
        self.sync = ApproximateTimeSynchronizer([self.sub_left, self.sub_right],
                                                self.queue_size, self.approx_slop)
        self.sync.registerCallback(self._on_stereo)
        rospy.Subscriber(self.left_ci_topic, CameraInfo, self._on_left_ci)
        rospy.Subscriber(self.right_ci_topic, CameraInfo, self._on_right_ci)
        rospy.Timer(rospy.Duration(1.0 / max(self.diagnostics_rate, 0.1)),
                    self._publish_statistics)

        rospy.loginfo("visual_odometry: %s + %s -> %s",
                      self.left_topic, self.right_topic, self.output_ns)

    # ------------------------------------------------------------------ #
    def _on_left_ci(self, msg: CameraInfo) -> None:
        self.P_left = np.asarray(msg.P, dtype=float).reshape(3, 4)

    def _on_right_ci(self, msg: CameraInfo) -> None:
        self.P_right = np.asarray(msg.P, dtype=float).reshape(3, 4)

    # ------------------------------------------------------------------ #
    def _build_vo(self) -> bool:
        """Create the VO engine once both projections are known."""
        if self.vo is not None or self.P_left is None or self.P_right is None:
            return self.vo is not None
        self.vo = StereoVisualOdometry(
            self.P_left, self.P_right,
            max_features=rospy.get_param("~max_features", 1500),
            quality_level=rospy.get_param("~quality_level", 0.01),
            min_distance=rospy.get_param("~min_distance", 10),
            min_inliers=rospy.get_param("~min_inliers", 15),
            ransac_reproj=rospy.get_param("~ransac_reproj", 3.0))
        rospy.loginfo("visual_odometry: engine ready (P_left/P_right loaded)")
        return True

    # ------------------------------------------------------------------ #
    def _base_cam_transform(self) -> np.ndarray:
        """camera_optical -> base_link transform (TF lookup, calib fallback)."""
        if self.T_base_cam is not None:
            return self.T_base_cam
        try:
            from tf2_ros import Buffer, TransformListener
            buf = Buffer()
            TransformListener(buf)
            stamped = buf.lookup_transform(self.base_frame, self.camera_frame,
                                          rospy.Time(0),
                                          timeout=rospy.Duration(2.0))
            t = stamped.transform
            T = np.eye(4)
            T[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
            T[:3, :3] = _quat_to_matrix(t.rotation)
            self.T_base_cam = T
            rospy.loginfo("visual_odometry: using TF %s -> %s",
                          self.camera_frame, self.base_frame)
        except Exception:  # noqa: BLE001
            try:
                from dataset_loader.kitti_parsers import KittiCalib, KittiPaths
                paths = KittiPaths(rospy.get_param("~dataset_root", "/data/kitti"),
                                   rospy.get_param("~date", "2011_09_26"),
                                   rospy.get_param("~drive", "2011_09_26_drive_0005"))
                calib = KittiCalib(paths.calib_dir, rospy.get_param("~date", "2011_09_26"))
                self.T_base_cam = calib.velo_to_cam_transform()
                rospy.loginfo("visual_odometry: using KITTI calib T_base_cam")
            except Exception as exc:  # noqa: BLE001
                rospy.logwarn_throttle(5.0,
                                       "visual_odometry: no base->cam transform: %s", exc)
                self.T_base_cam = np.eye(4)
        return self.T_base_cam

    # ------------------------------------------------------------------ #
    def _on_stereo(self, left: Image, right: Image) -> None:
        if not self._build_vo():
            self.skipped_frames += 1
            return
        start = rospy.Time.now()

        left_np = _image_to_np(left)
        right_np = _image_to_np(right)
        if left_np is None or right_np is None:
            self.skipped_frames += 1
            return

        try:
            T_w_cam = self.vo.process(left_np, right_np)
        except Exception as exc:  # noqa: BLE001
            rospy.logerr_throttle(5.0, "visual_odometry: %s", exc)
            self.skipped_frames += 1
            return

        if T_w_cam is None:
            rospy.loginfo_throttle(2.0, "visual_odometry: initializing "
                                        "(first frame)")
            return

        # ---- convert camera pose to base_link pose ------------------------------
        T_base_cam = self._base_cam_transform()
        T_w_base = T_w_cam @ invert_pose(T_base_cam)   # p_odom = T_w_base p_base

        # ---- publish Odometry -----------------------------------------------------
        odom = Odometry()
        odom.header.stamp = left.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose = _pose_from_matrix(T_w_base)
        self.pub_odom.publish(odom)

        # ---- publish Path ------------------------------------------------------------
        pose_stamped = PoseStamped()
        pose_stamped.header = odom.header
        pose_stamped.pose = odom.pose.pose
        self.path.header = odom.header
        self.path.poses.append(pose_stamped)
        self.pub_path.publish(self.path)

        # ---- TF odom -> base_link -------------------------------------------------------
        if self.enable_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = odom.header.stamp
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation = odom.pose.pose.position
            tf_msg.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)

        self.frames_processed += 1
        self.last_ms = (rospy.Time.now() - start).to_sec() * 1000.0

    # ------------------------------------------------------------------ #
    def _publish_statistics(self, _event=None) -> None:
        status = DiagnosticStatus()
        status.level = (DiagnosticStatus.OK if self.frames_processed > 0
                        else DiagnosticStatus.WARN)
        status.name = "visual_odometry"
        status.message = "{} frames processed".format(self.frames_processed)
        status.values = [
            KeyValue(key="frames_processed", value=str(self.frames_processed)),
            KeyValue(key="skipped_frames", value=str(self.skipped_frames)),
            KeyValue(key="last_process_ms", value="{:.2f}".format(self.last_ms)),
            KeyValue(key="last_pnp_inliers",
                     value=str(getattr(self.vo, "last_inliers", 0)
                               if self.vo else 0)),
        ]
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        array.status.append(status)
        self.pub_stats.publish(array)

    def run(self) -> None:
        rospy.spin()


# --------------------------------------------------------------------------- #
def _image_to_np(msg: Image):
    """sensor_msgs/Image -> numpy array (supports bgr8/rgb8/mono8)."""
    import numpy as np
    if msg.encoding in ("mono8", "8UC1"):
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
    if msg.encoding in ("bgr8", "rgb8"):
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3)
    rospy.logwarn_throttle(5.0, "visual_odometry: unsupported encoding %s",
                           msg.encoding)
    return None


def _pose_from_matrix(T):
    """4x4 matrix -> geometry_msgs/Pose (translation + quaternion)."""
    from dataset_loader.kitti_parsers import matrix_to_quaternion
    q = matrix_to_quaternion(T[:3, :3])
    pose = Pose()
    pose.position = Point(x=float(T[0, 3]), y=float(T[1, 3]), z=float(T[2, 3]))
    pose.orientation = Quaternion(x=float(q[0]), y=float(q[1]),
                                  z=float(q[2]), w=float(q[3]))
    return pose


def _quat_to_matrix(q) -> np.ndarray:
    import numpy as np
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    rospy.init_node("visual_odometry_node", anonymous=False)
    try:
        VisualOdometryNode().run()
    except rospy.ROSInterruptException:
        rospy.loginfo("visual_odometry interrupted.")
    except Exception as exc:
        rospy.logfatal("visual_odometry failed: %s", exc)
        raise


if __name__ == "__main__":
    main()
