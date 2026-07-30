#!/usr/bin/env python3
"""
Hesai LiDAR + ZED Camera Fusion Node
- YOLO segmentation via ultralytics
- URDF-based calibration (no ZED dynamic TF dependency)
- Latest-message matching (no timestamp sync required)
- Publishes obstacle relative coordinates in base_link frame
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import CompressedImage, Image, PointCloud2, CameraInfo
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener

import numpy as np
import cv2
import time

import sensor_msgs_py.point_cloud2 as pc2
from ultralytics import YOLO


def read_xyz(msg):
    """Fast XYZ from PointCloud2 (handles mixed field dtypes)"""
    n = msg.width * msg.height
    if n == 0:
        return np.empty((0, 3), dtype=np.float64)
    step = msg.point_step
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    offsets = {f.name: f.offset for f in msg.fields}
    x = np.ndarray(n, np.float32, buffer=raw, offset=offsets['x'], strides=(step,))
    y = np.ndarray(n, np.float32, buffer=raw, offset=offsets['y'], strides=(step,))
    z = np.ndarray(n, np.float32, buffer=raw, offset=offsets['z'], strides=(step,))
    valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    return np.column_stack([x[valid], y[valid], z[valid]]).astype(np.float64)

from hesai_zed_fusion.msg import ObstacleInfo, ObstacleArray


CLASS_MAP = {
    0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle',
    5: 'bus', 7: 'truck'
}
TARGET_CLASS_IDS = {0, 1, 2, 3, 5, 7}

# zedx_base_link (x-fwd, y-left, z-up) -> optical (x-right, y-down, z-fwd)
T_OPTICAL_FROM_BASE = np.array([
    [0, -1,  0, 0],
    [0,  0, -1, 0],
    [1,  0,  0, 0],
    [0,  0,  0, 1]
], dtype=np.float64)


def quat_to_rotation_matrix(qx, qy, qz, qw):
    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz
    return np.array([
        [1 - 2*(yy + zz),     2*(xy - wz),     2*(xz + wy)],
        [    2*(xy + wz), 1 - 2*(xx + zz),     2*(yz - wx)],
        [    2*(xz - wy),     2*(yz + wx), 1 - 2*(xx + yy)]
    ], dtype=np.float64)


def tf_to_matrix(tf_msg):
    tr = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    R = quat_to_rotation_matrix(q.x, q.y, q.z, q.w)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = [tr.x, tr.y, tr.z]
    return T


class HesaiZedFusionNode(Node):
    def __init__(self):
        super().__init__('hesai_zed_fusion')

        # Parameters
        self.declare_parameter('model_path', '/home/taewook/fs_ws/src/yolo26x-seg.engine')
        self.declare_parameter('image_topic', '/zed/zed_node/rgb/color/rect/image/compressed')
        self.declare_parameter('lidar_topic', '/lidar_points')
        self.declare_parameter('camera_info_topic', '/zed/zed_node/rgb/color/rect/image/camera_info')
        self.declare_parameter('lidar_frame', 'hesai_lidar')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_base_frame', 'zedx_base_link')
        self.declare_parameter('conf', 0.3)
        self.declare_parameter('mask_scale', 0.8)

        model_path = self.get_parameter('model_path').value
        image_topic = self.get_parameter('image_topic').value
        lidar_topic = self.get_parameter('lidar_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.lidar_frame = self.get_parameter('lidar_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.camera_base_frame = self.get_parameter('camera_base_frame').value
        self.conf = self.get_parameter('conf').value
        self.mask_scale = self.get_parameter('mask_scale').value

        # YOLO
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.yolo = YOLO(model_path, task='segment')
        self.get_logger().info('YOLO model loaded')

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State
        self.K = None
        self.T_optical_lidar = None
        self.T_base_lidar = None
        self.bridge = CvBridge()
        self.latest_image = None

        # Camera info (RELIABLE - matches rosbag publisher)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic,
            self.camera_info_cb, 10)

        # Image subscriber (RELIABLE - 원본 퍼블리셔와 동일)
        self.image_sub = self.create_subscription(
            CompressedImage, image_topic,
            self.image_cb, 10)

        # LiDAR subscriber (BEST_EFFORT)
        lidar_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5)
        self.lidar_sub = self.create_subscription(
            PointCloud2, lidar_topic,
            self.lidar_cb, lidar_qos)

        # Publishers
        self.pub_obstacles = self.create_publisher(
            ObstacleArray, 'hesai_zed/obstacles', 10)
        self.pub_overlay = self.create_publisher(
            Image, 'hesai_zed/overlay', 10)
        self.pub_obj_pc = self.create_publisher(
            PointCloud2, 'hesai_zed/object_pointcloud', 10)

        self.get_logger().info(f'  Image : {image_topic}')
        self.get_logger().info(f'  LiDAR : {lidar_topic}')
        self.get_logger().info(f'  Camera base frame : {self.camera_base_frame}')
        self.get_logger().info(f'  Base frame        : {self.base_frame}')
        self.get_logger().info('Node ready, waiting for data...')

    # ------------------------------------------------------------------
    def camera_info_cb(self, msg: CameraInfo):
        if self.K is None:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.get_logger().info(
                f'[DBG] camera_info OK: fx={self.K[0,0]:.1f} fy={self.K[1,1]:.1f} '
                f'cx={self.K[0,2]:.1f} cy={self.K[1,2]:.1f}')

    def image_cb(self, msg: CompressedImage):
        self.latest_image = msg
        self.get_logger().info(
            f'[DBG] image received: {len(msg.data)} bytes',
            throttle_duration_sec=3.0)

    def lidar_cb(self, msg: PointCloud2):
        self.get_logger().info(
            f'[DBG] lidar received: {msg.width*msg.height} pts, '
            f'image={self.latest_image is not None}, '
            f'K={self.K is not None}, '
            f'tf={self.T_optical_lidar is not None}',
            throttle_duration_sec=3.0)
        if self.latest_image is None:
            return
        if self.K is None:
            return
        if not self.cache_transforms():
            return
        self.process(self.latest_image, msg)

    # ------------------------------------------------------------------
    def _make_tf(self, xyz, rpy):
        roll, pitch, yaw = rpy
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        R = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr]
        ], dtype=np.float64)
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3, 3] = xyz
        return T

    def _compute_from_urdf(self):
        T_gps_base = self._make_tf([0.0, 0.0, 1.660], [0.0, 0.0, 0.0])
        T_gps_lidar = self._make_tf(
            [0.232668, 0.0, 0.113159], [0.0, 0.174533, 0.0])
        T_gps_zedx = self._make_tf([0.353894, 0.0, -0.0211], [0.0, 0.0, 0.0])

        T_zedx_lidar = np.linalg.inv(T_gps_zedx) @ T_gps_lidar
        self.T_optical_lidar = T_OPTICAL_FROM_BASE @ T_zedx_lidar
        self.T_base_lidar = np.linalg.inv(T_gps_base) @ T_gps_lidar
        self.get_logger().info('Transforms computed from URDF (hardcoded)')

    def cache_transforms(self):
        if self.T_optical_lidar is not None:
            return True
        try:
            tf_cam = self.tf_buffer.lookup_transform(
                self.camera_base_frame, self.lidar_frame,
                Time(), Duration(seconds=0.05))
            T_cambase_lidar = tf_to_matrix(tf_cam)
            self.T_optical_lidar = T_OPTICAL_FROM_BASE @ T_cambase_lidar

            tf_base = self.tf_buffer.lookup_transform(
                self.base_frame, self.lidar_frame,
                Time(), Duration(seconds=0.05))
            self.T_base_lidar = tf_to_matrix(tf_base)
            self.get_logger().info('Transforms cached from TF')
            return True
        except Exception:
            self._compute_from_urdf()
            return True

    # ------------------------------------------------------------------
    def process(self, img_msg: CompressedImage, lidar_msg: PointCloud2):
        t0 = time.time()

        # 1. Decompress image
        np_arr = np.frombuffer(img_msg.data, np.uint8)
        cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if cv_img is None:
            self.get_logger().error('Failed to decompress image')
            return
        img_h, img_w = cv_img.shape[:2]

        # ROI: black out bottom 25% to hide ego hood
        roi_h = int(img_h * 0.75)
        cv_img[roi_h:] = 0

        # 2. Read LiDAR
        pts_np = read_xyz(lidar_msg)
        if pts_np.shape[0] == 0:
            return

        N = pts_np.shape[0]

        # 3. Project LiDAR -> optical -> image
        pts_h = np.hstack([pts_np, np.ones((N, 1), dtype=np.float64)]).T
        pts_optical = self.T_optical_lidar @ pts_h

        proj = self.K @ pts_optical[:3, :]
        z = proj[2, :]

        with np.errstate(divide='ignore', invalid='ignore'):
            u_all = proj[0, :] / z
            v_all = proj[1, :] / z

        in_img = ((z > 0) & (u_all >= 0) & (u_all < img_w) &
                  (v_all >= 0) & (v_all < img_h))
        if not np.any(in_img):
            self.get_logger().warn('No LiDAR points project into image',
                                   throttle_duration_sec=5.0)
            return

        valid_mask_idx = np.where(in_img)[0]
        u = u_all[in_img].astype(np.int32)
        v = v_all[in_img].astype(np.int32)

        # 4. YOLO inference
        t_infer = time.time()
        results = self.yolo.predict(cv_img, conf=self.conf, verbose=False)
        dt_infer = time.time() - t_infer

        if len(results) == 0 or results[0].boxes is None:
            return

        result = results[0]
        boxes = result.boxes
        masks = result.masks

        # 5. Match detections with LiDAR
        obs_array = ObstacleArray()
        obs_array.header = lidar_msg.header
        obs_array.header.frame_id = self.base_frame

        is_object = np.zeros(N, dtype=bool)

        for i in range(len(boxes)):
            cls_id = int(boxes.cls[i].item())
            conf_val = float(boxes.conf[i].item())

            label = CLASS_MAP.get(cls_id, None)
            if label is None:
                continue

            x1, y1, x2, y2 = boxes.xyxy[i].cpu().numpy().astype(int)

            cv2.rectangle(cv_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(cv_img, label, (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            if cls_id not in TARGET_CLASS_IDS:
                continue
            if masks is None or i >= len(masks):
                continue

            mask_data = masks.data[i].cpu().numpy()
            mask_resized = cv2.resize(
                mask_data, (img_w, img_h), interpolation=cv2.INTER_LINEAR)
            mask_full = mask_resized > 0.5

            # Mask overlay
            colored = np.zeros_like(cv_img)
            colored[:, :] = (0, 0, 255)
            cv_img[mask_full] = cv2.addWeighted(
                cv_img[mask_full], 0.6, colored[mask_full], 0.4, 0)

            # Shrink mask
            check_mask = mask_full
            if self.mask_scale < 1.0:
                bx1, by1 = max(0, x1), max(0, y1)
                bx2, by2 = min(img_w, x2), min(img_h, y2)
                bw, bh = bx2 - bx1, by2 - by1
                if bw > 2 and bh > 2:
                    bbox_mask = mask_full[by1:by2, bx1:bx2].astype(
                        np.uint8) * 255
                    new_w = int(bw * self.mask_scale)
                    new_h = int(bh * self.mask_scale)
                    if new_w > 0 and new_h > 0:
                        shrunk = np.zeros((bh, bw), dtype=np.uint8)
                        resized_m = cv2.resize(
                            bbox_mask, (new_w, new_h),
                            interpolation=cv2.INTER_NEAREST)
                        ox = (bw - new_w) // 2
                        oy = (bh - new_h) // 2
                        shrunk[oy:oy+new_h, ox:ox+new_w] = resized_m
                        check_mask = np.zeros_like(mask_full)
                        check_mask[by1:by2, bx1:bx2] = shrunk > 0

            # LiDAR points in mask
            in_mask = check_mask[v, u]
            if not np.any(in_mask):
                continue

            mask_orig_indices = valid_mask_idx[in_mask]
            is_object[mask_orig_indices] = True

            # Closest point
            pts_in_mask = pts_np[mask_orig_indices]
            dists = np.linalg.norm(pts_in_mask, axis=1)
            closest_idx = np.argmin(dists)
            closest_pt = pts_in_mask[closest_idx]
            min_dist = float(dists[closest_idx])

            # Transform to base_link
            pt_h = np.array(
                [closest_pt[0], closest_pt[1], closest_pt[2], 1.0])
            pt_base = self.T_base_lidar @ pt_h

            obs = ObstacleInfo()
            obs.label = label
            obs.class_id = cls_id
            obs.confidence = float(conf_val)
            obs.rel_x = float(pt_base[0])
            obs.rel_y = float(pt_base[1])
            obs.rel_z = float(pt_base[2])
            obs.distance = min_dist
            obs_array.obstacles.append(obs)

            cv2.putText(cv_img, f'{min_dist:.1f}m',
                        (x1, y1 - 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # Publish
        self.pub_obstacles.publish(obs_array)

        overlay_msg = self.bridge.cv2_to_imgmsg(cv_img[:roi_h], 'bgr8')
        overlay_msg.header = img_msg.header
        self.pub_overlay.publish(overlay_msg)

        obj_pts = pts_np[is_object]
        if obj_pts.shape[0] > 0:
            obj_pc_msg = pc2.create_cloud_xyz32(
                lidar_msg.header, obj_pts.tolist())
            self.pub_obj_pc.publish(obj_pc_msg)

        dt_total = time.time() - t0
        self.get_logger().info(
            f'[PERF] Infer: {dt_infer*1000:.0f}ms | '
            f'Total: {dt_total*1000:.0f}ms | '
            f'Objects: {len(obs_array.obstacles)}')


def main(args=None):
    rclpy.init(args=args)
    node = HesaiZedFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
