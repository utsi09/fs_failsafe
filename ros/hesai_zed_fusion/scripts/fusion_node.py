#!/usr/bin/env python3
"""
Hesai LiDAR + ZED Camera Fusion Node
- YOLO segmentation via ultralytics
- URDF-based calibration (no ZED dynamic TF dependency)
- Latest-message matching (no timestamp sync required)
- Publishes obstacle relative coordinates in base_link frame
"""
# ============================================================================
# [이 파일의 역할]
# ★ 실차(real vehicle) 포팅용 모듈이다. CARLA 시뮬레이션이 아니라, 실제 차량에
#   달린 Hesai 라이다 + ZED 카메라를 융합하는 ROS2 노드.
#   C++ 버전(hesai_zed_fusion_node.cpp)과 같은 일을 하는 Python 프로토타입:
#   ultralytics 라이브러리로 YOLO를 돌리고, numpy 벡터 연산으로 투영을 처리한다.
#
# 하는 일 (한 프레임 기준):
#   1) 카메라 이미지에서 YOLO 세그멘테이션으로 물체(사람/차/트럭...)를 찾고
#   2) 라이다 3D 점들을 카메라 이미지 평면에 투영해서
#   3) "물체 마스크 안에 찍힌" 라이다 점만 골라낸 뒤
#   4) 가장 가까운 점의 3D 좌표를 차량 기준(base_link)으로 변환해
#   5) hesai_zed/obstacles (ObstacleArray) 토픽으로 발행한다.
#
# C++ 버전과 다른 점:
#   - 시간 동기화 없이 "가장 최근 이미지"를 라이다 콜백에서 그냥 가져다 씀
#   - TF 조회에 실패해도 URDF 수치를 하드코딩한 값으로 변환 행렬을 직접 계산
#     (ZED 드라이버의 TF 발행에 의존하지 않아도 동작하게 하기 위함)
# ============================================================================
import rclpy                                    # ROS2 파이썬 라이브러리
from rclpy.node import Node                     # 노드 기반 클래스
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy   # QoS(통신 품질) 설정
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import CompressedImage, Image, PointCloud2, CameraInfo
from cv_bridge import CvBridge                  # ROS 이미지 <-> OpenCV 이미지 변환
from tf2_ros import Buffer, TransformListener   # TF(좌표계 변환) 수신

import numpy as np
import cv2
import time

import sensor_msgs_py.point_cloud2 as pc2       # PointCloud2 메시지 생성 도우미
from ultralytics import YOLO                    # YOLO 모델 로드/추론 라이브러리


def read_xyz(msg):
    """Fast XYZ from PointCloud2 (handles mixed field dtypes)"""
    # PointCloud2는 점들이 바이트 덩어리로 들어있다. 파이썬 for문으로 한 점씩
    # 읽으면 너무 느려서, numpy로 바이트 배열을 직접 해석해 (N,3) 배열로 뽑는다.
    n = msg.width * msg.height                  # 전체 점 개수
    if n == 0:
        return np.empty((0, 3), dtype=np.float64)
    step = msg.point_step                       # 점 하나가 차지하는 바이트 수
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    offsets = {f.name: f.offset for f in msg.fields}   # 각 필드(x,y,z...)의 시작 위치
    # strides=(step,): step 바이트씩 건너뛰며 float32 하나씩 읽기 → 복사 없이 x,y,z 추출
    x = np.ndarray(n, np.float32, buffer=raw, offset=offsets['x'], strides=(step,))
    y = np.ndarray(n, np.float32, buffer=raw, offset=offsets['y'], strides=(step,))
    z = np.ndarray(n, np.float32, buffer=raw, offset=offsets['z'], strides=(step,))
    valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)   # NaN/무한대 점 제거
    return np.column_stack([x[valid], y[valid], z[valid]]).astype(np.float64)

from hesai_zed_fusion.msg import ObstacleInfo, ObstacleArray   # 이 패키지의 커스텀 메시지


# COCO 클래스 번호 → 이름. 이 목록에 있는 것만 장애물로 취급한다.
CLASS_MAP = {
    0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle',
    5: 'bus', 7: 'truck'
}
TARGET_CLASS_IDS = {0, 1, 2, 3, 5, 7}

# zedx_base_link (x-fwd, y-left, z-up) -> optical (x-right, y-down, z-fwd)
# 로봇 좌표계(x=앞)와 카메라 광학 좌표계(z=앞)는 축 방향이 다르다.
# 이 고정 행렬이 "축 이름 바꾸기"를 담당한다 (회전만 있고 이동은 없음).
T_OPTICAL_FROM_BASE = np.array([
    [0, -1,  0, 0],
    [0,  0, -1, 0],
    [1,  0,  0, 0],
    [0,  0,  0, 1]
], dtype=np.float64)


def quat_to_rotation_matrix(qx, qy, qz, qw):
    # 쿼터니언(회전을 4개 숫자로 표현) → 3x3 회전 행렬. 표준 변환 공식 그대로.
    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz
    return np.array([
        [1 - 2*(yy + zz),     2*(xy - wz),     2*(xz + wy)],
        [    2*(xy + wz), 1 - 2*(xx + zz),     2*(yz - wx)],
        [    2*(xz - wy),     2*(yz + wx), 1 - 2*(xx + yy)]
    ], dtype=np.float64)


def tf_to_matrix(tf_msg):
    # ROS TF 메시지(이동 + 회전 쿼터니언) → 4x4 동차변환행렬로 변환.
    tr = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    R = quat_to_rotation_matrix(q.x, q.y, q.z, q.w)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R         # 좌상단 3x3 = 회전
    T[:3, 3] = [tr.x, tr.y, tr.z]   # 마지막 열 = 평행이동
    return T


# 융합 노드 본체. Node를 상속(괄호 안의 Node)하면 ROS2 노드가 된다.
class HesaiZedFusionNode(Node):
    def __init__(self):
        super().__init__('hesai_zed_fusion')    # 노드 이름 등록

        # Parameters — 런치파일/명령행에서 바꿀 수 있는 설정값 (기본값 포함)
        self.declare_parameter('model_path', '/home/taewook/fs_ws/src/yolo26x-seg.engine')
        self.declare_parameter('image_topic', '/zed/zed_node/rgb/color/rect/image/compressed')
        self.declare_parameter('lidar_topic', '/lidar_points')
        self.declare_parameter('camera_info_topic', '/zed/zed_node/rgb/color/rect/image/camera_info')
        self.declare_parameter('lidar_frame', 'hesai_lidar')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_base_frame', 'zedx_base_link')
        self.declare_parameter('conf', 0.3)          # YOLO 신뢰도 문턱
        self.declare_parameter('mask_scale', 0.8)    # 마스크 축소 비율 (경계 오검출 방지)

        # 선언한 파라미터의 실제 값 읽기
        model_path = self.get_parameter('model_path').value
        image_topic = self.get_parameter('image_topic').value
        lidar_topic = self.get_parameter('lidar_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.lidar_frame = self.get_parameter('lidar_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.camera_base_frame = self.get_parameter('camera_base_frame').value
        self.conf = self.get_parameter('conf').value
        self.mask_scale = self.get_parameter('mask_scale').value

        # YOLO — ultralytics가 .engine(TensorRT) 파일도 알아서 로드해 준다
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.yolo = YOLO(model_path, task='segment')
        self.get_logger().info('YOLO model loaded')

        # TF2 — 좌표계 변환 정보를 자동으로 모아두는 버퍼 + 수신기
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State — 아직 안 받은 것들은 None으로 시작
        self.K = None                   # 카메라 내부 파라미터 행렬 (3D→픽셀 투영용)
        self.T_optical_lidar = None     # 라이다 → 카메라 광학 좌표 변환
        self.T_base_lidar = None        # 라이다 → 차량 기준(base_link) 변환
        self.bridge = CvBridge()
        self.latest_image = None        # 가장 최근에 받은 카메라 이미지 보관

        # Camera info (RELIABLE - matches rosbag publisher)
        # 구독자의 QoS가 발행자와 맞아야 통신이 된다 (안 맞으면 조용히 안 옴)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic,
            self.camera_info_cb, 10)

        # Image subscriber (RELIABLE - 원본 퍼블리셔와 동일)
        self.image_sub = self.create_subscription(
            CompressedImage, image_topic,
            self.image_cb, 10)

        # LiDAR subscriber (BEST_EFFORT)
        # BEST_EFFORT = 유실을 허용하는 빠른 전송 (센서 데이터에 흔한 설정)
        lidar_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5)
        self.lidar_sub = self.create_subscription(
            PointCloud2, lidar_topic,
            self.lidar_cb, lidar_qos)

        # Publishers — 장애물 목록 / 디버그 오버레이 이미지 / 물체 포인트클라우드
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
        # 최초 1회만 카메라 내부 파라미터 K(3x3)를 저장 (센서 고정값이므로 한 번이면 충분)
        if self.K is None:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.get_logger().info(
                f'[DBG] camera_info OK: fx={self.K[0,0]:.1f} fy={self.K[1,1]:.1f} '
                f'cx={self.K[0,2]:.1f} cy={self.K[1,2]:.1f}')

    def image_cb(self, msg: CompressedImage):
        # 이미지는 처리하지 않고 "가장 최근 것"만 보관 → 라이다가 올 때 함께 처리
        self.latest_image = msg
        self.get_logger().info(
            f'[DBG] image received: {len(msg.data)} bytes',
            throttle_duration_sec=3.0)          # 같은 로그를 3초에 한 번만 출력

    def lidar_cb(self, msg: PointCloud2):
        # 라이다 콜백이 처리의 "시동 버튼": 라이다가 도착할 때마다 최신 이미지와 묶어 처리
        self.get_logger().info(
            f'[DBG] lidar received: {msg.width*msg.height} pts, '
            f'image={self.latest_image is not None}, '
            f'K={self.K is not None}, '
            f'tf={self.T_optical_lidar is not None}',
            throttle_duration_sec=3.0)
        # 준비물(이미지, K, 변환행렬)이 하나라도 없으면 이번 프레임은 건너뜀
        if self.latest_image is None:
            return
        if self.K is None:
            return
        if not self.cache_transforms():
            return
        self.process(self.latest_image, msg)

    # ------------------------------------------------------------------
    def _make_tf(self, xyz, rpy):
        # (이동 xyz, 회전 roll/pitch/yaw) → 4x4 변환 행렬. URDF 수치 계산용 도우미.
        roll, pitch, yaw = rpy
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        # ZYX 순서 오일러각 회전 행렬 (yaw→pitch→roll 순으로 회전 적용)
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
        # TF를 못 받았을 때의 예비책: URDF(로봇 부품 배치 기술 파일)에 적힌
        # 센서 장착 위치 수치를 하드코딩해 변환 행렬을 직접 계산한다.
        # 기준점은 GPS 안테나: gps→base, gps→lidar, gps→zed 세 변환을 만들어 조합.
        T_gps_base = self._make_tf([0.0, 0.0, 1.660], [0.0, 0.0, 0.0])
        T_gps_lidar = self._make_tf(
            [0.232668, 0.0, 0.113159], [0.0, 0.174533, 0.0])   # 라이다는 10도(0.1745rad) 아래로 기울어 장착
        T_gps_zedx = self._make_tf([0.353894, 0.0, -0.0211], [0.0, 0.0, 0.0])

        # inv(A→B) @ (A→C) = (B→C): 공통 기준(gps)을 거쳐 zed→lidar 변환을 얻는다
        T_zedx_lidar = np.linalg.inv(T_gps_zedx) @ T_gps_lidar
        self.T_optical_lidar = T_OPTICAL_FROM_BASE @ T_zedx_lidar   # 광학 좌표계로 축 변환까지
        self.T_base_lidar = np.linalg.inv(T_gps_base) @ T_gps_lidar
        self.get_logger().info('Transforms computed from URDF (hardcoded)')

    def cache_transforms(self):
        # 변환 행렬 캐싱: 성공하면 다시 계산하지 않는다 (센서 위치는 고정).
        # 1순위 TF 조회, 실패하면 2순위 URDF 하드코딩 값 → 항상 True를 반환하게 됨.
        if self.T_optical_lidar is not None:
            return True
        try:
            tf_cam = self.tf_buffer.lookup_transform(
                self.camera_base_frame, self.lidar_frame,
                Time(), Duration(seconds=0.05))      # 최대 0.05초만 기다림
            T_cambase_lidar = tf_to_matrix(tf_cam)
            self.T_optical_lidar = T_OPTICAL_FROM_BASE @ T_cambase_lidar

            tf_base = self.tf_buffer.lookup_transform(
                self.base_frame, self.lidar_frame,
                Time(), Duration(seconds=0.05))
            self.T_base_lidar = tf_to_matrix(tf_base)
            self.get_logger().info('Transforms cached from TF')
            return True
        except Exception:
            self._compute_from_urdf()    # TF 실패 → URDF 수치로 대체
            return True

    # ------------------------------------------------------------------
    def process(self, img_msg: CompressedImage, lidar_msg: PointCloud2):
        # ★ 핵심 함수: 이미지 1장 + 라이다 1프레임 → 장애물 목록 발행.
        #   흐름: 디코딩 → 라이다 읽기 → 투영 → YOLO → 마스크 매칭 → 최근접점 → 발행
        t0 = time.time()

        # 1. Decompress image — JPEG 바이트 → OpenCV 이미지
        np_arr = np.frombuffer(img_msg.data, np.uint8)
        cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if cv_img is None:
            self.get_logger().error('Failed to decompress image')
            return
        img_h, img_w = cv_img.shape[:2]

        # ROI: black out bottom 25% to hide ego hood
        # 화면 아래 25%는 자기 차 보닛이 찍히므로 검게 칠해 YOLO 오검출을 막는다
        roi_h = int(img_h * 0.75)
        cv_img[roi_h:] = 0

        # 2. Read LiDAR — 바이트 덩어리에서 (N,3) 좌표 배열 추출
        pts_np = read_xyz(lidar_msg)
        if pts_np.shape[0] == 0:
            return

        N = pts_np.shape[0]

        # 3. Project LiDAR -> optical -> image
        # 모든 점을 한 번에 행렬 곱으로 처리 (for문 대신 numpy 벡터 연산 → 빠름)
        pts_h = np.hstack([pts_np, np.ones((N, 1), dtype=np.float64)]).T   # 동차좌표 (4,N)
        pts_optical = self.T_optical_lidar @ pts_h    # 라이다 좌표 → 카메라 광학 좌표

        proj = self.K @ pts_optical[:3, :]            # 카메라 행렬 K로 투영
        z = proj[2, :]

        with np.errstate(divide='ignore', invalid='ignore'):   # 0 나눗셈 경고 잠시 끔
            u_all = proj[0, :] / z                    # 동차좌표 나눗셈 → 픽셀 좌표
            v_all = proj[1, :] / z

        # 카메라 앞쪽(z>0)이면서 이미지 범위 안에 찍힌 점만 True
        in_img = ((z > 0) & (u_all >= 0) & (u_all < img_w) &
                  (v_all >= 0) & (v_all < img_h))
        if not np.any(in_img):
            self.get_logger().warn('No LiDAR points project into image',
                                   throttle_duration_sec=5.0)
            return

        valid_mask_idx = np.where(in_img)[0]          # 유효 점들의 원래 인덱스
        u = u_all[in_img].astype(np.int32)            # 유효 점들의 픽셀 좌표
        v = v_all[in_img].astype(np.int32)

        # 4. YOLO inference — 세그멘테이션 추론 (verbose=False: 로그 끔)
        t_infer = time.time()
        results = self.yolo.predict(cv_img, conf=self.conf, verbose=False)
        dt_infer = time.time() - t_infer

        if len(results) == 0 or results[0].boxes is None:
            return

        result = results[0]        # 배치 크기 1이므로 첫 결과만 사용
        boxes = result.boxes       # 박스 목록
        masks = result.masks       # 세그멘테이션 마스크 목록

        # 5. Match detections with LiDAR — 발행할 장애물 배열 준비 (base_link 기준)
        obs_array = ObstacleArray()
        obs_array.header = lidar_msg.header
        obs_array.header.frame_id = self.base_frame

        is_object = np.zeros(N, dtype=bool)    # 각 점이 물체에 속하는지 표시

        for i in range(len(boxes)):
            cls_id = int(boxes.cls[i].item())          # .item(): 텐서 → 파이썬 숫자
            conf_val = float(boxes.conf[i].item())

            label = CLASS_MAP.get(cls_id, None)
            if label is None:                          # 관심 없는 클래스는 통과
                continue

            x1, y1, x2, y2 = boxes.xyxy[i].cpu().numpy().astype(int)   # 박스 좌상단/우하단

            # 디버그 시각화: 초록 박스 + 클래스 이름
            cv2.rectangle(cv_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(cv_img, label, (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            if cls_id not in TARGET_CLASS_IDS:
                continue
            if masks is None or i >= len(masks):
                continue

            # 마스크는 모델 해상도로 나오므로 원본 이미지 크기로 확대 후 이진화
            mask_data = masks.data[i].cpu().numpy()
            mask_resized = cv2.resize(
                mask_data, (img_w, img_h), interpolation=cv2.INTER_LINEAR)
            mask_full = mask_resized > 0.5

            # Mask overlay — 마스크 영역을 빨간색 반투명으로 덧칠 (디버그용)
            colored = np.zeros_like(cv_img)
            colored[:, :] = (0, 0, 255)
            cv_img[mask_full] = cv2.addWeighted(
                cv_img[mask_full], 0.6, colored[mask_full], 0.4, 0)

            # Shrink mask — 마스크를 중심 기준으로 mask_scale배 축소.
            # 물체 경계에 걸친 배경 라이다 점(바닥, 뒷벽)이 섞이는 것을 줄인다.
            check_mask = mask_full
            if self.mask_scale < 1.0:
                bx1, by1 = max(0, x1), max(0, y1)              # 박스를 이미지 안으로 클램프
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
                        ox = (bw - new_w) // 2                 # 축소본을 박스 가운데 배치
                        oy = (bh - new_h) // 2
                        shrunk[oy:oy+new_h, ox:ox+new_w] = resized_m
                        check_mask = np.zeros_like(mask_full)
                        check_mask[by1:by2, bx1:bx2] = shrunk > 0

            # LiDAR points in mask — 투영된 픽셀 (v,u)가 마스크 안인지 한 번에 조회
            in_mask = check_mask[v, u]
            if not np.any(in_mask):
                continue

            mask_orig_indices = valid_mask_idx[in_mask]   # 마스크 안 점들의 원래 인덱스
            is_object[mask_orig_indices] = True

            # Closest point — 마스크 안 점들 중 라이다 원점에서 가장 가까운 점
            pts_in_mask = pts_np[mask_orig_indices]
            dists = np.linalg.norm(pts_in_mask, axis=1)   # 각 점의 직선거리
            closest_idx = np.argmin(dists)
            closest_pt = pts_in_mask[closest_idx]
            min_dist = float(dists[closest_idx])

            # Transform to base_link — 최근접점을 차량 기준 좌표로 변환
            pt_h = np.array(
                [closest_pt[0], closest_pt[1], closest_pt[2], 1.0])
            pt_base = self.T_base_lidar @ pt_h

            # 장애물 메시지 1개 채우기 (rel_x=앞, rel_y=왼쪽, rel_z=위 방향 상대좌표)
            obs = ObstacleInfo()
            obs.label = label
            obs.class_id = cls_id
            obs.confidence = float(conf_val)
            obs.rel_x = float(pt_base[0])
            obs.rel_y = float(pt_base[1])
            obs.rel_z = float(pt_base[2])
            obs.distance = min_dist
            obs_array.obstacles.append(obs)

            # 디버그 시각화: 박스 위에 거리 표시
            cv2.putText(cv_img, f'{min_dist:.1f}m',
                        (x1, y1 - 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # Publish — 장애물 목록 (핵심 출력. 빈 배열이어도 발행해 "살아있음"을 알림)
        self.pub_obstacles.publish(obs_array)

        # 디버그 오버레이 이미지 발행 (검게 칠한 아래 25%는 잘라냄)
        overlay_msg = self.bridge.cv2_to_imgmsg(cv_img[:roi_h], 'bgr8')
        overlay_msg.header = img_msg.header
        self.pub_overlay.publish(overlay_msg)

        # 물체에 속한 라이다 점만 모아 별도 클라우드로 발행 (RViz 확인용)
        obj_pts = pts_np[is_object]
        if obj_pts.shape[0] > 0:
            obj_pc_msg = pc2.create_cloud_xyz32(
                lidar_msg.header, obj_pts.tolist())
            self.pub_obj_pc.publish(obj_pc_msg)

        # 성능 로그: 추론 시간 / 전체 처리 시간 / 탐지된 장애물 수
        dt_total = time.time() - t0
        self.get_logger().info(
            f'[PERF] Infer: {dt_infer*1000:.0f}ms | '
            f'Total: {dt_total*1000:.0f}ms | '
            f'Objects: {len(obs_array.obstacles)}')


def main(args=None):
    # 프로그램 시작점: ROS2 초기화 → 노드 생성 → spin(콜백 처리 무한루프)
    rclpy.init(args=args)
    node = HesaiZedFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:      # Ctrl+C로 종료 시 에러 없이 빠져나옴
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
