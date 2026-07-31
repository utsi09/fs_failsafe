#!/usr/bin/env python3
"""[awcl_adapter.py] CARLA/Autoware(awcl) 토픽 → cdr_monitor 입력 토픽 어댑터.

cdr_monitor는 /vel, /heading, /hesai_zed/obstacles 같은 자기만의 입력 토픽을
기대하는데, Autoware는 다른 이름/형식으로 데이터를 낸다. 이 노드가 중간에서
"번역기" 역할을 한다. 변환 목록:

/localization/kinematic_state (nav_msgs/Odometry, 차의 위치+속도)
    -> /vel     (geometry_msgs/TwistStamped, 속도만 뽑아서)
    -> /heading (geometry_msgs/QuaternionStamped, 방향만 뽑아서)
/perception/object_recognition/objects (인식된 주변 물체들)
    -> /hesai_zed/obstacles (가장 가까운 물체까지의 거리 하나)
    -> /monitor/ttc_ms (시공간 TTC[ms]: 계획된 주행 경로와 물체들의 예측 경로가
       처음으로 안전 반경 안까지 가까워지는 시각. 예측 범위 안에 만남이 없으면 -1)

즉 이 파일 덕분에 cdr_monitor 본체는 CARLA/Autoware를 전혀 몰라도 된다.
"""
import math

import rclpy                    # ROS2 파이썬 클라이언트 라이브러리
from rclpy.node import Node     # ROS2 노드 베이스 클래스
from rclpy.qos import QoSProfile, DurabilityPolicy   # QoS = 통신 품질 설정(버퍼 크기 등)

# 주고받을 메시지 타입들
from nav_msgs.msg import Odometry                       # 위치+속도 (Autoware가 발행)
from std_msgs.msg import Float64                        # 숫자 하나짜리 메시지
from geometry_msgs.msg import TwistStamped, QuaternionStamped   # 속도 / 방향(쿼터니언)
from autoware_perception_msgs.msg import PredictedObjects       # 인식된 물체 + 예측 경로
from autoware_planning_msgs.msg import Trajectory               # 계획된 주행 경로
from hesai_zed_fusion.msg import ObstacleArray, ObstacleInfo    # cdr_monitor가 먹는 장애물 형식

# TTC 계산 파라미터 (튜닝 상수)
TTC_HORIZON_S = 6.0    # 미래를 몇 초까지 내다볼지
TTC_STEP_S = 0.25      # 시간을 몇 초 간격으로 잘라 검사할지
TTC_RADIUS_M = 2.5     # 차와 물체가 이 거리 안이면 "만남(충돌 위험)"으로 판정


class AwclAdapter(Node):
    def __init__(self):
        super().__init__("awcl_adapter")   # "awcl_adapter"라는 이름의 ROS2 노드로 등록
        # depth=1: 최신 메시지 1개만 유지 (오래된 건 버림). 감시용이라 최신값만 중요.
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE)

        # ── 출력(발행) 쪽: cdr_monitor가 구독하는 토픽들 ──
        self._vel_pub = self.create_publisher(TwistStamped, "/vel", qos)                    # 차 속도
        self._heading_pub = self.create_publisher(QuaternionStamped, "/heading", qos)       # 차 방향
        self._obstacle_pub = self.create_publisher(ObstacleArray, "/hesai_zed/obstacles", qos)   # 최근접 장애물 거리

        self._ttc_pub = self.create_publisher(Float64, "/monitor/ttc_ms", qos)              # 시공간 TTC
        # 실험 채점기(near-miss 판정)용: 가장 가까운 물체까지의 거리도 따로 발행
        self._gap_pub = self.create_publisher(Float64, "/monitor/nearest_obstacle_m", qos)

        # ── 입력(구독) 쪽: Autoware가 발행하는 토픽들 ──
        self._ego_xy = None    # 내 차의 최신 (x, y) 위치. 아직 못 받았으면 None
        self._traj = None      # 최신 계획 경로. 아직 못 받았으면 None
        # create_subscription(타입, 토픽, 콜백, qos):
        # 그 토픽에 새 메시지가 도착할 때마다 콜백 함수가 자동 호출된다
        self.create_subscription(Odometry, "/localization/kinematic_state",
                                 self._odom_cb, qos)
        self.create_subscription(PredictedObjects, "/perception/object_recognition/objects",
                                 self._objects_cb, qos)
        self.create_subscription(Trajectory, "/planning/scenario_planning/trajectory",
                                 self._traj_cb, qos)

    def _traj_cb(self, msg):
        # 계획 경로가 오면 저장만 해 둔다 (TTC 계산 때 꺼내 씀)
        self._traj = msg

    # Odometry(위치+속도) 한 덩어리를 받아 /vel 과 /heading 두 개로 쪼개 발행
    def _odom_cb(self, msg):
        # 내 차 위치 저장 (장애물 거리 계산에 필요)
        self._ego_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        # 속도 부분만 뽑아 /vel 로 발행
        vel = TwistStamped()
        vel.header = msg.header          # 시각/좌표계 정보는 원본 그대로
        vel.twist = msg.twist.twist      # twist = (직선속도, 회전속도) 묶음
        self._vel_pub.publish(vel)

        # 방향(쿼터니언) 부분만 뽑아 /heading 으로 발행
        heading = QuaternionStamped()
        heading.header = msg.header
        heading.quaternion = msg.pose.pose.orientation
        self._heading_pub.publish(heading)

    # 인식된 물체 목록을 받아 (1) 최근접 거리, (2) 시공간 TTC 를 계산해 발행
    def _objects_cb(self, msg):
        if self._ego_xy is None:
            return   # 내 차 위치를 아직 모르면 거리를 잴 수 없음 → 이번 건 건너뜀
        ex, ey = self._ego_xy
        # 모든 물체 중 내 차와의 최단 거리 찾기
        min_dist = float("inf")   # 무한대로 시작해서 더 작은 값으로 갱신
        for obj in msg.objects:
            p = obj.kinematics.initial_pose_with_covariance.pose.position   # 물체의 현재 위치
            min_dist = min(min_dist, math.hypot(p.x - ex, p.y - ey))        # hypot = 직선거리

        # cdr_monitor가 먹는 ObstacleArray 형식으로 포장
        out = ObstacleArray()
        out.header = msg.header
        ob = ObstacleInfo()
        ob.label = "nearest"
        # 물체가 하나도 없으면 min_dist가 무한대로 남는다 → 그대로 보내면
        # 뒤쪽 계산이 이상해지므로 "멀지만 정상적인" 1000m로 바꿔서,
        # FTTI가 기본값(850ms)에 머물게 한다
        ob.distance = min_dist if math.isfinite(min_dist) else 1000.0
        out.obstacles.append(ob)
        self._obstacle_pub.publish(out)

        # 시공간 TTC와 최근접 거리도 발행
        self._ttc_pub.publish(Float64(data=self._spacetime_ttc_ms(msg)))
        self._gap_pub.publish(Float64(data=ob.distance))

    # ------------------------------------------------- 시공간(spatio-temporal) TTC
    def _ego_samples(self):
        """계획 경로를 [(t, x, y)] 목록으로 변환: "t초 뒤 내 차는 (x,y)에 있을 것".

        경로점에는 시각이 없고 위치+목표속도만 있어서, 시각은
        구간 길이 / 그 구간의 명령 속도 를 누적해서 직접 만든다."""
        if self._traj is None or len(self._traj.points) < 2:
            return None   # 경로가 없거나 너무 짧으면 계산 불가
        out, t = [], 0.0
        prev = self._traj.points[0].pose.position
        out.append((0.0, prev.x, prev.y))   # 시작점: t=0에 첫 경로점
        for p in self._traj.points[1:]:
            pos = p.pose.position
            ds = math.hypot(pos.x - prev.x, pos.y - prev.y)   # 이전 점과의 거리
            # 속도가 0에 가까우면 시간이 무한대가 되므로 최소 0.5m/s로 깔아준다
            v = max(abs(p.longitudinal_velocity_mps), 0.5)
            t += ds / v                      # 거리/속도 = 이 구간에 걸리는 시간, 누적
            out.append((t, pos.x, pos.y))
            prev = pos
            if t > TTC_HORIZON_S:            # 6초 너머는 볼 필요 없음
                break
        return out

    @staticmethod
    def _interp(samples, t):
        """[(t, x, y)] 표에서 임의 시각 t의 위치를 선형 보간으로 구한다.
        표 범위를 지나면(경로 끝 이후면) None."""
        if t > samples[-1][0]:
            return None
        for i in range(1, len(samples)):
            t1, x1, y1 = samples[i]
            if t <= t1:   # t가 samples[i-1]과 samples[i] 사이에 있음
                t0, x0, y0 = samples[i - 1]
                # a = 두 점 사이에서 t가 어디쯤인지 (0~1 비율)
                a = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
                # 비율만큼 섞어서 중간 위치 계산
                return (x0 + a * (x1 - x0), y0 + a * (y1 - y0))
        return None

    def _spacetime_ttc_ms(self, msg):
        """내 차의 계획 경로와 어떤 물체의 예측 경로가 처음으로
        TTC_RADIUS_M(2.5m) 안까지 가까워지는 시각[ms]. 없으면 -1.

        핵심: 단순히 "지금 거리/지금 속도"가 아니라, 미래의 각 시각에
        "그때 내 차 위치"와 "그때 물체 위치"를 비교한다. 그래서 교차로에서
        옆에서 오는 차처럼 직선 계산으로는 안 잡히는 위험도 잡는다."""
        ego = self._ego_samples()   # 내 차의 (시각, 위치) 표
        if ego is None:
            return -1.0
        best = -1.0   # 지금까지 찾은 가장 이른 만남 시각 (초). -1 = 아직 없음
        for obj in msg.objects:
            paths = obj.kinematics.predicted_paths   # 물체마다 예측 경로가 여러 개일 수 있음
            if not paths:
                continue
            # 확신도(confidence)가 가장 높은 예측 경로 하나만 사용
            path = max(paths, key=lambda p: p.confidence)
            # 예측 경로의 점 간격(초). sec + nanosec를 합쳐 초 단위로
            step = path.time_step.sec + path.time_step.nanosec * 1e-9
            if step <= 0.0 or not path.path:
                continue   # 간격이 이상하거나 경로가 비어 있으면 건너뜀
            # 0.25초 간격으로 미래를 훑으며 "그때 둘 사이 거리"를 확인
            t = TTC_STEP_S
            while t <= TTC_HORIZON_S:
                e = self._interp(ego, t)         # t초 뒤 내 차 위치
                if e is None:
                    break                        # 내 경로가 끝났으면 이 물체는 그만
                # t초 뒤 물체 위치: 예측 경로에서 t에 해당하는 점 인덱스
                # (min으로 마지막 점을 넘지 않게 방어)
                idx = min(int(t / step), len(path.path) - 1)
                o = path.path[idx].position
                if math.hypot(o.x - e[0], o.y - e[1]) < TTC_RADIUS_M:
                    # 만남 발견! 여러 물체 중 가장 이른 시각을 유지
                    best = t if best < 0 else min(best, t)
                    break   # 이 물체는 첫 만남만 알면 되므로 종료
                t += TTC_STEP_S
        # 초 → 밀리초로 바꿔 반환. 만남이 없으면 -1
        return best * 1000.0 if best > 0 else -1.0


def main():
    rclpy.init()            # ROS2 통신 초기화
    node = AwclAdapter()    # 어댑터 노드 생성 (구독/발행 연결이 이때 걸림)
    rclpy.spin(node)        # 메시지가 올 때마다 콜백을 돌리며 무한 대기
    rclpy.shutdown()        # (Ctrl+C 등으로 spin이 끝나면) 정리


if __name__ == "__main__":
    main()
