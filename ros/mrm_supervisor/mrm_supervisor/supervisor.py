"""Score-based MRM supervisor.

Consumes /monitor/scores (cdr_monitor) and adds its own localization-jump
check, then runs the state machine:

  NORMAL      perception score healthy
  MRM_MOVE    degraded band (0.3 < s <= 0.7): pick the cheapest safe-stop
              candidate (shoulder / parking lot) by cost function and reroute
              Autoware to it via the AD API
  MRM_PARKED  arrived and stopped at the chosen spot
  MRM_DEFER   score below the hard floor (or localization too bad to drive):
              do nothing — the stock Autoware mrm_handler performs
              comfortable/emergency stop

eta(c)  = route_time(speed limits) + n_traffic_lights * tl_wait_s
          -- exposure time in the degraded state (risk accumulates per second,
             not per meter: ISO 26262 time-to-safe-state framing)

cost(c) = w_time * eta
        + w_tl * n_traffic_lights * g(cap_traffic_light)      [inf if camera dead]
        + w_sec * sum over required sectors of (1 - sector_score)
        + w_loc * eta * (1 - localization_score)
"""
import math
import os
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data

from std_msgs.msg import String
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from visualization_msgs.msg import Marker, MarkerArray
from autoware_adapi_v1_msgs.msg import RouteState, MrmState
from autoware_adapi_v1_msgs.srv import SetRoutePoints, ClearRoute
# [v4.9k] planning msgs는 autoware install 소스 시에만 존재 — 가드
# [sd 0827] 0.39의 외부속도제한 셀렉터는 autoware_internal_planning_msgs 타입을
# 구독 — tier4 타입 발행은 같은 토픽명에서 '무음 불달'(실측: 캡 발행 로그는 남고
# 실차속도는 도로 제한 그대로). internal 우선, 구환경 tier4 폴백.
try:
    from autoware_internal_planning_msgs.msg import VelocityLimit
except ImportError:
    try:
        from tier4_planning_msgs.msg import VelocityLimit
    except ImportError:
        VelocityLimit = None
from autoware_perception_msgs.msg import PredictedObjects

from mrm_supervisor.map_model import MapModel


def yaw_to_quat(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


class Supervisor(Node):
    def __init__(self):
        super().__init__("mrm_supervisor")

        self.declare_parameter("map_path", "/home/taewook/awcl_maps/Town07/lanelet2_map.osm")
        # baseline mode: identical trigger condition, but instead of the
        # shoulder/parking reroute hand the vehicle straight to the stock
        # Autoware mrm_handler (comfortable/emergency stop in lane). Used as
        # the comparison arm in the thesis experiments.
        self.declare_parameter("baseline", False)
        # 데모 전용: 갓길 후보 제외(주차장 후보가 최선이 되게) — 캠페인 기본 off
        self.declare_parameter("demo_no_shoulder", False)
        self.declare_parameter("upper_threshold", 0.7)
        self.declare_parameter("lower_threshold", 0.3)
        self.declare_parameter("min_loc_to_move", 0.5)
        # [v4.6] 라이다(장애물 인지) 이동 하한 — capability/obstacle이 이 밑이면
        # 이동형 MRC 대신 제자리 DEFER (ln-mid 실측 0.14 vs 정상 0.97)
        self.declare_parameter("min_obstacle_to_move", 0.4)
        self.declare_parameter("hold_ticks", 10)          # 1 s at 10 Hz
        # [0822 ablation] 유예·바닥 지속시간 파라미터화 — 기본값 = 기존 하드코딩값.
        # lowloc_hold_ticks: 이동게이트 loc 미달 시 회복 유예 (기존 50틱=5s)
        # floor_hold_ticks: 종단 DEFER에 요구하는 바닥 지속 증거 (기존 20틱=2s)
        self.declare_parameter("lowloc_hold_ticks", 50.0)   # float: env 오버라이드 루프가 float()로 셋
        self.declare_parameter("floor_hold_ticks", 20.0)
        # [0822] 하드 페일 고속 경로: 센서 사망(신선도 마감 경과, timeout_score=0)이
        # 확인되면 밴드 디바운스를 건너뛴다 — 침묵 850ms 자체가 지속 증거라
        # 디바운스 중복이기 때문. 1.0=켬 / 0.0=끔(본 캠페인 동작 보존 기본값).
        self.declare_parameter("hard_fail_fast", 0.0)
        self.declare_parameter("w_time", 7.0)     # cost per second of degraded driving
        self.declare_parameter("tl_wait_s", 15.0) # expected wait per traffic light
        self.declare_parameter("w_tl", 200.0)
        self.declare_parameter("w_sector", 500.0)
        self.declare_parameter("w_loc", 14.0)     # per second, scaled by (1 - loc)
        self.declare_parameter("arrive_dist", 4.0)
        self.declare_parameter("shoulder_mrc_penalty", 175.0)  # MRC 질 서열 [비용]
        # [η_max 0826] 저하 상태 노출 시간(eta)의 절대 상한 [s] — 건강도가
        # MOVE 밴드(θ_L~θ_H)라도 eta가 이를 넘는 후보는 제외(∞), 전부 초과면
        # 후보 없음 → DEFER. 근거: 안전 상태 도달 시간은 유계여야 한다는
        # 표준 요구(ISO 26262 time-to-safe-state, UNECE R157 MRM 신속 종결
        # 원칙, ISO 23793 저속 MRM 프레임워크) + 수치는 위험분석 몫 —
        # 60 s(3 m/s 캡 기준 180 m, 도심 블록 1~2개)로 설정. ODD 검증:
        # 루트 전 지점 최선 후보 eta 최악 28.9 s의 약 2배 여유. env MRM_ETA_MAX
        self.declare_parameter("eta_max_s", 60.0)
        # [v4.9k 사용자 설계] MRM 이동 중 속도 캡 [m/s] — 저하 상태 이동의
        # 운동에너지 저감 (UNECE 감속 MRM 원칙). 0이면 비활성. env MRM_MOVE_SPEED
        self.declare_parameter("mrm_move_speed_limit", 4.2)   # ≈15 km/h
        # [후방 점유 폴백] 주차 접근/기동 중 정지 상태로 후방 근접 차량이
        # rear_block_hold_s 이상 지속되면 주차장 후보 전체에 페널티를 걸어
        # 재선택 — 노측 정차(후진 불필요)가 비용에서 이기게 된다
        self.declare_parameter("rear_block_dist", 10.0)    # [m] 후방 감시 거리
        self.declare_parameter("rear_block_lat", 2.5)      # [m] 횡방향 허용 폭
        self.declare_parameter("rear_block_hold_s", 20.0)  # [s] 지속 시간
        # 예방적 바이어스 (기본 0 = off, env MRM_REAR_BIAS 로 켬 — ablation)
        self.declare_parameter("rear_bias_penalty", 0.0)
        # [Arm C 민감도 실험] env로 임계 오버라이드 — 미설정 시 완전 무동작.
        # 러너 CLI에 파라미터 배관 없이 케이스별 스윕을 돌리기 위한 통로.
        from rclpy.parameter import Parameter as _P
        for _pname, _ev in (("min_loc_to_move", "MRM_MIN_LOC"),
                            ("min_obstacle_to_move", "MRM_MIN_OBS"),
                            ("lower_threshold", "MRM_LOWER"),
                            ("shoulder_mrc_penalty", "MRM_SHOULDER_PEN"),
                            ("eta_max_s", "MRM_ETA_MAX"),
                            ("rear_bias_penalty", "MRM_REAR_BIAS"),
                            ("mrm_move_speed_limit", "MRM_MOVE_SPEED"),
                            ("lowloc_hold_ticks", "SUP_LOWLOC_HOLD"),
                            ("floor_hold_ticks", "SUP_FLOOR_HOLD"),
                            ("hard_fail_fast", "SUP_HARD_FAST")):
            _v = os.environ.get(_ev, "")
            if _v:
                self.set_parameters([_P(_pname, value=float(_v))])
                self.get_logger().warn(f"[armC] {_pname} <- {_v} (env {_ev})")
        # a pull-over needs approach distance (decel + lateral shift);
        # candidates closer than this are infeasible, not "cheap"
        self.declare_parameter("min_approach_dist", 25.0)
        self.declare_parameter("shoulder_sectors", ["F", "FR"])
        self.declare_parameter("parking_sectors", ["F", "FL", "FR", "B", "RL", "RR"])
        # traffic-light observability: cameras (mount yaw / FOV, deg) that could
        # cover a signal ahead. A TL route is allowed if ANY of them is healthy.
        self.declare_parameter("tl_camera_names", ["CAMERA_front"])
        self.declare_parameter("tl_camera_yaws_deg", [0.0])
        self.declare_parameter("tl_camera_fovs_deg", [90.0])
        self.declare_parameter("tl_visible_min_score", 0.5)
        self.declare_parameter("tl_bearing_deg", 30.0)  # signals sit within +-30 deg ahead
        # v2: raw-IMU watch (see _imu_cb). Topic follows the /sensing/* naming
        # of the other raw-input subscriptions (cdr sensors_awcl.yaml uses the
        # same /sensing/imu/imu_data).
        self.declare_parameter("imu_topic", "/sensing/imu/imu_data")
        self.declare_parameter("imu_window_s", 3.0)     # sliding evaluation window
        self.declare_parameter("imu_calib_s", 3.0)      # boot baseline window
        self.declare_parameter("imu_var_ratio", 25.0)   # (a) variance vs baseline
        self.declare_parameter("imu_bias_thresh", 0.2)  # (b) mean offset [rad/s]

        map_path = self.get_parameter("map_path").value
        self.map = MapModel(map_path)
        self.candidates = self.map.extract_candidates(
            list(self.get_parameter("shoulder_sectors").value),
            list(self.get_parameter("parking_sectors").value))
        self.get_logger().info(
            f"map loaded: {len(self.map.lanelets)} lanelets, "
            f"{len(self.candidates)} MRM candidates "
            f"({sum(1 for c in self.candidates if c.kind == 'shoulder')} shoulder, "
            f"{sum(1 for c in self.candidates if c.kind == 'parking_lot')} parking)")

        self.scores = {}           # name -> score from /monitor/scores
        self._sensor_timeout = {}  # sensor/<name> -> timeout_score (0.0 = 사망 확정)
        # [0822] 케이스 로그 로테이션에 안 지워지는 이벤트 원장 (충돌 원장과 동일 철학):
        # 사망 최초 관측/고속 경로 발화/모든 상태 전이를 epoch 스탬프로 append.
        self._events_path = ("/home/taewook/autoware_carla_leaderboard/"
                             "experiments/results/sup_events.log")
        self._dead_flagged = False
        self._dead_list = []
        self._fp_logged = False    # FAST_PATH 원장 에피소드당 1회
        # [0822] 후보 경로 사전계산 캐시: 결정 시점의 후보 선정(다익스트라×후보,
        # 실측 1.3~2.0 s)이 하드페일 1.5 s 마감을 깨는 병목 — NORMAL에서 2틱마다
        # 후보 1개씩 순환 재계산해 두고 결정 시엔 캐시(5 s 신선)를 쓴다.
        self._route_cache = {}     # cand.name -> (epoch, route tuple|None)
        self._precomp_tick = 0
        self.pose = None           # (x, y)
        self.speed = 0.0
        self.yaw_rate = 0.0
        self._ekf_wz_ema = 0.0
        from collections import deque as _dq
        self._ekf_win = _dq()             # (t, EKF wz) — IMU 창과 같은 길이
        self._imu_noise_bad_since = None  # 지속 조건 추적
        self._imu_bias_bad_since = None
        self.loc_jump_score = 1.0
        self.loc_meas_score = 1.0  # raw GNSS/NDT pose vs EKF output residual
        self._meas_base = 0.0      # static frame offset learned at boot
        self._meas_last_recv = None  # GNSS 입력 침묵(staleness) 감지용
        self._meas_n = 0
        self._last_odom = None
        # v2: raw-IMU health (angular velocity z) — noise/bias scores
        self.imu_noise_score = 1.0
        self.imu_bias_score = 1.0
        # [v4.7 ZUPT] 정지 중 자이로 영속도 제약 감시 — 정지=회전0이라는
        # 절대 정답이 있어 기준선 불필요, 잔차 채널의 정지 사각지대를 메꿈
        self.imu_zupt_score = 1.0
        self._zupt_win = deque()   # (t, wz) while stationary
        self._zupt_bad_since = None
        self._imu_win = deque()    # (t, wz) samples inside imu_window_s
        self._imu_t0 = None
        self._imu_calib_n = 0      # Welford accumulators for the boot baseline
        self._imu_dmean = 0.0      # mean of first differences (noise baseline)
        self._imu_base_m2 = 0.0    # M2 of first differences
        self._imu_base_mean = 0.0  # mean of raw gyro z (bias baseline)
        self._imu_mean_sum = 0.0
        self._imu_mean_n = 0
        # v2: sensors whose quality estimate itself is invalid (cdr v2
        # quality_valid=false) — counted separately from scored sensors
        self.unknown_sensors = set()

        self.state = "NORMAL"
        # armed only after real scores show a healthy system once — startup
        # transients (leaderboard still spawning sensors) must not trigger MRM
        self._armed = False
        self._armed_ticks = 0
        self._band_ticks = 0
        self._floor_ticks = 0   # consecutive ticks at/below the hard floor
        self._chosen = None
        self._goal_sent = False
        self._goal_sent_time = None
        self._goal_accepted = False
        self._noresp = 0
        self._select_failures = 0
        self._blacklist = set()      # candidates goal_planner failed to park at
        self._stuck_since = None
        self._accept_time = None
        self._accept_pose = None
        self._lowloc_ticks = 0  # consecutive band-confirmed ticks with loc < min_loc_to_move
        self._lowobs_ticks = 0  # consecutive band-confirmed ticks with obstacle < min_obstacle_to_move
        # [후방 점유 폴백] 공격적 후행 차량이 후진주차 공간을 내주지 않으면
        # 주차장 후보에 동적 페널티를 걸어 노측(후진 불필요)으로 재라우팅
        self._rear_block_since = None
        self._parking_denied = False
        self._objects = []        # [(x, y)] map frame, perception 객체 위치
        self.ego_yaw = None

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(DiagnosticArray, "/monitor/scores", self._scores_cb, qos)
        self.create_subscription(Odometry, "/localization/kinematic_state", self._odom_cb, qos)
        # Autoware's own arrival signal: goal_planner may refine the pull-over
        # pose tens of meters away from our candidate midpoint, so a pure
        # euclidean arrive_dist check can never fire (observed 19 m offset)
        self.route_state = RouteState.UNKNOWN
        self.create_subscription(
            RouteState, "/api/routing/state", self._route_state_cb,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # GNSS/NDT measurement quality: the EKF smooths noisy pose inputs so
        # its OUTPUT looks clean (loc_jump misses gn faults entirely) — the
        # input-vs-output residual is where measurement noise is visible
        self.create_subscription(
            PoseWithCovarianceStamped, "/sensing/gnss/pose_with_covariance",
            self._meas_pose_cb, qos)
        # 잔차 기반 감시의 IMU 사각지대(백색노이즈 흡수·바이어스 누적) 보완 —
        # v1 캠페인에서 imn 40/40 미감지 실증. EKF 출력(loc_jump)도
        # 입력-출력 잔차(loc_meas)도 자이로 백색노이즈는 흡수해 버리고
        # 바이어스는 천천히 누적되므로, 원신호를 직접 본다.
        self.create_subscription(
            Imu, self.get_parameter("imu_topic").value,
            self._imu_cb, qos_profile_sensor_data)
        # [후방 점유 폴백] 인지 객체로 후방 근접 차량 감시
        self.create_subscription(
            PredictedObjects, "/perception/object_recognition/objects",
            self._objects_cb, qos)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, "/mrm_supervisor/state", latched)
        # [v5 절제] SUP_SHADOW=1: 상태 전이·로그만, 일체 행동 금지 (A1 섀도 감지기)
        self._shadow = os.environ.get("SUP_SHADOW", "") == "1"
        if self._shadow:
            self.get_logger().warn("SHADOW mode: transitions only, no actuation")
        self.marker_pub = self.create_publisher(MarkerArray, "/mrm_supervisor/candidates", qos)
        self.route_cli = self.create_client(SetRoutePoints, "/api/routing/change_route_points")
        # 행동계획기 동결(빈 궤적, 재전송 무효) 강제 리셋용 — 스톨 재시도 2회째
        # clear_route 후 2 s 뒤 재전송 (8/11 1003003: 재전송 2회 무효 실측)
        self.clear_cli = self.create_client(ClearRoute, "/api/routing/clear_route")
        # clear 후엔 경로가 UNSET이라 change_route_points 는 "route is not set yet"
        # 으로 전부 거부됨(8/11 rep1 실측: 거부 15연속→블랙리스트) — UNSET일 땐
        # set_route_points 로 새로 설정해야 한다
        self.set_route_cli = self.create_client(
            SetRoutePoints, "/api/routing/set_route_points")
        # DEFER hand-over to the stock Autoware MRM: the diagnostic graph links
        # "mrm_supervisor: defer_request" into /autoware/modes/autonomous, so
        # ERROR here makes autonomous unavailable -> mrm_handler performs
        # comfortable/emergency stop. Covers DEFER causes that the perception
        # score alone does not reach (TL gate, localization too low to move).
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        # [v4.9k] MRM 저속 캡 — planning 외부 속도제한 토픽 (latched)
        self.vlimit_pub = None
        if VelocityLimit is not None:
            # [v4.9u] max_velocity(셀렉터 "출력")에 직접 쏘면 셀렉터와 발행
            # 경합 -> MRM 재출발 불능 (0816 실측: 2.5/3.0 캡 3케이스 연속
            # AW_EMERGENCY, 발행 OFF 프로브는 정상 출발). 정식 입구인
            # candidates(셀렉터 입력)로 발행한다.
            self.vlimit_pub = self.create_publisher(
                VelocityLimit,
                "/planning/scenario_planning/max_velocity_candidates",
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        # [HUD] ego 위를 따라다니는 상태·건강점수 텍스트 (RViz MarkerArray)
        self.hud_pub = self.create_publisher(MarkerArray, "/mrm_supervisor/hud", qos)
        # [패널 반영] 좌상단 AutowareStatePanel의 MRM State/Behavior에 우리
        # 판정을 매핑해 표시 (mrm_handler와 동일 토픽 공유 — 우리 MRM 중일
        # 때만 발행해 충돌 최소화)
        # ADAPI 토픽은 latched 구독(transient_local) — volatile로 쏘면 QoS
        # 불일치로 패널이 못 받음 (8/11 실측: Inactive 유지)
        self.adapi_mrm_pub = self.create_publisher(MrmState, "/api/fail_safe/mrm_state", latched)
        self.create_timer(0.5, self._publish_hud)
        self.create_timer(0.1, self._tick)
        self._publish_state()

    # ------------------------------------------------------------- callbacks
    def _scores_cb(self, msg):
        for st in msg.status:
            quality_valid = True
            for kv in st.values:
                if kv.key == "score":
                    try:
                        self.scores[st.name] = float(kv.value)
                    except ValueError:
                        pass
                # [0822 hard-fail fast path] 센서별 신선도(타임아웃) 점수 —
                # 0.0 = 신선도 마감(850ms) 경과 확정. 침묵 850ms는 그 자체가
                # 지속 증거이므로 밴드 디바운스를 중복 적용하지 않기 위한 재료.
                elif kv.key == "timeout_score" and st.name.startswith("sensor/"):
                    try:
                        self._sensor_timeout[st.name] = float(kv.value)
                    except ValueError:
                        pass
                elif kv.key == "quality_valid" and kv.value.strip().lower() == "false":
                    quality_valid = False
            # cdr v2: quality_valid=false — the quality estimate itself is not
            # trustworthy (e.g. not enough data to judge). Treat the sensor as
            # UNKNOWN: keep a separate count, and weigh it conservatively as
            # 0.5 in health computations (see _score) instead of trusting a
            # possibly-optimistic score.
            if not quality_valid:
                if st.name not in self.unknown_sensors:
                    self.get_logger().warn(
                        f"{st.name}: quality_valid=false -> UNKNOWN "
                        f"(now {len(self.unknown_sensors) + 1} unknown)")
                self.unknown_sensors.add(st.name)
            else:
                self.unknown_sensors.discard(st.name)

    def _score(self, name, default=1.0):
        """Score lookup that caps UNKNOWN (quality_valid=false) sensors at a
        conservative 0.5: never trust an unverifiable sensor to be healthier
        than 'degraded', but keep an even worse reported score if present."""
        s = self.scores.get(name, default)
        return min(s, 0.5) if name in self.unknown_sensors else s

    def _imu_cb(self, msg):
        """v2: raw gyro-z watch. 잔차 기반 감시의 IMU 사각지대(백색노이즈
        흡수·바이어스 누적) 보완 — v1 캠페인에서 imn 40/40 미감지 실증.
        (a) imu_noise: 최근 3초 창의 표본 분산이 부팅(정지/직진) 기준선의
            imu_var_ratio배(기본 25배)를 초과하면 이상. 선회 시 실제 yaw
            운동이 창 분산에 섞이지 않도록 1차 차분(고주파 성분)의 분산으로
            비교한다 — 백색노이즈는 차분에서 2σ²로 그대로 보이고, 저주파의
            실제 회전 운동은 소거된다.
        (b) imu_bias: 창의 이동평균이 부팅 기준선 대비 imu_bias_thresh(기본
            0.2 rad/s) 이상 오프셋이면 이상. 장시간 선회는 임계 근처까지
            접근할 수 있으므로 임계는 파라미터로 조정 가능하다."""
        wz = msg.angular_velocity.z
        t = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds / 1e9
        # [v4.7 ZUPT] 정지(속도<0.1) 중엔 자이로가 0이어야 한다 — 평균 오프셋
        # (imb/imd)이나 과대 분산(imn) 모두 즉시 거짓말 확정. 1 s 창 + 2 s
        # 지속 요구(신호 미세 크립·정지 직후 과도 무시). 출발하면 창 리셋.
        if self.speed is not None and self.speed < 0.1:
            self._zupt_win.append((t, wz))
            while self._zupt_win and t - self._zupt_win[0][0] > 1.0:
                self._zupt_win.popleft()
            if len(self._zupt_win) >= 10:
                zvals = [v for _, v in self._zupt_win]
                zmean = sum(zvals) / len(zvals)
                zvar = sum((v - zmean) ** 2 for v in zvals) / len(zvals)
                if abs(zmean) > 0.05 or zvar > 0.01:
                    if self._zupt_bad_since is None:
                        self._zupt_bad_since = t
                    if t - self._zupt_bad_since >= 2.0:
                        excess = max(abs(zmean) - 0.05, 0.0) + max(zvar - 0.01, 0.0)
                        self.imu_zupt_score = 0.9 * self.imu_zupt_score + \
                            0.1 * math.exp(-excess / 0.1)
                else:
                    self._zupt_bad_since = None
                    self.imu_zupt_score = min(1.0, self.imu_zupt_score + 0.02)
        else:
            self._zupt_win.clear()
            self._zupt_bad_since = None
            # 주행 재개 시 서서히 회복 (잔차 채널이 이어받음)
            self.imu_zupt_score = min(1.0, self.imu_zupt_score + 0.005)
        # 주행 전(부팅 정지 상태)의 분산은 ~0이라 기준선으로 쓰면 출발 직후
        # 정상 진동조차 25배를 넘어 오경보가 난다(v2 스모크에서 실측).
        # 기준선 학습과 판정 모두 실주행(속도>=1m/s) 이후에만 수행한다.
        if self._imu_t0 is None:
            if self.speed < 1.0:
                return
            self._imu_t0 = t
        if t - self._imu_t0 < self.get_parameter("imu_calib_s").value:
            # boot baseline: faults only arm after driving starts, so this
            # window is clean. Mean of the raw signal for the bias check,
            # Welford variance of the FIRST DIFFERENCES for the noise check.
            if self._imu_win:
                d = wz - self._imu_win[-1][1]       # first difference
                self._imu_calib_n += 1
                delta = d - self._imu_dmean
                self._imu_dmean += delta / self._imu_calib_n
                self._imu_base_m2 += delta * (d - self._imu_dmean)
            self._imu_mean_sum += wz
            self._imu_mean_n += 1
            self._imu_base_mean = self._imu_mean_sum / self._imu_mean_n
            self._imu_win.append((t, wz))
            return
        win = self.get_parameter("imu_window_s").value
        self._imu_win.append((t, wz))
        while self._imu_win and t - self._imu_win[0][0] > win:
            self._imu_win.popleft()
        n = len(self._imu_win)
        if n < 10 or self._imu_calib_n < 5:
            return
        # (a) variance of first differences vs (adaptive) driving baseline.
        # nf 오탐 재발 원인: 저속 출발 구간에서 배운 기준선이 고속/거친 노면의
        # 정상 진동 증가를 못 따라감. -> 건강할 때(ratio<5) 기준선을 1%씩
        # 현재 값 쪽으로 적응시키고(라이다 baseline과 동일 패턴, 급변은 불추종),
        # 초과는 2초 이상 "지속"될 때만 감점한다(순간 스파이크 무시).
        diffs = [self._imu_win[i][1] - self._imu_win[i - 1][1] for i in range(1, n)]
        dmean = sum(diffs) / len(diffs)
        dvar = sum((d - dmean) ** 2 for d in diffs) / len(diffs)
        base_var = max(self._imu_base_m2 / (self._imu_calib_n - 1), 1e-8)
        ratio = dvar / base_var
        n_max = self.get_parameter("imu_var_ratio").value
        if ratio < 5.0:
            self._imu_base_m2 = 0.99 * self._imu_base_m2 + \
                0.01 * dvar * (self._imu_calib_n - 1)
        if ratio > n_max:
            if self._imu_noise_bad_since is None:
                self._imu_noise_bad_since = t
            sustained = t - self._imu_noise_bad_since >= 2.0
        else:
            self._imu_noise_bad_since = None
            sustained = False
        # [v2 비활성] 진동(분산) 채널: 기준선이 속도 종속(저속 학습치 << 고속
        # 정상 진동)이라 가속 구간에서 구조적 오탐 — nf 스모크 2회 실측.
        # 속도-구간별 기준선(binned baseline)로 재도전 예정. 바이어스 채널은 유지.
        noise_inst = 1.0
        # (b) 바이어스: 같은 창의 "IMU 평균 vs EKF 요레이트 평균" 차이.
        # (EMA는 창평균과 시간 스케일이 달라 선회 과도 구간에서 가짜 차이를
        # 만들었음 -> 동일 창 평균끼리 비교로 교체). 3초 지속 시에만 감점.
        mean = sum(v for _, v in self._imu_win) / n
        while self._ekf_win and t - self._ekf_win[0][0] > win:
            self._ekf_win.popleft()
        th = self.get_parameter("imu_bias_thresh").value
        if len(self._ekf_win) >= 5:
            ekf_mean = sum(v for _, v in self._ekf_win) / len(self._ekf_win)
            off = abs(mean - ekf_mean)
        else:
            off = 0.0
        if off > th:
            if self._imu_bias_bad_since is None:
                self._imu_bias_bad_since = t
            b_sustained = t - self._imu_bias_bad_since >= 3.0
        else:
            self._imu_bias_bad_since = None
            b_sustained = False
        bias_inst = math.exp(-max(0.0, off - th) / th) if b_sustained else 1.0
        self.imu_noise_score = 0.9 * self.imu_noise_score + 0.1 * noise_inst
        self.imu_bias_score = 0.9 * self.imu_bias_score + 0.1 * bias_inst

    def _route_state_cb(self, msg):
        self.route_state = msg.state

    def _meas_pose_cb(self, msg):
        # 시뮬 스탬프 기준(벽시계는 시뮬 감속 시 정상 공백을 침묵으로 오인
        # — nf 4/49 오탐 실측, v2.1 수정)
        self._meas_last_recv = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds / 1e9
        if self.pose is None:
            return
        # residual between the raw localization measurement and the EKF pose.
        # There is a STATIC offset between the two (vehicle-center vs base_link,
        # ~1.4 m) — learn it during boot (faults only fire after driving
        # starts) and score only the EXCESS over that baseline.
        r = math.dist(self.pose, (msg.pose.pose.position.x, msg.pose.pose.position.y))
        if self._meas_n < 60:                     # ~3 s calibration window
            self._meas_base = r if self._meas_n == 0 else \
                0.9 * self._meas_base + 0.1 * r
            self._meas_n += 1
            return
        excess = max(0.0, r - self._meas_base - 0.2)
        # gn mid (sigma 1 m, excess ~0.3) -> ~0.58: degraded band, still above
        # min_loc_to_move -> shoulder pull-over; gn high (sigma 2.5, excess
        # ~1.6) -> ~0.05: floor -> DEFER
        inst = math.exp(-excess / 0.55)
        self.loc_meas_score = 0.9 * self.loc_meas_score + 0.1 * inst

    def _odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.speed = math.hypot(vx, vy)
        self._sim_now = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds / 1e9
        self.yaw_rate = msg.twist.twist.angular.z
        self._ekf_wz_ema = 0.9 * self._ekf_wz_ema + 0.1 * self.yaw_rate
        # 'now'는 이 함수 아래쪽에서 정의되므로 여기서 쓰면 UnboundLocalError
        # (v2 배터리 전 케이스 출발불가 사고의 원인) — 스탬프를 직접 계산한다
        _t_ekf = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds / 1e9
        self._ekf_win.append((_t_ekf, self.yaw_rate))
        now = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds / 1e9
        if self._last_odom is not None:
            px, py, pt, pv = self._last_odom
            dt = now - pt
            if 0.0 < dt < 1.0:
                # innovation: actual displacement vs what the twist predicts
                jump = abs(math.dist((x, y), (px, py)) - pv * dt)
                inst = math.exp(-max(0.0, jump - 0.3) / 0.5)
                self.loc_jump_score = 0.9 * self.loc_jump_score + 0.1 * inst
        self._last_odom = (x, y, now, self.speed)
        self.pose = (x, y)
        q = msg.pose.pose.orientation
        self.ego_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _objects_cb(self, msg):
        self._objects = [
            (o.kinematics.initial_pose_with_covariance.pose.position.x,
             o.kinematics.initial_pose_with_covariance.pose.position.y)
            for o in msg.objects]

    def _rear_blocked(self):
        """ego 바로 뒤(진행축 기준)에 인지 객체가 있으면 True."""
        if self.pose is None or self.ego_yaw is None:
            return False
        dist = self.get_parameter("rear_block_dist").value
        lat = self.get_parameter("rear_block_lat").value
        cy, sy = math.cos(self.ego_yaw), math.sin(self.ego_yaw)
        for ox, oy in self._objects:
            dx, dy = ox - self.pose[0], oy - self.pose[1]
            lon = dx * cy + dy * sy          # ego 전방 +
            side = -dx * sy + dy * cy
            if -dist <= lon <= -1.0 and abs(side) <= lat:
                return True
        return False

    def _publish_defer_diag(self):
        st = DiagnosticStatus()
        st.name = "mrm_supervisor: defer_request"
        st.hardware_id = "mrm_supervisor"
        st.level = (DiagnosticStatus.ERROR if (self.state == "MRM_DEFER"
                    and not self._shadow) else DiagnosticStatus.OK)
        st.message = self.state
        st.values.append(KeyValue(key="state", value=self.state))
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status.append(st)
        self.diag_pub.publish(arr)

    # ------------------------------------------------------------------ tick
    def _tick(self):
        # [sd 0827] MRM_MOVE 동안 속도 캡 1 s 주기 재발행 — 외부 속도제한이
        # 셀렉터/리루트 경로에서 소실돼 도로 제한속도로 복귀하는 실측 방어.
        if (self.state == "MRM_MOVE"
                and getattr(self, "_active_vcap", 0.0) > 0.0
                and self.vlimit_pub is not None and not self._shadow):
            _now_s = self.get_clock().now().nanoseconds * 1e-9
            if _now_s - getattr(self, "_last_vcap_pub", 0.0) >= 1.0:
                vm = VelocityLimit()
                vm.stamp = self.get_clock().now().to_msg()
                vm.max_velocity = self._active_vcap
                vm.use_constraints = False
                vm.sender = "mrm_supervisor"
                self.vlimit_pub.publish(vm)
                self._last_vcap_pub = _now_s
        self._publish_defer_diag()
        # v2: periodic state publish alongside the immediate publish inside
        # _transition — external observers (judge) must never depend on the
        # latched one-shot alone (v1: detection 0.93 s vs observed 2.2 s)
        self._publish_state()
        perception = self._score("capability/perception")
        # v2: raw-IMU health joins the localization capability via min()
        # GNSS 입력 침묵 감지: loc_meas는 콜백에서만 갱신되어 입력이 끊기면
        # 마지막(건강한) 값에 얼어붙는다 — gb 블랙아웃 미감지의 원인(v2 실측).
        # 1초 이상 무수신이면 잔차 신뢰를 지수 감쇠시킨다. 수신기는 정차 중에도
        # 고정 주기로 발행하므로 침묵 검사에는 속도 게이트를 걸지 않는다
        # (값 기반 검사(ZUPT·점프)만 정차 무여기 오판 방지용 게이트 유지).
        if (self._armed
                and self._meas_last_recv is not None
                and getattr(self, "_sim_now", None) is not None):
            _stale = self._sim_now - self._meas_last_recv
            if _stale > 1.0:
                self.loc_meas_score = min(self.loc_meas_score,
                                          math.exp(-(_stale - 1.0) / 0.8))
        imu_health = min(self.imu_noise_score, self.imu_bias_score,
                         self.imu_zupt_score)
        cap_loc = min(self._score("capability/localization"),
                      self.loc_jump_score, self.loc_meas_score, imu_health)

        upper = self.get_parameter("upper_threshold").value
        lower = self.get_parameter("lower_threshold").value

        # trigger on the WORST capability: perception faults (lidar/camera)
        # and localization faults (IMU/GNSS -> EKF wobble, caught by the
        # odometry-innovation loc_jump score) must both be able to fire MRM
        health = min(perception, cap_loc)
        # HUD용 스냅샷: loc_jump/loc_meas는 내부 파생 점수라 scores에 없음 —
        # "센서 0.9인데 왜 정지?"의 답(loc 붕괴)이 화면에 보이게 (8/11)
        self._hud_health = (perception, cap_loc, health)

        # [0822] 사망 센서 목록 갱신 + 최초 관측 원장 (지연 분해용 스탬프)
        _dn = [n for n, v in self._sensor_timeout.items() if v <= 0.02]
        if _dn and not self._dead_flagged:
            self._dead_flagged = True
            self._ledger("DEAD_SEEN", ",".join(_dn))
        elif not _dn:
            self._dead_flagged = False
        self._dead_list = _dn

        if not self._armed:
            # require sustained health (3 s), not one blip: a transient 0.9
            # during boot armed the supervisor right into the boot dip, firing
            # a false MRM seconds after engagement
            if "capability/perception" in self.scores and perception >= upper:
                self._armed_ticks += 1
                if self._armed_ticks >= 30:
                    self._armed = True
                    self.get_logger().info(f"armed: perception {perception:.2f} healthy for 3 s")
            else:
                self._armed_ticks = 0
            return

        # MRM_DEFER is terminal, so a single bad tick must not trigger it
        self._floor_ticks = self._floor_ticks + 1 if health <= lower else 0

        if self.state == "NORMAL":
            # [0822] 후보 경로 사전계산 (2틱마다 1후보, 전 후보 ~3.2 s 주기)
            self._precomp_tick += 1
            if (self._armed and self.pose is not None and self.candidates
                    and self._precomp_tick % 2 == 0):
                c = self.candidates[(self._precomp_tick // 2) % len(self.candidates)]
                try:
                    r = None
                    for s_ in self.map.nearest_road_lanelets(*self.pose, k=3):
                        r = self.map.route(s_, c.target_lanelet)
                        if r is not None:
                            break
                    self._route_cache[c.name] = (time.time(), r)
                except Exception:
                    pass
            # 5틱(0.5s)은 gn 순간 스파이크(loc 0.13 실측)에 과민 — 종단 결정이므로
            # MOVE 브랜치와 동일하게 2s 지속 요구 (gn-high의 지속 붕괴는 여전히 포착)
            if self._floor_ticks >= self.get_parameter("floor_hold_ticks").value:
                self._transition("MRM_DEFER", f"health {health:.2f} below floor "
                                 f"(perception {perception:.2f}, loc {cap_loc:.2f})")
            elif health < upper:
                self._band_ticks += 1
                if self._band_ticks == 1 or self._band_ticks % 10 == 0:
                    self.get_logger().info(
                        f"degraded band: health {health:.2f} (p {perception:.2f}/l {cap_loc:.2f}) "
                        f"({self._band_ticks}/{self.get_parameter('hold_ticks').value} ticks)")
                # [0822 hard-fail fast path] 신선도 마감을 넘긴 죽은 센서가
                # 있으면(이미 850ms 지속 증거) 밴드 디바운스를 1틱으로 단축.
                # 단, 전 센서 동시 사망은 시뮬 프레임 스톨 아티팩트일 가능성이
                # 높으므로(0822 원장 실측: cf 케이스에서 라이다 전후+카메라 동시
                # 사망 관측) "살아있는 센서가 하나라도 있을 때"만 발화한다 —
                # 진짜 단일 센서 사망엔 지연 비용 0, 스톨엔 기존 디바운스 유지.
                _dead = self._dead_list
                _hold_req = self.get_parameter("hold_ticks").value
                _partial_death = _dead and len(_dead) < max(len(self._sensor_timeout), 1)
                if self.get_parameter("hard_fail_fast").value >= 1.0 and _partial_death:
                    if not self._fp_logged:
                        self._fp_logged = True
                        self.get_logger().warn(
                            f"hard-fail fast path: dead sensor(s) {_dead} — band hold skipped")
                        self._ledger("FAST_PATH", ",".join(_dead))
                    _hold_req = 1
                if self._band_ticks >= _hold_req:
                    if self.get_parameter("baseline").value:
                        self._transition("MRM_DEFER",
                                         f"baseline: health {health:.2f}, stock MRM")
                    elif cap_loc < self.get_parameter("min_loc_to_move").value:
                        # gn 온셋에서 EMA가 설계점(≈0.58)보다 깊게 패이는 과도구간을
                        # 순간 샘플링해 즉시 DEFER(8/10 실측 0.44) — 5 s 유지될 때만
                        # 포기하고, 회복하면 MOVE로 진행
                        _hold = self.get_parameter("lowloc_hold_ticks").value
                        self._lowloc_ticks += 1
                        if self._lowloc_ticks == 1 or self._lowloc_ticks % 20 == 0:
                            self.get_logger().warn(
                                f"loc {cap_loc:.2f} below move floor "
                                f"({self._lowloc_ticks}/{_hold:.0f} ticks) — holding")
                        if self._lowloc_ticks >= _hold:
                            self._transition("MRM_DEFER",
                                             f"degraded but localization {cap_loc:.2f} "
                                             f"too low to move for {_hold/10:.1f} s")
                    elif (min(self._score("capability/obstacle"),
                              self._score("sector/F"),
                              self._score("sector/FL"),
                              self._score("sector/FR"))
                          < self.get_parameter("min_obstacle_to_move").value
                          # 히스테리시스: 홀딩 시작 후엔 +0.05 넘어야 이동 허용
                          # (경계 진동 한 틱에 MOVE로 새는 플래핑 방지, v4.6b 실측)
                          + (0.05 if self._lowobs_ticks > 0 else 0.0)):
                        # [v4.6] 라이다 이동 게이트 — min_loc_to_move와 대칭.
                        # 강수·오염·블루밍 계열 열화는 경로 위 유령 장애물을
                        # 만들어 이동형 MRC를 차선 한복판 스턱으로 끝낸다
                        # (0812 ln-mid 11/11 실측: obstacle 0.14, route-obstacle
                        # 정지 → 전 재라우팅 "not safe" 거부 → 560 s 스턱).
                        # 앞이 안 보이면 이동 대신 제자리 DEFER가 안전.
                        _o = self._score("capability/obstacle")
                        _f = self._score("sector/F")
                        _fl = self._score("sector/FL")
                        _fr = self._score("sector/FR")
                        obs_gate = min(_o, _f, _fl, _fr)
                        self._lowobs_ticks += 1
                        if self._lowobs_ticks == 1 or self._lowobs_ticks % 20 == 0:
                            self.get_logger().warn(
                                f"obstacle perception {obs_gate:.2f} "
                                f"(obs {_o:.2f} F {_f:.2f} FL {_fl:.2f} FR {_fr:.2f}) "
                                f"below move floor "
                                f"({self._lowobs_ticks}/50 ticks) — holding")
                        # [0823] 하드페일(라이다 사망)로 무너진 전방 시야는 5 s를
                        # 기다려도 회복 불가 — lowloc·후보전멸과 동일하게 즉시 위임
                        # (lf-ours 실측 5.95 s = 0.95 판정 + 5.0 무의미 유예의 수리)
                        _hf_dead = (self.get_parameter("hard_fail_fast").value >= 1.0
                                    and self._dead_list
                                    and len(self._dead_list) < max(len(self._sensor_timeout), 1))
                        if _hf_dead or self._lowobs_ticks >= 50:
                            self._transition(
                                "MRM_DEFER",
                                f"degraded but obstacle perception {obs_gate:.2f} "
                                + ("too low to move (hard-fail, immediate)" if _hf_dead
                                   else "too low to move for 5 s"))
                    else:
                        self._enter_mrm_move(cap_loc)
            else:
                # decay instead of reset so a borderline score oscillating
                # around the threshold still accumulates
                self._band_ticks = max(0, self._band_ticks - 1)
                self._lowloc_ticks = 0
                self._lowobs_ticks = 0
                self._fp_logged = False

        elif self.state == "MRM_MOVE":
            # 5틱(0.5s)은 트래픽 회피 급제동+gn 노이즈의 innovation 스파이크에
            # 과민(8/11 def 2건 실측: p 0.95+인데 loc 순간 0) — 2s 지속 요구
            if self._floor_ticks >= self.get_parameter("floor_hold_ticks").value:
                self._transition("MRM_DEFER", f"health {health:.2f} collapsed en route")
                return
            # reject-loop 등으로 현재 타깃이 블랙리스트됐으면 즉시 재선택
            if self._chosen is not None and self._chosen.name in self._blacklist:
                best = self._select_candidate(cap_loc)
                if best is None:
                    self._transition("MRM_DEFER", "no viable candidate left — stock MRM")
                    return
                self._chosen = best
                self._goal_sent = False
                self._publish_state()
                self.get_logger().warn(f"fallback target: {best.name}")
            # route-change watchdog: near its own goal Autoware's goal_planner
            # takes over and the change service can go unanswered — the car
            # then silently finishes the OLD route. Retry, then hand over to
            # the stock MRM instead of driving on as if nothing happened.
            if (self._goal_sent and not self._goal_accepted
                    and self._goal_sent_time is not None
                    and (self.get_clock().now() - self._goal_sent_time).nanoseconds > 5e9):
                self._noresp += 1
                self.get_logger().error(
                    f"route change unanswered for 5 s (try {self._noresp}/3) — retrying")
                self._goal_sent = False
                if self._noresp >= 3:
                    self._transition("MRM_DEFER", "route change unanswered — stock MRM")
                    return
            if not self._goal_sent:
                now_s = self.get_clock().now().nanoseconds * 1e-9
                if now_s >= getattr(self, "_reject_until", 0.0):
                    self._send_goal()
            elif self._chosen and self.pose:
                d = math.dist(self.pose, (self._chosen.x, self._chosen.y))
                # decompose into along-shoulder / across-shoulder: a plain
                # radius check declared "parked" while stopped IN THE LANE
                # 3.5 m abeam of the shoulder marker
                dx = self.pose[0] - self._chosen.x
                dy = self.pose[1] - self._chosen.y
                cy, sy = math.cos(self._chosen.yaw), math.sin(self._chosen.yaw)
                lon = dx * cy + dy * sy
                lat = -dx * sy + dy * cy
                on_shoulder = (abs(lon) < self.get_parameter("arrive_dist").value
                               and abs(lat) < 1.5)
                arrived = on_shoulder or self.route_state == RouteState.ARRIVED
                # 주차장: mission 라우트가 골 수정으로 링/진입로에서 조기 ARRIVED를
                # 쏴서 4.3m 앞에서 PARKED 처리, freespace 기동이 잘림(8/10 실측)
                # — 실제 freespace 완주는 골 0.3~0.5m 안에 꽂히므로 1.2m 밖이면
                # 미도달. 3.0m는 진입로 정차(차로 41% 점유)를 PARKED 로 오판했다
                # (8/11 agg rep1 실측: d=2.9m 가짜 성공)
                if self._chosen.kind == "parking_lot" and d > 1.2:
                    arrived = False
                if arrived and self.speed < 0.2:
                    self._transition("MRM_PARKED",
                                     f"stopped at {self._chosen.name} "
                                     f"(d={d:.1f}m, lat={lat:.1f}m, "
                                     f"route_state={self.route_state})")
                    return
                # creep guard: noisy-lidar phantom stops can keep the ego
                # crawling (0.2-1 m/s) forever — below the full-stop stall
                # detector. If 90 s after accept we have moved <25 m, the
                # exposure outweighs the pull-over benefit: try the next
                # candidate (exhausted -> DEFER).
                # 주차장 후보는 제외: 시나리오 전환+costmap+A*+기동에 60~120 s가
                # 걸려 90 s/25 m 기준이 기동 중 골을 뺏는다(8/10 밤 39건 전패 원인,
                # 45 s stall 타이머와 동일한 실수) — 주차장은 180 s stall 검출이 담당
                if (self._goal_accepted and self._accept_time is not None
                        and self._accept_pose is not None
                        and self._chosen.kind != "parking_lot"
                        and (self.get_clock().now() - self._accept_time).nanoseconds > 90e9
                        and math.dist(self.pose, self._accept_pose) < 25.0):
                    self._blacklist.add(self._chosen.name)
                    self.get_logger().error(
                        f"crawling: <25 m progress in 90 s toward "
                        f"{self._chosen.name} — trying next candidate")
                    self._goal_sent = False
                    self._goal_accepted = False
                    self._accept_time = None
                    best = self._select_candidate(cap_loc)
                    if best is None:
                        self._transition("MRM_DEFER",
                                         "no viable candidate left — stock MRM")
                        return
                    self._chosen = best
                    self._publish_state()
                    self.get_logger().warn(f"fallback target: {best.name}")
                    return
                # goal_planner can end up with "no path candidate" (too-narrow
                # or overshot shoulder): ego stalls without ever reaching the
                # shoulder. Blacklist that candidate and fall back to the next.
                # 45 s > a full Town07 red phase, so light waits don't trigger.
                # [주차장 데모] 폴리곤 안 정차 순간 골 1회 재전송(넛지):
                # 새 route uuid -> scenario EMPTY 리셋 -> 위치 재판정 -> Parking 전환.
                # 모든 성공 사례의 실제 전환 메커니즘(실측).
                # _moved_since_goal 가드: fault 시점 ego가 이미 폴리곤 안(신호 대기
                # 등)이면 출발 전에 넛지 3발이 13 s 만에 소진되며 route 리셋만
                # 반복(8/10 실측) — "주행해서 lot 도착 후 정차" 시에만 발사
                now_s = self.get_clock().now().nanoseconds * 1e-9
                if self.speed < 0.2:
                    if getattr(self, "_stopped_since_s", None) is None:
                        self._stopped_since_s = now_s
                else:
                    self._stopped_since_s = None
                # [v4.5] 넛지는 15 s+ 연속 정지에만: 셀렉터 8 m 거리판정 픽스로
                # Parking 전환은 자력으로 일어나므로, freespace 기동 중 creep
                # 정지(2~5 s)에 넛지가 발사되면 route 리셋 -> scenario EMPTY ->
                # LANE 폴백으로 주차 진행만 파괴한다 (0811 v4.5 게이트 1차:
                # none 블록 2/5 churn DEFER 실측). 진짜 전환 불발은 15 s를
                # 훌쩍 넘겨 정지하므로 복구 기능은 유지된다.
                if (self._goal_accepted and self.speed < 0.2
                        and getattr(self, "_stopped_since_s", None) is not None
                        and now_s - self._stopped_since_s > 15.0
                        and getattr(self, "_moved_since_goal", False)
                        and self._chosen is not None
                        and self._chosen.kind == "parking_lot"
                        and getattr(self, "_nudge_count", 0) < 3
                        and self._ego_in_lot_polygon()):
                    if now_s - getattr(self, "_last_nudge_s", 0.0) > 8.0:
                        self._nudge_count = getattr(self, "_nudge_count", 0) + 1
                        self._last_nudge_s = now_s
                        self._goal_sent = False
                        self._send_goal()
                        self.get_logger().warn(
                            f"parking nudge #{self._nudge_count}: goal re-sent at in-lot stop")
                if self._goal_accepted and self.speed < 0.2:
                    now = self.get_clock().now()
                    # [후방 점유 폴백] 주차 접근/기동 중 정지 + 후방 근접 차량
                    # 20s+ 지속 → 공격적 교통이 후진 공간을 내주지 않는 상황
                    # = 도로 혼잡 판단. 주차장 후보 전부 제외하고 노측으로만
                    # 재라우팅(후진 불필요), 노측이 없으면 DEFER.
                    if (self._chosen is not None
                            and self._chosen.kind == "parking_lot"
                            and not self._parking_denied
                            and getattr(self, "_moved_since_goal", False)):
                        if self._rear_blocked():
                            if self._rear_block_since is None:
                                self._rear_block_since = now
                            elif ((now - self._rear_block_since).nanoseconds
                                  > self.get_parameter("rear_block_hold_s").value * 1e9):
                                self._parking_denied = True
                                self.get_logger().error(
                                    "REAR_BLOCK: rear vehicle held the reverse "
                                    "corridor — parking denied, rerouting")
                                self._rear_block_since = None
                                self._stuck_since = None
                                self._moved_since_goal = False
                                self._goal_sent = False
                                self._goal_accepted = False
                                best = self._select_candidate(cap_loc)
                                if best is None:
                                    self._transition(
                                        "MRM_DEFER",
                                        "no viable candidate after rear-block")
                                    return
                                self._chosen = best
                                self._publish_state()
                                self.get_logger().warn(
                                    f"rear-block fallback target: {best.name}")
                                return
                        else:
                            self._rear_block_since = None
                    if not getattr(self, "_moved_since_goal", False):
                        # 골 수락 후 미출발: 보통은 engage 지연이지만, 교차로에서
                        # 리루트가 오면 behavior의 out-of-route 래치(마진 없는
                        # 폴리곤 검사 + gn 노이즈)로 영구 웨지가 됨(8/11 실측
                        # 3건: (-2,-17),(-101,33),(-123,106) 전부 교차로).
                        # 20s 미출발 시 골 재전송으로 route/모듈 재초기화(최대 3회)
                        if self._wedge_since is None:
                            self._wedge_since = now
                        elif ((now - self._wedge_since).nanoseconds > 20e9
                              and self._wedge_retries < 3):
                            self._wedge_retries += 1
                            self._wedge_since = None
                            self._goal_sent = False
                            self._goal_accepted = False
                            self.get_logger().warn(
                                f"departure wedge retry #{self._wedge_retries}: "
                                f"re-sending goal to {self._chosen.name}")
                    elif self._stuck_since is None:
                        self._stuck_since = now
                    elif (now - self._stuck_since).nanoseconds > (
                            180e9 if self._chosen.kind == "parking_lot" else 45e9):
                        # 주차장: Parking 전환(정차 5s)+costmap+A*+기동에 60~120s 소요 —
                        # 45s 도약이 기동 중 골을 뺏어 DEFER시키는 것을 실측 후 복원
                        # 주차장 후보는 도착 후 주차 시나리오 전환·freespace 기동에
                        # 60~90초가 걸린다 — 45초 타이머가 기동 중 목표를 도약시켜
                        # freespace가 격자 밖 목표를 받는 기각 루프를 만들었다(실측)
                        if (self._chosen.kind == "parking_lot"
                                and getattr(self, "_stall_retries", 0) < 2):
                            # 트래픽이 freespace 기동을 막아 생기는 일시 정체는
                            # 후보 폐기 사유가 아님(8/10 def/agg 4건 실측) — 같은
                            # 골 재전송으로 2회까지 재시도 후에만 폐기
                            self._stall_retries = getattr(self, "_stall_retries", 0) + 1
                            self._stuck_since = None
                            self._goal_sent = False
                            if self._stall_retries >= 2:
                                # 재전송만으로 안 풀리는 행동계획기 동결:
                                # 경로를 완전히 비워 모듈 상태를 리셋한 뒤 재전송
                                try:
                                    (None if self._shadow else self.clear_cli.call_async(ClearRoute.Request()))
                                    now_s = self.get_clock().now().nanoseconds * 1e-9
                                    self._reject_until = now_s + 2.0
                                    self._goal_accepted = False
                                    self.get_logger().warn(
                                        "hard reset: clear_route before re-send")
                                except Exception as e:
                                    self.get_logger().warn(f"clear_route failed: {e}")
                            self.get_logger().warn(
                                f"parking stall retry #{self._stall_retries}: "
                                f"re-sending goal to {self._chosen.name}")
                            return
                        self._blacklist.add(self._chosen.name)
                        self.get_logger().error(
                            f"stalled off-shoulder near {self._chosen.name} "
                            f"(d={d:.0f}m, lat={lat:.1f}m) — trying next candidate")
                        self._stuck_since = None
                        self._moved_since_goal = False
                        self._goal_sent = False
                        self._goal_accepted = False
                        best = self._select_candidate(cap_loc)
                        if best is None:
                            self._transition("MRM_DEFER",
                                             "no viable candidate left — stock MRM")
                            return
                        self._chosen = best
                        self._publish_state()
                        self.get_logger().warn(f"fallback target: {best.name}")
                else:
                    self._stuck_since = None
                    self._wedge_since = None
                    self._rear_block_since = None
                    if self._goal_accepted and self.speed >= 0.2:
                        self._moved_since_goal = True

        # MRM_PARKED / MRM_DEFER are terminal for this run

    # ---------------------------------------------------------- MRM planning
    def _enter_mrm_move(self, cap_loc):
        if self.pose is None:
            # keep waiting in the degraded band until localization arrives
            self.get_logger().warn("degraded, but no /localization/kinematic_state yet — waiting")
            return
        best = self._select_candidate(cap_loc)
        if best is None:
            self._select_failures += 1
            # [0822] 하드페일(센서 사망)로 인한 후보 전멸은 회복이 불가능하므로
            # 3 s 재시도 없이 즉시 위임 — 노이즈 고장은 회복 여지가 있어 재시도 유지
            _hf_dead = (self.get_parameter("hard_fail_fast").value >= 1.0
                        and self._dead_list
                        and len(self._dead_list) < max(len(self._sensor_timeout), 1))
            if _hf_dead or self._select_failures >= 30:   # ~3 s of consecutive failures
                self._transition("MRM_DEFER", "no reachable MRM candidate"
                                 + (" (hard-fail, immediate)" if _hf_dead
                                    else " (after retries)"))
            return
        self._select_failures = 0
        self._chosen = best
        self._goal_sent = False
        # [v4.9k] 저하 상태 이동은 감속 기동으로 (0 = off)
        _vcap = float(self.get_parameter("mrm_move_speed_limit").value)
        # [v4.9m 사용자 설계] 고장 비례 감속: health가 낮을수록 더 느리게.
        # health 0.7(경미)->약 25km/h, 0.3(심각)->약 11km/h. 상식 범위 클램프.
        if os.environ.get("MRM_CAP_MODE", "") == "scaled":
            _h = min(self._score("capability/perception"), cap_loc)
            _vcap = max(3.0, min(7.0, 10.0 * _h))
        if _vcap > 0.0 and self.vlimit_pub is not None and not self._shadow:
            vm = VelocityLimit()
            vm.stamp = self.get_clock().now().to_msg()
            vm.max_velocity = _vcap
            vm.use_constraints = False
            vm.sender = "mrm_supervisor"    # 셀렉터의 sender별 최소값 병합용
            self.vlimit_pub.publish(vm)
            self.get_logger().warn(f"MRM speed cap {_vcap:.1f} m/s published")
            # [sd 0827] 지속 재발행용 활성 캡 저장 — 1회 발행이 리루트/후속
            # 발행에 밀려 소실되는 실측(스모크: 캡 3.0인데 4.2 순항 구간) 대응
            self._active_vcap = _vcap
            self._last_vcap_pub = self.get_clock().now().nanoseconds * 1e-9
        self._stall_retries = 0   # 타깃별 stall 재시도 카운터 리셋
        self._wedge_since = None
        self._wedge_retries = 0   # 출발 웨지(out-of-route 래치) 재시도 리셋
        self._transition("MRM_MOVE", f"target {best.name}")

    def _tl_observable(self):
        """A traffic light ahead is observable if any camera whose FOV covers
        the forward bearing band is still healthy. Function-level availability:
        the front camera dying is fine when another camera sees the signal."""
        names = list(self.get_parameter("tl_camera_names").value)
        yaws = list(self.get_parameter("tl_camera_yaws_deg").value)
        fovs = list(self.get_parameter("tl_camera_fovs_deg").value)
        th = self.get_parameter("tl_visible_min_score").value
        bearing = self.get_parameter("tl_bearing_deg").value
        for name, yaw, fov in zip(names, yaws, fovs):
            covers = abs(yaw) - fov / 2.0 <= bearing   # FOV overlaps [-bearing, bearing]
            healthy = self._score(f"sensor/{name}") >= th
            if covers and healthy:
                return True
        return False

    def _select_candidate(self, cap_loc):
        if self.pose is None:
            return None
        src = self.map.nearest_road_lanelet(*self.pose)
        self.get_logger().info(f"selecting from pose ({self.pose[0]:.1f}, {self.pose[1]:.1f}), "
                               f"current lanelet {src}")
        # 교차로 내부 lanelet은 그래프 출구가 없어 전 후보 INF가 날 수 있음 —
        # 최근접 3개 중 라우팅 가능한 소스로 폴백 (8/11 agg 실측)
        src_candidates = self.map.nearest_road_lanelets(*self.pose, k=3)
        cap_tl = self._score("capability/traffic_light")
        w_t = self.get_parameter("w_time").value
        tl_wait = self.get_parameter("tl_wait_s").value
        w_tl = self.get_parameter("w_tl").value
        w_sec = self.get_parameter("w_sector").value
        w_loc = self.get_parameter("w_loc").value

        best, best_cost, evaluated = None, float("inf"), []
        no_shoulder = self.get_parameter("demo_no_shoulder").value
        # 검증용: MRM_FORCE_TARGET=parking_1003001 식으로 특정 후보 강제
        # (lot별 개별 검증 — 비용 순위에서 밀리는 lot도 지정 실험 가능)
        force = os.environ.get("MRM_FORCE_TARGET", "")
        for c in self.candidates:
            if force and c.name != force:
                continue
            if c.name in self._blacklist:
                continue     # goal_planner already failed to park there
            if no_shoulder and c.kind == "shoulder":
                continue     # 데모: 주차장 후보만 평가
            # [0822] 사전계산 캐시 우선 (5 s 신선) — 없으면 동기 폴백
            cached = self._route_cache.get(c.name)
            if cached is not None and time.time() - cached[0] < 5.0:
                r = cached[1]
            else:
                r = None
                for s in src_candidates:
                    r = self.map.route(s, c.target_lanelet)
                    if r is not None:
                        break
            if r is None:
                evaluated.append((c, float("inf"), "unreachable"))
                continue
            length, time_s, n_tl = r
            approach = max(length, math.dist(self.pose, (c.x, c.y)))
            if approach < self.get_parameter("min_approach_dist").value:
                evaluated.append((c, float("inf"), f"too close ({approach:.0f}m)"))
                continue
            if n_tl > 0 and not self._tl_observable():
                evaluated.append((c, float("inf"),
                                  f"{n_tl} traffic lights, no healthy camera covers them"))
                continue
            g = (1.0 / max(cap_tl, 0.05)) - 1.0
            sector_pen = sum(1.0 - self._score(f"sector/{k}")
                             for k in c.required_sectors)
            # exposure time in the degraded state, incl. expected signal waits
            eta = time_s + n_tl * tl_wait
            eta_max = self.get_parameter("eta_max_s").value
            if eta_max > 0.0 and eta > eta_max:
                evaluated.append((c, float("inf"),
                                  f"eta {eta:.0f}s > eta_max {eta_max:.0f}s"))
                continue
            cost = (w_t * eta
                    + w_tl * n_tl * g
                    + w_sec * sector_pen
                    + w_loc * eta * (1.0 - cap_loc))
            # MRC 질 서열(주차장 > 노측)을 비용에 인코딩: 노측 정차는 차로
            # 부분 점유가 남는 열등 MRC이므로 고정 페널티. 값 175 ≈ 저하 주행
            # 25초 상당 — 주차장이 그 이내 추가 비용이면 주차장을 선호.
            # (경로 실측 gap +80~+270에서 혼합 선택이 나오도록 보정, 8/10)
            # no-shoulder 모드에선 갓길이 아예 후보 제외라 결과 불변(Arm B 무영향).
            if c.kind == "shoulder":
                cost += self.get_parameter("shoulder_mrc_penalty").value
            # [예방적 후방 바이어스] 선택 시점에 이미 후방 근접 차량이 있으면
            # 후진이 필요한 주차장에 페널티 — 갓길(전진 정차)이 비용에서
            # 이기게 한다. 반응적 폴백(20 s rear-block)의 앞단 예방책.
            # 기본 0 = 무동작; MRM_REAR_BIAS env 로 켠다 (ablation 용)
            if (c.kind == "parking_lot" and self._rear_blocked()
                    and self.get_parameter("rear_bias_penalty").value > 0.0):
                cost += self.get_parameter("rear_bias_penalty").value
            # 후방 점유로 주차 거부 판정 후에는 "도로 혼잡" 판단 — 주차장
            # 후보 전부 제외, 노측으로만 재라우팅 (노측이 없으면 DEFER)
            if c.kind == "parking_lot" and self._parking_denied:
                evaluated.append((c, float("inf"), "parking denied (rear block)"))
                continue
            evaluated.append((c, cost,
                              f"len {length:.0f}m eta {eta:.0f}s tl {n_tl} sec {sector_pen:.2f}"))
            if cost < best_cost:
                best, best_cost = c, cost

        for c, cost, why in sorted(evaluated, key=lambda e: e[1]):
            self.get_logger().info(f"  candidate {c.name} [{c.kind}]: cost "
                                   f"{'INF' if math.isinf(cost) else f'{cost:.0f}'} ({why})")
        self._publish_markers(evaluated, best)
        return best

    def _ego_in_lot_polygon(self):
        """현재 타깃 주차장 폴리곤 안에 ego가 있는지 (ray casting)."""
        if self.pose is None or self._chosen is None:
            return False
        wid = self._chosen.name.rsplit("_", 1)[-1]
        pts = self.map.parking_lots.get(wid)
        if not pts:
            return False
        x, y = self.pose
        inside = False
        n = len(pts)
        for i in range(n):
            x1, y1 = pts[i]
            x2, y2 = pts[(i + 1) % n]
            if (y1 > y) != (y2 > y):
                xi = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
                if xi > x:
                    inside = not inside
        return inside

    def _send_goal(self):
        if self._shadow:
            self._goal_sent = True
            return
        if not self.route_cli.service_is_ready():
            self.get_logger().warn("route service not ready, retrying...")
            return
        req = SetRoutePoints.Request()
        req.header.frame_id = "map"
        req.option.allow_goal_modification = True
        # not present in older autoware_adapi_v1_msgs versions
        if hasattr(req.option, "allow_while_using_route"):
            req.option.allow_while_using_route = True
        req.goal = Pose(position=Point(x=self._chosen.x, y=self._chosen.y, z=0.0),
                        orientation=yaw_to_quat(self._chosen.yaw))
        cli = self.route_cli
        if ((self.route_state == RouteState.UNSET
                or getattr(self, "_use_set_route", False))
                and self.set_route_cli.service_is_ready()):
            cli = self.set_route_cli
            self.get_logger().warn("route UNSET — using set_route_points")
        cli.call_async(req).add_done_callback(self._goal_response)
        self._goal_sent = True
        self._goal_sent_time = self.get_clock().now()
        self._goal_accepted = False
        self.get_logger().warn(f"MRM goal sent: {self._chosen.name} "
                               f"({self._chosen.x:.1f}, {self._chosen.y:.1f})")

    def _goal_response(self, future):
        try:
            res = future.result()
            if not res.status.success:
                in_lot = (self._chosen is not None
                          and self._chosen.kind == "parking_lot"
                          and self._ego_in_lot_polygon())
                if in_lot:
                    # lot 폴리곤 안에서의 리루트 거부 = Parking 전환/기동 진행 중.
                    # 사유 불문 조용히 대기 — "planned route is empty"(전환 순간
                    # 레이스)를 스트릭으로 처리하면 주차 중인 lot 을 블랙리스트
                    # 하는 자멸 스파이럴(8/11 gate rep1: 0.2m 완주 주차를 DEFER 로
                    # 오판). freespace 완주는 arrive 게이트(d<=1.2m)가 잡는다.
                    self._goal_sent = True
                    self.get_logger().warn(
                        f"route change rejected in-lot ({res.status.message[:40]}) "
                        "— parking underway, standing by")
                elif "not in lane following" in res.status.message:
                    # lot 밖에서 Parking 래치(폴리곤 0.6m 이탈 정차 등) — 이
                    # 거부는 재시도해도 영원히 거부됨(8/11 rep4: 15연속→블랙
                    # 리스트→건강 붕괴 DEFER). 즉시 clear_route 로 시나리오를
                    # 리셋하고 2 s 뒤 set_route 로 재설정한다.
                    try:
                        (None if self._shadow else self.clear_cli.call_async(ClearRoute.Request()))
                        self.get_logger().warn(
                            "parking-latched reject — clear_route escalation")
                    except Exception as e:
                        self.get_logger().warn(f"clear_route failed: {e}")
                    self._goal_sent = False
                    self._goal_accepted = False
                    self._reject_until = (
                        self.get_clock().now().nanoseconds * 1e-9 + 2.0)
                else:
                    # 주행 중 거부: 옛 경로로 lot을 지나치는 함정 — 재시도하되
                    # 2s 쿨다운(스팸 방지). 거부는 '응답'이므로 unanswered 아님.
                    # lot 밖 Parking 래치(goal 지나침)면 거부가 무한 반복됨(8/11
                    # 0359 실측) — 연속 15회(~30s)면 이 타깃 폐기, 다음 후보로
                    self._reject_streak = getattr(self, "_reject_streak", 0) + 1
                    if "not set" in res.status.message:
                        # clear_route 뒤에는 change 가 영원히 거부됨 — 다음
                        # 전송부터 set_route_points 로 (메시지 구동: route_state
                        # 토픽 갱신 지연에 안 기댐, 8/11 gate rep1 실측)
                        self._use_set_route = True
                    self.get_logger().error(
                        f"route change rejected: {res.status.message} — retry in 2 s "
                        f"(streak {self._reject_streak})")
                    self._goal_sent = False
                    self._reject_until = self.get_clock().now().nanoseconds * 1e-9 + 2.0
                    if self._reject_streak >= 15 and self._chosen is not None:
                        self._blacklist.add(self._chosen.name)
                        self._reject_streak = 0
                        self.get_logger().error(
                            f"reject loop: blacklisting {self._chosen.name}, reselecting")
                        self._goal_accepted = False
                        self._moved_since_goal = False
            else:
                self._goal_accepted = True
                self._noresp = 0
                self._reject_streak = 0
                self._use_set_route = False
                self._accept_time = self.get_clock().now()
                self._accept_pose = self.pose
                self.get_logger().warn("route change ACCEPTED by Autoware")
        except Exception as e:
            self.get_logger().error(f"route change failed: {e} — retrying")
            self._goal_sent = False

    # ------------------------------------------------------------- reporting
    def _ledger(self, event, detail):
        # 케이스 로테이션과 무관한 append-only 이벤트 원장 — 실패해도 절대 죽지 않게
        try:
            import time as _time
            _sim = getattr(self, "_sim_now", None)
            _sim_s = f"{_sim:.3f}" if isinstance(_sim, float) else "-1"
            with open(self._events_path, "a") as _f:
                _f.write(f"{_time.time():.3f} sim={_sim_s} {event} {detail}\n")
        except Exception:
            pass

    def _transition(self, state, why):
        self._ledger("TRANSITION", f"{self.state}->{state} | {why}")
        # [안전망] DEFER 선언 순간 이미 어떤 주차 goal 1.2 m 안에 정지해 있으면
        # 그건 완주다 — 거부 스파이럴이 상태만 꼬은 경우(8/11 gate rep1:
        # freespace 는 0.2 m 완주, 상태기계는 타깃 갈아치우고 DEFER 오판)
        if (state == "MRM_DEFER" and self.pose is not None
                and self.speed is not None and self.speed < 0.2):
            for c in getattr(self, "candidates", []):
                if (c.kind == "parking_lot"
                        and math.dist(self.pose, (c.x, c.y)) <= 1.2):
                    self._chosen = c
                    state = "MRM_PARKED"
                    why = f"parked at {c.name} (recovered from: {why})"
                    break
        if state != self.state:
            self.get_logger().warn(f"state {self.state} -> {state}: {why}")
            self.state = state
            # v2: publish ONCE immediately at the transition (in addition to
            # the 10 Hz periodic publish in _tick) so the measured detection
            # latency is the real one, not detection + publish-cycle jitter
            self._publish_state()

    def _publish_hud(self):
        # 상태별 색 + 점수 블록. 고장 부위는 최저 점수 항목으로 표시.
        colors = {"NORMAL": (0.2, 1.0, 0.2), "MRM_MOVE": (1.0, 0.8, 0.1),
                  "MRM_PARKED": (0.2, 0.6, 1.0), "MRM_DEFER": (1.0, 0.2, 0.2)}
        r, g, b = colors.get(self.state, (1.0, 1.0, 1.0))
        # 표시 규칙(사용자 지시): 상태 + 현재 건강도는 항상, 고장 시엔
        # 문제 센서(원인)와 그 점수만 추가 — 전체 점수 나열은 안 함
        lines = [f"[{self.state}]" + (f" -> {self._chosen.name}" if self._chosen else "")]
        bad = [(k.split("/")[-1], v) for k, v in self.scores.items() if v < 0.5]
        if getattr(self, "_hud_health", None):
            p, l, h = self._hud_health
            lines.append(f"health {h:.2f} (percep {p:.2f} / loc {l:.2f})")
            if l < 0.5:
                bad.append(("localization(EKF)", l))
        for name, v in sorted(bad, key=lambda kv: kv[1])[:4]:
            lines.append(f"FAULT {name}: {v:.2f}")
        m = Marker()
        m.header.frame_id = "base_link"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "mrm_hud"; m.id = 0
        m.type = Marker.TEXT_VIEW_FACING; m.action = Marker.ADD
        m.pose.position.z = 3.5
        m.scale.z = 0.9
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.95
        m.text = "\n".join(lines)
        arr = MarkerArray(); arr.markers.append(m)
        self.hud_pub.publish(arr)
        # 좌상단 패널 반영: 우리 MRM 활성 구간에만 ADAPI mrm_state 발행
        if self.state != "NORMAL":
            ms = MrmState()
            ms.stamp = self.get_clock().now().to_msg()
            ms.state = (MrmState.MRM_SUCCEEDED if self.state == "MRM_PARKED"
                        else MrmState.MRM_OPERATING)
            ms.behavior = (MrmState.PULL_OVER if self.state in ("MRM_MOVE", "MRM_PARKED")
                           else MrmState.COMFORTABLE_STOP)
            self.adapi_mrm_pub.publish(ms)

    def _publish_state(self):
        detail = f":{self._chosen.name}" if self._chosen else ""
        self.state_pub.publish(String(data=self.state + detail))

    def _publish_markers(self, evaluated, best):
        arr = MarkerArray()
        for i, (c, cost, _) in enumerate(evaluated):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "mrm_candidates"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = Point(x=c.x, y=c.y, z=1.0)
            m.pose.orientation = yaw_to_quat(c.yaw)
            m.scale.x = m.scale.y = m.scale.z = 2.5
            if c is best:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 0.9
            elif math.isinf(cost):
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 0.6
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.6, 0.0, 0.7
            arr.markers.append(m)

            t = Marker()
            t.header = m.header
            t.ns = "mrm_costs"
            t.id = i
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position = Point(x=c.x, y=c.y, z=3.5)
            t.scale.z = 1.5
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = f"{c.name}\n{'INF' if math.isinf(cost) else f'{cost:.0f}'}"
            arr.markers.append(t)
        self.marker_pub.publish(arr)


def main():
    rclpy.init()
    node = Supervisor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
