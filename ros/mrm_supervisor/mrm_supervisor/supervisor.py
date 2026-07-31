"""점수 기반 MRM(최소 위험 기동, Minimum Risk Maneuver) 감독 노드.

[이 파일이 하는 일 한 줄 요약]
  cdr_monitor가 계산해 주는 "모니터 점수"(/monitor/scores)를 받아서
  차량의 건강도(health)를 계산하고, 그 값에 따라
  NORMAL(정상 주행) / MRM_MOVE(갓길로 이동해서 정차) /
  MRM_DEFER(Autoware 기본 정지 기능에 위임) 중 무엇을 할지 결정하는 "뇌"다.

[상태기계(state machine) 다이어그램]
                 health <= 0.3 이 5틱 연속 (바닥)
   NORMAL ─────────────────────────────────────────────► MRM_DEFER (종료 상태)
     │ 0.3 < health < 0.7 이 10틱 지속(성능 저하 밴드)        ▲
     ▼                                                      │ 이동 중 붕괴 /
   MRM_MOVE ──(갓길 후보로 경로 변경, 이동)──► MRM_PARKED     │ 후보 소진 /
     └──────────────────────────────────────────────────────┘ 위치추정 불량

  - MRM_PARKED, MRM_DEFER 는 "종료 상태": 한 번 들어가면 이번 주행에서는
    다시 나오지 않는다.
  - MRM_DEFER 는 "우리가 직접 움직이는 건 위험하니, Autoware가 원래 갖고 있는
    차선 내 정지(comfortable/emergency stop)에게 맡긴다"는 뜻이다.

[비용함수 개요] — 갓길/주차장 후보 중 어디로 갈지 고르는 수식
  eta(c)  = 경로 주행 시간(제한속도 기준) + 신호등 개수 * 신호 대기 시간
            → "고장난 상태로 도로에 노출되는 시간". 위험은 거리(미터)가 아니라
              시간(초)마다 쌓인다는 ISO 26262의 time-to-safe-state 관점.
  cost(c) = w_time * eta                          (오래 걸릴수록 나쁨)
          + w_tl * 신호등수 * g(신호등 인식 점수)   (카메라가 죽었으면 무한대)
          + w_sec * Σ(1 - 섹터 점수)               (그 방향 감지가 죽었으면 나쁨)
          + w_loc * eta * (1 - 위치추정 점수)      (위치가 불안하면 오래 못 감)
  각 항의 자세한 설명은 _select_candidate() 안의 주석 참고.

[ROS2 기초 용어 미리보기] (아래 코드에서 처음 나올 때마다 다시 설명함)
  - 노드(Node): ROS 세계에서 실행되는 하나의 프로그램. 이 파일이 노드 하나다.
  - 토픽(topic): 노드들이 데이터를 주고받는 "방송 채널" 이름 (예: /monitor/scores).
  - 구독(subscribe): 그 방송 채널을 틀어놓고 메시지가 올 때마다 콜백 함수 실행.
  - 발행(publish): 그 방송 채널로 메시지를 내보내기.
  - 파라미터(parameter): 노드 실행 시 밖에서 바꿔 넣을 수 있는 설정값.
"""
import math

import rclpy                      # ROS2 파이썬 클라이언트 라이브러리 (ROS Client Library for PYthon)
from rclpy.node import Node       # 모든 ROS2 노드가 상속받는 기본 클래스
from rclpy.qos import QoSProfile, DurabilityPolicy  # QoS: 메시지 전달 품질 설정 (아래에서 설명)

# --- 메시지 타입들: 토픽으로 주고받는 데이터의 "형식(구조체)" 정의 ---
from std_msgs.msg import String                       # 단순 문자열 메시지
from nav_msgs.msg import Odometry                     # 위치 + 속도 (주행 기록계)
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped  # 자세/좌표
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
#   DiagnosticArray = Autoware의 "진단 게시판"에 올리는 쪽지 묶음.
#   각 쪽지(DiagnosticStatus)에는 이름, 레벨(OK/WARN/ERROR), key-value 목록이 들어간다.
from visualization_msgs.msg import Marker, MarkerArray  # RViz 화면에 그리는 도형(구, 글자 등)
from autoware_adapi_v1_msgs.msg import RouteState       # Autoware가 알려주는 경로 상태(도착 여부 등)
from autoware_adapi_v1_msgs.srv import SetRoutePoints   # 서비스: "경로를 이 지점으로 바꿔줘" 요청 형식

from mrm_supervisor.map_model import MapModel  # 같은 패키지의 지도 처리 모듈 (map_model.py)


def yaw_to_quat(yaw):
    """yaw(진행 방향 각도, 라디안)를 쿼터니언으로 변환.

    ROS에서 회전은 항상 쿼터니언(x,y,z,w 4개 숫자)으로 표현한다.
    평면 주행에서는 z축 회전(yaw)만 있으므로 공식이 이렇게 단순해진다:
    z = sin(yaw/2), w = cos(yaw/2), x = y = 0.
    """
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


class Supervisor(Node):
    """MRM 감독 노드 본체. rclpy.node.Node를 상속해서 ROS2 노드가 된다."""

    def __init__(self):
        # 부모 클래스(Node) 초기화. "mrm_supervisor"가 ROS 세계에서 이 노드의 이름.
        super().__init__("mrm_supervisor")

        # ------------------------------------------------------------ 파라미터
        # declare_parameter(이름, 기본값): 이 노드가 받을 설정값을 등록한다.
        # 런치 파일이나 명령행에서 값을 바꿔 넣을 수 있고, 안 넣으면 기본값 사용.
        # 나중에 self.get_parameter("이름").value 로 꺼내 쓴다.
        self.declare_parameter("map_path", "/home/taewook/awcl_maps/Town07/lanelet2_map.osm")
        # baseline 모드: 발동 조건은 완전히 동일하지만, 갓길/주차장으로 이동하는
        # 대신 곧바로 Autoware 기본 mrm_handler(차선 내 정지)에 넘긴다.
        # 논문 실험에서 "우리 방법 vs 기본 동작" 비교군(대조군)으로 쓰는 스위치.
        self.declare_parameter("baseline", False)
        # 건강도 밴드 경계값 두 개:
        #   health >= 0.7  → 정상 (아직 괜찮음)
        #   0.3 < health < 0.7 → "성능 저하 밴드": 아직 스스로 움직일 수 있으니
        #                        갓길로 이동해서 정차 (MRM_MOVE)
        #   health <= 0.3  → "바닥(floor)": 움직이는 것 자체가 위험 → MRM_DEFER
        # 즉 0.7/0.3 두 단계로 나눈 이유: 고장에도 "아직 운전 가능한 수준"과
        # "당장 멈춰야 하는 수준"이 있고, 대응이 달라야 하기 때문.
        self.declare_parameter("upper_threshold", 0.7)
        self.declare_parameter("lower_threshold", 0.3)
        # 위치추정 점수가 이 값보다 낮으면 갓길 이동 자체를 포기하고 DEFER.
        # 내가 어디 있는지 모르면서 갓길을 찾아가는 것은 더 위험하기 때문.
        self.declare_parameter("min_loc_to_move", 0.5)
        # 성능 저하 밴드에 몇 틱(0.1초 단위) 연속으로 머물러야 MRM을 발동할지.
        # 10틱 = 1초. 순간적인 점수 출렁임 한 번으로 바로 발동하지 않기 위한 버퍼.
        self.declare_parameter("hold_ticks", 10)          # 10 Hz 기준 1초
        # ---- 비용함수 가중치들 (자세한 의미는 _select_candidate 참고) ----
        self.declare_parameter("w_time", 7.0)     # 고장 상태로 주행하는 "1초당" 비용
        self.declare_parameter("tl_wait_s", 15.0) # 신호등 하나당 예상 대기 시간(초)
        self.declare_parameter("w_tl", 200.0)     # 신호등 통과 자체에 대한 벌점 가중치
        self.declare_parameter("w_sector", 500.0) # 필요한 감지 섹터가 죽어있을 때 벌점
        self.declare_parameter("w_loc", 14.0)     # 위치추정 불안 시 초당 추가 비용
        # 도착 판정 반경(미터): 목표 지점에서 이 거리 안이면 "도착"으로 본다.
        self.declare_parameter("arrive_dist", 4.0)
        # 갓길 정차(pull-over)는 감속 + 옆으로 붙이는 거리(접근 구간)가 필요하다.
        # 이보다 가까운 후보는 "싸게 갈 수 있는 곳"이 아니라 "물리적으로 불가능한
        # 곳"이므로 후보에서 제외한다.
        self.declare_parameter("min_approach_dist", 25.0)
        # 후보 종류별로 "반드시 살아있어야 하는 감지 섹터" 목록.
        # 섹터 = 차 주변을 방향별로 나눈 감지 구역 (F=앞, FR=앞오른쪽, B=뒤, ...).
        # 갓길: 앞으로 가서 오른쪽으로 붙이므로 F, FR만 필요.
        # 주차장: 후진/회전이 섞이므로 거의 전 방향 필요.
        self.declare_parameter("shoulder_sectors", ["F", "FR"])
        self.declare_parameter("parking_sectors", ["F", "FL", "FR", "B", "RL", "RR"])
        # 신호등 관측 가능성 설정: 앞쪽 신호등을 볼 수 있는 카메라들의
        # (이름, 장착 방향 yaw, 시야각 FOV). 이 중 "하나라도" 살아있으면
        # 신호등이 있는 경로를 허용한다. (기능 수준의 가용성: 전방 카메라가
        # 죽어도 다른 카메라가 신호를 볼 수 있으면 OK)
        self.declare_parameter("tl_camera_names", ["CAMERA_front"])
        self.declare_parameter("tl_camera_yaws_deg", [0.0])
        self.declare_parameter("tl_camera_fovs_deg", [90.0])
        self.declare_parameter("tl_visible_min_score", 0.5)
        self.declare_parameter("tl_bearing_deg", 30.0)  # 신호등은 전방 ±30도 안에 있다고 가정

        # ------------------------------------------------------------ 지도 로드
        map_path = self.get_parameter("map_path").value
        self.map = MapModel(map_path)   # lanelet2 지도 파싱 (map_model.py)
        # 지도에서 갓길/주차장 후보를 미리 전부 뽑아 둔다 (주행 중에는 고르기만 함).
        self.candidates = self.map.extract_candidates(
            list(self.get_parameter("shoulder_sectors").value),
            list(self.get_parameter("parking_sectors").value))
        self.get_logger().info(
            f"map loaded: {len(self.map.lanelets)} lanelets, "
            f"{len(self.candidates)} MRM candidates "
            f"({sum(1 for c in self.candidates if c.kind == 'shoulder')} shoulder, "
            f"{sum(1 for c in self.candidates if c.kind == 'parking_lot')} parking)")

        # ------------------------------------------------------ 내부 상태 변수들
        self.scores = {}           # /monitor/scores 에서 받은 점수들: 이름 -> 점수(0~1)
        self.pose = None           # 현재 위치 (x, y). 아직 못 받았으면 None
        self.speed = 0.0           # 현재 속력 [m/s]
        self.loc_jump_score = 1.0  # 위치추정 "점프" 점수 (EKF 출력이 갑자기 튀는지)
        self.loc_meas_score = 1.0  # 위치추정 "측정" 점수 (GNSS 원시 측정 vs EKF 출력 차이)
        self._meas_base = 0.0      # 부팅 시 학습하는 GNSS-EKF 좌표계 고정 오프셋 (아래 설명)
        self._meas_n = 0           # 위 오프셋 캘리브레이션에 쓴 샘플 개수
        self._last_odom = None     # 직전 odometry (x, y, 시각, 속력) — 점프 계산용

        self.state = "NORMAL"      # 상태기계의 현재 상태
        # armed(무장) 플래그: "진짜 점수가 들어오고 시스템이 건강하다"를 한 번
        # 확인한 뒤에만 MRM 판단을 시작한다. 부팅 직후(시뮬레이터가 센서를
        # 아직 만드는 중)에는 점수가 요동치는데, 그걸 고장으로 오인해서
        # MRM을 오발동하면 안 되기 때문.
        self._armed = False
        self._armed_ticks = 0      # 건강한 상태가 연속 몇 틱 지속됐는지 (30틱 = 3초 필요)
        self._band_ticks = 0       # 성능 저하 밴드에 연속 몇 틱 머물렀는지
        self._floor_ticks = 0      # 바닥(lower) 이하에 연속 몇 틱 머물렀는지
        self._chosen = None        # 선택된 갓길/주차장 후보 (Candidate 객체)
        self._goal_sent = False    # 경로 변경 요청을 보냈는지
        self._goal_sent_time = None
        self._goal_accepted = False  # Autoware가 경로 변경을 수락했는지
        self._noresp = 0           # 경로 변경 요청이 무응답으로 끝난 횟수
        self._select_failures = 0  # 후보 선택 실패(도달 가능한 후보 없음)가 연속된 횟수
        self._blacklist = set()    # goal_planner가 주차에 실패한 후보 이름들 (재시도 금지)
        self._stuck_since = None   # 정체(멈춤)가 시작된 시각
        self._accept_time = None   # 경로 변경이 수락된 시각 (크립 가드용)
        self._accept_pose = None   # 수락 시점의 위치 (크립 가드용)

        # ------------------------------------------------------- 구독/발행 설정
        # QoS(Quality of Service): 메시지를 "어떻게" 전달할지에 대한 옵션.
        #   depth=1: 최신 메시지 1개만 큐에 보관 (오래된 건 버림)
        #   VOLATILE: 구독을 시작한 "이후"의 메시지만 받음 (일반적인 실시간 방송)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE)
        # create_subscription = "이 방송 채널을 구독하고, 메시지가 올 때마다
        # 이 콜백 함수를 불러줘". 인자: (메시지타입, 토픽이름, 콜백, QoS)
        self.create_subscription(DiagnosticArray, "/monitor/scores", self._scores_cb, qos)
        self.create_subscription(Odometry, "/localization/kinematic_state", self._odom_cb, qos)
        # Autoware 자체의 "도착" 신호도 같이 본다. 이유: Autoware의 goal_planner는
        # 우리가 준 후보 중점에서 수십 미터 떨어진 곳으로 정차 위치를 스스로
        # 다듬을 수 있어서(실측 19 m 오프셋), 순수한 거리 기준(arrive_dist)만으로는
        # 도착 판정이 영영 안 떨어질 수 있다.
        self.route_state = RouteState.UNKNOWN
        self.create_subscription(
            RouteState, "/api/routing/state", self._route_state_cb,
            # TRANSIENT_LOCAL: "래치(latch)" 방송. 늦게 구독해도 마지막 메시지를
            # 한 번 받아볼 수 있다. 상태처럼 가끔 바뀌는 값에 쓴다.
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # GNSS/NDT 측정 품질 감시: EKF(칼만 필터)는 노이즈 낀 위치 입력을
        # 부드럽게 만들어 주기 때문에, EKF의 "출력"만 보면 깨끗해 보인다
        # (그래서 loc_jump 점수는 GNSS 노이즈 고장을 아예 못 잡는다).
        # 노이즈가 실제로 보이는 곳은 "입력(원시 측정) vs 출력(EKF)"의 차이(잔차)다.
        self.create_subscription(
            PoseWithCovarianceStamped, "/sensing/gnss/pose_with_covariance",
            self._meas_pose_cb, qos)

        # create_publisher = "이 방송 채널로 송출할 마이크를 만든다".
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, "/mrm_supervisor/state", latched)
        self.marker_pub = self.create_publisher(MarkerArray, "/mrm_supervisor/candidates", qos)
        # create_client = 서비스(요청-응답 방식 통신)를 부르는 전화기.
        # 토픽이 일방향 방송이라면, 서비스는 "요청 보내고 답을 기다리는" 전화다.
        self.route_cli = self.create_client(SetRoutePoints, "/api/routing/change_route_points")
        # DEFER를 Autoware 기본 MRM에 넘기는 통로: Autoware의 진단 그래프
        # (diagnostic_graph_aggregator)가 "mrm_supervisor: defer_request"라는
        # 진단 항목을 /autoware/modes/autonomous 에 연결해 두었다.
        # 우리가 이 항목을 ERROR로 올리면 → 자율주행 모드가 "불가능" 판정
        # → Autoware의 mrm_handler가 차선 내 정지(comfortable/emergency stop)를
        # 수행한다. 인지 점수만으로는 못 잡는 DEFER 원인(신호등 게이트,
        # 위치추정 불량)까지 이 경로 하나로 처리된다.
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)

        # create_timer(주기, 콜백) = 일정 주기 알람. 0.1초마다 _tick을 실행
        # → 이 노드의 "심장 박동". 상태기계 판단은 전부 _tick 안에서 일어난다.
        self.create_timer(0.1, self._tick)
        self._publish_state()

    # ------------------------------------------------------------- 콜백들
    # (콜백 = 구독 중인 토픽에 메시지가 도착할 때마다 ROS가 대신 불러주는 함수)

    def _scores_cb(self, msg):
        """/monitor/scores 수신: 진단 메시지에서 'score' 값만 뽑아 저장.

        DiagnosticArray 안에는 여러 status(쪽지)가 있고, 각 쪽지에는
        key-value 목록이 있다. 그중 key가 "score"인 값을 이름별로 저장한다.
        예: scores["capability/perception"] = 0.85
        """
        for st in msg.status:
            for kv in st.values:
                if kv.key == "score":
                    try:
                        self.scores[st.name] = float(kv.value)
                    except ValueError:
                        pass  # 숫자가 아닌 값은 조용히 무시

    def _route_state_cb(self, msg):
        """Autoware의 경로 상태(SET/ARRIVED 등)를 그대로 기억해 둔다."""
        self.route_state = msg.state

    def _meas_pose_cb(self, msg):
        """GNSS 원시 위치 수신: EKF 출력과의 차이(잔차)로 측정 품질을 점수화.

        [왜 부팅 캘리브레이션이 필요한가]
        GNSS 측정점과 EKF 출력점 사이에는 좌표 기준 차이(차량 중심 vs base_link,
        약 1.4 m)로 인한 "고정 오프셋"이 항상 존재한다. 이건 고장이 아니다.
        그래서 부팅 직후 약 3초 동안 이 고정 오프셋을 학습해서(평균 내서)
        기억해 두고, 그 이후에는 "기준선을 넘는 초과분"만 고장 신호로 본다.
        (고장 주입은 주행 시작 후에만 일어나므로 부팅 구간은 안전한 학습 구간)
        """
        if self.pose is None:
            return  # 아직 EKF 위치를 못 받았으면 비교 자체가 불가능
        # r = 원시 측정 위치와 EKF 위치 사이 거리 (잔차)
        r = math.dist(self.pose, (msg.pose.pose.position.x, msg.pose.pose.position.y))
        if self._meas_n < 60:                     # 처음 60샘플(~3초)은 캘리브레이션
            # 지수이동평균(EMA): 새 값 10%, 기존 값 90%를 섞어 부드럽게 평균.
            self._meas_base = r if self._meas_n == 0 else \
                0.9 * self._meas_base + 0.1 * r
            self._meas_n += 1
            return
        # 기준선 + 여유 0.2 m를 넘는 "초과분"만 잔차로 취급
        excess = max(0.0, r - self._meas_base - 0.2)
        # 초과분 → 점수(0~1) 변환: exp(-x/0.55)는 x=0에서 1, 커질수록 0으로.
        # 튜닝 근거(실험값):
        #   gn 중간 고장(sigma 1 m, 초과 ~0.3) → 점수 ~0.58: 성능 저하 밴드지만
        #     min_loc_to_move(0.5)보다는 높음 → 갓길 정차 가능
        #   gn 심각 고장(sigma 2.5, 초과 ~1.6) → 점수 ~0.05: 바닥 → DEFER
        inst = math.exp(-excess / 0.55)
        # 여기도 EMA로 부드럽게: 한 샘플의 튐이 점수를 확 흔들지 않게.
        self.loc_meas_score = 0.9 * self.loc_meas_score + 0.1 * inst

    def _odom_cb(self, msg):
        """위치/속도 수신 + 위치추정 '점프' 감지.

        아이디어(이노베이션 검사): 속도를 알면 "다음 순간 위치"를 예측할 수
        있다. 실제 이동 거리가 그 예측과 크게 다르면 = 위치추정이 순간이동
        (점프)했다는 뜻 → 위치추정 이상 신호.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x   # twist = 속도 (선속도 + 각속도)
        vy = msg.twist.twist.linear.y
        self.speed = math.hypot(vx, vy)  # 속력 = sqrt(vx^2 + vy^2)
        # 메시지에 찍힌 시각을 초 단위 실수로 변환
        now = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds / 1e9
        if self._last_odom is not None:
            px, py, pt, pv = self._last_odom
            dt = now - pt
            if 0.0 < dt < 1.0:  # 시간이 역행하거나 너무 오래 끊겼으면 스킵
                # 점프량 = |실제 이동 거리 - 속도로 예측한 이동 거리|
                jump = abs(math.dist((x, y), (px, py)) - pv * dt)
                # 0.3 m까지는 정상 오차로 봐주고, 초과분을 점수로 변환
                inst = math.exp(-max(0.0, jump - 0.3) / 0.5)
                self.loc_jump_score = 0.9 * self.loc_jump_score + 0.1 * inst
        self._last_odom = (x, y, now, self.speed)
        self.pose = (x, y)

    def _publish_defer_diag(self):
        """진단 게시판(/diagnostics)에 우리 상태 쪽지를 올린다.

        state가 MRM_DEFER면 레벨을 ERROR로 → Autoware 진단 그래프를 통해
        자율주행 모드가 막히고 기본 MRM(차선 내 정지)이 발동된다.
        그 외 상태에서는 OK를 계속 올려 "우리는 살아있고 정상"임을 알린다.
        """
        st = DiagnosticStatus()
        st.name = "mrm_supervisor: defer_request"
        st.hardware_id = "mrm_supervisor"
        st.level = (DiagnosticStatus.ERROR if self.state == "MRM_DEFER"
                    else DiagnosticStatus.OK)
        st.message = self.state
        st.values.append(KeyValue(key="state", value=self.state))
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status.append(st)
        self.diag_pub.publish(arr)

    # ------------------------------------------------------------------ 틱
    def _tick(self):
        """0.1초마다 실행되는 메인 루프. 상태기계 판단은 전부 여기서."""
        self._publish_defer_diag()
        # 점수가 아직 안 들어왔으면 1.0(건강)으로 간주 — dict.get(키, 기본값)
        perception = self.scores.get("capability/perception", 1.0)
        # 위치추정 건강도 = 세 신호 중 최악:
        #   cdr_monitor의 localization 점수, 점프 점수, 측정 잔차 점수
        cap_loc = min(self.scores.get("capability/localization", 1.0),
                      self.loc_jump_score, self.loc_meas_score)

        upper = self.get_parameter("upper_threshold").value
        lower = self.get_parameter("lower_threshold").value

        # [핵심 설계] health = 여러 능력 중 "최악"의 값 (min).
        # 왜 min인가: 인지(라이다/카메라)와 위치추정(IMU/GNSS→EKF 흔들림) 중
        # 하나라도 죽으면 차량 전체가 위험하다. 사슬은 가장 약한 고리에서
        # 끊어진다(약한 고리 법칙). 평균을 쓰면 한쪽이 멀쩡할 때 고장이 가려진다.
        health = min(perception, cap_loc)

        if not self._armed:
            # [armed 게이트] 건강한 상태가 30틱(=3초) "지속"돼야 무장한다.
            # 왜 한 번의 좋은 점수로는 부족한가: 부팅 중 순간적으로 0.9가 찍혀서
            # 무장했다가, 곧바로 이어지는 부팅 딥(점수 하락)에 걸려 주행 시작
            # 몇 초 만에 가짜 MRM을 발동한 실제 사례가 있었다.
            if "capability/perception" in self.scores and perception >= upper:
                self._armed_ticks += 1
                if self._armed_ticks >= 30:
                    self._armed = True
                    self.get_logger().info(f"armed: perception {perception:.2f} healthy for 3 s")
            else:
                self._armed_ticks = 0  # 중간에 나쁜 틱이 끼면 처음부터 다시 센다
            return  # 무장 전에는 아무 판단도 하지 않음

        # [floor 5틱 규칙] MRM_DEFER는 되돌릴 수 없는 종료 상태이므로,
        # 나쁜 틱 "한 번"으로 발동하면 안 된다. 일시적인 점수 스파이크
        # (노이즈 한 방)에 즉시 포기하지 않도록 5틱(0.5초) 연속을 요구한다.
        self._floor_ticks = self._floor_ticks + 1 if health <= lower else 0

        if self.state == "NORMAL":
            if self._floor_ticks >= 5:
                # 바닥을 0.5초 연속으로 찍음 → 움직이는 것 자체가 위험 → 위임
                self._transition("MRM_DEFER", f"health {health:.2f} below floor "
                                 f"(perception {perception:.2f}, loc {cap_loc:.2f})")
            elif health < upper:
                # 성능 저하 밴드 (0.3 < health < 0.7): 카운트 누적
                self._band_ticks += 1
                if self._band_ticks == 1 or self._band_ticks % 10 == 0:
                    self.get_logger().info(
                        f"degraded band: health {health:.2f} (p {perception:.2f}/l {cap_loc:.2f}) "
                        f"({self._band_ticks}/{self.get_parameter('hold_ticks').value} ticks)")
                if self._band_ticks >= self.get_parameter("hold_ticks").value:
                    # 1초간 지속 확인 완료 → 어떤 MRM으로 갈지 결정
                    if self.get_parameter("baseline").value:
                        # 비교 실험용: 무조건 기본 MRM에 위임
                        self._transition("MRM_DEFER",
                                         f"baseline: health {health:.2f}, stock MRM")
                    elif cap_loc < self.get_parameter("min_loc_to_move").value:
                        # 위치추정이 너무 나쁘면 갓길까지 "운전해 가는" 것이
                        # 더 위험 → 이동 포기, 그 자리 정지에 위임
                        self._transition("MRM_DEFER",
                                         f"degraded but localization {cap_loc:.2f} too low to move")
                    else:
                        self._enter_mrm_move(cap_loc)
            else:
                # 건강 회복 시 카운터를 0으로 "리셋"하지 않고 1씩만 "감쇠"시킨다.
                # 이유: 점수가 문턱값 근처에서 진동(0.69↔0.71)해도 위험은
                # 계속 쌓이고 있는 것이므로, 누적이 유지되게 하기 위함.
                self._band_ticks = max(0, self._band_ticks - 1)

        elif self.state == "MRM_MOVE":
            if self._floor_ticks >= 5:
                # 갓길로 가는 도중에 건강도가 바닥으로 붕괴 → 즉시 위임
                self._transition("MRM_DEFER", f"health {health:.2f} collapsed en route")
                return
            # [경로 변경 감시견(watchdog)] Autoware가 자기 목표 근처에 있으면
            # goal_planner가 주도권을 잡아서 경로 변경 서비스가 "무응답"이 될
            # 수 있다 — 그러면 차는 아무 일 없었다는 듯 "옛" 경로를 조용히
            # 끝까지 달린다. 5초 무응답이면 재시도, 3번 실패하면 기본 MRM에 위임.
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
                self._send_goal()
            elif self._chosen and self.pose:
                d = math.dist(self.pose, (self._chosen.x, self._chosen.y))
                # [도착 판정: lon/lat 분해] 목표까지의 오차를 경로 방향(세로,
                # lon)과 옆 방향(가로, lat)으로 나눠서 본다.
                # 왜: 단순 반경(원) 검사만 쓰면, 갓길 옆 "차선 위"에서 3.5 m
                # 비껴 멈춘 것도 "주차 완료"로 오판한 사례가 있었다.
                # 세로로는 arrive_dist(4 m)까지 봐주되, 가로(옆으로 벗어난
                # 정도)는 1.5 m 이내여야 진짜 갓길 위에 있는 것.
                # 아래는 목표 yaw 기준 좌표계로의 2D 회전 변환이다.
                dx = self.pose[0] - self._chosen.x
                dy = self.pose[1] - self._chosen.y
                cy, sy = math.cos(self._chosen.yaw), math.sin(self._chosen.yaw)
                lon = dx * cy + dy * sy    # 경로 방향 성분 (앞뒤 오차)
                lat = -dx * sy + dy * cy   # 옆 방향 성분 (좌우 오차)
                on_shoulder = (abs(lon) < self.get_parameter("arrive_dist").value
                               and abs(lat) < 1.5)
                # Autoware 스스로의 ARRIVED 신호도 도착으로 인정 (위 구독 주석 참고)
                arrived = on_shoulder or self.route_state == RouteState.ARRIVED
                if arrived and self.speed < 0.2:   # 도착 + 사실상 정지 상태
                    self._transition("MRM_PARKED",
                                     f"stopped at {self._chosen.name} "
                                     f"(d={d:.1f}m, lat={lat:.1f}m, "
                                     f"route_state={self.route_state})")
                    return
                # [크립(creep) 가드: 90초 / 25 m] 라이다 노이즈가 만든 유령
                # 장애물 때문에 차가 0.2~1 m/s로 "기어가기만" 하는 상태가
                # 영원히 지속될 수 있다 — 완전 정지가 아니라서 아래의 정지
                # 감지(45초 규칙)에는 안 걸린다. 경로 수락 후 90초가 지났는데
                # 이동 거리가 25 m도 안 되면, 도로 위 노출 시간이 갓길 정차의
                # 이득을 넘어선 것: 이 후보는 블랙리스트에 넣고 다음 후보로.
                # (후보가 다 떨어지면 DEFER)
                if (self._goal_accepted and self._accept_time is not None
                        and self._accept_pose is not None
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
                # [정체 45초 → 블랙리스트] goal_planner가 "만들 수 있는 정차
                # 경로 후보가 0개"(갓길이 너무 좁거나 이미 지나침)라며 그냥
                # 멈춰버릴 수 있다: 갓길에 닿지도 못한 채 도로에서 정지.
                # 45초 동안 그런 상태면 그 갓길 후보는 포기하고(블랙리스트)
                # 다음 후보로 넘어간다.
                # 왜 45초인가: Town07의 빨간불 한 사이클보다 길게 잡아서,
                # 신호 대기 중인 정상 정지를 정체로 오판하지 않기 위해.
                if self._goal_accepted and self.speed < 0.2:
                    now = self.get_clock().now()
                    if self._stuck_since is None:
                        self._stuck_since = now   # 정체 시작 시각 기록
                    elif (now - self._stuck_since).nanoseconds > 45e9:
                        self._blacklist.add(self._chosen.name)
                        self.get_logger().error(
                            f"stalled 45 s off-shoulder near {self._chosen.name} "
                            f"(d={d:.0f}m, lat={lat:.1f}m) — trying next candidate")
                        self._stuck_since = None
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
                    self._stuck_since = None  # 다시 움직이면 정체 타이머 리셋

        # MRM_PARKED / MRM_DEFER 는 종료 상태: 이번 주행에서는 여기서 끝.

    # ---------------------------------------------------------- MRM 계획
    def _enter_mrm_move(self, cap_loc):
        """성능 저하 확정 → 갓길/주차장 후보를 골라 MRM_MOVE로 전환."""
        if self.pose is None:
            # 내 위치를 모르면 후보를 고를 수 없다. 성능 저하 밴드에 머물며
            # 위치추정 토픽이 들어올 때까지 기다린다.
            self.get_logger().warn("degraded, but no /localization/kinematic_state yet — waiting")
            return
        best = self._select_candidate(cap_loc)
        if best is None:
            # 지금 당장 도달 가능한 후보가 없을 수도 있다 (예: 라우팅 그래프가
            # 잠깐 이상). 매 틱 재시도하고, 30틱(~3초) 연속 실패면 포기 → DEFER.
            self._select_failures += 1
            if self._select_failures >= 30:
                self._transition("MRM_DEFER", "no reachable MRM candidate (after retries)")
            return
        self._select_failures = 0
        self._chosen = best
        self._goal_sent = False
        self._transition("MRM_MOVE", f"target {best.name}")

    def _tl_observable(self):
        """전방 신호등을 '지금 볼 수 있는' 카메라가 하나라도 있는지 검사.

        [왜 이 검사가 필요한가] 카메라가 죽으면 신호를 못 읽는다.
        빨간불을 못 보고 교차로에 진입하는 것은 최악의 시나리오이므로,
        신호등이 낀 경로는 이 검사를 통과 못 하면 비용 무한대(INF)로
        게이트(차단)된다 (_select_candidate 참고).

        판정: 카메라의 시야각(FOV)이 전방 신호등 방위 범위 [-30°, +30°]와
        겹치고(covers), 그 카메라의 센서 점수가 기준 이상(healthy)이면 OK.
        전방 카메라가 죽어도 다른 카메라가 신호를 볼 수 있으면 통과 —
        "부품 단위"가 아니라 "기능 단위"의 가용성 판단.
        """
        names = list(self.get_parameter("tl_camera_names").value)
        yaws = list(self.get_parameter("tl_camera_yaws_deg").value)
        fovs = list(self.get_parameter("tl_camera_fovs_deg").value)
        th = self.get_parameter("tl_visible_min_score").value
        bearing = self.get_parameter("tl_bearing_deg").value
        for name, yaw, fov in zip(names, yaws, fovs):
            covers = abs(yaw) - fov / 2.0 <= bearing   # FOV가 [-bearing, bearing]과 겹침
            healthy = self.scores.get(f"sensor/{name}", 1.0) >= th
            if covers and healthy:
                return True
        return False

    def _select_candidate(self, cap_loc):
        """모든 후보에 대해 비용을 계산하고 가장 싼(안전한) 후보를 반환.

        비용함수 (파일 맨 위 docstring의 수식과 같다):
          cost = w_time * eta                      … (1) 노출 시간 항
               + w_tl * n_tl * g(cap_tl)           … (2) 신호등 항
               + w_sec * Σ(1 - sector_score)       … (3) 섹터 항
               + w_loc * eta * (1 - cap_loc)       … (4) 위치추정 항
        각 항의 의미는 아래 계산 지점의 주석 참고.
        """
        if self.pose is None:
            return None
        # 내가 지금 서 있는 도로 lanelet을 찾는다 (경로 탐색의 출발점)
        src = self.map.nearest_road_lanelet(*self.pose)
        self.get_logger().info(f"selecting from pose ({self.pose[0]:.1f}, {self.pose[1]:.1f}), "
                               f"current lanelet {src}")
        cap_tl = self.scores.get("capability/traffic_light", 1.0)
        w_t = self.get_parameter("w_time").value
        tl_wait = self.get_parameter("tl_wait_s").value
        w_tl = self.get_parameter("w_tl").value
        w_sec = self.get_parameter("w_sector").value
        w_loc = self.get_parameter("w_loc").value

        best, best_cost, evaluated = None, float("inf"), []
        for c in self.candidates:
            if c.name in self._blacklist:
                continue     # goal_planner가 이미 주차 실패한 곳 — 다시 안 감
            # 지도 라우팅: (거리 m, 시간 s, 경로상 신호등 개수) 또는 None(도달 불가)
            r = self.map.route(src, c.target_lanelet)
            if r is None:
                evaluated.append((c, float("inf"), "unreachable"))
                continue
            length, time_s, n_tl = r
            # 접근 거리 = 경로 길이와 직선 거리 중 큰 쪽 (보수적으로)
            approach = max(length, math.dist(self.pose, (c.x, c.y)))
            if approach < self.get_parameter("min_approach_dist").value:
                # 너무 가까움 = 감속+차선 이동을 할 공간이 없음 → 불가능 판정
                evaluated.append((c, float("inf"), f"too close ({approach:.0f}m)"))
                continue
            if n_tl > 0 and not self._tl_observable():
                # [INF 게이트] 경로에 신호등이 있는데 그걸 볼 카메라가 하나도
                # 없다 → 빨간불 무시 위험. 이 후보는 비용이 아니라 "금지".
                evaluated.append((c, float("inf"),
                                  f"{n_tl} traffic lights, no healthy camera covers them"))
                continue
            # (2) 신호등 항의 g: 신호등 인식 점수가 낮을수록 벌점이 커지는 함수.
            #     cap_tl=1이면 g=0 (벌점 없음), 0.5면 g=1, 0.05 이하면 g=19 (매우 큼).
            #     즉 "신호는 보이긴 하는데 인식 성능이 떨어진 상태"일수록
            #     신호등 낀 경로를 점점 더 회피하게 만든다.
            g = (1.0 / max(cap_tl, 0.05)) - 1.0
            # (3) 섹터 항: 이 후보의 정차 기동에 "필요한 방향 섹터"들의 점수
            #     결손 합. 예: 갓길은 F, FR 섹터가 필요한데 FR 라이다가 반쯤
            #     죽었으면(0.5) 0.5만큼의 벌점이 쌓인다. 그 방향을 못 보는
            #     기동은 위험하니 그런 후보를 뒤로 밀어낸다.
            sector_pen = sum(1.0 - self.scores.get(f"sector/{k}", 1.0)
                             for k in c.required_sectors)
            # (1) eta = 고장 난 채로 도로에 노출되는 총 시간
            #         = 주행 시간 + 신호등 개수 * 예상 대기 시간.
            #     신호 대기 중에도 위험 노출은 계속되므로 대기 시간을 더한다.
            eta = time_s + n_tl * tl_wait
            cost = (w_t * eta                       # (1) 노출 시간: 초당 w_time 벌점
                    + w_tl * n_tl * g               # (2) 신호등: 인식 성능 나쁠수록 회피
                    + w_sec * sector_pen            # (3) 섹터: 필요한 방향 감지 결손 벌점
                    + w_loc * eta * (1.0 - cap_loc))  # (4) 위치추정: 불안할수록
                                                      #     "오래 걸리는 경로"가 더 비싸짐
            evaluated.append((c, cost,
                              f"len {length:.0f}m eta {eta:.0f}s tl {n_tl} sec {sector_pen:.2f}"))
            if cost < best_cost:
                best, best_cost = c, cost

        # 모든 후보의 비용을 로그로 출력 (디버깅/논문 그림용)
        for c, cost, why in sorted(evaluated, key=lambda e: e[1]):
            self.get_logger().info(f"  candidate {c.name} [{c.kind}]: cost "
                                   f"{'INF' if math.isinf(cost) else f'{cost:.0f}'} ({why})")
        self._publish_markers(evaluated, best)
        return best

    def _send_goal(self):
        """선택된 후보 위치로 경로 변경을 Autoware AD API 서비스에 요청."""
        if not self.route_cli.service_is_ready():
            # 서비스 서버(Autoware 쪽)가 아직 안 떠 있으면 다음 틱에 재시도
            self.get_logger().warn("route service not ready, retrying...")
            return
        req = SetRoutePoints.Request()
        req.header.frame_id = "map"   # 좌표가 map 좌표계 기준임을 명시
        # goal_planner가 우리가 준 지점을 "주차 가능한 근처 지점"으로
        # 스스로 다듬을 수 있게 허용
        req.option.allow_goal_modification = True
        # 구버전 autoware_adapi_v1_msgs에는 이 필드가 없어서 있는지 확인 후 설정
        if hasattr(req.option, "allow_while_using_route"):
            req.option.allow_while_using_route = True
        req.goal = Pose(position=Point(x=self._chosen.x, y=self._chosen.y, z=0.0),
                        orientation=yaw_to_quat(self._chosen.yaw))
        # call_async = 비동기 호출: 응답을 기다리며 멈추지 않고, 응답이 오면
        # _goal_response 콜백이 불린다. (동기로 기다리면 노드 전체가 멈춘다)
        self.route_cli.call_async(req).add_done_callback(self._goal_response)
        self._goal_sent = True
        self._goal_sent_time = self.get_clock().now()
        self._goal_accepted = False
        self.get_logger().warn(f"MRM goal sent: {self._chosen.name} "
                               f"({self._chosen.x:.1f}, {self._chosen.y:.1f})")

    def _goal_response(self, future):
        """경로 변경 서비스의 응답 처리 (call_async의 콜백)."""
        try:
            res = future.result()
            if not res.status.success:
                # 거절됨 → _goal_sent를 False로 되돌려 다음 틱에 재전송
                self.get_logger().error(f"route change rejected: {res.status.message} — retrying")
                self._goal_sent = False
            else:
                self._goal_accepted = True
                self._noresp = 0
                # 크립 가드(90초/25 m)의 기준점: 수락 시각과 그때의 위치
                self._accept_time = self.get_clock().now()
                self._accept_pose = self.pose
                self.get_logger().warn("route change ACCEPTED by Autoware")
        except Exception as e:
            self.get_logger().error(f"route change failed: {e} — retrying")
            self._goal_sent = False

    # ------------------------------------------------------------- 보고/시각화
    def _transition(self, state, why):
        """상태 전이 + 이유 로그. 같은 상태로의 전이는 무시."""
        if state != self.state:
            self.get_logger().warn(f"state {self.state} -> {state}: {why}")
            self.state = state
            self._publish_state()

    def _publish_state(self):
        """현재 상태(+선택된 후보 이름)를 /mrm_supervisor/state 로 방송."""
        detail = f":{self._chosen.name}" if self._chosen else ""
        self.state_pub.publish(String(data=self.state + detail))

    def _publish_markers(self, evaluated, best):
        """후보들을 RViz용 마커(3D 도형)로 방송 — 사람 눈으로 확인하기 위한 것.

        각 후보마다 구(SPHERE) 하나 + 이름/비용 텍스트 하나를 그린다.
        색: 초록 = 선택된 후보, 빨강 = 불가능(INF), 주황 = 평가만 된 후보.
        """
        arr = MarkerArray()
        for i, (c, cost, _) in enumerate(evaluated):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "mrm_candidates"   # ns(namespace) + id 조합이 마커의 고유 주소
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = Point(x=c.x, y=c.y, z=1.0)
            m.pose.orientation = yaw_to_quat(c.yaw)
            m.scale.x = m.scale.y = m.scale.z = 2.5   # 지름 2.5 m 구
            if c is best:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 0.9
            elif math.isinf(cost):
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 0.6
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.6, 0.0, 0.7
            arr.markers.append(m)

            # 구 위에 띄우는 텍스트 (후보 이름 + 비용)
            t = Marker()
            t.header = m.header
            t.ns = "mrm_costs"
            t.id = i
            t.type = Marker.TEXT_VIEW_FACING   # 항상 카메라를 바라보는 글자
            t.action = Marker.ADD
            t.pose.position = Point(x=c.x, y=c.y, z=3.5)
            t.scale.z = 1.5   # 글자 높이 1.5 m
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = f"{c.name}\n{'INF' if math.isinf(cost) else f'{cost:.0f}'}"
            arr.markers.append(t)
        self.marker_pub.publish(arr)


def main():
    """진입점: ROS 초기화 → 노드 생성 → 이벤트 루프(spin) → 종료 정리.

    rclpy.spin(node)은 "이 노드를 계속 살려두고, 타이머/구독 콜백이
    올 때마다 실행해라"는 무한 루프다. Ctrl+C 등으로 끝날 때까지 여기 머문다.
    """
    rclpy.init()
    node = Supervisor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
