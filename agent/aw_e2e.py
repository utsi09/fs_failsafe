#!/usr/bin/env python

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
[이 파일이 하는 일 — 큰 그림]

CARLA 리더보드가 매 시뮬레이션 틱마다 호출하는 "에이전트"이다.
Autoware(자율주행 소프트웨어)를 대신해 CARLA(시뮬레이터)와 데이터를
주고받으며, 실험용 고장 주입(센서 노이즈 / 센서 파괴)과 NPC 교통
(공격적 운전 스타일, 꼬리차)을 관리한다.

용어 정리:
  - CARLA 리더보드: CARLA 위에서 자율주행을 "채점"하는 프레임워크.
    시나리오(맵, 경로, 이벤트)를 만들어 놓고 매 틱 run_step()을 불러
    "이번 틱에 차를 어떻게 조작할래?"라고 묻는다.
  - ROS2: 프로세스들이 "토픽"이라는 이름의 채널로 메시지를 주고받는
    미들웨어. publisher(발행자)가 토픽에 쓰면 subscriber(구독자)가 받는다.
    Autoware는 수십 개의 ROS2 노드로 이루어져 있다.
  - 이 에이전트 = CARLA와 Autoware 사이의 "통역사"이자 실험 조작자.

데이터 흐름 (매 틱 반복):
  1. CARLA -> 에이전트: 차량 위치/속도/가속도, 센서 데이터를 읽는다.
  2. 에이전트 -> Autoware(ROS2 토픽 발행): 위치(/localization/...),
     속도/조향 상태(/vehicle/status/...), IMU, GNSS, 목적지(goal) 등을
     Autoware가 알아듣는 메시지로 변환해 흘려보낸다.
     (라이다/카메라 원본은 별도의 C++ 브리지 노드가 담당한다.)
  3. Autoware -> 에이전트: Autoware가 계산한 제어 명령
     (/control/command/control_cmd: 조향+가감속)을 구독해 받는다.
  4. 에이전트 -> CARLA: 받은 제어 명령을 carla.VehicleControl로 바꿔
     run_step()의 반환값으로 돌려준다. 리더보드가 이것을 차에 적용한다.

여기에 더해, 연구용으로 두 가지를 얹는다:
  - 고장 주입: 정해진 시각에 카메라/라이다를 파괴하거나 노이즈를 섞어
    "센서 고장" 상황을 만든다 (환경변수 cf/cn/lf/ln/imn/gn/... 로 설정).
  - NPC 교통 조작: 주변 차들을 공격적으로 만들거나(npc=agg),
    자차 뒤에 바짝 따라붙는 꼬리차(tail NPC)를 유지한다.
"""


from __future__ import print_function
import os
import time
import threading
import queue
import numpy as np
import math
import re

# CARLA 파이썬 API. 시뮬레이터 안의 세계(world), 차량/센서(actor) 등을
# 파이썬 객체로 조작할 수 있게 해 준다.
import carla

# 리더보드/시나리오러너가 제공하는 도우미들.
# CarlaDataProvider: 현재 시뮬레이션의 world, 자차(hero), 맵 등을
# 어디서든 꺼내 쓸 수 있게 해 주는 전역 저장소 같은 것.
from leaderboard.utils.route_manipulation import downsample_route
from leaderboard.autoagents.autonomous_agent import Track
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

# rclpy = ROS2의 파이썬 클라이언트 라이브러리.
# "노드"를 만들고 그 노드로 토픽을 발행/구독한다.
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.task import Future

# 아래는 전부 ROS2 메시지 타입들(주고받는 데이터의 "서식지").
from ackermann_msgs.msg import AckermannDrive
from diagnostic_msgs.msg import KeyValue, DiagnosticStatus
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3, TransformStamped, AccelWithCovarianceStamped, PoseStamped, PoseWithCovarianceStamped, TwistWithCovarianceStamped
from sensor_msgs.msg import NavSatFix, Imu, PointCloud2, Image, CameraInfo
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from visualization_msgs.msg import MarkerArray

# TUMROSBaseAgent: 리더보드 에이전트의 공통 기능(제어 큐 등)을 담은 부모 클래스.
# AutowareConverter: CARLA 좌표계 <-> Autoware(ROS) 좌표계 변환 담당.
from autoware_agent.tum_ros_base_agent import TUMROSBaseAgent, BridgeHelper
from autoware_agent.aw_converter import AutowareConverter
# 아래는 Autoware 전용 메시지/서비스 타입들.
from autoware_adapi_v1_msgs.msg import LocalizationInitializationState, RouteState, OperationModeState
from autoware_vehicle_msgs.msg import SteeringReport, VelocityReport, Engage, ControlModeReport
from autoware_control_msgs.msg import Control
from autoware_perception_msgs.msg import PredictedObjects, TrafficLightGroupArray, TrafficLightGroup, TrafficLightElement
from tier4_vehicle_msgs.msg import ActuationStatusStamped
from autoware_adapi_v1_msgs.srv import SetRoutePoints
# SetParameters: 다른 ROS2 노드의 파라미터를 원격으로 바꾸는 표준 서비스.
# (고장 주입 때 브리지 노드의 noise_sigma를 바꾸는 데 쓴다.)
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as RclParameter, ParameterValue, ParameterType

def get_entry_point():
    # 리더보드가 이 파일에서 어떤 클래스를 에이전트로 쓸지 물어볼 때
    # 호출하는 약속된 함수. 클래스 이름을 문자열로 돌려준다.
    return 'AutowareE2EAgent'

EPSILON = 0.001

def wait_for_message(node, topic, topic_type, timeout=None):
    # 특정 토픽에서 "메시지 딱 한 개"가 올 때까지 기다렸다가 받아오는 도우미.
    # 임시 구독자를 만들고, 첫 메시지가 도착하면 Future(약속 상자)에 담은 뒤
    # 구독자를 지우고 그 값을 반환한다.

    s = None
    try:
        future = Future()
        s = node.create_subscription(
            topic_type,
            topic,
            lambda msg: future.set_result(msg.data),
            qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        rclpy.spin_until_future_complete(node, future, timeout)

    finally:
        if s is not None:
            node.destroy_subscription(s)

    return future.result()


class AutowareE2EAgent(TUMROSBaseAgent):


    ROS_VERSION = 2

    def __init__(self, carla_host, carla_port, debug=False):
        # 리더보드가 시나리오 시작 전에 에이전트를 생성하면서 호출한다.
        # 여기서는 ROS2 노드를 하나 만들고, Autoware와 주고받을
        # 모든 토픽의 발행자(publisher)/구독자(subscription)를 등록한다.
        #
        # ROS2 미니 문법:
        #   create_publisher(타입, "토픽이름", QoS) -> 이 토픽에 "쓰는" 손잡이
        #   create_subscription(타입, "토픽이름", 콜백, QoS)
        #     -> 이 토픽에 메시지가 오면 콜백 함수가 자동 호출됨
        #   QoS의 TRANSIENT_LOCAL = "늦게 구독한 쪽도 마지막 메시지를 받게
        #     보관해 둠"(래치), VOLATILE = 보관 안 함(실시간 스트림용).
        super(AutowareE2EAgent, self).__init__(self.ROS_VERSION, carla_host, carla_port, debug)
        rclpy.init(args=None)
        self.ros_node = rclpy.create_node("autoware_e2e_node")

        # --Autoware 연동부 시작--
        # [시간 동기화] Autoware 쪽 시계(/autoware_time)를 받아 둔다.
        # 시뮬레이션은 현실 시간과 다르게 흐르므로 양쪽 시계를 맞추는 것이 중요.
        self._aw_time_sub = self.ros_node.create_subscription(Clock, "/autoware_time", self._aw_time_callback, qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        self._ready_state_pub = self.ros_node.create_publisher(DiagnosticStatus, "/ready_state", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # [센서] 라이다 점군 발행자 (sensor_data QoS = 유실 허용, 최신 우선)
        self._localization_acceleration_subscriber = self.ros_node.create_subscription(AccelWithCovarianceStamped, "/localization/acceleration", self._acc_callback, qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        self._lidar_pub_front = self.ros_node.create_publisher(PointCloud2, "/sensor/lidar/front", qos_profile=qos_profile_sensor_data)
        self._lidar_pub_rear = self.ros_node.create_publisher(PointCloud2, "/sensor/lidar/rear", qos_profile=qos_profile_sensor_data)
        # [센서] 신호등 인식용 카메라 이미지 발행자
        self._camera_pub = self.ros_node.create_publisher(Image, "/sensing/camera/traffic_light/image_raw", qos_profile=qos_profile_sensor_data)
        self._camera_info_pub = self.ros_node.create_publisher(CameraInfo, "/sensing/camera/traffic_light/camera_info", qos_profile=qos_profile_sensor_data)
        # [시계] ROS 표준 시계 토픽 /clock 발행자 (Autoware 전체가 이 시각을 따름)
        self._clock_pub = self.ros_node.create_publisher(Clock, "/clock", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        # [위치추정] CARLA가 알고 있는 "진짜 위치"를 Autoware의 위치추정
        # 토픽들로 흘려보내는 발행자들.
        self._localization_publisher = self.ros_node.create_publisher(Odometry, "/localization/kinematic_state", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        self._tf_publisher = self.ros_node.create_publisher(TFMessage, "/tf", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        #self._localization_init_state_publisher = self.ros_node.create_publisher(LocalizationInitializationState, "/localization/initialization_state", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._localization_acceleration_publisher = self.ros_node.create_publisher(AccelWithCovarianceStamped, "/localization/acceleration", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        # [위치추정-네이티브] Autoware의 EKF(칼만필터)에 먹이는 IMU/자세 입력들.
        # 고장 주입 실험에서 imn/gn 노이즈가 섞이는 지점이 바로 이 토픽들이다.
        self._imu_pub = self.ros_node.create_publisher(Imu, "/sensing/imu/imu_data", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        self._ndt_pose_pub = self.ros_node.create_publisher(PoseWithCovarianceStamped, "/localization/pose_estimator/pose_with_covariance", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        #self._vel_pub = self.ros_node.create_publisher(TwistWithCovarianceStamped, "/sensing/vehicle_velocity_converter/twist_with_covariance", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        self._init_pose_pub = self.ros_node.create_publisher(PoseWithCovarianceStamped, "/initialpose", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        self._gnss_pose_pub = self.ros_node.create_publisher(PoseWithCovarianceStamped, "/sensing/gnss/pose_with_covariance", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        # [차량 상태 보고] Autoware의 계획/제어 모듈이 "차가 지금 어떤 상태인지"
        # 알 수 있도록 조향각/속도/제어모드를 보고하는 발행자들.
        self._steering_report_publisher = self.ros_node.create_publisher(SteeringReport, "/vehicle/status/steering_status", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        self._velocity_report_publisher = self.ros_node.create_publisher(VelocityReport, "/vehicle/status/velocity_status", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        self._control_mode_report_publisher = self.ros_node.create_publisher(ControlModeReport, "/vehicle/status/control_mode", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        #self._actuation_report_publisher = self.ros_node.create_publisher(ActuationStatusStamped, "/vehicle/status/actuation_status", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        # [목적지/출발] rough_goal = RViz에서 지도를 클릭해 목적지를 찍는 것과
        # 같은 토픽. 여기로 최종 목적지를 발행하면 Autoware가 경로를 계산한다.
        # engage = "출발해도 좋다" 버튼에 해당.
        self._goal_publisher = self.ros_node.create_publisher(PoseStamped, "/rviz/routing/rough_goal", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._engage_publisher = self.ros_node.create_publisher(Engage, "/autoware/engage", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        # [실험 신호] /fault_injection/active = "고장이 켜졌음"을 알리는 래치 신호.
        # TRANSIENT_LOCAL이라 한 번 켜지면 나중에 구독한 쪽도 받는다(유지됨).
        # 실험 판정 스크립트가 이 신호의 시각을 기준(고장 시점 t=0)으로 쓴다.
        self._fault_active_pub = self.ros_node.create_publisher(Bool, "/fault_injection/active", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._autoware_mode_subscriber = self.ros_node.create_subscription(OperationModeState, "/system/operation_mode/state", self._operation_mode_callback, qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        # [서비스] 중간 경유지를 갈아끼울 때 쓰는 Autoware 경로 변경 서비스.
        # 서비스 = 토픽과 달리 "요청 -> 응답" 1회성 호출(함수 호출과 비슷).
        self._route_service_client = self.ros_node.create_client(SetRoutePoints, "/api/routing/change_route_points")
        # MRM(최소위험기동, 비상 정지/회피) 중에는 mrm_supervisor가 경로의
        # 주인이다 — 그동안 이 에이전트는 목적지 갱신을 멈추고 물러난다.
        self._mrm_active = False
        self._mrm_state_subscriber = self.ros_node.create_subscription(
            String, "/mrm_supervisor/state", self._mrm_state_callback,
            qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        # [제어 수신] Autoware가 계산한 최종 제어 명령(조향+가감속)을 받는 구독자.
        # 이게 도착해야 run_step()이 CARLA에 돌려줄 조작값이 생긴다.
        self._control_subscriber = self.ros_node.create_subscription(Control, "/control/command/control_cmd", self._vehicle_control_cmd_callback, qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))

        self._object_publisher = self.ros_node.create_publisher(PredictedObjects, "/perception/object_recognition/objects", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))

        # [V2X 신호등] 카메라 대신 "지도에서 신호등 ID를 뽑아 상태를 직접
        # 알려주는" 치트 경로용 구독/발행자 (기본은 꺼져 있음, 아래 setup 참고).
        self._lanelet_map_subscriber = self.ros_node.create_subscription(MarkerArray, "/map/vector_map_marker", self._get_traffic_lights_from_lanelet, qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._external_traffic_light_publisher = self.ros_node.create_publisher(TrafficLightGroupArray, "/perception/traffic_light_recognition/traffic_signals", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))

        # 동기화용 상태 변수들
        self._aw_time = None
        self.ready_status = DiagnosticStatus()
        self.ready_status.level = DiagnosticStatus.OK
        self.ready_status.name = "AutowareE2EAgent"

        # --Autoware 연동부 끝--
        self._client = None
        self._vehicle = None
        self._world = None
        self._map = None

        self._camera_height = None
        self._camera_width = None
        self._camera_fov = None

        # rclpy.spin = "메시지 오면 콜백 실행"을 무한 반복하는 이벤트 루프.
        # run_step()과 별개로 돌아야 하므로 전용 스레드에서 돌린다.
        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self.ros_node,))
        self.spin_thread.start()

    def setup(self, path_to_conf_file):
        # 리더보드가 시나리오를 다 차려 놓은 뒤 호출하는 초기화 훅.
        # 여기서 CARLA 세계/자차 핸들을 얻고, 고장 주입 설정을 읽어 둔다.
        self.track = Track.MAP

        # CARLA 접속 핸들 얻기.
        # CarlaDataProvider는 리더보드가 미리 채워 둔 전역 저장소이고,
        # get_hero_actor() = 우리가 조종할 자차(ego) 객체를 꺼내는 것.
        # world = 시뮬레이션 세계 전체, map = 도로망 정보.
        self._client = CarlaDataProvider.get_client()
        self._vehicle = CarlaDataProvider.get_hero_actor()
        self._world = self._vehicle.get_world()
        self._map = self._world.get_map()
        self._awp_converter = AutowareConverter(self._vehicle, self._world)
        # carla.VehicleControl = CARLA에 줄 "핸들/액셀/브레이크" 묶음.
        # 아직 명령이 없으므로 "가만히 있기"로 초기화.
        self._last_control = carla.VehicleControl(throttle=0.0)
        self._route_index = 1 # 0번은 출발점이므로 1부터 시작
        self._last_published_route_index = None
        self._published_latest = False
        self._goal_mod_failure_counter = 0
        self._is_autonomous = False
        self._pending_service_call = False
        self._route_extended = False
        # 전체 경로를 한 번에 세팅하기 위한 플래그 (주행 중 "재경로 불가" 회피)
        self._bootstrap_goal_sent = False
        self._route_sent = False
        # ===== 고장 주입 설정 (연구용) =====
        # 환경변수로 켠다. 아무것도 안 주면 전부 꺼짐:
        #   cf=20       주행 시작 20초 뒤 카메라 파괴 (하류에서 이미지가 멈춤)
        #   cn=20:15    20초 뒤 카메라에 가우시안 픽셀 노이즈 (0-255 기준 sigma 15)
        #   ln=20:0.1   20초 뒤 라이다에 거리 노이즈 (sigma 0.1 m)
        # 실행 중에도 ROS 파라미터로 바꿀 수 있다:
        #   ros2 param set /autoware_e2e_node cf true
        #   ros2 param set /autoware_e2e_node cn "0:15"      (0 = 지금 즉시 발동)
        cf_env = os.environ.get("cf", "")
        try:
            cf_default = bool(cf_env)
            cf_delay_default = float(cf_env) if cf_env else 20.0
        except ValueError:
            print(f"[FAULT INJECTION] invalid cf value {cf_env!r} — fault injection disabled")
            cf_default, cf_delay_default = False, 20.0
        # 환경변수 값을 ROS 파라미터로 옮겨 둔다.
        # (파라미터로 만들어 두면 실행 중에 ros2 param set으로 조작 가능)
        if not self.ros_node.has_parameter("cf"):
            self.ros_node.declare_parameter("cf", cf_default)
            self.ros_node.declare_parameter("cf_delay", cf_delay_default)
            self.ros_node.declare_parameter("cn", os.environ.get("cn", ""))
            # lf=<t>: t초 뒤 전방(front) 라이다 액터 자체를 파괴 (하드웨어 사망 상황)
            self.ros_node.declare_parameter("lf", os.environ.get("lf", ""))
            self.ros_node.declare_parameter("ln", os.environ.get("ln", ""))
            self.ros_node.declare_parameter("lnf", os.environ.get("lnf", ""))
            self.ros_node.declare_parameter("lnr", os.environ.get("lnr", ""))
            self.ros_node.declare_parameter("imn", os.environ.get("imn", ""))
            self.ros_node.declare_parameter("gn", os.environ.get("gn", ""))
            # tail=<gap_m>: 자차 뒤 <gap_m> 미터에 추격 NPC를 항상 유지.
            # (리더보드의 배경 교통은 대부분 자차 "앞"에 생긴다 — 차선 내
            # 비상정지(MRM) 시 뒤에서 들이받을 위협을 만들려면 명시적인
            # 꼬리차가 필요하다.)
            self.ros_node.declare_parameter("tail", os.environ.get("tail", ""))
        # 고장 타이머/상태 플래그 초기화
        self._fault_t0 = None            # 타이머 기준 시각 (아직 안 켜짐)
        self._camera_killed = False
        self._lidar_killed = False
        self._tail_spawned = False
        self._tail_actor = None
        self._fault_announced = False    # /fault_injection/active 발행 여부
        self._bridge_noise_started = set()
        # npc=agg: TrafficManager로 배경 교통을 공격적으로 만들기 (아직 안 함)
        self._npc_styled = False
        self._imu_noise_sigma = 0.0
        self._gnss_noise_sigma = 0.0

        # 신호등 인식 방식 선택:
        # 카메라 이미지(_publish_cam_image) 또는 V2X 치트(_use_v2x_traffic_light)
        # 중 하나는 켜야 한다. 시각화 목적이라면 둘 다 켜도 됨.
        self._use_v2x_traffic_light = False
        self._publish_cam_image = True
        self._traffic_light_ids = set()

        self._steps_before_init = 0
        self._counter = 0

        # 첫 제어 명령을 기다리는 최대 시간(초). Autoware가 켜지는 데
        # 오래 걸리므로 처음엔 아주 길게 두고, 이후 0.2초로 줄인다(run_step 참고).
        self._time_to_wait_for_control = 99.0

    def sensors(self) -> list:
        # 리더보드가 "어떤 센서를 차에 달아 줄까?" 물어볼 때 호출하는 함수.
        # 여기 적은 대로 리더보드가 CARLA 센서 액터를 자차에 부착해 준다.
        # x/y/z/roll/pitch/yaw = 차 기준 부착 위치와 방향.
        sensors = [
        {'type': 'sensor.lidar.ray_cast', 'id': 'LIDAR_front',
         'x': 0.3, 'y': 0.0, 'z': 1.70, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
         'channels': 128, 'rotation_frequency': 20, 'points_per_second': 1800000,
         'ros_name': 'sensor/lidar/front'},
        {'type': 'sensor.lidar.ray_cast', 'id': 'LIDAR_rear',
         'x': -1.45, 'y': 0.0, 'z': 1.70, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
         'channels': 64, 'rotation_frequency': 20, 'points_per_second': 700000,
         'ros_name': 'sensor/lidar/back'},
        {'type': 'sensor.other.gnss', 'id': 'GPS',
         'x': 0.0, 'y': 0.0, 'z': 0.0}
        ]

        if self._publish_cam_image:
            sensors.append({ "type": "sensor.camera.rgb", "id": "CAMERA_front",
            "x": 0.0, "y": 0.0, "z": 2.0,
            "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "width": 800 , "height": 400,
            "fov": 90.0,
            'ros_name': 'sensor/camera'})

            self._camera_height = next(s for s in sensors if s["type"] == "sensor.camera.rgb")["height"]
            self._camera_width = next(s for s in sensors if s["type"] == "sensor.camera.rgb")["width"]
            self._camera_fov = next(s for s in sensors if s["type"] == "sensor.camera.rgb")["fov"]

        return sensors

    # 라이다 부분 고장용 "부채꼴" 정의 (도 단위): 중심 방위각 / 폭.
    # 예: "R" = 오른쪽 절반에만 노이즈 (센서 좌표계 기준).
    _NOISE_SECTORS = {"R": (-90.0, 180.0), "L": (90.0, 180.0),
                      "F": (0.0, 180.0), "B": (180.0, 180.0)}

    @staticmethod
    def _parse_fault(spec, default_mag):
        # 고장 설정 문자열 파서. "시각:크기:부채꼴" 형식을 튜플로 바꾼다.
        # "20:0.1"   -> (20.0, 0.1, None)   센서 전체에 노이즈
        # "20:0.1:R" -> (20.0, 0.1, "R")    오른쪽 절반만 (라이다 전용)
        # "20"       -> (20.0, 기본크기, None); "" -> None (고장 없음)
        if not spec:
            return None
        try:
            parts = str(spec).split(":")
            t = float(parts[0])
            mag = float(parts[1]) if len(parts) > 1 and parts[1] else default_mag
            sector = parts[2].upper() if len(parts) > 2 and parts[2] else None
            return t, mag, sector
        except ValueError:
            print(f"[FAULT INJECTION] invalid fault spec {spec!r} — ignored")
            return None

    def _set_bridge_noise(self, node_names, sigma, sector=None):
        # [노이즈 주입의 실제 통로]
        # 노이즈는 시뮬레이터가 아니라 C++ 브리지 노드(camera_bridge/
        # lidar_bridge) "안에서" 섞인다. (이 CARLA 서버 빌드는 실행 중
        # 센서를 새로 스폰하면 멈춰 버려서, 시뮬레이터는 건드리지 않는다.)
        # 방법: 각 브리지 노드의 ROS 파라미터 noise_sigma를 원격으로 바꾼다.
        # 브리지는 매 프레임 이 파라미터를 읽어, 0보다 크면 그만큼의
        # 가우시안 노이즈를 데이터에 섞는다. 즉 "노이즈 크기 다이얼"을
        # 밖에서 돌려 주는 것이다.
        for name in node_names:
            try:
                # /<노드이름>/set_parameters = 모든 ROS2 노드가 자동으로 갖는
                # "내 파라미터 바꿔 줘" 서비스.
                cli = self.ros_node.create_client(SetParameters, f"/{name}/set_parameters")
                req = SetParameters.Request()
                params = []
                # 부채꼴 지정이 있으면(라이다만) 노이즈를 그 방향 절반에만 적용
                if sector in self._NOISE_SECTORS and name.startswith("lidar"):
                    c, w = self._NOISE_SECTORS[sector]
                    params += [
                        RclParameter(name="noise_yaw_center",
                                     value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                                          double_value=c)),
                        RclParameter(name="noise_yaw_width",
                                     value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                                          double_value=w)),
                    ]
                params.append(RclParameter(
                    name="noise_sigma",
                    value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                         double_value=float(sigma))))
                req.parameters = params
                # call_async = 응답을 기다리지 않고 요청만 보냄 (틱을 막지 않기 위해)
                cli.call_async(req)
                import time as _time
                print(f"[FAULT INJECTION] noise_sigma={sigma}"
                      f"{f' sector={sector}' if sector else ''} requested on /{name}"
                      f" (wall {_time.strftime('%H:%M:%S')})")
            except Exception as e:
                print(f"[FAULT INJECTION] noise request on /{name} failed:", e)

    # ============================================================
    # run_step: 리더보드가 매 시뮬레이션 틱마다 부르는 심장부.
    # 입력: 센서 데이터(input_data)와 시각 -> 출력: 이번 틱의 차량 조작.
    # 순서: 고장 주입 체크 -> NPC 관리 -> Autoware에 상태 발행 ->
    #       Autoware의 제어 명령 수신 -> CARLA에 반환.
    # ============================================================

    def run_step(self, input_data, timestamp):
        # 시각은 인자 대신 CARLA 스냅샷(현재 프레임의 상태 사진)에서 직접 읽는다.
        timestamp = carla_timestamp = CarlaDataProvider.get_world().get_snapshot().timestamp.elapsed_seconds
        print('COUNTER', self._counter)
        self._counter += 1

        # --- 고장 주입 타이머 (연구용) ---
        # 고장 시각 t는 "자율주행 모드(engaged) AND 속도 > 1.0 m/s",
        # 즉 차가 실제로 출발한 순간부터 센다.
        # 이유: engage 시점부터 세면 빨간불에 서서 기다리는 동안 t가 흘러
        # "주차된 차"에 고장이 주입되는 문제가 있었다 (출발 전 주입 방지).
        if self._fault_t0 is None:
            moving = False
            if self._is_autonomous:
                try:
                    ego = CarlaDataProvider.get_hero_actor()
                    # get_velocity().length() = 속도 벡터의 크기 (m/s)
                    moving = ego is not None and ego.get_velocity().length() > 1.0
                except Exception:
                    pass
            if moving:
                self._fault_t0 = timestamp   # 타이머 시작!
                print(f"[FAULT INJECTION] timer armed at sim {timestamp:.1f}s (driving)")
            fault_elapsed = -1.0             # 아직 출발 전: 어떤 고장도 발동 안 함
        else:
            fault_elapsed = timestamp - self._fault_t0

        # --- npc=agg: 배경 교통을 공격적으로 (스폰 완료 후 한 번만 적용) ---
        # TrafficManager(TM) = CARLA가 NPC 차들을 자동 운전시키는 관제탑.
        # 차마다 아래 다이얼을 돌려 운전 성향을 바꿀 수 있다:
        #   vehicle_percentage_speed_difference(v, -20): 제한속도 대비 속도 차이(%).
        #     음수 = 더 빠르게. -20 = 제한속도보다 20% 과속.
        #   distance_to_leading_vehicle(v, 1.2): 앞차와 유지할 최소 거리(m).
        #     1.2m = 바짝 붙는 꼬리물기. (0.5m는 정지한 자차 범퍼에 붙어
        #     재출발을 영영 막아 버렸음)
        #   ignore_vehicles_percentage(v, 20): 다른 차를 무시할 확률(%).
        #     끼어들기/양보 안 함이 늘어남.
        #   ignore_lights_percentage(v, 10): 신호를 무시할 확률(%). 10% 확률로 신호 위반.
        if not self._npc_styled and self._counter > 40 and os.environ.get("npc", "") == "agg":
            self._npc_styled = True
            try:
                client = CarlaDataProvider.get_client()
                tm = client.get_trafficmanager(CarlaDataProvider.get_traffic_manager_port())
                styled = 0
                # world.get_actors() = 현재 씬에 존재하는 모든 물체(차, 보행자,
                # 센서, 신호등...) 목록. .filter("vehicle.*")로 차만 골라낸다.
                for v in CarlaDataProvider.get_world().get_actors().filter("vehicle.*"):
                    # role_name이 hero/ego_vehicle인 것 = 우리 자차. 건드리지 않음.
                    if v.attributes.get("role_name") in ("hero", "ego_vehicle"):
                        continue
                    tm.vehicle_percentage_speed_difference(v, -20.0)  # 제한속도 +20% 과속
                    tm.distance_to_leading_vehicle(v, 1.2)            # 1.2m 꼬리물기 (0.5는 자차를 못 움직이게 함)
                    tm.ignore_vehicles_percentage(v, 20.0)            # 20% 확률로 양보 안 함
                    tm.ignore_lights_percentage(v, 10.0)              # 10% 확률로 신호 무시
                    styled += 1
                print(f"[NPC] aggressive style applied to {styled} vehicles")
            except Exception as e:
                print("[NPC] aggressive style failed:", e)

        # --- cf: 카메라를 통째로 파괴 (하류 파이프라인에서 이미지가 멈춤) ---
        # actor.destroy() = 그 물체를 시뮬레이션 세계에서 삭제하는 CARLA API.
        # 센서를 destroy하면 데이터가 더 이상 안 나온다 = "하드웨어 사망" 재현.
        if not self._camera_killed and self.ros_node.get_parameter("cf").value:
            fault_camera_delay = float(self.ros_node.get_parameter("cf_delay").value)
            if fault_elapsed >= fault_camera_delay:
                try:
                    world = CarlaDataProvider.get_world()
                    killed = 0
                    for s in world.get_actors().filter('sensor.camera.rgb'):
                        s.destroy(); killed += 1
                    print(f"[FAULT INJECTION] destroyed {killed} camera sensor(s) at t+{fault_camera_delay:.0f}s")
                except Exception as e:
                    print("[FAULT INJECTION] camera destroy failed:", e)
                self._camera_killed = True

        # --- tail=<gap_m>: 실험 내내 자차 뒤에 추격 NPC 유지 ---
        # 한 번만 스폰하면 TM이 교차로에서 제멋대로 방향을 골라 떨어져
        # 나간다 — 그래서 약 5초마다(100틱) 살아 있는지/가까운지 재확인하고,
        # 잃어버렸거나 60m 넘게 멀어졌으면 부수고 다시 스폰한다.
        # 스폰 시점은 반드시 자차가 출발한 "이후"(_fault_t0가 그때 켜짐):
        # 아직 출발 안 한 자차 범퍼에 공격적 꼬리차(0.5m 간격)가 주차해
        # 버리면 자차가 출발을 못 하고, 리더보드가 "차 막힘"(AgentBlockedTest)
        # 판정으로 실험 자체를 죽여 버린다.
        if (self._fault_t0 is not None and self._counter % 100 == 0
                and self.ros_node.get_parameter("tail").value):
            try:
                gap = float(self.ros_node.get_parameter("tail").value)
                world = CarlaDataProvider.get_world()
                ego = CarlaDataProvider.get_hero_actor()
                tail_ok = False
                if self._tail_actor is not None:
                    try:
                        # 살아 있고 자차에서 60m 안이면 그대로 둔다
                        tail_ok = (self._tail_actor.is_alive and
                                   self._tail_actor.get_location().distance(
                                       ego.get_location()) < 60.0)
                    except Exception:
                        tail_ok = False
                if not tail_ok:
                    # 옛 꼬리차가 남아 있으면 삭제하고 새로 만든다
                    if self._tail_actor is not None:
                        try:
                            self._tail_actor.destroy()
                        except Exception:
                            pass
                        self._tail_actor = None
                    # waypoint = 맵의 도로 중심선 위의 한 점. previous(gap) =
                    # 도로를 따라 gap 미터 "뒤"의 지점 목록을 얻는다.
                    wp = CarlaDataProvider.get_map().get_waypoint(ego.get_location())
                    prev = wp.previous(gap)
                    if prev:
                        # transform = 위치 + 회전 묶음 (어디에, 어느 방향으로).
                        # z를 0.5m 올려 스폰: 지면에 파묻혀 스폰 실패하는 것 방지.
                        tf = prev[0].transform
                        tf.location.z += 0.5
                        # blueprint = 차종 설계도. find로 아우디 TT를 고르고,
                        # try_spawn_actor = 그 자리에 실제 차를 생성(자리가
                        # 막혀 있으면 None 반환, 예외 없이 조용히 실패).
                        bp = world.get_blueprint_library().find("vehicle.audi.tt")
                        bp.set_attribute("role_name", "tail_npc")
                        npc = world.try_spawn_actor(bp, tf)
                        if npc is not None:
                            client = CarlaDataProvider.get_client()
                            tm_port = CarlaDataProvider.get_traffic_manager_port()
                            tm = client.get_trafficmanager(tm_port)
                            # set_autopilot(True) = 이 차의 운전을 TM에 맡김
                            npc.set_autopilot(True, tm_port)
                            aggressive = os.environ.get("npc", "") == "agg"
                            # 차간거리 튜닝의 역사: 0.5m로 하면 자차가 설 때마다
                            # 범퍼에 딱 붙어 자차가 다시는 못 움직였다(영구 고착).
                            # 공격 모드 1.5m = 여전히 위협적이지만 재출발은 가능,
                            # 방어 모드 2.5m = 안전거리 유지.
                            tm.distance_to_leading_vehicle(npc, 1.5 if aggressive else 2.5)
                            # 공격: 제한속도 +30% / 방어: +10%
                            tm.vehicle_percentage_speed_difference(
                                npc, -30.0 if aggressive else -10.0)
                            # 공격: 30% 확률로 신호 무시 / 방어: 항상 준수
                            tm.ignore_lights_percentage(npc, 30.0 if aggressive else 0.0)
                            self._tail_actor = npc
                            print(f"[TAIL NPC] (re)spawned {gap:.0f}m behind ego"
                                  f" (aggressive={aggressive})")
            except Exception as e:
                print("[TAIL NPC] ensure failed:", e)

        # --- lf=<t>: 전방 라이다 액터를 아예 파괴 ---
        # 노이즈(ln)와 달리 이것은 CARLA 액터 자체를 destroy한다.
        # 토픽이 완전히 침묵하므로, 감시자(cdr_monitor)의 "노이즈 검사"가
        # 아니라 "타임아웃 검사"가 잡아내야 하는 고장 유형이다.
        if not self._lidar_killed:
            spec = self._parse_fault(self.ros_node.get_parameter("lf").value, 0.0)
            if spec and fault_elapsed >= spec[0]:
                try:
                    world = CarlaDataProvider.get_world()
                    killed = 0
                    # 씬의 모든 라이다 중 이름표에 "front"가 붙은 것만 골라 파괴
                    for s in world.get_actors().filter('sensor.lidar.ray_cast'):
                        tag = (s.attributes.get("role_name", "")
                               + s.attributes.get("ros_name", "")).lower()
                        if "front" in tag:
                            s.destroy(); killed += 1
                    print(f"[FAULT INJECTION] destroyed {killed} front lidar(s) at t+{spec[0]:.0f}s")
                except Exception as e:
                    print("[FAULT INJECTION] lidar destroy failed:", e)
                self._lidar_killed = True

        # --- 브리지 경유 노이즈 고장들 ---
        # cn=<t>:<sigma>       카메라 픽셀에 가우시안 노이즈 (camera_bridge에서 주입)
        # ln/lnf/lnr=<t>:<sigma>  라이다 거리값 노이즈 (양쪽/전방/후방 lidar_bridge)
        # 형식: (플래그 이름, 기본 sigma, 노이즈를 주입할 브리지 노드 이름들)
        bridge_faults = [
            ("cn", 15.0, ["camera_bridge_1"]),
            ("ln", 0.1, ["lidar_bridge_1", "lidar_bridge_2"]),
            ("lnf", 0.1, ["lidar_bridge_1"]),
            ("lnr", 0.1, ["lidar_bridge_2"]),
        ]
        for flag, default_sigma, nodes in bridge_faults:
            if flag in self._bridge_noise_started:
                continue   # 이미 발동한 고장은 다시 안 건드림
            spec = self._parse_fault(self.ros_node.get_parameter(flag).value, default_sigma)
            if spec and fault_elapsed >= spec[0]:
                # 발동 시각 도달 -> 브리지에 noise_sigma 파라미터 전송
                self._set_bridge_noise(nodes, spec[1], spec[2])
                self._bridge_noise_started.add(flag)

        # --- 에이전트가 직접 섞는 노이즈들 ---
        # imn=<t>:<sigma>: IMU 노이즈 (자이로에 sigma rad/s, 가속도에 10*sigma m/s^2)
        # gn=<t>:<sigma>: EKF에 먹이는 자세(pose)의 위치에 노이즈 (미터)
        # 여기서는 sigma 값만 정해 두고, 실제로 섞는 곳은 아래 메시지 발행부다.
        spec = self._parse_fault(self.ros_node.get_parameter("imn").value, 0.05)
        self._imu_noise_sigma = spec[1] if spec and fault_elapsed >= spec[0] else 0.0
        spec = self._parse_fault(self.ros_node.get_parameter("gn").value, 1.0)
        self._gnss_noise_sigma = spec[1] if spec and fault_elapsed >= spec[0] else 0.0

        # --- 고장 발동 알림 (래치) ---
        # 어떤 종류든 고장이 하나라도 켜진 첫 순간에 /fault_injection/active에
        # True를 딱 한 번 발행한다. TRANSIENT_LOCAL QoS라 한 번 켜지면 계속
        # 유지되는 래치 신호다. 실험 판정 스크립트가 이 메시지의 시각을
        # "고장 시점 t=0"의 시계 기준으로 삼는다.
        if not self._fault_announced and (
                self._bridge_noise_started or self._camera_killed
                or self._lidar_killed or self._imu_noise_sigma > 0.0
                or self._gnss_noise_sigma > 0.0):
            self._fault_announced = True
            self._fault_active_pub.publish(Bool(data=True))
            print(f"[FAULT INJECTION] fault ACTIVE at t+{fault_elapsed:.1f}s (announced)")

        # --- Autoware 시계 대기 ---
        # Autoware 쪽 시계가 아직 안 왔으면(부팅 중) 차를 세워 둔 채 반환.
        if self._aw_time is None:
            self._steps_before_init += 1
            control = carla.VehicleControl(steer=0,throttle=0,brake=0)
            return control

        aw_time = self._aw_time

        # --- CARLA에서 차량 상태 읽기 (converter가 좌표계 변환까지 처리) ---
        position = self._awp_converter._get_localization()
        velocity = self._awp_converter._get_twist()
        loc_acc = self._awp_converter._get_acceleration()

        # --- GNSS 자세 발행 ---
        # CARLA의 "진짜 위치"를 GPS 측정값인 척 Autoware에 보낸다.
        # gn 고장이 켜져 있으면 여기서 x, y에 가우시안 오차를 섞는다
        # -> Autoware의 EKF(위치 추정 필터)가 흔들리는 GPS를 받게 됨.
        ndt_pose_msg = PoseWithCovarianceStamped()
        ndt_pose_msg.header.stamp = aw_time
        ndt_pose_msg.header.frame_id = "map"
        ndt_pose_msg.pose.pose.position = Point(**position["position"])
        ndt_pose_msg.pose.pose.orientation = Quaternion(**position["orientation"])
        if self._gnss_noise_sigma > 0.0:
            ndt_pose_msg.pose.pose.position.x += float(np.random.normal(0.0, self._gnss_noise_sigma))
            ndt_pose_msg.pose.pose.position.y += float(np.random.normal(0.0, self._gnss_noise_sigma))
        self._gnss_pose_pub.publish(ndt_pose_msg)

        # --- 조향각 보고 ---
        steering_angle = self._awp_converter._get_steering()
        steering_angle_msg = SteeringReport()
        steering_angle_msg.stamp = aw_time
        steering_angle_msg.steering_tire_angle = steering_angle
        self._steering_report_publisher.publish(steering_angle_msg)

        # --- 속도 보고 ---
        velocity_report_msg = VelocityReport()
        velocity_report_msg.header.stamp = aw_time
        velocity_report_msg.header.frame_id = 'base_link'
        velocity_report_msg.longitudinal_velocity = velocity["position"]["x"]
        self._velocity_report_publisher.publish(velocity_report_msg)

        # --- 제어 모드 보고: "차는 자율주행 모드로 움직이는 중"이라고 알림 ---
        control_mode_report_msg  = ControlModeReport()
        control_mode_report_msg.stamp = aw_time
        control_mode_report_msg.mode = ControlModeReport.AUTONOMOUS
        self._control_mode_report_publisher.publish(control_mode_report_msg)

        # --- 목적지 갱신 + 출발(engage) ---
        # 목적지가 발행되었는데 아직 자율주행 모드가 아니면 engage를 눌러 준다.
        self.publish_global_plan(position["position"])
        if self._published_latest and not self._is_autonomous:
            self._engage_publisher.publish(Engage(engage=True))
            print('enage')

        # --- IMU 발행 ---
        # 각속도(자이로) + 선형가속도. imn 고장이 켜져 있으면 여기서
        # 여섯 축 모두에 가우시안 노이즈를 섞는다 (가속도 쪽은 10배 크게).
        imu_msg = Imu()
        imu_msg.header.stamp = aw_time
        imu_msg.header.frame_id = "imu_link"
        imu_msg.angular_velocity = Vector3(
            x=velocity["orientation"]["roll"],
            y=velocity["orientation"]["pitch"],
            z=velocity["orientation"]["yaw"]
        )
        imu_msg.linear_acceleration = Vector3(**loc_acc["position"])
        if self._imu_noise_sigma > 0.0:
            s = self._imu_noise_sigma
            imu_msg.angular_velocity.x += float(np.random.normal(0.0, s))
            imu_msg.angular_velocity.y += float(np.random.normal(0.0, s))
            imu_msg.angular_velocity.z += float(np.random.normal(0.0, s))
            imu_msg.linear_acceleration.x += float(np.random.normal(0.0, 10.0 * s))
            imu_msg.linear_acceleration.y += float(np.random.normal(0.0, 10.0 * s))
            imu_msg.linear_acceleration.z += float(np.random.normal(0.0, 10.0 * s))
        self._imu_pub.publish(imu_msg)

        # --- V2X 신호등 (치트 모드, 기본 꺼짐) ---
        # 카메라로 신호등을 "보는" 대신, CARLA에서 정답 상태를 읽어
        # 지도상의 모든 신호등 ID에 그대로 뿌려 준다.
        if self._use_v2x_traffic_light:
            traffict_light_state = self._awp_converter.get_current_traffic_light_state()
            traffic_light_msg = TrafficLightGroupArray()
            traffic_light_msg.stamp = aw_time
            for traffic_light_id in self._traffic_light_ids:
                traffic_light_group = TrafficLightGroup()
                traffic_light_group.traffic_light_group_id = traffic_light_id
                traffic_light_group.elements.append(TrafficLightElement())
                traffic_light_group.elements[0].color = traffict_light_state
                traffic_light_group.elements[0].shape = 1
                traffic_light_group.elements[0].status = 2
                traffic_light_group.elements[0].confidence = 1.0
                traffic_light_msg.traffic_light_groups.append(traffic_light_group)
            self._external_traffic_light_publisher.publish(traffic_light_msg)


        self._ready_state_pub.publish(self.ready_status)

        # 첫 5틱은 일부러 기다린다: 첫 라이다 메시지가 도착할 시간을 벌어 줌.
        if self._steps_before_init < 5:
            time.sleep(0.2)
            self._steps_before_init += 1
            control = carla.VehicleControl(steer=0,throttle=0,brake=0)
            return control
        # wait 100ms
        #if time_sec > 5: time.sleep(0.05)
        #time.sleep(0.2)
        # --- Autoware의 제어 명령 받기 ---
        # _control_queue에는 제어 콜백(_vehicle_control_cmd_callback)이
        # 검증을 통과한 명령을 넣어 둔다. 최대 _time_to_wait_for_control초
        # 기다렸다가, 새 명령이 없으면(queue.Empty) 직전 명령을 재사용한다.
        try:
            control_timestamp, control = self._control_queue.get(True, self._time_to_wait_for_control)
            self._last_control = control

        except queue.Empty:
            control_timestamp, control = timestamp, self._last_control

        # 긴 대기(99초)는 초기화 단계에만 필요했다. 이후로는 0.2초.
        # 0.2초인 이유(원래 1.0초였음): 재경로/재계획 중에는 제어 명령
        # 스트림이 잠깐 끊기는데, 틱마다 1초씩 기다리면 시뮬레이션이
        # ~1Hz로 무너지고, 감시자(monitor)는 이것을 "모든 센서 타임아웃"
        # (FTTI 850ms)으로 오판해 버렸다.
        self._time_to_wait_for_control = 0.2

        # 이번 틱의 차량 조작을 리더보드에 반환 -> 리더보드가 CARLA에 적용.
        return control

    @staticmethod
    def get_ros_version():
        return AutowareE2EAgent.ROS_VERSION

    def set_global_plan(self, global_plan_gps, global_plan_world_coord):
        # 리더보드가 시나리오의 정답 경로(웨이포인트 목록)를 넘겨줄 때 호출.
        # 촘촘한 원본 경로를 20m 간격 정도로 솎아내서(downsample) 보관한다.
        ds_ids = downsample_route(global_plan_world_coord, 20)

        # 마지막 웨이포인트(진짜 도착점)는 반드시 포함시킨다
        if ds_ids[-1] != len(global_plan_world_coord) - 1:
            ds_ids.append(len(global_plan_world_coord) - 1)

        self._global_plan_world_coord = [(global_plan_world_coord[x][0], global_plan_world_coord[x][1]) for x in ds_ids]
        self._global_plan = [global_plan_gps[x] for x in ds_ids]

    def _get_extended_final_waypoint(self, global_plan_world_coord, offset_meters=50.0):
        # 최종 웨이포인트에서 도로를 따라 50m 더 나아간 지점을 계산한다.
        # 이유: 리더보드 시나리오는 "정확히 마지막 좌표"에서 끝나지 않으므로,
        # 목적지를 조금 더 앞으로 밀어 두어야 차가 도착 직전에 멈춰서
        # 시나리오가 안 끝나는 문제를 피할 수 있다.
        # CARLA waypoint API의 .next(2.0) = 도로 위상을 따라 2m 앞 지점.

        last_wp_transform = global_plan_world_coord[-1][0]
        last_carla_wp = self._map.get_waypoint(last_wp_transform.location)

        accumulated = 0.0
        current_wp = last_carla_wp

        while accumulated < offset_meters:
            next_wps = current_wp.next(2.0)  # 도로를 따라 한 번에 2m씩 전진
            if not next_wps:
                break
            next_wp = next_wps[0]
            accumulated += 2.0
            current_wp = next_wp

        return (current_wp.transform, global_plan_world_coord[-1][1])

    def _mrm_state_callback(self, msg):
        # mrm_supervisor의 상태를 듣는 콜백. 상태 문자열이 "MRM"으로 시작하면
        # 비상 대응이 진행 중 = 경로의 주도권이 supervisor에게 있음.
        # 그동안 이 에이전트는 목적지 갱신을 멈춘다 (publish_global_plan 참고).
        was = self._mrm_active
        self._mrm_active = msg.data.startswith("MRM")
        if self._mrm_active and not was:
            print(f"[MRM] supervisor took over the route ({msg.data}) — pausing goal updates")

    def publish_global_plan(self, current_position):
        # 매 틱 호출됨. 리더보드가 준 경로를 Autoware에 "목적지"의 형태로
        # 흘려보내는 함수. 중간 경유지는 서비스 호출로, 최종 목적지는
        # /rviz/routing/rough_goal 토픽으로 보낸다.
        if self._mrm_active:
            return   # MRM 중에는 supervisor가 경로 주인 — 아무것도 안 함
        if not self._global_plan_world_coord:
            return

        # 최초 1회: 경로 끝을 50m 연장 (위 _get_extended_final_waypoint 참고)
        if not self._route_extended:
            try:
                carla_map = CarlaDataProvider.get_map()
                extended = self._get_extended_final_waypoint(self._global_plan_world_coord, offset_meters=50.0)
                if extended:
                    self._global_plan_world_coord.append(extended)
                self._route_extended = True
            except Exception as e:
                print(f"Map not ready yet, will retry: {e}")
                return

        total_waypoints = len(self._global_plan_world_coord)

        # dg=1: 직행(direct-goal) 모드 — 중간 경유지를 전부 건너뛰고
        # Autoware에 최종 목적지 하나만 준다. 중간 목적지가 없으면
        # 정상 주행 중 목적지가 갓길(shoulder)로 스냅되는 교란 요인이
        # 원천 차단된다. (아래 is_last_waypoint 분기에서 dg=<x>:<y>로
        # 커스텀 좌표를 주는 확장도 지원.)
        if os.environ.get("dg", "") and self._route_index < total_waypoints - 1:
            self._route_index = total_waypoints - 1
            self._published_latest = False

        is_last_waypoint = self._route_index >= total_waypoints - 1

        # --- 경유지 전진: 현재 목표에 50m 안까지 접근하면 다음 목표로 ---
        if not is_last_waypoint:
            goal_wp = self._global_plan_world_coord[self._route_index]
            # CARLA 좌표 -> ROS 좌표 변환 (CARLA는 y축이 반대인 왼손 좌표계)
            goal_pose_dict = BridgeHelper.carla2ros_pose(
                goal_wp[0].location.x, goal_wp[0].location.y, goal_wp[0].location.z,
                np.deg2rad(goal_wp[0].rotation.roll),
                np.deg2rad(goal_wp[0].rotation.pitch),
                np.deg2rad(goal_wp[0].rotation.yaw),
                to_quat=True
            )

            dist = math.sqrt(
                (goal_pose_dict["position"]["x"] - current_position["x"])**2 +
                (goal_pose_dict["position"]["y"] - current_position["y"])**2 +
                (goal_pose_dict["position"]["z"] - current_position["z"])**2
            )

            # 50m 안에 들어왔거나, 이 목표 설정이 5번 넘게 실패했으면 다음으로
            if dist < 50.0 or self._goal_mod_failure_counter > 5:
                self._route_index += 1
                self._published_latest = False
                self._goal_mod_failure_counter = 0
                is_last_waypoint = self._route_index >= total_waypoints - 1

        # --- 아직 발행 안 한 목표가 있으면 발행 ---
        if self._published_latest:
            return   # 현재 목표는 이미 보냈음 — 할 일 없음

        next_wp = self._global_plan_world_coord[min(self._route_index, total_waypoints - 1)]
        next_pose_dict = BridgeHelper.carla2ros_pose(
            next_wp[0].location.x, next_wp[0].location.y, next_wp[0].location.z,
            np.deg2rad(next_wp[0].rotation.roll),
            np.deg2rad(next_wp[0].rotation.pitch),
            np.deg2rad(next_wp[0].rotation.yaw),
            to_quat=True
        )

        if is_last_waypoint:
            # 최종 목적지는 반드시 토픽(/rviz/routing/rough_goal)으로 발행해야
            # Autoware가 "시나리오 완주"를 제대로 인식한다.
            goal_pose_stamped = PoseStamped()
            goal_pose_stamped.header.frame_id = "map"
            dg_env = os.environ.get("dg", "")
            if ":" in dg_env:
                # dg=<x>:<y>[:<yaw_deg>] — ROS/map 좌표계로 지정한 커스텀 최종
                # 목적지. 시나리오 경로 대신 이 좌표가 rough_goal로 발행된다.
                # 주의: Autoware의 mission_planner는 목적지의 "방향(각도)"까지
                # 차선과 맞는지 검사한다. yaw를 생략하면 0(동쪽)인데, 차선이
                # 다른 방향이면 거부되므로 yaw_deg를 명시해야 한다.
                parts = dg_env.split(":")
                gx, gy = float(parts[0]), float(parts[1])
                yaw = math.radians(float(parts[2])) if len(parts) > 2 else 0.0
                goal_pose_stamped.pose = Pose(
                    position=Point(x=gx, y=gy, z=0.0),
                    # yaw각 -> 쿼터니언(3D 회전 표현) 변환 공식
                    orientation=Quaternion(x=0.0, y=0.0,
                                           z=math.sin(yaw / 2.0),
                                           w=math.cos(yaw / 2.0))
                )
                print(f"Publishing CUSTOM final goal ({gx:.1f}, {gy:.1f}, "
                      f"yaw {math.degrees(yaw):.0f}deg) via topic")
            else:
                goal_pose_stamped.pose = Pose(
                    position=Point(**next_pose_dict["position"]),
                    orientation=Quaternion(**next_pose_dict["orientation"])
                )
                print("Publishing FINAL goal via topic")
            self._goal_publisher.publish(goal_pose_stamped)
            self._published_latest = True
        else:
            # 중간 경유지는 서비스(change_route_points)로 전달
            if self._pending_service_call:
                return   # 이전 서비스 응답을 아직 기다리는 중이면 중복 호출 금지

            service_request = SetRoutePoints.Request()
            service_request.header.frame_id = "map"
            # allow_goal_modification은 반드시 켜 둔다: CARLA 원본 웨이포인트는
            # 반대 차선에 걸리거나 자차 뒤에 떨어질 수 있어서, 그대로 쓰면
            # 플래너가 블록을 한 바퀴 돌아가는 경로를 만든다. 목적지가 갓길로
            # 스냅되는 교란은 여기가 아니라 goal_planner의
            # margin_from_boundary 설정으로 막는다 (좁은 기본 갓길에는 차가
            # 못 들어가고, MRM은 넓혀 둔 경로 밖 갓길만 노리게 됨).
            service_request.option.allow_goal_modification = True
            service_request.option.allow_while_using_route = True
            service_request.goal = Pose(
                position=Point(**next_pose_dict["position"]),
                orientation=Quaternion(**next_pose_dict["orientation"])
            )
            self._pending_service_call = True
            # 비동기 호출: 응답이 오면 _handle_service_response가 실행됨
            future = self._route_service_client.call_async(service_request)
            future.add_done_callback(self._handle_service_response)



    def _handle_service_response(self, future):
        # 경로 변경 서비스의 응답 처리 콜백.
        self._pending_service_call = False
        try:
            result = future.result()
            success = result.status.success
            message = result.status.message

            print('Response:', result.status)

            if success:
                self._published_latest = True
                self._goal_mod_failure_counter = 0

            elif message == 'The route is not set yet.' and not self._mrm_active:
                # 아직 경로가 초기화되지 않은 상태(맨 처음) — 서비스는 못 쓰니
                # 첫 목표를 토픽으로 발행해 Autoware의 경로를 부트스트랩한다.
                print("Route not set yet, publishing initial goal via topic")
                next_wp = self._global_plan_world_coord[self._route_index]
                next_pose_dict = BridgeHelper.carla2ros_pose(
                    next_wp[0].location.x, next_wp[0].location.y, next_wp[0].location.z,
                    np.deg2rad(next_wp[0].rotation.roll),
                    np.deg2rad(next_wp[0].rotation.pitch),
                    np.deg2rad(next_wp[0].rotation.yaw),
                    to_quat=True
                )
                goal_pose_stamped = PoseStamped()
                goal_pose_stamped.header.frame_id = "map"
                goal_pose_stamped.pose = Pose(
                    position=Point(**next_pose_dict["position"]),
                    orientation=Quaternion(**next_pose_dict["orientation"])
                )
                self._goal_publisher.publish(goal_pose_stamped)
                self._published_latest = True

            else:
                # 그 외 실패 — 재시도할 수 있게 두고 실패 횟수만 센다
                # (5회 초과 시 publish_global_plan이 그 경유지를 포기하고 넘어감)
                self._published_latest = False
                self._goal_mod_failure_counter += 1

        except Exception as e:
            self._pending_service_call = False
            self._published_latest = False
            print('Service call failed:', e)

    def _vehicle_control_cmd_callback(self, control_msg):
        # Autoware의 제어 명령(/control/command/control_cmd)이 도착할 때마다
        # 실행되는 콜백. 타임스탬프 검증을 통과한 명령만 큐에 넣고,
        # run_step()이 큐에서 꺼내 CARLA에 적용한다.
        control_timestamp, control = self._awp_converter.convert_control(control_msg)



        # 검증 1: 같은 타임스탬프의 명령이 두 번 오면 중복이므로 무시.
        if self._last_control_timestamp is not None and abs(self._last_control_timestamp - control_timestamp) < EPSILON:
            print(
                "\033[93mWARNING 11111: A new vehicle command with a repeated timestamp has been received {} .\033[0m".format(control_timestamp),
                "\033[93mThis vehicle command will be ignored.\033[0m",
                sep=" ")
            return

        # 검증 2: 명령의 타임스탬프가 "지금 시각"과 맞는지 확인.
        # (GameTime 객체 대신 시각을 직접 계산하는 이유: 이 콜백 실행과
        # GameTime 내부 변수 갱신 사이의 경쟁 상태(race condition)를 피하려고.)

        # 우리는 자체 autoware 시계를 쓰므로 CARLA 시각 대신 그것으로 비교한다.
        carla_timestamp = self._aw_time.sec + self._aw_time.nanosec * 1e-9
        # 허용 오차를 약간 크게(0.051초): 콜백 실행 타이밍의 지터 때문에
        # 드물게 정상 명령이 거부되어 시뮬레이션이 더듬거리는 것을 방지.
        if abs(control_timestamp - carla_timestamp) > 0.051:
            print(
                "\033[93mWARNING2222: Expecting a vehicle command with timestamp {} but the timestamp received was {} .\033[0m".format(carla_timestamp, control_timestamp),
                "\033[93mThis vehicle command will be ignored.\033[0m",
                sep=" ")
            return

        self._last_control_timestamp = control_timestamp
        try:
            self._control_queue.put_nowait((control_timestamp, control))
        except queue.Full:
            print(
                "\033[93mWARNING3333: A new vehicle command has been received while the previous one has not been yet processed.\033[0m",
                "\033[93mThis vehicle command will be ignored.\033[0m",
                sep=" ")

    def _vehicle_control_cmd_callback2(self, aw_ackermann_control_command_msg):
        # (미사용 대안) Ackermann 방식 제어 변환 콜백 — 현재 구독에 연결 안 됨.
        carla_ackermann_control = AckermannDrive()
        carla_ackermann_control.steering_angle = aw_ackermann_control_command_msg.lateral.steering_tire_angle * 1.2
        carla_ackermann_control.steering_angle_velocity = aw_ackermann_control_command_msg.lateral.steering_tire_rotation_rate * 1.2
        carla_ackermann_control.speed = aw_ackermann_control_command_msg.longitudinal.speed
        carla_ackermann_control.acceleration = aw_ackermann_control_command_msg.longitudinal.acceleration
        carla_ackermann_control.jerk = aw_ackermann_control_command_msg.longitudinal.jerk
        self._control_publiher.publish(carla_ackermann_control)

    def _vehicle_control_cmd_callback3(self, control_msg):
        # (미사용 대안) 조향/스로틀/브레이크를 그대로 전달하는 콜백 — 연결 안 됨.

        control_timestamp, control = CarlaDataProvider.get_world().get_snapshot().timestamp.elapsed_seconds, carla.VehicleControl(
            steer = control_msg.steer,
            throttle = control_msg.throttle,
            brake = control_msg.brake,
        )


        # 중복 타임스탬프 검사 (위 콜백과 동일한 로직)
        if self._last_control_timestamp is not None and abs(self._last_control_timestamp - control_timestamp) < EPSILON:
            print(
                "\033[93mWARNING 11111: A new vehicle command with a repeated timestamp has been received {} .\033[0m".format(control_timestamp),
                "\033[93mThis vehicle command will be ignored.\033[0m",
                sep=" ")
            return

        # 시각 일치 검사: CARLA 스냅샷 시각과 비교 (race condition 회피 목적)
        carla_timestamp = CarlaDataProvider.get_world().get_snapshot().timestamp.elapsed_seconds
        if abs(control_timestamp - carla_timestamp) > EPSILON:
            print(
                "\033[93mWARNING2222: Expecting a vehicle command with timestamp {} but the timestamp received was {} .\033[0m".format(carla_timestamp, control_timestamp),
                "\033[93mThis vehicle command will be ignored.\033[0m",
                sep=" ")
            return

        self._last_control_timestamp = control_timestamp
        try:
            self._control_queue.put_nowait((control_timestamp, control))
        except queue.Full:
            print(
                "\033[93mWARNING3333: A new vehicle command has been received while the previous one has not been yet processed.\033[0m",
                "\033[93mThis vehicle command will be ignored.\033[0m",
                sep=" ")

    def _acc_callback(self, acc_msg):
        # Autoware가 추정한 가속도를 converter에 전달 (제어 변환에 사용)
        self._awp_converter.getAcc(acc_msg)

    def _aw_time_callback(self, clock_msg):
        # Autoware 시계 갱신 — run_step은 이 시각으로 메시지에 도장을 찍는다
        self._aw_time = clock_msg.clock

    def _operation_mode_callback(self, operation_mode_msg):
        # Autoware의 운행 모드 변화 감지. AUTONOMOUS = 자율주행 중.
        # 이 플래그가 고장 주입 타이머의 시작 조건 중 하나다 (run_step 참고).
        if operation_mode_msg.mode == OperationModeState.AUTONOMOUS:
            self._is_autonomous = True
        else:
            self._is_autonomous = False

    def _get_traffic_lights_from_lanelet(self, lanelet_marker_msg):
        # 지도(lanelet) 시각화 마커의 텍스트에서 신호등 규제 ID를 정규식으로
        # 긁어모은다. V2X 치트 모드에서 "모든 신호등"의 ID 목록으로 사용됨.
        print("Fetch Traffic Lights from Lanelet")
        pattern = re.compile(r"TLRegElemId:(\d+)")
        for marker in lanelet_marker_msg.markers:
            if marker.text:
                match = pattern.search(marker.text)
                if match:
                    self._traffic_light_ids.add(int(match.group(1)))

    def destroy(self):
        """
        시나리오 종료 시 리더보드가 호출하는 정리(clean-up) 함수.
        ROS 노드를 내리고 spin 스레드를 끝낸 뒤 부모 정리로 넘어간다.
        """
        self.ros_node.destroy_node()
        rclpy.shutdown()
        self.spin_thread.join()

        super(AutowareE2EAgent, self).destroy()
