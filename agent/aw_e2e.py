#!/usr/bin/env python

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides a ROS autonomous agent interface to control the ego vehicle via a ROS2 stack
"""


from __future__ import print_function
import os
import time
import threading
import queue
import numpy as np
import math
import re

import carla

from leaderboard.utils.route_manipulation import downsample_route
from leaderboard.autoagents.autonomous_agent import Track
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.task import Future

from ackermann_msgs.msg import AckermannDrive
from diagnostic_msgs.msg import KeyValue, DiagnosticStatus
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3, TransformStamped, AccelWithCovarianceStamped, PoseStamped, PoseWithCovarianceStamped, TwistWithCovarianceStamped
from sensor_msgs.msg import NavSatFix, Imu, PointCloud2, Image, CameraInfo
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from visualization_msgs.msg import MarkerArray

from autoware_agent.tum_ros_base_agent import TUMROSBaseAgent, BridgeHelper
from autoware_agent.aw_converter import AutowareConverter
from autoware_adapi_v1_msgs.msg import LocalizationInitializationState, RouteState, OperationModeState
from autoware_vehicle_msgs.msg import SteeringReport, VelocityReport, Engage, ControlModeReport, GearReport, GearCommand
from autoware_control_msgs.msg import Control
from autoware_perception_msgs.msg import PredictedObjects, TrafficLightGroupArray, TrafficLightGroup, TrafficLightElement
from tier4_vehicle_msgs.msg import ActuationStatusStamped
from autoware_adapi_v1_msgs.srv import SetRoutePoints
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as RclParameter, ParameterValue, ParameterType

def get_entry_point():
    return 'AutowareE2EAgent'

EPSILON = 0.001

def wait_for_message(node, topic, topic_type, timeout=None):

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
        super(AutowareE2EAgent, self).__init__(self.ROS_VERSION, carla_host, carla_port, debug)
        rclpy.init(args=None)
        self.ros_node = rclpy.create_node("autoware_e2e_node")

        # --Beginning of Autoware Stuff--
        # Timing and Synchronization
        self._aw_time_sub = self.ros_node.create_subscription(Clock, "/autoware_time", self._aw_time_callback, qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        self._ready_state_pub = self.ros_node.create_publisher(DiagnosticStatus, "/ready_state", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # Lidar Publisher using best effort
        self._localization_acceleration_subscriber = self.ros_node.create_subscription(AccelWithCovarianceStamped, "/localization/acceleration", self._acc_callback, qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        self._lidar_pub_front = self.ros_node.create_publisher(PointCloud2, "/sensor/lidar/front", qos_profile=qos_profile_sensor_data)
        self._lidar_pub_rear = self.ros_node.create_publisher(PointCloud2, "/sensor/lidar/rear", qos_profile=qos_profile_sensor_data)
        # Camera Publisher
        self._camera_pub = self.ros_node.create_publisher(Image, "/sensing/camera/traffic_light/image_raw", qos_profile=qos_profile_sensor_data)
        self._camera_info_pub = self.ros_node.create_publisher(CameraInfo, "/sensing/camera/traffic_light/camera_info", qos_profile=qos_profile_sensor_data)
        # Subscriber for Autoware Priviliged
        self._clock_pub = self.ros_node.create_publisher(Clock, "/clock", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        # Publisher for Localization
        self._localization_publisher = self.ros_node.create_publisher(Odometry, "/localization/kinematic_state", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        self._tf_publisher = self.ros_node.create_publisher(TFMessage, "/tf", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        #self._localization_init_state_publisher = self.ros_node.create_publisher(LocalizationInitializationState, "/localization/initialization_state", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._localization_acceleration_publisher = self.ros_node.create_publisher(AccelWithCovarianceStamped, "/localization/acceleration", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        # Native Loc
        self._imu_pub = self.ros_node.create_publisher(Imu, "/sensing/imu/imu_data", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        self._ndt_pose_pub = self.ros_node.create_publisher(PoseWithCovarianceStamped, "/localization/pose_estimator/pose_with_covariance", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        #self._vel_pub = self.ros_node.create_publisher(TwistWithCovarianceStamped, "/sensing/vehicle_velocity_converter/twist_with_covariance", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        self._init_pose_pub = self.ros_node.create_publisher(PoseWithCovarianceStamped, "/initialpose", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        self._gnss_pose_pub = self.ros_node.create_publisher(PoseWithCovarianceStamped, "/sensing/gnss/pose_with_covariance", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        # Publisher for Vehicle Infos
        self._steering_report_publisher = self.ros_node.create_publisher(SteeringReport, "/vehicle/status/steering_status", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        self._velocity_report_publisher = self.ros_node.create_publisher(VelocityReport, "/vehicle/status/velocity_status", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        self._control_mode_report_publisher = self.ros_node.create_publisher(ControlModeReport, "/vehicle/status/control_mode", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        # 기어 루프백: Autoware가 후진 궤적 실행 전 gear_status로 R 확인을
        # 기다림 — CARLA엔 기어 개념이 없어(reverse 플래그) 명령을 즉시 반향
        self._gear_report_publisher = self.ros_node.create_publisher(GearReport, "/vehicle/status/gear_status", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        self._gear_cmd_value = 2  # DRIVE
        self._gear_cmd_subscriber = self.ros_node.create_subscription(GearCommand, "/control/command/gear_cmd", self._gear_cmd_callback, qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        #self._actuation_report_publisher = self.ros_node.create_publisher(ActuationStatusStamped, "/vehicle/status/actuation_status", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        # Publisher & Subscriber for Goal Planning
        self._goal_publisher = self.ros_node.create_publisher(PoseStamped, "/rviz/routing/rough_goal", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._engage_publisher = self.ros_node.create_publisher(Engage, "/autoware/engage", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        self._fault_active_pub = self.ros_node.create_publisher(Bool, "/fault_injection/active", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._autoware_mode_subscriber = self.ros_node.create_subscription(OperationModeState, "/system/operation_mode/state", self._operation_mode_callback, qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        # Services for Goal Publishing
        self._route_service_client = self.ros_node.create_client(SetRoutePoints, "/api/routing/change_route_points")
        # MRM supervisor owns the route while an MRM is active — stand down then
        self._mrm_active = False
        self._mrm_state_subscriber = self.ros_node.create_subscription(
            String, "/mrm_supervisor/state", self._mrm_state_callback,
            qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        
        # Subscriber for Control
        self._control_subscriber = self.ros_node.create_subscription(Control, "/control/command/control_cmd", self._vehicle_control_cmd_callback, qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))

        self._object_publisher = self.ros_node.create_publisher(PredictedObjects, "/perception/object_recognition/objects", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))

        # V2X Traffic Light
        self._lanelet_map_subscriber = self.ros_node.create_subscription(MarkerArray, "/map/vector_map_marker", self._get_traffic_lights_from_lanelet, qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # [v4.6f] QoSProfile(depth=1, durability=VOLATILE) 부분지정 구성에서 이
        # publisher만 wire 무발신(동일 노드 타 토픽 정상, 컨테이너 rclpy 자가
        # 테스트 정상, 그래프엔 등록 — 0812 새벽 실측). 기본 QoS로 교체.
        self._external_traffic_light_publisher = self.ros_node.create_publisher(TrafficLightGroupArray, "/perception/traffic_light_recognition/traffic_signals", 10)
        # [v4.6e] arbiter 정식 V2X 입력(external) — 직접 발행은 arbiter의 judged
        # 출력과 경합해 신호가 됐다 안 됐다 한다(0812 nf 교차로 스턱 실측).
        # arbiter가 camera(internal)와 병합하도록 external에도 같은 메시지 발행.
        self._external_v2x_arbiter_publisher = self.ros_node.create_publisher(TrafficLightGroupArray, "/perception/traffic_light_recognition/external/traffic_signals", 10)

        # Sync stuff
        self._aw_time = None
        self.ready_status = DiagnosticStatus()
        self.ready_status.level = DiagnosticStatus.OK
        self.ready_status.name = "AutowareE2EAgent"

        # --End of Autoware Stuff--
        self._client = None
        self._vehicle = None
        self._world = None
        self._map = None

        self._camera_height = None
        self._camera_width = None
        self._camera_fov = None

        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self.ros_node,))
        self.spin_thread.start()

    def setup(self, path_to_conf_file):
        self.track = Track.MAP
        
        # Autoware Priviliged Stuff
        self._client = CarlaDataProvider.get_client()
        self._vehicle = CarlaDataProvider.get_hero_actor()
        self._world = self._vehicle.get_world()
        self._map = self._world.get_map()
        self._awp_converter = AutowareConverter(self._vehicle, self._world)
        self._last_control = carla.VehicleControl(throttle=0.0)
        self._route_index = 1 # Start with 1, because 0 is the start point
        self._last_published_route_index = None
        self._published_latest = False
        self._goal_mod_failure_counter = 0
        self._is_autonomous = False
        self._pending_service_call = False
        self._route_extended = False
        # One-shot full-route setup (avoids mid-drive "cannot reroute")
        self._bootstrap_goal_sent = False
        self._route_sent = False
        # Fault injection (research). All disabled unless given:
        #   cf=20       destroy camera 20 s in (image freezes downstream)
        #   cn=20:15    respawn camera at 20 s with gaussian pixel noise (sigma 15 on 0-255)
        #   ln=20:0.1   respawn lidars at 20 s with CARLA-native noise_stddev=0.1 m
        #   imb=20:0.02 constant gyro z (yaw-rate) bias of 0.02 rad/s from 20 s on
        #   rain=20     switch the CARLA weather to a heavy downpour at 20 s
        #   gb=20:30    GNSS blackout: stop the GNSS pose publish for 30 s
        #               (duration omitted = blackout forever)
        # Runtime override:
        #   ros2 param set /autoware_e2e_node cf true
        #   ros2 param set /autoware_e2e_node cn "0:15"      (0 = trigger now)
        cf_env = os.environ.get("cf", "")
        try:
            cf_default = bool(cf_env)
            cf_delay_default = float(cf_env) if cf_env else 20.0
        except ValueError:
            print(f"[FAULT INJECTION] invalid cf value {cf_env!r} — fault injection disabled")
            cf_default, cf_delay_default = False, 20.0
        if not self.ros_node.has_parameter("cf"):
            self.ros_node.declare_parameter("cf", cf_default)
            self.ros_node.declare_parameter("cf_delay", cf_delay_default)
            self.ros_node.declare_parameter("cn", os.environ.get("cn", ""))
            # lf=<t>: destroy the FRONT lidar actor at t (hardware-dead case)
            self.ros_node.declare_parameter("lf", os.environ.get("lf", ""))
            self.ros_node.declare_parameter("ln", os.environ.get("ln", ""))
            self.ros_node.declare_parameter("lnf", os.environ.get("lnf", ""))
            self.ros_node.declare_parameter("lnr", os.environ.get("lnr", ""))
            # lso=<t>:<ratio>:<width_deg>: 전방 라이다 오염 섹터 드롭아웃 (기본 0.6:90)
            self.ros_node.declare_parameter("lso", os.environ.get("lso", ""))
            # lcl=<t>:<clutter_n>:<far_ratio>: 강수 근거리 클러터+원거리 감쇠 (기본 1500:0.15)
            self.ros_node.declare_parameter("lcl", os.environ.get("lcl", ""))
            # gm=<t>:<sigma_m>:<tau_s>: GNSS 1차 Gauss-Markov 상관편향 (멀티패스/
            # NLOS 모사 — 백색 gn과 달리 EKF가 믿고 따라가는 최악 케이스)
            self.ros_node.declare_parameter("gm", os.environ.get("gm", ""))
            self.ros_node.declare_parameter("imn", os.environ.get("imn", ""))
            self.ros_node.declare_parameter("gn", os.environ.get("gn", ""))
            # imb=<t>:<bias_rad_s>: constant gyro z bias (see run_step)
            self.ros_node.declare_parameter("imb", os.environ.get("imb", ""))
            # imd=<t>:<slope_rad_s2>: gyro z bias RAMP (드리프트, see run_step)
            self.ros_node.declare_parameter("imd", os.environ.get("imd", ""))
            # rain=<t>: heavy-rain weather preset (lidar/camera degrade physically)
            self.ros_node.declare_parameter("rain", os.environ.get("rain", ""))
            # gb=<t>[:<dur_s>]: GNSS blackout (publish gated in this agent)
            self.ros_node.declare_parameter("gb", os.environ.get("gb", ""))
            # cfl=<t>: 근접 TTC ablation용 복합 고장 — t초에 선행차 급제동,
            # t+0.8초에 카메라 파괴. TTC가 감지예산(850ms) 아래로 붕괴하는
            # 순간의 침묵 고장 감지를 고정/동적 데드라인으로 비교한다.
            self.ros_node.declare_parameter("cfl", os.environ.get("cfl", ""))
            # cfl2=<t>: cfl 개선판 — 카메라 파괴를 고정 지연이 아니라
            # "실측 TTC < 0.9s" 순간에 트리거 (동적 감지예산 이득 분리용)
            self.ros_node.declare_parameter("cfl2", os.environ.get("cfl2", ""))
            # lfl2=<t>: 선행차 급제동 + TTC<1.4s 순간 "전방 라이다" 파괴.
            # 장애물 추적이 사라져 감지->DEFER 제동이 유일한 브레이크가 되는
            # 칼날 시나리오 — 고정/동적 감지예산의 결과(충돌 여부) 차이 시험
            self.ros_node.declare_parameter("lfl2", os.environ.get("lfl2", ""))
            # tail=<gap_m>: guarantee an NPC tailing the ego at <gap_m> meters
            # (background traffic spawns mostly ahead — rear-end pressure for
            # in-lane MRM stops needs an explicit chaser)
            self.ros_node.declare_parameter("tail", os.environ.get("tail", ""))
            # clearnpc=1: 배경 NPC 차량 전부 제거(우측 실주차장 데모 —
            # 정차 ego 주변에 교통이 몰려 freespace 경로를 막는 것 방지)
            self.ros_node.declare_parameter("clearnpc", os.environ.get("clearnpc", ""))
            # npccap=<N>: 배경 NPC를 N대로 상한(주기 컬링, actor id 순 유지).
            # npccap=0 은 clearnpc 와 동일 — 트래픽 밀도 차원(none/low) 실험용
            self.ros_node.declare_parameter("npccap", os.environ.get("npccap", ""))
            # npcadd=<N>: 배경 NPC N대 추가 스폰(TM_SEED 결정적) — 밀도 high 실험용
            self.ros_node.declare_parameter("npcadd", os.environ.get("npcadd", ""))
        self._fault_t0 = None
        self._camera_killed = False
        self._lidar_killed = False
        self._tail_spawned = False
        self._npc_clear_tick = 0
        self._npc_cap_tick = 0
        self._npc_added = False
        self._tail_actor = None
        self._fault_announced = False
        self._bridge_noise_started = set()
        # npc=agg: make background traffic aggressive via the TrafficManager
        self._npc_styled = False
        self._imu_noise_sigma = 0.0
        self._gnss_noise_sigma = 0.0
        self._gm_sigma = 0.0        # [gm] 정상상태 편향 크기 [m] (0=비활성)
        self._gm_tau = 30.0         # [gm] 상관시간 [s]
        self._gm_bias = [0.0, 0.0]  # [gm] 현재 편향 상태 (x, y)
        self._imu_gyro_bias = 0.0
        self._imu_gyro_drift = 0.0
        self._rain_applied = False
        self._gnss_blackout = False
        self._gb_bridge_sent = False
        self._cfl_lead = None       # 선행 NPC 액터
        self._cfl_braked = False
        self._cfl_cam_killed = False

        # V2X sens gt traffic light states
        # Either the camera image or the v2x need to be set true
        # For visualisation you can also set both to true 
        # 신호등 소스 선택: 카메라 계열 fault(cf/cn/cfl/cfl2/mix) 케이스는
        # 고장이 신호 인식에도 미치는 게 실험 취지이므로 카메라 인식 유지.
        # 그 외(gn/ln/imn/nf 등) 케이스는 신호등이 실험 변인이 아니므로 V2X로
        # 받아 "초록불 인식 미스 → 교차로 프리징" 무관 노이즈를 제거 (8/11).
        _cam_fault = any(os.environ.get(k) for k in ("cf", "cn", "cfl", "cfl2", "mix"))
        self._use_v2x_traffic_light = not _cam_fault
        self._publish_cam_image = True
        print(f"[TL SOURCE] {'camera (camera-fault case)' if _cam_fault else 'V2X ground truth'}")
        self._traffic_light_ids = set()
        # [v4.6d] 마커 스크레이프는 발행/수신 레이스로 ID가 새는 수가 있다
        # (0812 실측: 9416 누락 -> "signal data not received" -> nf 교차로 영구
        # 대기). lanelet2 osm에서 traffic_light 규제요소를 직접 파싱해 합집합.
        try:
            import xml.etree.ElementTree as _ET
            # repo 마운트 사본을 1순위로 (awcl_maps 마운트는 agent 프로세스에서
            # errno2가 나는 미스터리 — 0812 실측. 맵 수정 시 사본 cp 동기화 필수)
            _map_osm = os.environ.get("LL2_MAP_OSM", "")
            if not _map_osm:
                for _cand in (
                        "/home/taewook/autoware_carla_leaderboard/experiments/"
                        "results/lanelet2_map_for_snapshot.osm",
                        "/home/taewook/awcl_maps/Town07/lanelet2_map.osm"):
                    if os.path.exists(_cand):
                        _map_osm = _cand
                        break
            for _rel in _ET.parse(_map_osm).getroot().findall("relation"):
                _tags = {t.get("k"): t.get("v") for t in _rel.findall("tag")}
                if (_tags.get("type") == "regulatory_element"
                        and _tags.get("subtype") == "traffic_light"):
                    self._traffic_light_ids.add(int(_rel.get("id")))
            print(f"[TL SOURCE] lanelet2 osm: "
                  f"{len(self._traffic_light_ids)} traffic-light reg elems")
        except Exception as _e:
            print(f"[TL SOURCE] osm parse failed ({_e}) — marker scrape only")

        self._steps_before_init = 0
        self._counter = 0

        self._time_to_wait_for_control = 99.0
    
    def sensors(self) -> list:
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

    # lidar sector gates (deg): center azimuth / width in the sensor frame
    _NOISE_SECTORS = {"R": (-90.0, 180.0), "L": (90.0, 180.0),
                      "F": (0.0, 180.0), "B": (180.0, 180.0)}

    @staticmethod
    def _parse_fault(spec, default_mag):
        # "20:0.1"   -> (20.0, 0.1, None)   whole sensor
        # "20:0.1:R" -> (20.0, 0.1, "R")    right half only (lidar)
        # "20"       -> (20.0, default_mag, None); "" -> None
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
        # Noise is injected inside the cpp bridge nodes (this CARLA server build
        # hangs on runtime sensor spawn, so the simulator is left untouched).
        # Flip each bridge's noise_sigma parameter via its parameter service.
        for name in node_names:
            try:
                cli = self.ros_node.create_client(SetParameters, f"/{name}/set_parameters")
                req = SetParameters.Request()
                params = []
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
                cli.call_async(req)
                import time as _time
                print(f"[FAULT INJECTION] noise_sigma={sigma}"
                      f"{f' sector={sector}' if sector else ''} requested on /{name}"
                      f" (wall {_time.strftime('%H:%M:%S')})")
            except Exception as e:
                print(f"[FAULT INJECTION] noise request on /{name} failed:", e)

    def _set_bridge_params(self, node_names, values):
        # [v4.6] lso/lcl 등 임의 브리지 파라미터 주입 (noise_sigma 전용이던
        # _set_bridge_noise의 일반화 버전)
        for name in node_names:
            try:
                cli = self.ros_node.create_client(SetParameters, f"/{name}/set_parameters")
                req = SetParameters.Request()
                ps = []
                for k, v in values.items():
                    if isinstance(v, int):
                        pv = ParameterValue(type=ParameterType.PARAMETER_INTEGER,
                                            integer_value=v)
                    else:
                        pv = ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                            double_value=float(v))
                    ps.append(RclParameter(name=k, value=pv))
                req.parameters = ps
                cli.call_async(req)
                print(f"[FAULT INJECTION] {values} requested on /{name}")
            except Exception as e:
                print(f"[FAULT INJECTION] param request on /{name} failed:", e)

    # After initial initialization the run step method should
    # wait for the next control signal

    def run_step(self, input_data, timestamp):
        timestamp = carla_timestamp = CarlaDataProvider.get_world().get_snapshot().timestamp.elapsed_seconds
        print('COUNTER', self._counter)
        self._counter += 1

        # --- Fault injection (research) ---
        # t counts from the moment the ego actually STARTS DRIVING (engaged
        # AND moving): counting from engagement made t elapse while waiting
        # at a red light, injecting faults into a parked car.
        if self._fault_t0 is None:
            moving = False
            if self._is_autonomous:
                try:
                    ego = CarlaDataProvider.get_hero_actor()
                    moving = ego is not None and ego.get_velocity().length() > 1.0
                except Exception:
                    pass
            if moving:
                self._fault_t0 = timestamp
                print(f"[FAULT INJECTION] timer armed at sim {timestamp:.1f}s (driving)")
            fault_elapsed = -1.0
        else:
            fault_elapsed = timestamp - self._fault_t0

        # npc=agg: aggressive background traffic (applied once, after spawns)
        if not self._npc_styled and self._counter > 40 and os.environ.get("npc", "") == "agg":
            self._npc_styled = True
            try:
                client = CarlaDataProvider.get_client()
                tm = client.get_trafficmanager(CarlaDataProvider.get_traffic_manager_port())
                styled = 0
                # [v4.6i] 차량별 랜덤 부주의: TM 차량은 전지적이라 "시야 제한"은
                # 불가 — ignore 확률로 사람의 부주의를 분포로 흉내낸다.
                # TM_SEED 시드 고정 -> ours/baseline 짝케이스가 같은 성격 분포.
                import random as _rnd
                _r = _rnd.Random(os.environ.get("TM_SEED", "0"))
                for v in CarlaDataProvider.get_world().get_actors().filter("vehicle.*"):
                    if v.attributes.get("role_name") in ("hero", "ego_vehicle"):
                        continue
                    tm.vehicle_percentage_speed_difference(v, -_r.uniform(5.0, 30.0))
                    tm.distance_to_leading_vehicle(v, 1.2)            # tailgating (0.5 pinned stopped egos)
                    tm.ignore_vehicles_percentage(v, _r.uniform(0.0, 40.0))
                    tm.ignore_lights_percentage(v, _r.uniform(0.0, 20.0))
                    styled += 1
                print(f"[NPC] aggressive style applied to {styled} vehicles")
            except Exception as e:
                print("[NPC] aggressive style failed:", e)

        # cf: destroy the camera entirely (image freezes downstream)
        if not self._camera_killed and self.ros_node.get_parameter("cf").value:
            fault_camera_delay = float(self.ros_node.get_parameter("cf_delay").value)
            if fault_elapsed >= fault_camera_delay:
                try:
                    world = CarlaDataProvider.get_world()
                    killed = 0
                    for s in world.get_actors().filter('sensor.camera.rgb'):
                        # stop() 먼저: destroy는 서버측 적용이 지연돼 ~1.2 s
                        # 유령 프레임이 남는다(0822 원장 실측) — 실제 센서 사망은
                        # 즉시 침묵이므로 스트림부터 끊는다
                        try: s.stop()
                        except Exception: pass
                        s.destroy(); killed += 1
                    print(f"[FAULT INJECTION] destroyed {killed} camera sensor(s) at t+{fault_camera_delay:.0f}s")
                except Exception as e:
                    print("[FAULT INJECTION] camera destroy failed:", e)
                self._camera_killed = True

        # clearnpc=1: 배경 NPC 차량을 주기적으로 전부 제거(우측 실주차장 데모).
        # 앞마당 주차 차량들은 정적 메시라 영향 없음 — 움직이는 교통만 사라짐.
        try:
            if (self.ros_node.get_parameter("clearnpc").value
                    and self._counter - self._npc_clear_tick >= 5):
                self._npc_clear_tick = self._counter
                world = CarlaDataProvider.get_world()
                killed = 0
                for v in world.get_actors().filter("vehicle.*"):
                    if v.attributes.get("role_name") in ("hero", "ego_vehicle"):
                        continue
                    try:
                        v.destroy(); killed += 1
                    except Exception:
                        pass
                if killed:
                    print(f"[DEMO] cleared {killed} background NPC vehicle(s)")
        except Exception:
            pass

        # npccap=<N>: 배경 NPC를 N대까지만 유지. 스폰 순서(actor id 오름차순)로
        # 남길 차를 고정해 rep 간 재현성 유지. tail_npc 는 별도 축이므로 제외.
        try:
            capv = self.ros_node.get_parameter("npccap").value
            if capv != "" and self._counter - self._npc_cap_tick >= 5:
                self._npc_cap_tick = self._counter
                cap = int(capv)
                world = CarlaDataProvider.get_world()
                background = sorted(
                    (v for v in world.get_actors().filter("vehicle.*")
                     if v.attributes.get("role_name")
                     not in ("hero", "ego_vehicle", "tail_npc")),
                    key=lambda v: v.id)
                killed = 0
                for v in background[cap:]:
                    try:
                        v.destroy(); killed += 1
                    except Exception:
                        pass
                if killed:
                    print(f"[NPC] capped background traffic to {cap} "
                          f"(destroyed {killed})")
        except Exception:
            pass

        # npcadd=<N>: 스폰 완료 후 1회, 오토파일럿 배경차 N대 추가(밀도 high).
        # TM_SEED 로 스폰 지점/차종을 고정해 rep 간 재현성 유지.
        if (not self._npc_added and self._counter > 60
                and self.ros_node.get_parameter("npcadd").value):
            self._npc_added = True
            try:
                import random as _random
                n = int(self.ros_node.get_parameter("npcadd").value)
                world = CarlaDataProvider.get_world()
                tm_port = CarlaDataProvider.get_traffic_manager_port()
                lib = [bp for bp in world.get_blueprint_library().filter("vehicle.*")
                       if int(bp.get_attribute("number_of_wheels")) == 4]
                sps = world.get_map().get_spawn_points()
                rng = _random.Random(int(os.environ.get("TM_SEED", "100")))
                rng.shuffle(sps)
                added = 0
                for tf in sps:
                    if added >= n:
                        break
                    bp = rng.choice(lib)
                    bp.set_attribute("role_name", "extra_npc")
                    v = world.try_spawn_actor(bp, tf)   # 점유된 지점은 None -> skip
                    if v is not None:
                        v.set_autopilot(True, tm_port)
                        added += 1
                print(f"[NPC] spawned {added}/{n} extra background vehicle(s)")
            except Exception as e:
                print("[NPC] extra spawn failed:", e)

        # tail=<gap_m>: keep a chasing NPC behind the ego for the WHOLE case.
        # A one-shot spawn drifts away (the TM picks its own turns at
        # junctions) — re-check every ~5 s and respawn when lost/far.
        # Spawn only AFTER the ego starts driving (_fault_t0 arms then): an
        # aggressive tail (0.5 m gap) parked on the bumper of the not-yet-
        # departed ego blocks departure entirely (AgentBlockedTest failures).
        if (self._fault_t0 is not None and self._counter % 100 == 0
                and self.ros_node.get_parameter("tail").value):
            try:
                gap = float(self.ros_node.get_parameter("tail").value)
                world = CarlaDataProvider.get_world()
                ego = CarlaDataProvider.get_hero_actor()
                tail_ok = False
                if self._tail_actor is not None:
                    try:
                        tail_ok = (self._tail_actor.is_alive and
                                   self._tail_actor.get_location().distance(
                                       ego.get_location()) < 60.0)
                    except Exception:
                        tail_ok = False
                # [주차 실험] def(비공격) 후행차는 주차장 부근(35 m)에서 리스폰
                # 금지 — 후진주차 코앞 리스폰이 기동을 영구 봉쇄(8/11 실측).
                # agg 는 그대로: 공격 교통이 공간을 안 내주는 조건 자체가 실험
                # 변인이고, 그 경우 supervisor 가 노측으로 폴백한다.
                _aggressive = os.environ.get("npc", "") == "agg"
                _park_anchors = ((-22.6, -8.2), (-17.0, -14.0),
                                 (-51.8, -69.2), (-148.0, -98.2),
                                 (-147.5, -91.9), (-140.0, -110.0))
                _el = ego.get_location()
                _near_lot = any(math.hypot(_el.x - ax, _el.y - ay) < 35.0
                                for ax, ay in _park_anchors)
                if not tail_ok and not (_near_lot and not _aggressive):
                    if self._tail_actor is not None:
                        try:
                            self._tail_actor.destroy()
                        except Exception:
                            pass
                        self._tail_actor = None
                    wp = CarlaDataProvider.get_map().get_waypoint(ego.get_location())
                    prev = wp.previous(gap)
                    if prev:
                        tf = prev[0].transform
                        tf.location.z += 0.5
                        bp = world.get_blueprint_library().find("vehicle.audi.tt")
                        bp.set_attribute("role_name", "tail_npc")
                        npc = world.try_spawn_actor(bp, tf)
                        if npc is not None:
                            client = CarlaDataProvider.get_client()
                            tm_port = CarlaDataProvider.get_traffic_manager_port()
                            tm = client.get_trafficmanager(tm_port)
                            npc.set_autopilot(True, tm_port)
                            aggressive = os.environ.get("npc", "") == "agg"
                            # 0.5 m glued to the bumper at every stop and the
                            # ego could never resume -> permanent pin. 1.5 m
                            # still threatens but allows restart.
                            # def 는 여유롭게 따라오는 교통(7 m, 제한속도 -15%):
                            # 후진주차에 공간을 내준다. agg 만 밀착 유지.
                            tm.distance_to_leading_vehicle(npc, 1.5 if aggressive else 7.0)
                            tm.vehicle_percentage_speed_difference(
                                npc, -30.0 if aggressive else 15.0)
                            tm.ignore_lights_percentage(npc, 30.0 if aggressive else 0.0)
                            self._tail_actor = npc
                            print(f"[TAIL NPC] (re)spawned {gap:.0f}m behind ego"
                                  f" (aggressive={aggressive})")
            except Exception as e:
                print("[TAIL NPC] ensure failed:", e)

        # lf=<t>: destroy the FRONT lidar actor — topic goes silent, the
        # monitor's timeout checker (not the noise checker) must catch it
        if not self._lidar_killed:
            spec = self._parse_fault(self.ros_node.get_parameter("lf").value, 0.0)
            if spec and fault_elapsed >= spec[0]:
                try:
                    world = CarlaDataProvider.get_world()
                    killed = 0
                    for s in world.get_actors().filter('sensor.lidar.ray_cast'):
                        tag = (s.attributes.get("role_name", "")
                               + s.attributes.get("ros_name", "")).lower()
                        if "front" in tag:
                            try: s.stop()
                            except Exception: pass
                            s.destroy(); killed += 1
                    print(f"[FAULT INJECTION] destroyed {killed} front lidar(s) at t+{spec[0]:.0f}s")
                except Exception as e:
                    print("[FAULT INJECTION] lidar destroy failed:", e)
                self._lidar_killed = True

        # cn=<t>:<sigma>: gaussian pixel noise, injected in camera_bridge
        # ln/lnf/lnr=<t>:<sigma>: radial distance noise in both/front/rear lidar_bridges
        bridge_faults = [
            ("cn", 15.0, ["camera_bridge_1"]),
            ("ln", 0.1, ["lidar_bridge_1", "lidar_bridge_2"]),
            ("lnf", 0.1, ["lidar_bridge_1"]),
            ("lnr", 0.1, ["lidar_bridge_2"]),
        ]
        for flag, default_sigma, nodes in bridge_faults:
            if flag in self._bridge_noise_started:
                continue
            spec = self._parse_fault(self.ros_node.get_parameter(flag).value, default_sigma)
            if spec and fault_elapsed >= spec[0]:
                self._set_bridge_noise(nodes, spec[1], spec[2])
                self._bridge_noise_started.add(flag)

        # [v4.6] lso=<t>:<T>:<width_deg> — 전방 라이다 오염 섹터 (커버 투과율 T,
        # Trierweiler 2019 실측 + Goodin 검출문턱; 전방 센서만)
        if "lso" not in self._bridge_noise_started:
            raw = str(self.ros_node.get_parameter("lso").value or "")
            if raw:
                parts = raw.split(":")
                try:
                    t0 = float(parts[0])
                    trans = float(parts[1]) if len(parts) > 1 and parts[1] else 0.35
                    width = float(parts[2]) if len(parts) > 2 and parts[2] else 90.0
                    if fault_elapsed >= t0:
                        self._set_bridge_params(["lidar_bridge_1"], {
                            "cover_transmission": trans,
                            "dropout_yaw_center": 0.0,
                            "dropout_yaw_width": width})
                        self._bridge_noise_started.add("lso")
                except ValueError:
                    print(f"[FAULT INJECTION] invalid lso spec {raw!r} — ignored")

        # [v4.6] lcl=<t>:<rain_mmh>:<clutter_n> — 강수 (Beer-Lambert 감쇠
        # alpha=0.01*Rr^0.6 + Goodin 거리지터 + 근거리 클러터; 양쪽 라이다)
        if "lcl" not in self._bridge_noise_started:
            raw = str(self.ros_node.get_parameter("lcl").value or "")
            if raw:
                parts = raw.split(":")
                try:
                    t0 = float(parts[0])
                    rain = float(parts[1]) if len(parts) > 1 and parts[1] else 5.0
                    clutter_n = int(float(parts[2])) if len(parts) > 2 and parts[2] else 1500
                    if fault_elapsed >= t0:
                        self._set_bridge_params(
                            ["lidar_bridge_1", "lidar_bridge_2"], {
                                "rain_rate_mmh": rain,
                                "clutter_points": clutter_n,
                                "clutter_range_m": 10.0})
                        self._bridge_noise_started.add("lcl")
                except ValueError:
                    print(f"[FAULT INJECTION] invalid lcl spec {raw!r} — ignored")

        # imn=<t>:<sigma>: IMU noise (sigma rad/s on gyro, 10*sigma m/s^2 on accel)
        # gn=<t>:<sigma>: position noise (meters) on the pose fed to the EKF
        spec = self._parse_fault(self.ros_node.get_parameter("imn").value, 0.05)
        self._imu_noise_sigma = spec[1] if spec and fault_elapsed >= spec[0] else 0.0
        spec = self._parse_fault(self.ros_node.get_parameter("gn").value, 1.0)
        self._gnss_noise_sigma = spec[1] if spec and fault_elapsed >= spec[0] else 0.0
        # [gm] t:sigma:tau — _parse_fault의 3번째 필드(sector 자리)를 tau로 재활용
        gm_spec = self._parse_fault(self.ros_node.get_parameter("gm").value, 3.0)
        if gm_spec and fault_elapsed >= gm_spec[0]:
            self._gm_sigma = gm_spec[1]
            try:
                self._gm_tau = float(gm_spec[2]) if gm_spec[2] else 30.0
            except (TypeError, ValueError):
                self._gm_tau = 30.0
        else:
            self._gm_sigma = 0.0

        # imb=<t>:<bias_rad_s>: constant gyro z (yaw-rate) bias from t on.
        # White noise (imn) is zero-mean and gets absorbed by the EKF's
        # measurement update, but a constant bias is INTEGRATED into a
        # steadily growing yaw error — the dangerous real-world MEMS IMU
        # failure mode is bias instability, not white noise (Allan-variance
        # bias-instability term, IEEE Std 952 / El-Sheimy et al. 2008).
        spec = self._parse_fault(self.ros_node.get_parameter("imb").value, 0.02)
        self._imu_gyro_bias = spec[1] if spec and fault_elapsed >= spec[0] else 0.0

        # imd=<t>:<slope_rad_s2>: gyro z bias RAMP (드리프트). 계단형 imb와
        # 달리 바이어스 자체가 시간에 비례해 커진다 — Allan variance의 rate
        # random walk/온도 드리프트 근사. 감지 지연이 램프 기울기에 반비례하는
        # CUSUM 특성 곡선 검증용 (작을수록 늦게, 그러나 반드시 잡힘).
        spec = self._parse_fault(self.ros_node.get_parameter("imd").value, 0.005)
        if spec and fault_elapsed >= spec[0]:
            self._imu_gyro_drift = spec[1] * (fault_elapsed - spec[0])
        else:
            self._imu_gyro_drift = 0.0

        # rain=<t>: switch the CARLA weather to a heavy downpour at t.
        # Environmental (not component) degradation: rain physically
        # attenuates/scatters lidar returns and degrades the camera image,
        # so every exteroceptive sensor worsens at once. One-shot;
        # keeps the current sun position, only precipitation-related
        # fields are overridden (CARLA 0.9.16 carla.WeatherParameters).
        if not self._rain_applied:
            spec = self._parse_fault(self.ros_node.get_parameter("rain").value, 0.0)
            if spec and fault_elapsed >= spec[0]:
                try:
                    world = CarlaDataProvider.get_world()
                    weather = world.get_weather()
                    weather.cloudiness = 90.0
                    weather.precipitation = 90.0
                    weather.precipitation_deposits = 80.0
                    weather.wetness = 80.0
                    weather.fog_density = 20.0
                    world.set_weather(weather)
                    print(f"[FAULT INJECTION] heavy rain applied at t+{spec[0]:.0f}s")
                except Exception as e:
                    print("[FAULT INJECTION] weather change failed:", e)
                self._rain_applied = True

        # gb=<t>[:<dur_s>]: GNSS blackout — this agent publishes the GNSS
        # pose itself (see the GNSS-PosePub block below), so the blackout is
        # a publish gate right here, NOT a bridge noise parameter like gn.
        # The topic simply goes silent (tunnel / urban-canyon outage),
        # exercising the monitor's timeout path instead of its noise path.
        # Duration omitted -> blackout lasts for the rest of the run.
        spec = self._parse_fault(self.ros_node.get_parameter("gb").value, float("inf"))
        self._gnss_blackout = bool(spec) and spec[0] <= fault_elapsed < spec[0] + spec[1]
        # the cpp bridge publishes the SAME topic (second publisher), so the
        # local gate below is not enough — flip the bridge's gnss_mute too
        # (v2 battery finding: agent-only mute left EKF fully fed)
        if self._gnss_blackout != self._gb_bridge_sent:
            try:
                cli = self.ros_node.create_client(
                    SetParameters, "/aw_carla_bridge/set_parameters")
                req = SetParameters.Request()
                p = RclParameter(name="gnss_mute",
                                 value=ParameterValue(type=1, bool_value=self._gnss_blackout))
                req.parameters = [p]
                cli.call_async(req)
                self._gb_bridge_sent = self._gnss_blackout
                print(f"[FAULT INJECTION] bridge gnss_mute={self._gnss_blackout}")
            except Exception as e:
                print("[FAULT INJECTION] gnss_mute set failed:", e)

        # cfl: 선행차 스폰(타이머 무장 직후) -> t초 급제동 -> t+0.8s 카메라 파괴
        spec = self._parse_fault(self.ros_node.get_parameter("cfl").value, 0.0)
        if spec:
            try:
                world = CarlaDataProvider.get_world()
                ego = CarlaDataProvider.get_hero_actor()
                if self._cfl_lead is None and ego is not None and fault_elapsed >= 0.0:
                    tf = ego.get_transform()
                    fwd = tf.get_forward_vector()
                    sp = carla.Transform(
                        carla.Location(tf.location.x + fwd.x * 22.0,
                                       tf.location.y + fwd.y * 22.0,
                                       tf.location.z + 0.3), tf.rotation)
                    bp = world.get_blueprint_library().filter("vehicle.audi.tt")[0]
                    self._cfl_lead = world.try_spawn_actor(bp, sp)
                    if self._cfl_lead is not None:
                        self._cfl_lead.set_autopilot(True, CarlaDataProvider.get_traffic_manager_port())
                        tm = CarlaDataProvider.get_client().get_trafficmanager(
                            CarlaDataProvider.get_traffic_manager_port())
                        tm.vehicle_percentage_speed_difference(self._cfl_lead, 20.0)
                        print("[FAULT INJECTION] cfl lead spawned 22m ahead")
                if (self._cfl_lead is not None and not self._cfl_braked
                        and fault_elapsed >= spec[0]):
                    self._cfl_lead.set_autopilot(False)
                    self._cfl_lead.apply_control(
                        carla.VehicleControl(throttle=0.0, brake=1.0))
                    self._cfl_braked = True
                    print(f"[FAULT INJECTION] cfl lead HARD BRAKE at t+{spec[0]:.0f}s")
                if (self._cfl_braked and not self._cfl_cam_killed
                        and fault_elapsed >= spec[0] + 0.8):
                    for actor in world.get_actors().filter("sensor.camera.rgb"):
                        if actor.parent is not None and actor.parent.id == ego.id:
                            actor.destroy()
                    self._cfl_cam_killed = True
                    self._camera_killed = True
                    print(f"[FAULT INJECTION] cfl camera destroyed at t+{spec[0]+0.8:.1f}s (TTC collapsing)")
            except Exception as e:
                print("[FAULT INJECTION] cfl failed:", e)

        # cfl2: 선행차 급제동 -> 에이전트가 차간/접근속도로 TTC 직접 계산
        # -> TTC<0.9s 순간 카메라 파괴 (주입 시점의 TTC를 통제)
        spec = self._parse_fault(self.ros_node.get_parameter("cfl2").value, 0.0)
        if spec:
            try:
                world = CarlaDataProvider.get_world()
                ego = CarlaDataProvider.get_hero_actor()
                # JIT 스폰: 제동 2초 전, 에고 전방 20m, 에고 속도 매칭
                # (조기 스폰 + autopilot은 90m 도주 실측 -> 폐기)
                if (self._cfl_lead is None and ego is not None
                        and fault_elapsed >= spec[0] - 2.0):
                    tf = ego.get_transform(); fwd = tf.get_forward_vector()
                    for ahead in (20.0, 26.0, 32.0):
                        sp = carla.Transform(
                            carla.Location(tf.location.x + fwd.x * ahead,
                                           tf.location.y + fwd.y * ahead,
                                           tf.location.z + 0.5), tf.rotation)
                        bp = world.get_blueprint_library().filter("vehicle.audi.tt")[0]
                        self._cfl_lead = world.try_spawn_actor(bp, sp)
                        if self._cfl_lead is not None:
                            ev = ego.get_velocity()
                            self._cfl_lead.set_target_velocity(ev)
                            print(f"[FAULT INJECTION] cfl2 lead JIT-spawn {ahead:.0f}m ahead, v-matched")
                            break
                if (self._cfl_lead is not None and not self._cfl_braked
                        and fault_elapsed >= spec[0]):
                    self._cfl_lead.set_autopilot(False)
                    self._cfl_lead.apply_control(
                        carla.VehicleControl(throttle=0.0, brake=1.0))
                    self._cfl_braked = True
                    print(f"[FAULT INJECTION] cfl2 lead HARD BRAKE at t+{spec[0]:.0f}s")
                if (self._cfl_braked and not self._cfl_cam_killed
                        and ego is not None and self._cfl_lead is not None):
                    el = ego.get_location(); ll = self._cfl_lead.get_location()
                    gap = ((el.x-ll.x)**2 + (el.y-ll.y)**2) ** 0.5 - 4.5
                    ev = ego.get_velocity(); lv = self._cfl_lead.get_velocity()
                    closing = ((ev.x-lv.x)**2+(ev.y-lv.y)**2)**0.5
                    ttc = gap / closing if closing > 0.5 else 99.0
                    if self._counter % 10 == 0:
                        print(f"[TTCDBG] gap={gap:.1f} closing={closing:.2f} ttc={ttc:.2f}")
                    # 0.9->1.8 상향에도 불발: 에고 AEB가 더 빨랐음. 계측 후 재결정
                    if ttc < 1.8:
                        for actor in world.get_actors().filter("sensor.camera.rgb"):
                            if actor.parent is not None and actor.parent.id == ego.id:
                                actor.destroy()
                        self._cfl_cam_killed = True
                        self._camera_killed = True
                        print(f"[FAULT INJECTION] cfl2 camera destroyed at TTC={ttc:.2f}s gap={gap:.1f}m")
            except Exception as e:
                print("[FAULT INJECTION] cfl2 failed:", e)

        # lfl2: 선행차 급제동 -> 에이전트가 차간/접근속도로 TTC 직접 계산
        # -> TTC<0.9s 순간 카메라 파괴 (주입 시점의 TTC를 통제)
        spec = self._parse_fault(self.ros_node.get_parameter("lfl2").value, 0.0)
        if spec:
            try:
                world = CarlaDataProvider.get_world()
                ego = CarlaDataProvider.get_hero_actor()
                # JIT 스폰: 제동 2초 전, 에고 전방 20m, 에고 속도 매칭
                # (조기 스폰 + autopilot은 90m 도주 실측 -> 폐기)
                if (self._cfl_lead is None and ego is not None
                        and fault_elapsed >= spec[0] - 2.0):
                    tf = ego.get_transform(); fwd = tf.get_forward_vector()
                    for ahead in (20.0, 26.0, 32.0):
                        sp = carla.Transform(
                            carla.Location(tf.location.x + fwd.x * ahead,
                                           tf.location.y + fwd.y * ahead,
                                           tf.location.z + 0.5), tf.rotation)
                        bp = world.get_blueprint_library().filter("vehicle.audi.tt")[0]
                        self._cfl_lead = world.try_spawn_actor(bp, sp)
                        if self._cfl_lead is not None:
                            ev = ego.get_velocity()
                            self._cfl_lead.set_target_velocity(ev)
                            print(f"[FAULT INJECTION] lfl2 lead JIT-spawn {ahead:.0f}m ahead, v-matched")
                            break
                if (self._cfl_lead is not None and not self._cfl_braked
                        and fault_elapsed >= spec[0]):
                    self._cfl_lead.set_autopilot(False)
                    self._cfl_lead.apply_control(
                        carla.VehicleControl(throttle=0.0, brake=1.0))
                    self._cfl_braked = True
                    print(f"[FAULT INJECTION] lfl2 lead HARD BRAKE at t+{spec[0]:.0f}s")
                if (self._cfl_braked and not self._cfl_cam_killed
                        and ego is not None and self._cfl_lead is not None):
                    el = ego.get_location(); ll = self._cfl_lead.get_location()
                    gap = ((el.x-ll.x)**2 + (el.y-ll.y)**2) ** 0.5 - 4.5
                    ev = ego.get_velocity(); lv = self._cfl_lead.get_velocity()
                    closing = ((ev.x-lv.x)**2+(ev.y-lv.y)**2)**0.5
                    ttc = gap / closing if closing > 0.5 else 99.0
                    if ttc < 1.8:
                        for actor in world.get_actors().filter("sensor.lidar.ray_cast"):
                            if (actor.parent is not None and actor.parent.id == ego.id
                                    and "front" in (actor.attributes.get("role_name","")
                                                    + actor.attributes.get("ros_name",""))):
                                actor.destroy()
                        self._cfl_cam_killed = True
                        self._lidar_killed = True
                        print(f"[FAULT INJECTION] lfl2 FRONT LIDAR destroyed at TTC={ttc:.2f}s gap={gap:.1f}m")
            except Exception as e:
                print("[FAULT INJECTION] lfl2 failed:", e)

        # single latched "fault is now live" signal for the experiment judge
        if not self._fault_announced and (
                self._bridge_noise_started or self._camera_killed
                or self._lidar_killed or self._imu_noise_sigma > 0.0
                or self._gnss_noise_sigma > 0.0 or self._gm_sigma > 0.0
                or self._imu_gyro_bias != 0.0
                or self._imu_gyro_drift != 0.0
                or self._rain_applied or self._gnss_blackout):
            self._fault_announced = True
            self._fault_active_pub.publish(Bool(data=True))
            # [0823] 고장 활성화 심시간 원장 — 저지 실명과 무관하게 지연 산출 가능
            try:
                _t = self._aw_time
                _sim = _t.sec + _t.nanosec * 1e-9 if _t is not None else -1.0
                import time as _tm
                with open("/home/taewook/autoware_carla_leaderboard/experiments/"
                          "results/fault_ledger.log", "a") as _f:
                    _f.write(f"{_tm.time():.3f} FAULT_ACTIVE sim={_sim:.3f}\n")
            except Exception:
                pass
            print(f"[FAULT INJECTION] fault ACTIVE at t+{fault_elapsed:.1f}s (announced)")

        # Autoware Timing
        if self._aw_time is None:
            self._steps_before_init += 1
            control = carla.VehicleControl(steer=0,throttle=0,brake=0)
            return control
        
        aw_time = self._aw_time
        
        # Get Infos from Vehicle
        position = self._awp_converter._get_localization()
        velocity = self._awp_converter._get_twist()
        loc_acc = self._awp_converter._get_acceleration()

        # GNSS-PosePub
        ndt_pose_msg = PoseWithCovarianceStamped()
        ndt_pose_msg.header.stamp = aw_time
        ndt_pose_msg.header.frame_id = "map"
        ndt_pose_msg.pose.pose.position = Point(**position["position"])
        ndt_pose_msg.pose.pose.orientation = Quaternion(**position["orientation"])
        if self._gnss_noise_sigma > 0.0:
            # [v5 절제] gn 고장의 실제 주입 지점 — 전용 RNG + 파형 트레이스
            if not hasattr(self, "_fault_rng"):
                import os as _os
                self._fault_rng = np.random.default_rng(
                    int(_os.environ.get("FAULT_SEED", "0")))
                self._fault_idx = 0
                _tp = _os.environ.get("FAULT_TRACE", "")
                self._fault_tf = open(_tp, "a", buffering=1) if _tp else None
            _gn = self._fault_rng.normal(0.0, self._gnss_noise_sigma, 2)
            ndt_pose_msg.pose.pose.position.x += float(_gn[0])
            ndt_pose_msg.pose.pose.position.y += float(_gn[1])
            if self._fault_tf is not None:
                self._fault_tf.write(f"{self._fault_idx},{_gn[0]:.6f},{_gn[1]:.6f}\n")
            self._fault_idx += 1
        # [gm] 1차 Gauss-Markov 상관편향: b <- a*b + N(0, sigma*sqrt(1-a^2)).
        # a=exp(-dt/tau) — 편향이 tau 시간 스케일로 같은 방향을 유지하며 흘러
        # EKF가 실제 이동으로 믿고 추종하는 멀티패스/NLOS형 최악 케이스.
        if self._gm_sigma > 0.0:
            _dt = 0.05  # sim tick
            _a = math.exp(-_dt / max(self._gm_tau, 0.1))
            _drive = self._gm_sigma * math.sqrt(max(1.0 - _a * _a, 1e-9))
            if not hasattr(self, "_fault_rng"):
                import os as _os
                self._fault_rng = np.random.default_rng(
                    int(_os.environ.get("FAULT_SEED", "0")))
                self._fault_idx = 0
                _tp = _os.environ.get("FAULT_TRACE", "")
                self._fault_tf = open(_tp, "a", buffering=1) if _tp else None
            for _i in (0, 1):
                self._gm_bias[_i] = _a * self._gm_bias[_i] + float(
                    self._fault_rng.normal(0.0, _drive))
            ndt_pose_msg.pose.pose.position.x += self._gm_bias[0]
            ndt_pose_msg.pose.pose.position.y += self._gm_bias[1]
        # gb: during a GNSS blackout the pose is simply not published
        if not self._gnss_blackout:
            self._gnss_pose_pub.publish(ndt_pose_msg)

        # Steering Angle
        steering_angle = self._awp_converter._get_steering()
        steering_angle_msg = SteeringReport()
        steering_angle_msg.stamp = aw_time
        steering_angle_msg.steering_tire_angle = steering_angle
        self._steering_report_publisher.publish(steering_angle_msg)

        # Velocity Report
        velocity_report_msg = VelocityReport()
        velocity_report_msg.header.stamp = aw_time
        velocity_report_msg.header.frame_id = 'base_link'
        velocity_report_msg.longitudinal_velocity = velocity["position"]["x"]
        self._velocity_report_publisher.publish(velocity_report_msg)

        # Control Mode Report
        control_mode_report_msg  = ControlModeReport()
        control_mode_report_msg.stamp = aw_time
        control_mode_report_msg.mode = ControlModeReport.AUTONOMOUS
        self._control_mode_report_publisher.publish(control_mode_report_msg)

        # Gear Report (명령 반향)
        gear_report_msg = GearReport()
        gear_report_msg.stamp = aw_time
        gear_report_msg.report = self._gear_cmd_value
        self._gear_report_publisher.publish(gear_report_msg)

        # Goal
        self.publish_global_plan(position["position"])
        # Engage if goal is published and not autonomous
        if self._published_latest and not self._is_autonomous:
            self._engage_publisher.publish(Engage(engage=True))
            print('enage')

        # IMU
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
            # [v5 절제실험] 전용 fault RNG — 전역 np.random과 분리해 팔 간
            # 코드 경로 차이가 파형을 오염 못 하게. FAULT_SEED로 실현 고정,
            # FAULT_TRACE로 샘플 로그 (same-realization 증빙).
            if not hasattr(self, "_fault_rng"):
                import os as _os
                self._fault_rng = np.random.default_rng(
                    int(_os.environ.get("FAULT_SEED", "0")))
                self._fault_idx = 0
                _tp = _os.environ.get("FAULT_TRACE", "")
                self._fault_tf = open(_tp, "a", buffering=1) if _tp else None
            s = self._imu_noise_sigma
            _n = self._fault_rng.normal(0.0, s, 6)
            imu_msg.angular_velocity.x += float(_n[0])
            imu_msg.angular_velocity.y += float(_n[1])
            imu_msg.angular_velocity.z += float(_n[2])
            imu_msg.linear_acceleration.x += float(_n[3] * 10.0)
            imu_msg.linear_acceleration.y += float(_n[4] * 10.0)
            imu_msg.linear_acceleration.z += float(_n[5] * 10.0)
            if self._fault_tf is not None:
                self._fault_tf.write(
                    f"{self._fault_idx},{_n[2]:.6f}\n")
            self._fault_idx += 1
        # imb: constant yaw-rate bias on the gyro z axis (integrates into a
        # growing heading error downstream — see the imb comment in run_step)
        # imd: ramp bias(드리프트)도 같은 축에 합산
        if self._imu_gyro_bias != 0.0 or self._imu_gyro_drift != 0.0:
            imu_msg.angular_velocity.z += float(self._imu_gyro_bias + self._imu_gyro_drift)
        self._imu_pub.publish(imu_msg)
        
        if self._use_v2x_traffic_light:
            # We set all autoware traffic lights to the state of the currently affecting traffic light
            # [v4.6e 계측] 발행 경로 무발신 미스터리 — 100틱마다 상태 출력
            if self._counter % 100 == 0:
                print(f"[TL V2X] tick={self._counter} ids={len(self._traffic_light_ids)}"
                      f" v2x={self._use_v2x_traffic_light}")
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
            self._external_v2x_arbiter_publisher.publish(traffic_light_msg)


        self._ready_state_pub.publish(self.ready_status)

        # Execute this 5 Times, to give time to receive first lidar messages
        if self._steps_before_init < 5:
            time.sleep(0.2)
            self._steps_before_init += 1
            control = carla.VehicleControl(steer=0,throttle=0,brake=0)
            return control
        # wait 100ms
        #if time_sec > 5: time.sleep(0.05)
        #time.sleep(0.2)
        try:
            control_timestamp, control = self._control_queue.get(True, self._time_to_wait_for_control)
            self._last_control = control

        except queue.Empty:
            control_timestamp, control = timestamp, self._last_control

        # We only wanted a longer wait time for the init phase.
        # 0.2 s (was 1.0): during reroute/replan the control stream gaps, and a
        # 1 s blocking wait per tick collapses the sim to ~1 Hz, which the
        # monitor then misreads as every sensor timing out (FTTI 850 ms).
        self._time_to_wait_for_control = 0.2

        return control

    @staticmethod
    def get_ros_version():
        return AutowareE2EAgent.ROS_VERSION

    def set_global_plan(self, global_plan_gps, global_plan_world_coord):
        ds_ids = downsample_route(global_plan_world_coord, 20)
        
        # Ensure the last waypoint is included
        if ds_ids[-1] != len(global_plan_world_coord) - 1:
            ds_ids.append(len(global_plan_world_coord) - 1)
        
        self._global_plan_world_coord = [(global_plan_world_coord[x][0], global_plan_world_coord[x][1]) for x in ds_ids]
        self._global_plan = [global_plan_gps[x] for x in ds_ids]
    
    def _get_extended_final_waypoint(self, global_plan_world_coord, offset_meters=50.0):
        # Use CARLA's waypoint API to follow road topology beyond the final waypoint.
        # This is need because the scenario does not excit at the exact last position
        
        last_wp_transform = global_plan_world_coord[-1][0]
        last_carla_wp = self._map.get_waypoint(last_wp_transform.location)

        accumulated = 0.0
        current_wp = last_carla_wp

        while accumulated < offset_meters:
            next_wps = current_wp.next(2.0)  # step 2m at a time along road
            if not next_wps:
                break
            next_wp = next_wps[0]
            accumulated += 2.0
            current_wp = next_wp

        return (current_wp.transform, global_plan_world_coord[-1][1])

    def _mrm_state_callback(self, msg):
        was = self._mrm_active
        self._mrm_active = msg.data.startswith("MRM")
        if self._mrm_active and not was:
            print(f"[MRM] supervisor took over the route ({msg.data}) — pausing goal updates")

    def publish_global_plan(self, current_position):
        if self._mrm_active:
            return
        if not self._global_plan_world_coord:
            return

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

        # dg=1: direct-goal mode — skip all intermediate waypoints and give
        # Autoware only the final goal. No intermediate goals means nothing
        # can get snapped onto a shoulder while driving fault-free.
        if os.environ.get("dg", "") and self._route_index < total_waypoints - 1:
            self._route_index = total_waypoints - 1
            self._published_latest = False

        is_last_waypoint = self._route_index >= total_waypoints - 1

        # --- Advance route index based on distance ---
        if not is_last_waypoint:
            goal_wp = self._global_plan_world_coord[self._route_index]
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

            if dist < 50.0 or self._goal_mod_failure_counter > 5:
                self._route_index += 1
                self._published_latest = False
                self._goal_mod_failure_counter = 0
                is_last_waypoint = self._route_index >= total_waypoints - 1

        # --- Publish next/final waypoint if not yet published ---
        if self._published_latest:
            return

        next_wp = self._global_plan_world_coord[min(self._route_index, total_waypoints - 1)]
        next_pose_dict = BridgeHelper.carla2ros_pose(
            next_wp[0].location.x, next_wp[0].location.y, next_wp[0].location.z,
            np.deg2rad(next_wp[0].rotation.roll),
            np.deg2rad(next_wp[0].rotation.pitch),
            np.deg2rad(next_wp[0].rotation.yaw),
            to_quat=True
        )

        if is_last_waypoint:
            # Final goal MUST be published via topic so Autoware registers scenario completion
            goal_pose_stamped = PoseStamped()
            goal_pose_stamped.header.frame_id = "map"
            dg_env = os.environ.get("dg", "")
            if ":" in dg_env:
                # dg=<x>:<y>[:<yaw_deg>] — custom final goal in ROS/map coords.
                # mission_planner validates the goal ANGLE against the lane, so
                # the yaw must roughly match the lane heading (identity = east);
                # pass yaw_deg explicitly for lanes pointing elsewhere.
                parts = dg_env.split(":")
                gx, gy = float(parts[0]), float(parts[1])
                yaw = math.radians(float(parts[2])) if len(parts) > 2 else 0.0
                goal_pose_stamped.pose = Pose(
                    position=Point(x=gx, y=gy, z=0.0),
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
            # Intermediate goals via service
            if self._pending_service_call:
                return

            service_request = SetRoutePoints.Request()
            service_request.header.frame_id = "map"
            # Modification must stay on: raw CARLA waypoints can match the
            # opposite lane / land behind the ego, making the planner loop
            # around the block. Shoulder-snap confounds are prevented by
            # goal_planner margin_from_boundary instead (narrow map shoulders
            # can't host the car; MRM targets the widened off-route ones).
            service_request.option.allow_goal_modification = True
            service_request.option.allow_while_using_route = True
            service_request.goal = Pose(
                position=Point(**next_pose_dict["position"]),
                orientation=Quaternion(**next_pose_dict["orientation"])
            )
            self._pending_service_call = True
            future = self._route_service_client.call_async(service_request)
            future.add_done_callback(self._handle_service_response)


            
    def _handle_service_response(self, future):
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
                # Route not initialized yet — publish initial goal via topic to bootstrap Autoware
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
                # Other failure — allow retry, increment failure counter
                self._published_latest = False
                self._goal_mod_failure_counter += 1

        except Exception as e:
            self._pending_service_call = False
            self._published_latest = False
            print('Service call failed:', e)
    
    def _vehicle_control_cmd_callback(self, control_msg):
        control_timestamp, control = self._awp_converter.convert_control(control_msg)



        # Checks that the received control timestamp is not repeated.
        if self._last_control_timestamp is not None and abs(self._last_control_timestamp - control_timestamp) < EPSILON:
            print(
                "\033[93mWARNING 11111: A new vehicle command with a repeated timestamp has been received {} .\033[0m".format(control_timestamp),
                "\033[93mThis vehicle command will be ignored.\033[0m",
                sep=" ")
            return

        # Checks that the received control timestamp is the expected one.
        # We need to retrieve the simulation time directly from the CARLA snapshot instead of using the GameTime object to avoid
        # a race condition between the execution of this callback and the update of the GameTime internal variables.

        # Since we are tracking our own autoware time we need to overwrite the actual carla time
        carla_timestamp = self._aw_time.sec + self._aw_time.nanosec * 1e-9
        # Allow a slightly larger epsilon here to account for some potential jitter in the callback execution
        # Happends rarely but makes the sim stutter.
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
        carla_ackermann_control = AckermannDrive()
        carla_ackermann_control.steering_angle = aw_ackermann_control_command_msg.lateral.steering_tire_angle * 1.2
        carla_ackermann_control.steering_angle_velocity = aw_ackermann_control_command_msg.lateral.steering_tire_rotation_rate * 1.2
        carla_ackermann_control.speed = aw_ackermann_control_command_msg.longitudinal.speed
        carla_ackermann_control.acceleration = aw_ackermann_control_command_msg.longitudinal.acceleration
        carla_ackermann_control.jerk = aw_ackermann_control_command_msg.longitudinal.jerk
        self._control_publiher.publish(carla_ackermann_control)

    def _vehicle_control_cmd_callback3(self, control_msg):
        
        control_timestamp, control = CarlaDataProvider.get_world().get_snapshot().timestamp.elapsed_seconds, carla.VehicleControl(
            steer = control_msg.steer,
            throttle = control_msg.throttle,
            brake = control_msg.brake,
        )


        # Checks that the received control timestamp is not repeated.
        if self._last_control_timestamp is not None and abs(self._last_control_timestamp - control_timestamp) < EPSILON:
            print(
                "\033[93mWARNING 11111: A new vehicle command with a repeated timestamp has been received {} .\033[0m".format(control_timestamp),
                "\033[93mThis vehicle command will be ignored.\033[0m",
                sep=" ")
            return

        # Checks that the received control timestamp is the expected one.
        # We need to retrieve the simulation time directly from the CARLA snapshot instead of using the GameTime object to avoid
        # a race condition between the execution of this callback and the update of the GameTime internal variables.
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
        
    def _gear_cmd_callback(self, msg):
        self._gear_cmd_value = msg.command

    def _acc_callback(self, acc_msg):
        self._awp_converter.getAcc(acc_msg)

    def _aw_time_callback(self, clock_msg):
        self._aw_time = clock_msg.clock

    def _operation_mode_callback(self, operation_mode_msg):
        if operation_mode_msg.mode == OperationModeState.AUTONOMOUS:
            self._is_autonomous = True
        else:
            self._is_autonomous = False
    
    def _get_traffic_lights_from_lanelet(self, lanelet_marker_msg):
        print("Fetch Traffic Lights from Lanelet")
        pattern = re.compile(r"TLRegElemId:(\d+)")
        for marker in lanelet_marker_msg.markers:
            if marker.text:
                match = pattern.search(marker.text)
                if match:
                    self._traffic_light_ids.add(int(match.group(1)))

    def destroy(self):
        """
        Destroy (clean-up) the agent
        :return:
        """
        self.ros_node.destroy_node()
        rclpy.shutdown()
        self.spin_thread.join()

        super(AutowareE2EAgent, self).destroy()
