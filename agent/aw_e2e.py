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
from autoware_vehicle_msgs.msg import SteeringReport, VelocityReport, Engage, ControlModeReport
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
        #self._actuation_report_publisher = self.ros_node.create_publisher(ActuationStatusStamped, "/vehicle/status/actuation_status", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        # Publisher & Subscriber for Goal Planning
        self._goal_publisher = self.ros_node.create_publisher(PoseStamped, "/rviz/routing/rough_goal", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._engage_publisher = self.ros_node.create_publisher(Engage, "/autoware/engage", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
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
        self._external_traffic_light_publisher = self.ros_node.create_publisher(TrafficLightGroupArray, "/perception/traffic_light_recognition/traffic_signals", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))

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
            self.ros_node.declare_parameter("imn", os.environ.get("imn", ""))
            self.ros_node.declare_parameter("gn", os.environ.get("gn", ""))
            # tail=<gap_m>: guarantee an NPC tailing the ego at <gap_m> meters
            # (background traffic spawns mostly ahead — rear-end pressure for
            # in-lane MRM stops needs an explicit chaser)
            self.ros_node.declare_parameter("tail", os.environ.get("tail", ""))
        self._fault_t0 = None
        self._camera_killed = False
        self._lidar_killed = False
        self._tail_spawned = False
        self._bridge_noise_started = set()
        # npc=agg: make background traffic aggressive via the TrafficManager
        self._npc_styled = False
        self._imu_noise_sigma = 0.0
        self._gnss_noise_sigma = 0.0

        # V2X sens gt traffic light states
        # Either the camera image or the v2x need to be set true
        # For visualisation you can also set both to true 
        self._use_v2x_traffic_light = False
        self._publish_cam_image = True
        self._traffic_light_ids = set()  

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

    # After initial initialization the run step method should
    # wait for the next control signal

    def run_step(self, input_data, timestamp):
        timestamp = carla_timestamp = CarlaDataProvider.get_world().get_snapshot().timestamp.elapsed_seconds
        print('COUNTER', self._counter)
        self._counter += 1

        # --- Fault injection (research) ---
        # t counts from AUTONOMOUS engagement, not scenario start: boot eats
        # 40-50 s, so counting from scenario start made "t=60" hit ~10 s into
        # the actual drive and look like a pre-fault MRM.
        if self._fault_t0 is None:
            if self._is_autonomous:
                self._fault_t0 = timestamp
                print(f"[FAULT INJECTION] timer armed at sim {timestamp:.1f}s (engaged)")
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
                for v in CarlaDataProvider.get_world().get_actors().filter("vehicle.*"):
                    if v.attributes.get("role_name") in ("hero", "ego_vehicle"):
                        continue
                    tm.vehicle_percentage_speed_difference(v, -20.0)  # 20% above limit
                    tm.distance_to_leading_vehicle(v, 0.5)            # tailgating
                    tm.ignore_vehicles_percentage(v, 20.0)            # yields less
                    tm.ignore_lights_percentage(v, 10.0)              # runs some lights
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
                        s.destroy(); killed += 1
                    print(f"[FAULT INJECTION] destroyed {killed} camera sensor(s) at t+{fault_camera_delay:.0f}s")
                except Exception as e:
                    print("[FAULT INJECTION] camera destroy failed:", e)
                self._camera_killed = True

        # tail=<gap_m>: spawn a chasing NPC <gap_m> behind the ego once driving
        if not self._tail_spawned and self._is_autonomous and self._counter > 40:
            tail_env = self.ros_node.get_parameter("tail").value
            if tail_env:
                self._tail_spawned = True
                try:
                    gap = float(tail_env)
                    world = CarlaDataProvider.get_world()
                    ego = CarlaDataProvider.get_hero_actor()
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
                            tm.distance_to_leading_vehicle(npc, 0.5 if aggressive else 2.0)
                            tm.vehicle_percentage_speed_difference(
                                npc, -30.0 if aggressive else -10.0)
                            tm.ignore_lights_percentage(npc, 30.0 if aggressive else 0.0)
                            print(f"[TAIL NPC] spawned {gap:.0f}m behind ego"
                                  f" (aggressive={aggressive})")
                        else:
                            print("[TAIL NPC] spawn blocked (occupied), skipping")
                    else:
                        print("[TAIL NPC] no lane waypoint behind ego, skipping")
                except Exception as e:
                    print("[TAIL NPC] spawn failed:", e)

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

        # imn=<t>:<sigma>: IMU noise (sigma rad/s on gyro, 10*sigma m/s^2 on accel)
        # gn=<t>:<sigma>: position noise (meters) on the pose fed to the EKF
        spec = self._parse_fault(self.ros_node.get_parameter("imn").value, 0.05)
        self._imu_noise_sigma = spec[1] if spec and fault_elapsed >= spec[0] else 0.0
        spec = self._parse_fault(self.ros_node.get_parameter("gn").value, 1.0)
        self._gnss_noise_sigma = spec[1] if spec and fault_elapsed >= spec[0] else 0.0

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
            ndt_pose_msg.pose.pose.position.x += float(np.random.normal(0.0, self._gnss_noise_sigma))
            ndt_pose_msg.pose.pose.position.y += float(np.random.normal(0.0, self._gnss_noise_sigma))
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
            s = self._imu_noise_sigma
            imu_msg.angular_velocity.x += float(np.random.normal(0.0, s))
            imu_msg.angular_velocity.y += float(np.random.normal(0.0, s))
            imu_msg.angular_velocity.z += float(np.random.normal(0.0, s))
            imu_msg.linear_acceleration.x += float(np.random.normal(0.0, 10.0 * s))
            imu_msg.linear_acceleration.y += float(np.random.normal(0.0, 10.0 * s))
            imu_msg.linear_acceleration.z += float(np.random.normal(0.0, 10.0 * s))
        self._imu_pub.publish(imu_msg)
        
        if self._use_v2x_traffic_light:
            # We set all autoware traffic lights to the state of the currently affecting traffic light
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
