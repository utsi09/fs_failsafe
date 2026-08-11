#!/usr/bin/env python3
"""출발 거부 디버그: ego 최근접 궤적점 속도 / motion·routing 상태 / 컨트롤 명령을
2초마다 기록. MRM_MOVE 경로 수락 후 차가 교차로에서 안 움직이는 원인 추적용."""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from autoware_planning_msgs.msg import Trajectory
from autoware_control_msgs.msg import Control
from autoware_adapi_v1_msgs.msg import MotionState, RouteState, OperationModeState

OUT = "/home/taewook/autoware_carla_leaderboard/experiments/results/junction_probe.log"

class P(Node):
    def __init__(self):
        import os
        # 노드명 중복 → duplicated_node_checker ERROR → 자율모드 불가 →
        # emergency 래치 (8/10 pkdemo114 실측). PID로 유일성 보장.
        super().__init__(f"junction_probe_{os.getpid()}")
        q = QoSProfile(depth=1)
        qt = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pose = None; self.traj = None; self.ctrl = None
        self.motion = None; self.route = None; self.opmode = None
        self.create_subscription(Odometry, "/localization/kinematic_state",
                                 lambda m: setattr(self, "pose", m), q)
        self.create_subscription(Trajectory, "/planning/trajectory",
                                 lambda m: setattr(self, "traj", m), q)
        self.create_subscription(Control, "/control/command/control_cmd",
                                 lambda m: setattr(self, "ctrl", m), q)
        self.create_subscription(MotionState, "/api/motion/state",
                                 lambda m: setattr(self, "motion", m), qt)
        self.create_subscription(RouteState, "/api/routing/state",
                                 lambda m: setattr(self, "route", m), qt)
        self.create_subscription(OperationModeState, "/api/operation_mode/state",
                                 lambda m: setattr(self, "opmode", m), qt)
        self.f = open(OUT, "a", buffering=1)
        self.create_timer(2.0, self.tick)

    def tick(self):
        if self.pose is None:
            return
        px = self.pose.pose.pose.position.x
        py = self.pose.pose.pose.position.y
        v = math.hypot(self.pose.twist.twist.linear.x, self.pose.twist.twist.linear.y)
        line = f"pos=({px:.1f},{py:.1f}) v={v:.2f}"
        if self.traj and self.traj.points:
            pts = self.traj.points
            dists = [(math.hypot(p.pose.position.x - px, p.pose.position.y - py), i)
                     for i, p in enumerate(pts)]
            d, i = min(dists)
            ahead = len(pts) - 1 - i
            nz = next((j for j in range(i, len(pts))
                       if abs(pts[j].longitudinal_velocity_mps) > 0.05), None)
            stopdist = (sum(math.hypot(pts[k+1].pose.position.x - pts[k].pose.position.x,
                                       pts[k+1].pose.position.y - pts[k].pose.position.y)
                            for k in range(i, len(pts) - 1))
                        if nz is None else -1.0)
            line += (f" traj[n={len(pts)} near_i={i} near_d={d:.1f}"
                     f" near_v={pts[i].longitudinal_velocity_mps:.2f}"
                     f" ahead={ahead} first_moving_i={nz} len_ahead={stopdist:.1f}]")
        if self.ctrl:
            line += (f" ctrl[v={self.ctrl.longitudinal.velocity:.2f}"
                     f" a={self.ctrl.longitudinal.acceleration:.2f}]")
        if self.motion:
            line += f" motion={self.motion.state}"
        if self.route:
            line += f" route={self.route.state}"
        if self.opmode:
            line += f" op={self.opmode.mode}/{self.opmode.is_autonomous_mode_available}"
        import time as _t
        self.f.write(_t.strftime("%H:%M:%S ") + line + "\n")

rclpy.init()
rclpy.spin(P())
