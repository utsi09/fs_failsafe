#!/usr/bin/env python3
"""비단조 기제 녹화기: 인식 객체 수와 에고 전방 최근접 객체 거리를 1Hz 기록.

ln mid(유령 유지) vs high(필터 기각) 가설의 직접 증거용. 러너와 독립 실행.
출력: results/mech_objects.log  (wall_ts, n_objects, nearest_ahead_m, ego_x, ego_y)
"""
import math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from autoware_perception_msgs.msg import PredictedObjects
from nav_msgs.msg import Odometry

OUT = "/home/taewook/autoware_carla_leaderboard/experiments/results/mech_objects.log"

class Rec(Node):
    def __init__(self):
        super().__init__("mech_recorder")
        self.ego = None
        self.yaw = 0.0
        self.n = 0
        self.nearest = float("inf")
        self.create_subscription(Odometry, "/localization/kinematic_state",
                                 self.odom, qos_profile_sensor_data)
        self.create_subscription(PredictedObjects,
                                 "/perception/object_recognition/objects",
                                 self.objs, qos_profile_sensor_data)
        self.f = open(OUT, "a", buffering=1)
        self.create_timer(1.0, self.tick)

    def odom(self, m):
        p = m.pose.pose.position
        self.ego = (p.x, p.y)
        q = m.pose.pose.orientation
        self.yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

    def objs(self, m):
        self.n = len(m.objects)
        if self.ego is None:
            return
        best = float("inf")
        for o in m.objects:
            p = o.kinematics.initial_pose_with_covariance.pose.position
            dx, dy = p.x-self.ego[0], p.y-self.ego[1]
            # 전방 +-60도 원뿔 내 객체만 (진로 봉쇄 후보)
            ang = math.atan2(dy, dx) - self.yaw
            ang = (ang+math.pi) % (2*math.pi) - math.pi
            if abs(ang) < math.radians(60):
                best = min(best, math.hypot(dx, dy))
        self.nearest = best

    def tick(self):
        e = self.ego or (float("nan"), float("nan"))
        near = "" if self.nearest == float("inf") else f"{self.nearest:.1f}"
        self.f.write(f"{time.time():.1f} n={self.n} ahead={near} "
                     f"ego={e[0]:.1f},{e[1]:.1f}\n")

rclpy.init()
rclpy.spin(Rec())
